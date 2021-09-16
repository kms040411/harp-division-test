`include "cci_mpf_if.vh"
`include "csr_mgr.vh"
`include "afu_json_info.vh"

module app_afu(
    input logic clk,
    cci_mpf_if.to_fiu fiu,      // Connection toward the host.
    app_csrs.app csrs,          // CSR connections
    input logic c0NotEmpty,     // MPF tracks outstanding requests. These will be true as long as
    input logic c1NotEmpty      // reads or unacknowledged writes are still in flight.
);

    // Local reset to reduce fan-out
    logic reset = 1'b1;
    always @(posedge clk)
    begin
        reset <= fiu.reset;
    end

    logic clk_div2;
    clock_divider clk_divider(
        .clk(clk),
        .reset(reset),
        .clk_div2(clk_div2)
    );

    // =========================================================================
    //   Byte Address (CPU uses) <-> Line Address (FPGA uses)
    // =========================================================================

    localparam CL_BYTE_IDX_BITS = 6;
    typedef logic [$bits(t_cci_clAddr) + CL_BYTE_IDX_BITS - 1 : 0] t_byteAddr;

    function automatic t_cci_clAddr byteAddrToClAddr(t_byteAddr addr);
        return addr[CL_BYTE_IDX_BITS +: $bits(t_cci_clAddr)];
    endfunction

    function automatic t_byteAddr clAddrToByteAddr(t_cci_clAddr addr);
        return {addr, CL_BYTE_IDX_BITS'(0)};
    endfunction

    // =========================================================================
    //   CSR Handling
    // =========================================================================

    // Initialize Read CSRs
    always_comb begin
        csrs.afu_id = `AFU_ACCEL_UUID;

        for (int i=0; i<NUM_APP_CSRS; i=i+1) begin
            csrs.cpu_rd_csrs[i].data = 64'(0);
        end
    end

    // CSR write handling variables
    logic is_fn_written;
    assign is_fn_written = csrs.cpu_wr_csrs[0].en;
    logic is_input_buf_written;
    assign is_input_buf_written = csrs.cpu_wr_csrs[1].en;
    logic is_output_buf_written;
    assign is_output_buf_written = csrs.cpu_wr_csrs[2].en;
    logic is_reset_signal_written;
    assign is_reset_signal_written = csrs.cpu_wr_csrs[3].en;

    // =========================================================================
    //   Main AFU logic
    // =========================================================================
    typedef enum logic [4:0] {
        STATE_WAITING_INPUT,
        STATE_WAITING_OUTPUT,
        STATE_IDLE,

        STATE_REQUEST,
        STATE_OP,
        STATE_WAIT,
        STATE_RESPONSE,

        STATE_RESET
    } t_state;
    t_state state;

    t_ccip_clAddr input_addr;
    t_ccip_clAddr output_addr;

    // Input buffer Read Header
    t_cci_mpf_c0_ReqMemHdr input_buffer_read_hdr;
    t_cci_mpf_ReqMemHdrParams input_buffer_read_params;
    always_comb begin
        input_buffer_read_params = cci_mpf_defaultReqHdrParams(1);
        input_buffer_read_params.vc_sel = eVC_VL0;
        input_buffer_read_params.cl_len = eCL_LEN_1;
        input_buffer_read_hdr = cci_mpf_c0_genReqHdr(eREQ_RDLINE_I,
                                                     input_addr,
                                                     t_cci_mdata'(0),
                                                     input_buffer_read_params);
    end

    // Output buffer Write Header
    t_cci_mpf_c1_ReqMemHdr output_buffer_write_hdr;
    t_cci_mpf_ReqMemHdrParams output_buffer_write_params;
    always_comb begin
        output_buffer_write_params = cci_mpf_defaultReqHdrParams(1);
        output_buffer_write_params.vc_sel = eVC_VL0;
        output_buffer_write_params.cl_len = eCL_LEN_1;
        output_buffer_write_hdr = cci_mpf_c1_genReqHdr(eREQ_WRLINE_I,
                                                       output_addr,
                                                       t_cci_mdata'(0),
                                                       output_buffer_write_params);
    end

    // DUT
    localparam DATA_LEN = 32;
    localparam PIPELINE_STAGE = 2;

    logic d_reset;
    logic [DATA_LEN-1:0] d_a;
    logic [DATA_LEN-1:0] d_b;
    logic [DATA_LEN-1:0] d_result;

    assign d_reset = (reset || (state == STATE_RESET));

    multiplier #(
        .DATA_LEN(DATA_LEN),
        .PIPELINE_STAGE(PIPELINE_STAGE)
    ) multiplier_unit (
        .clk(clk_div2),
        .reset(d_reset),
        .a(d_a),
        .b(d_b),
        .result(d_result)
    );

    // State Machine
    assign fiu.c0Tx.valid = (state == STATE_REQUEST);
    assign fiu.c1Tx.valid = (state == STATE_RESPONSE);

    int cycle_wait;
    always_ff @(posedge clk_div2) begin
        if(reset) begin
            input_addr <= t_cci_clAddr'(0);
            output_addr <= t_cci_clAddr'(0);

            d_a <= {DATA_LEN{1'b0}};
            d_b <= {DATA_LEN{1'b0}};

            cycle_wait <= PIPELINE_STAGE;

            state <= STATE_WAITING_INPUT;
        end else begin
            if((state == STATE_WAITING_INPUT) && is_input_buf_written) begin
                state <= STATE_WAITING_OUTPUT;
                $display("AFU read Input buffer address");

                input_addr <= byteAddrToClAddr(csrs.cpu_wr_csrs[1].data);
            end else if((state == STATE_WAITING_OUTPUT) && is_output_buf_written) begin
                state <= STATE_IDLE;
                $display("AFU read Output buffer address");

                output_addr <= byteAddrToClAddr(csrs.cpu_wr_csrs[2].data);
            end else if(state == STATE_IDLE) begin
                cycle_wait <= PIPELINE_STAGE;

                if(is_fn_written) begin
                    $display("AFU got start signal, send it to divider");
                    state <= STATE_REQUEST;
                end else if(is_reset_signal_written) begin
                    $display("AFU got reset signal, send it to divider");
                    state <= STATE_RESET;
                end else begin
                    state <= STATE_IDLE;
                end
            end else if(state == STATE_REQUEST) begin
                $display("AFU sent input buffer read request");
                state <= STATE_OP;
                
                fiu.c0Tx.hdr <= input_buffer_read_hdr;
            end else if(state == STATE_OP) begin
                if(cci_c0Rx_isReadRsp(fiu.c0Rx)) begin
                    $display("AFU got two number a(%d), b(%d)", fiu.c0Rx.data[31:0], fiu.c0Rx.data[63:32]);
                    state <= STATE_WAIT;

                    d_a <= fiu.c0Rx.data[31:0];
                    d_b <= fiu.c0Rx.data[63:32];
                end
            end else if(state == STATE_WAIT) begin
                d_a <= {DATA_LEN{1'b0}};
                d_b <= {DATA_LEN{1'b0}};

                if(cycle_wait == 0) begin
                    $display("Stop waiting");
                    state <= STATE_RESPONSE;
                end else begin
                    $display("Remain %d cycles", cycle_wait);
                    cycle_wait <= cycle_wait - 1;
                end 
            end else if(state == STATE_RESPONSE) begin
                $display("AFU sent result (%d)", d_result);
                state <= STATE_IDLE;

                fiu.c1Tx.hdr <= output_buffer_write_hdr;
                fiu.c1Tx.data <= t_ccip_clData'({448'b0, d_result, 32'b1});
            end else if(state == STATE_RESET) begin
                $display("Reset divider");
                state <= STATE_IDLE;

                d_a <= {DATA_LEN{1'b0}};
                d_b <= {DATA_LEN{1'b0}};
                cycle_wait <= PIPELINE_STAGE;
            end else begin
                d_a <= {DATA_LEN{1'b0}};
                d_b <= {DATA_LEN{1'b0}};
                cycle_wait <= PIPELINE_STAGE;
            end
        end
    end

    assign fiu.c2Tx.mmioRdValid = 1'b0;

endmodule