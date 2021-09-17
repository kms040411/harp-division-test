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

    // Clock divider
    logic clk_div2;
    always_ff @(posedge clk) begin
        if(reset) begin
            clk_div2 <= 1'b0;
        end else begin
            clk_div2 <= ~clk_div2;
        end
    end

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
        CLK_WAITING_INPUT,
        CLK_WAITING_OUTPUT,
        CLK_IDLE,

        CLK_REQUEST,
        CLK_REQUEST_FILL_BUF,
        CLK_WAIT,
        CLK_RESPONSE,

        CLK_RESET
    } _clk_state;
    _clk_state clk_state;

    typedef enum logic [4:0] {
        CLKDIV2_IDLE,

        CLKDIV2_OP,
        CLKDIV2_WAIT,
        CLKDIV2_RESULT,

        CLKDIV2_RESET
    } _clk_div2_state;
    _clk_div2_state clk_div2_state;

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

    // DUT @ clk_div2 domain
    localparam DATA_LEN = 32;
    localparam PIPELINE_STAGE = 2;

    logic d_reset;
    logic [DATA_LEN-1:0] d_a;
    logic [DATA_LEN-1:0] d_b;
    logic [DATA_LEN-1:0] d_result;

    assign d_reset = (reset || (clk_div2_state == CLKDIV2_RESET));

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

    logic [511:0] read_buffer;
    logic [DATA_LEN-1:0] result_buffer;

    // State Machine @ clk domain
    always_ff @(posedge clk) begin
        if(reset) begin
            fiu.c0Rx.valid <= 1'b0;
            fiu.c1Rx.valid <= 1'b0;

            read_buffer <= {512{1'b0}};

            clk_state <= CLK_WAITING_INPUT;
        end else begin
            if((clk_state == CLK_WAITING_INPUT) && is_input_buf_written) begin
                $display("CLK: Read input buffer address");
                clk_state <= CLK_WAITING_OUTPUT;

                input_addr <= byteAddrToClAddr(csrs.cpu_wr_csrs[1].data);
            end else if((clk_state == CLK_WAITING_OUTPUT) && is_output_buf_written) begin
                $display("CLK: Read output buffer address");
                clk_state <= CLK_IDLE;

                output_addr <= byteAddrToClAddr(csrs.cpu_wr_csrs[2].data);
            end else if(clk_state == CLK_IDLE) begin
                fiu.c0Tx.valid <= 1'b0;
                fiu.c1Tx.valid <= 1'b0;

                if(is_fn_written) begin
                    $display("CLK: Got start signal, send it to divider");
                    clk_state <= CLK_REQUEST;
                end else if(is_reset_signal_written) begin
                    $display("CLK: Got reset signal, send it to divider");
                    clk_state <= CLK_RESET;
                end else begin
                    clk_state <= CLK_IDLE;
                end
            end else if(clk_state == CLK_REQUEST) begin
                $display("CLK: Send input buffer read request");
                clk_state <= CLK_REQUEST_FILL_BUF;
                
                fiu.c0Tx.valid <= 1'b1;
                fiu.c0Tx.hdr <= input_buffer_read_hdr;
            end else if(clk_state == CLK_REQUEST_FILL_BUF) begin
                fiu.c0Tx.valid <= 1'b0;
                if(cci_c0Rx_isReadRsp(fiu.c0Rx)) begin
                    $display("CLK: Got two number a(%d), b(%d)", fiu.c0Rx.data[0+:DATA_LEN], fiu.c0Rx.data[DATA_LEN+:DATA_LEN]);
                    clk_state <= CLK_WAIT;

                    read_buffer <= fiu.c0Rx.data[511:0];
                end
            end else if(clk_state == CLK_WAIT) begin
                $display("CLK: Wait until CLK_DIV2 proceeds");
                if(clk_div2_state == CLKDIV2_RESULT) begin
                    clk_state <= CLK_RESPONSE;
                end
            end else if(clk_state == CLK_RESPONSE) begin
                $display("CLK: Send output buffer write request with result(%d)", result_buffer[DATA_LEN-1:0]);
                clk_state <= CLK_IDLE;

                fiu.c1Tx.valid <= 1'b1;
                fiu.c1Tx.hdr <= output_buffer_write_hdr;
                fiu.c1Tx.data <= t_ccip_clData'({448'b0, result_buffer[DATA_LEN-1:0], 32'b1});
            end else if(clk_state == CLK_RESET) begin
                $display("CLK: Got reset signal, wait for synchronizing with CLKDIV2");

                fiu.c0Rx.valid <= 1'b0;
                fiu.c1Rx.valid <= 1'b0;
                read_buffer <= {512{1'b0}};

                if(clk_div2_state == CLKDIV2_RESET) begin
                    clk_state <= CLK_IDLE;
                end
            end
        end
    end

    // State Machine @ clk_div2 domain
    always_ff @(posedge clk_div2) begin
        if(reset) begin
            clk_div2_state <= CLKDIV2_IDLE;
            result_buffer <= {DATA_LEN{1'b0}};
        end else begin
            if(clk_div2_state == CLKDIV2_IDLE) begin
                $display("CLKDIV2: Wait until CLK proceeds");
                if(clk_state == CLK_WAIT) begin
                    clk_div2_state <= CLKDIV2_OP;
                end else if(clk_state == CLK_RESET) begin
                    clk_div2_state <= CLKDIV2_RESET;
                end
            end else if(clk_div2_state == CLKDIV2_OP) begin
                $display("CLKDIV2: Read the buffer with a(%d), b(%d)", read_buffer[0+:DATA_LEN], read_buffer[DATA_LEN+:DATA_LEN]);
                clk_div2_state <= CLKDIV2_WAIT;

                d_a <= read_buffer[0+:DATA_LEN];
                d_b <= read_buffer[DATA_LEN+:DATA_LEN];
            end else if(clk_div2_state == CLKDIV2_WAIT) begin
                $display("CLKDIV2: Wait for 1 cycle");
                clk_div2_state <= CLKDIV2_RESULT;

                d_a <= {DATA_LEN{1'b0}};
                d_b <= {DATA_LEN{1'b0}};
            end else if(clk_div2_state == CLKDIV2_RESULT) begin
                $display("CLKDIV2: Operation done with result(%d)", d_result);
                clk_div2_state <= CLKDIV2_IDLE;

                result_buffer <= d_result;
            end else if(clk_div2_state == CLKDIV2_RESET) begin
                $display("CLKDIV2: Got reset signal");
                clk_div2_state <= CLKDIV2_IDLE;

                result_buffer <= {DATA_LEN{1'b0}};
            end
        end
    end

    assign fiu.c2Tx.mmioRdValid = 1'b0;

endmodule