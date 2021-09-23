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

        CLK_RESET
    } _clk_state;
    _clk_state clk_state;

    typedef enum logic [4:0] {
        CLK2_POLLING,
        CLK2_RESPONSE,
    } _clk_state2;
    _clk_state2 clk_state2;

    typedef enum logic [4:0] {
        CLKDIV2_POLLING,
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

    logic [DATA_LEN*2-1:0] clk_operand;         // Input
    logic [DATA_LEN*2-1:0] clkdiv2_operand;     // Output
    logic clk_to_clkdiv2_empty;                 // Output
    logic clk_to_clkdiv2_full;                  // Output
    logic clk_wrt_en;                           // Input
    logic clkdiv2_rd_en;                        // Input

    async_fifo #(
        .DATA_LEN(DATA_LEN * 2)
    ) clk_to_clkdiv2 (
        .reset(reset),
        .wclk(clk),
        .data_in(clk_operand),
        .wrt_en(clk_wrt_en),
        .wrt_full(clk_to_clkdiv2_full),
        .rclk(clk_div2),
        .data_out(clkdiv2_operand),
        .rd_en(clkdiv2_rd_en),
        .rd_empty(clk_to_clkdiv2_empty)
    );

    logic [DATA_LEN-1:0] clk_result;            // Output
    logic [DATA_LEN-1:0] clkdiv2_result;        // Input
    logic clkdiv2_to_clk_empty;                 // Output
    logic clkdiv2_to_clk_full;                  // Output
    logic clkdiv2_wrt_en;                       // Input
    logic clk_rd_en;                            // Input

    async_fifo #(
        .DATA_LEN(DATA_LEN)
    ) clkdiv2_to_clk (
        .reset(reset),
        .wclk(clk_div2),
        .data_in(clkdiv2_result),
        .wrt_en(clkdiv2_wrt_en),
        .wrt_full(clkdiv2_to_clk_full),
        .rclk(clk),
        .data_out(clk_result),
        .rd_en(clk_rd_en),
        .rd_empty(clkdiv2_to_clk_empty)
    );

    logic clk_reset_signal;
    logic clkdiv2_reset_signal;
    logic clk_reset_en;
    logic clkdiv2_reset_en;
    logic clk_reset_full;
    logic clkdiv2_reset_empty;

    async_fifo #(
        .DATA_LEN(1)
    ) signal_clk_to_clkdiv2 (
        .reset(reset),
        .wclk(clk),
        .data_in(clk_reset_signal),
        .wrt_en(clk_reset_en),
        .wrt_full(clk_reset_full),
        .rclk(clk_div2),
        .data_out(clkdiv2_reset_signal),
        .rd_en(clkdiv2_reset_en),
        .rd_empty(clkdiv2_reset_empty)
    );

    always_ff @(posedge clk) begin
        if(reset) begin
            fiu.c0Tx.valid <= 1'b0;
            fiu.c1Tx.valid <= 1'b0;

            clk_rd_en <= 1'b0;
            clk_wrt_en <= 1'b0;
            clk_operand <= {DATA_LEN*2{1'b0}};

            clk_reset_en <= 1'b0;
            clk_reset_signal <= 1'b0;

            clk_state <= CLK_WAITING_INPUT;
            clk_state2 <= CLK2_POLLING;
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

                clk_wrt_en <= 1'b0;
                clk_operand <= {DATA_LEN*2{1'b0}};

                clk_reset_en <= 1'b0;
                clk_reset_signal <= 1'b0;

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
                    clk_state <= CLK_IDLE;

                    clk_operand <= fiu.c0Rx.data[0+:DATA_LEN*2];
                    clk_wrt_en <= 1'b1;
                end
            end else if(clk_state == CLK_RESET) begin
                $display("CLK: Got reset signal, wait for synchronizing with CLKDIV2");
                clk_state <= CLK_IDLE;

                fiu.c0Tx.valid <= 1'b0;
                clk_wrt_en <= 1'b0;
                clk_operand <= {DATA_LEN*2{1'b0}};

                clk_reset_en <= 1'b1;
                clk_reset_signal <= 1'b1;
            end

            if(clk_state2 == CLK2_POLLING) begin
                fiu.c1Tx.valid <= 1'b0;
                if(!clkdiv2_to_clk_empty) begin
                    $display("CLK: FIFO has data, read it");
                    clk_state2 <= CLK2_RESPONSE;

                    clk_rd_en <= 1'b1;
                end
            end else if(clk_state2 == CLK2_RESPONSE) begin
                $display("CLK: Got a number result(%d)", clk_result);
                clk_state2 <= CLK2_POLLING;

                clk_rd_en <= 1'b0;

                fiu.c1Tx.valid <= 1'b1;
                fiu.c1Tx.hdr <= output_buffer_write_hdr;
                fiu.c1Tx.data <= t_ccip_clData'({448'b0, clk_result[DATA_LEN-1:0], 32'b1});
            end
        end
    end

    always_ff @(posedge clk_div2) begin
        if(reset) begin
            d_a <= {DATA_LEN{1'b0}};
            d_b <= {DATA_LEN{1'b0}};

            clkdiv2_rd_en <= 1'b0;
            clkdiv2_wrt_en <= 1'b0;
            clkdiv2_result <= {DATA_LEN{1'b0}};

            clk_div2_state <= CLKDIV2_POLLING;
        end else begin
            if(clk_div2_state == CLKDIV2_POLLING) begin
                clkdiv2_wrt_en <= 1'b0;
                clkdiv2_result <= {DATA_LEN{1'b0}};

                if(!clk_to_clkdiv2_empty) begin
                    $display("CLKDIV2: FIFO has data, read it");
                    clk_div2_state <= CLKDIV2_OP;

                    clkdiv2_rd_en <= 1'b1;
                end else if(!clkdiv2_reset_empty) begin
                    $display("CLKDIV2: Got reset signal");
                    clk_div2_state <= CLKDIV2_RESET;

                    clkdiv2_reset_en <= 1'b1;
                end
            end else if(clk_div2_state == CLKDIV2_OP) begin
                $display("CLKDIV2: Got two operands a(%d), b(%d)", clkdiv2_operand[0+:DATA_LEN], clkdiv2_operand[DATA_LEN+:DATA_LEN]);
                clk_div2_state <= CLKDIV2_WAIT;

                clkdiv2_rd_en <= 1'b0;

                d_a <= clkdiv2_operand[0+:DATA_LEN];
                d_b <= clkdiv2_operand[DATA_LEN+:DATA_LEN];
            end else if(clk_div2_state == CLKDIV2_WAIT) begin
                $display("CLKDIV2: Wait for 1 cycle");
                clk_div2_state <= CLKDIV2_RESULT;

                d_a <= {DATA_LEN{1'b0}};
                d_b <= {DATA_LEN{1'b0}};
            end else if(clk_div2_state == CLKDIV2_RESULT) begin
                $display("CLKDIV2: Operation done with result(%d)", d_result);
                clk_div2_state <= CLKDIV2_POLLING;

                clkdiv2_wrt_en <= 1'b1;
                clkdiv2_result <= d_result;
            end else if(clk_div2_state == CLKDIV2_RESET) begin
                $display("CLKDIV2: Reset");
                clk_div2_state <= CLKDIV2_POLLING;

                clkdiv2_reset_en <= 1'b0;

                d_a <= {DATA_LEN{1'b0}};
                d_b <= {DATA_LEN{1'b0}};
            end
        end
    end

    assign fiu.c2Tx.mmioRdValid = 1'b0;

endmodule