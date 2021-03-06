module async_fifo
    #(
        parameter DATA_LEN = 16,
        parameter ADDR_LEN = 4,
        parameter FIFO_DEPTH = 1 << ADDR_LEN
    )(
        input reset,

        input wclk,
        input [DATA_LEN-1:0] data_in,
        input wrt_en,
        output wrt_full,

        input rclk,
        output [DATA_LEN-1:0] data_out,
        input rd_en,
        output rd_empty
    );

    dcfifo #(
        .lpm_width(DATA_LEN),
        .lpm_widthu(ADDR_LEN),
        .lpm_numwords(FIFO_DEPTH),
        .lpm_showahead("OFF"),
        .lpm_type("dcfifo"),
        .lpm_hint("RAM_BLOCK_TYPE=M20K,MAXIMUM_DEPTH=32,DISABLE_EMBEDDED_TIMING_CONSTRAINT=TRUE"),
        .rdsync_delaypipe(0),
        .wrsync_delaypipe(0),
        .read_aclr_synch("ON"),
        .write_aclr_synch("ON"),
        .enable_ecc("FALSE")
    ) dcfifo_unit (
        .aclr(reset),

        .wrclk(wclk),
        .wrreq(wrt_en),
        .data(data_in),
        .wrfull(wrt_full),
        .wrempty(),
        .wrusedw(),

        .rdclk(rclk),
        .rdreq(rd_en),
        .q(data_out),
        .rdempty(rd_empty),
        .rdfull(),
        .rdusedw(),

        .eccstatus()
    );

endmodule

/*module async_fifo
    #(
        parameter DATA_LEN = 16,
        parameter ADDR_LEN = 4,
        parameter FIFO_DEPTH = 1 << ADDR_LEN
    )(
        input reset,

        input wclk,
        input [DATA_LEN-1:0] data_in,
        input wrt_en,
        output reg wrt_full,

        input rclk,
        output reg [DATA_LEN-1:0] data_out,
        input rd_en,
        output reg rd_empty
    );
    // Reference: http://www.asic-world.com/examples/verilog/asyn_fifo.html

    reg [DATA_LEN-1:0] mem [FIFO_DEPTH-1:0];
    wire [ADDR_LEN-1:0] pNextWordToWrite;   // Write Pointer
    wire [ADDR_LEN-1:0] pNextWordToRead;    // Read Pointer
    wire EqualAddresses;
    wire NextWriteAddressEn;
    wire NextReadAddressEn;
    wire Set_Status;
    wire Rst_Status;
    reg Status;
    wire PresetFull;
    wire PresetEmpty;

    always_ff @(posedge rclk) begin
        if(rd_en & !rd_empty) begin
            data_out <= mem[pNextWordToRead];
        end
    end

    always_ff @(posedge wclk) begin
        if(wrt_en & !wrt_full) begin
            mem[pNextWordToWrite] <= data_in;
        end
    end

    assign NextWriteAddressEn = wrt_en & ~wrt_full;
    assign NextReadAddressEn = rd_en & ~rd_empty;

    gray_counter #(
        .ADDR_LEN(ADDR_LEN)
    ) graycounter_wr (
        .clk(wclk),
        .reset(reset),
        .en(NextWriteAddressEn),
        .out(pNextWordToWrite)
    );

    gray_counter #(
        .ADDR_LEN(ADDR_LEN)
    ) graycounter_rd (
        .clk(rclk),
        .reset(reset),
        .en(NextReadAddressEn),
        .out(pNextWordToRead)
    );

    assign EqualAddresses = (pNextWordToRead == pNextWordToWrite);

    assign Set_Status = (pNextWordToWrite[ADDR_LEN-2] ~^ pNextWordToRead[ADDR_LEN-1]) &
                         (pNextWordToWrite[ADDR_LEN-1] ^  pNextWordToRead[ADDR_LEN-2]);
    assign Rst_Status = (pNextWordToWrite[ADDR_LEN-2] ^  pNextWordToRead[ADDR_LEN-1]) &
                         (pNextWordToWrite[ADDR_LEN-1] ~^ pNextWordToRead[ADDR_LEN-2]);

    always @(Set_Status, Rst_Status, reset) begin
        if (Rst_Status | reset) begin
            Status = 0; // Going Empty
        end else if(Set_Status) begin
            Status = 1; // Going Full
        end
    end

    assign PresetFull = Status & EqualAddresses;

    always_ff @(posedge wclk or posedge PresetFull) begin
        if(reset) begin
            wrt_full <= 1'b0;
        end else begin
            if(PresetFull) begin
                wrt_full <= 1'b1;
            end else begin
                wrt_full <= 1'b0;
            end
        end
    end

    assign PresetEmpty = ~Status & EqualAddresses;

    always_ff @(posedge rclk or posedge PresetEmpty) begin
        if(reset) begin
            rd_empty <= 1'b1;
        end else begin
            if(PresetEmpty) begin
                rd_empty <= 1'b1;
            end else begin
                rd_empty <= 1'b0;
            end
        end
    end

endmodule*/