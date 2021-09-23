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

    assign Set_Status = (pNextWordToWrite[ADDRESS_LEN-2] ~^ pNextWordToRead[ADDRESS_LEN-1]) &
                         (pNextWordToWrite[ADDRESS_LEN-1] ^  pNextWordToRead[ADDRESS_LEN-2]);
    assign Rst_Status = (pNextWordToWrite[ADDRESS_LEN-2] ^  pNextWordToRead[ADDRESS_LEN-1]) &
                         (pNextWordToWrite[ADDRESS_LEN-1] ~^ pNextWordToRead[ADDRESS_LEN-2]);

    always @(Set_Status, Rst_Status, reset) begin
        if (Rst_Status | reset) begin
            Status = 0; // Going Empty
        end else if(Set_Status) begin
            Status = 1; // Going Full
        end
    end

    assign PresetFull = Status & EqualAddresses;

    always_ff @(posedge wclk or posedge PresetFull) begin
        if(PresetFull) begin
            wrt_full <= 1'b1;
        end else begin
            wrt_full <= 1'b0;
        end
    end

    assign PresetEmpty = ~Status & EqualAddresses;

    always_ff @(posedge rclk or posedge PresetEmpty) begin
        if(PresetEmpty) begin
            rd_empty <= 1'b1;
        end else begin
            rd_empty <= 1'b0;
        end
    end

endmodule