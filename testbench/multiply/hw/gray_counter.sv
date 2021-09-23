module gray_counter
    #(
        parameter ADDR_LEN = 4
    )(
        input clk;
        input reset;
        input en;

        output [ADDR_LEN-1:0] out;  // gray code counter
    );
    // Reference: http://www.asic-world.com/examples/verilog/asyn_fifo.html

    reg [ADDR_LEN-1:0] counter; // binary code counter

    always_ff @(posedge clk) begin
        if(reset) begin
            counter <= {ADDR_LEN{1'b0}} + 1;
            out <= {ADDR_LEN{1'b0}};
        end else if(en) begin
            counter <= counter + 1;
            // Convert binary code to gray code [num ^ (num >> 1)]
            out <= {counter[ADDR_LEN-1], counter[ADDR_LEN-2:0] ^ counter[ADDR_LEN-1:1]};
        end
    end

endmodule