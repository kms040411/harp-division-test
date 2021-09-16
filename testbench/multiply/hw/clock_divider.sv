module clock_divider
    (
        input clk,
        input reset,

        output reg clk_div2
    );

    always @(posedge clk) begin
        if(reset) begin
            clk_div2 <= 1'b0;
        end else begin
            clk_div2 <= ~clk_div2;
        end
    end

endmodule