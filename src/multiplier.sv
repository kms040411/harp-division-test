module multiplier
    #(
        parameter DATA_LEN = 32,
        parameter PIPELINE_STAGE = 2
    )(
        input clk,
        input reset,
        
        input [DATA_LEN-1:0] a,
        input [DATA_LEN-1:0] b,

        output reg [DATA_LEN-1:0] result
    );

    wire signed [DATA_LEN-1:0] input1, input2;
    assign input1 = $signed(a);
    assign input2 = $signed(b);

    wire [2*DATA_LEN-1:0] mult_out;
    assign mult_out = input1 * input2;

    always_ff @(posedge clk) begin
        if(reset) begin
            result <= {DATA_LEN{1'b0}};
        end else begin
            result <= mult_out[DATA_LEN-1:0 ];
        end
    end

endmodule