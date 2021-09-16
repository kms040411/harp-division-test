module multiplier
    #(
        parameter DATA_LEN = 32,
        parameter PIPELINE_STAGE = 2
    )(
        input clk,
        input reset,
        
        input [DATA_LEN-1:0] a,
        input [DATA_LEN-1:0] b,

        output [DATA_LEN-1:0] result
    );

    wire signed [DATA_LEN-1:0] input1, input2;
    assign input1 = $signed(a);
    assign input2 = $signed(b);

    /*wire [DATA_LEN-1:0] immMult;
    lpm_mult #(
        .lpm_type("lpm_mult"),
        .lpm_widtha(DATA_LEN),
        .lpm_widthb(DATA_LEN),
        .lpm_widthp(DATA_LEN),
        .lpm_widths(2 * DATA_LEN),
        .lpm_representation("SIGNED"),
        .lpm_pipeline(PIPELINE_STAGE)
    ) mult_unit (
        .clock(clk),
        .clken(1'b1),
        .aclr(reset),
        .sclr(reset),
        .dataa(input1),
        .datab(input2),
        .sum({DATA_LEN{1'b0}}),
        .result(immMult)
    );*/
    reg [DATA_LEN-1:0] mult_out;
    reg [DATA_LEN-1:0] mult_out_q;
    always_ff @(posedge clk) begin
        if(reset) begin
            mult_out <= {DATA_LEN{1'b0}};
            mult_out_q <= {DATA_LEN{1'b0}};
        end else begin
            mult_out <= input1 * input2;
            mult_out_q <= mult_out;
        end
    end

    reg [DATA_LEN-1:0] result_out;
    always_ff @(posedge clk) begin
        $display("%d %d immMult: %d", input1, input2,immMult);
        if(reset) begin
            result_out <= {DATA_LEN{1'b0}};
        end else begin
            result_out <= mult_out_q;
        end
    end

    assign result = result_out;

endmodule