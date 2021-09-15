module divider
    #(
        parameter DATA_LEN = 32
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

    wire [DATA_LEN-1:0] immDiv;
    lpm_divide #(
        .lpm_type("lpm_divide"),
        .lpm_widthn(DATA_LEN),
        .lpm_widthd(DATA_LEN),
        .lpm_nrepresentation("SIGNED"),
        .lpm_drepresentation("SIGNED"),
        .lpm_pipeline(5)
    ) divider (
        .clock(clk),
        .clken(1'b1),
        .aclr(reset),
        .numer(input1),
        .denom(input2),
        .quotient(immDiv),
        .remain()
    );

    reg [DATA_LEN-1:0] result_out;
    always_ff @(posedge clk) begin
        if(reset) begin
            result_out <= {DATA_LEN{1'b0}};
        end else begin
            result_out <= immDiv;
        end
    end

    assign result = result_out;

endmodule