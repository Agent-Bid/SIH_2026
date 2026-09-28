// Fixed-point scale: out = in * factor / 4096 (factor 4096 = 1.0).
module scale (
    input  wire               clk,
    input  wire signed [11:0] in,
    input  wire        [12:0] factor,
    output reg  signed [11:0] out
);

    wire signed [13:0] factor_s = {1'b0, factor};   // zero-extend so the multiply is signed
    wire signed [25:0] product  = in * factor_s;

    always @(posedge clk)
        out <= product >>> 12;

endmodule
