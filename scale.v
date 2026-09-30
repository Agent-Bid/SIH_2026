// Fixed-point scale: out = in * factor / 4096 (factor 4096 = 1.0), over two clocks.
// The multiply is built from logic (the hardware multipliers compute unsigned in this toolchain),
// and in one clock it was too slow on the chip in some layouts, although the timing report showed
// slack: some products came out wrong. So it is split: clock 1 multiplies by the low 7 and the high
// 6 bits of factor separately (two shorter multipliers), clock 2 adds them. Same result, 2 clocks late.
module scale (
    input  wire               clk,
    input  wire signed [11:0] in,
    input  wire        [12:0] factor,
    output reg  signed [11:0] out
);

    wire signed [7:0]  f_lo = {1'b0, factor[6:0]};     // zero-extend so the multiplies are signed
    wire signed [6:0]  f_hi = {1'b0, factor[12:7]};
    reg  signed [19:0] p_lo;                          // in * factor[6:0]
    reg  signed [18:0] p_hi;                          // in * factor[12:7]
    wire signed [25:0] product = (p_hi <<< 7) + p_lo;

    always @(posedge clk) begin
        p_lo <= in * f_lo;
        p_hi <= in * f_hi;
        out  <= product >>> 12;
    end

endmodule
