// Frequency sweep.
//   LFM (geo = 0):       ftw moves by step every clock, so the frequency changes by the same
//                        number of Hz each clock.
//   Geometric (geo = 1): every 32 clocks, ftw grows by ftw * step / 2^32, so the frequency is
//                        multiplied by the same ratio (1 + step / 2^32) each time: equal time
//                        per octave.
// The product ftw * step / 2^32 comes from a shift-and-add multiplier that handles one bit of
// step per clock (bit n on clock n), so it needs only an adder, not a 32 x 32 multiplier.
module sweep (
    input  wire        clk,
    input  wire        rst,
    input  wire        geo,
    input  wire [31:0] start,
    input  wire [31:0] step,    // LFM: two's complement, negative sweeps down.  Geometric: ratio - 1, x 2^32
    output reg  [31:0] ftw
);

    reg  [4:0]  n;
    reg  [31:0] acc;                                  // ftw * step[n-1:0] / 2^n, rounded down
    wire [32:0] sum = acc + (step[n] ? ftw : 32'd0);

    always @(posedge clk)
        if (rst) begin
            ftw <= start;
            n   <= 0;
            acc <= 0;
        end
        else if (!geo)
            ftw <= ftw + step;
        else begin
            n <= n + 1;
            if (n == 31) begin
                ftw <= ftw + sum[32:1];
                acc <= 0;
            end
            else
                acc <= sum[32:1];
        end

endmodule
