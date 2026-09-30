// Frequency sweep. ftw is the top 32 bits of a 48-bit register that also keeps 16 bits
// below the FTW, so a slow sweep can move by a fraction of an FTW per clock.
//   LFM (geo = 0):       the register moves by step every clock (step = FTW per clock x 2^16),
//                        so the frequency changes by the same number of Hz each clock.
//   Geometric (geo = 1): every 32 clocks, the register grows by register * step / 2^32, so the
//                        frequency is multiplied by the same ratio (1 + step / 2^32) each time:
//                        equal time per octave.
// The product comes from a shift-and-add multiplier that handles one bit of step per clock
// (bit n on clock n), so it needs only an adder, not a 48 x 32 multiplier.
module sweep (
    input  wire        clk,
    input  wire        rst,
    input  wire        geo,
    input  wire [31:0] start,   // FTW
    input  wire [31:0] step,    // LFM: two's complement, negative sweeps down.  Geometric: ratio - 1, x 2^32
    output wire [31:0] ftw
);

    reg  [47:0] acc_ftw;                              // FTW x 2^16
    reg  [4:0]  n;
    reg  [47:0] prod;                                 // acc_ftw * step[n-1:0] / 2^n, rounded down
    wire [48:0] sum = prod + (step[n] ? acc_ftw : 48'd0);

    assign ftw = acc_ftw[47:16];

    always @(posedge clk)
        if (rst) begin
            acc_ftw <= {start, 16'd0};
            n       <= 0;
            prod    <= 0;
        end
        else if (!geo)
            acc_ftw <= acc_ftw + {{16{step[31]}}, step};
        else begin
            n <= n + 1;
            if (n == 31) begin
                acc_ftw <= acc_ftw + sum[48:1];
                prod    <= 0;
            end
            else
                prod <= sum[48:1];
        end

endmodule
