// Frequency sweep for LFM chirps: ftw starts at start and moves by step every clock.
module sweep (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] start,
    input  wire [31:0] step,    // two's complement: negative sweeps down
    output reg  [31:0] ftw
);

    always @(posedge clk)
        if (rst)
            ftw <= start;
        else
            ftw <= ftw + step;

endmodule
