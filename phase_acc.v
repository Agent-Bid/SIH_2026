// Phase accumulator: phase += ftw every clock, wrapping at 2^32.
module phase_acc (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] ftw,
    output reg  [31:0] phase
);

    always @(posedge clk)
        if (rst)
            phase <= 32'd0;
        else
            phase <= phase + ftw;

endmodule
