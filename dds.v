// DDS: phase accumulator + 1024-entry sine table. f_out = ftw * f_clk / 2^32.
module dds (
    input  wire               clk,
    input  wire               rst,
    input  wire        [31:0] ftw,
    input  wire        [31:0] offset,
    output reg  signed [11:0] sample
);

    reg signed [11:0] sine_rom [0:1023];
    initial $readmemh("sine.hex", sine_rom);

    wire [31:0] phase;
    phase_acc u_acc (.clk(clk), .rst(rst), .ftw(ftw), .phase(phase));
    wire [31:0] shifted = phase + offset;

    always @(posedge clk)
        sample <= sine_rom[shifted[31:22]];

endmodule
