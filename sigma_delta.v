// 1-bit DAC for looking at the waveform on an oscilloscope: a first-order sigma-delta modulator.
// out is 1 for a share of the clocks equal to the sample's level (0 at -2048, 1 at +2047), so a
// resistor-capacitor low-pass filter on the pin turns it into the analog waveform. At 50 MHz,
// R = 1 kOhm and C = 100 pF (cut-off ~1.6 MHz) pass the 100-500 kHz waves and smooth the bits.
module sigma_delta (
    input  wire               clk,
    input  wire               rst,
    input  wire signed [11:0] in,
    output reg                out
);

    wire [11:0] level = {~in[11], in[10:0]};      // offset binary: 0 .. 4095
    reg  [12:0] acc = 0;

    always @(posedge clk)
        if (rst) begin
            acc <= 0;
            out <= 1'b0;
        end
        else begin
            acc <= acc[11:0] + level;              // the carry out is the output bit
            out <= acc[12];
        end

endmodule
