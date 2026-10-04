// 1-bit DAC: a sigma-delta modulator. out is 1 for a share of the clocks equal to the sample's
// level (0 at -2048, 1 at +2047), so a low-pass filter on the pin turns it into the waveform.
//   ORDER 1: one accumulator. Its error is spread evenly over all frequencies, and near the
//            midpoint it makes slow repeating patterns (idle tones) that get through the filter.
//   ORDER 2 (default): two integrators, so the error is shaped by (1 - z^-1)^2 and pushed far
//            above 100-500 kHz, where the filter removes it. The input is limited to +-95% of
//            full scale, which keeps the integrators bounded (|i1| < 10k, |i2| < 50k in
//            simulation; held at full scale, i2 would grow without limit).
module sigma_delta #(parameter ORDER = 2) (
    input  wire               clk,
    input  wire               rst,
    input  wire signed [11:0] in,
    output reg                out
);

    generate
        if (ORDER == 1) begin : first
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
        end
        else begin : second
            localparam signed [12:0] LIM = 13'sd1945;
            wire signed [12:0] x  = (in > LIM) ? LIM : (in < -LIM) ? -LIM : in;
            wire signed [12:0] fb = out ? 13'sd2048 : -13'sd2048;   // what the last bit stood for
            reg  signed [15:0] i1 = 0;
            reg  signed [19:0] i2 = 0;
            wire signed [15:0] i1_next = i1 + x - fb;
            wire signed [19:0] i2_next = i2 + i1_next - fb;

            always @(posedge clk)
                if (rst) begin
                    i1  <= 0;
                    i2  <= 0;
                    out <= 1'b0;
                end
                else begin
                    i1  <= i1_next;
                    i2  <= i2_next;
                    out <= ~i2_next[19];                   // 1 when the second integrator is >= 0
                end
        end
    endgenerate

endmodule
