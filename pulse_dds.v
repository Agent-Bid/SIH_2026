// Windowed pulse: swept (LFM or geometric), phase-coded tone * Hann window * amp, for len clocks after each start.
module pulse_dds (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire        [23:0] len,
    input  wire        [31:0] ftw_start,
    input  wire        [31:0] ftw_step,   // LFM: FTW per clock x 2^16 (0 = plain tone); geo: see sweep.v
    input  wire               geo,        // 1 = geometric sweep (see sweep.v)
    input  wire        [31:0] win_step,   // 2^32 / len
    input  wire        [15:0] code,
    input  wire        [23:0] chip_len,
    input  wire        [12:0] amp,        // 4096 = 1.0
    output wire signed [11:0] out,
    output wire               active
);

    pulse_ctrl u_ctrl (.clk(clk), .rst(rst), .start(start), .len(len), .active(active));

    wire run_rst = rst | ~active;

    wire [31:0] ftw_now;

    sweep u_sweep (.clk(clk), .rst(run_rst), .geo(geo), .start(ftw_start), .step(ftw_step), .ftw(ftw_now));

    wire flip;
    phase_code u_phase (.clk(clk), .rst(run_rst), .code(code), .chip_len(chip_len), .flip(flip));

    wire signed [11:0] tone;
    dds u_dds (.clk(clk), .rst(run_rst), .ftw(ftw_now), .offset(flip ? 32'h8000_0000 : 32'd0), .sample(tone));

    reg [12:0] hann_rom [0:1023];
    initial $readmemh("hann.hex", hann_rom);

    wire [31:0] win_phase;
    reg  [12:0] win;

    phase_acc u_win (.clk(clk), .rst(run_rst), .ftw(win_step), .phase(win_phase));

    always @(posedge clk)
        win <= hann_rom[win_phase[31:22]];

    // Pipeline register: gives the table read and the multiply a clock each
    reg signed [11:0] tone_r;
    reg        [12:0] win_r;
    always @(posedge clk) begin
        tone_r <= tone;
        win_r  <= win;
    end

    wire signed [11:0] shaped;
    scale u_win_scale (.clk(clk), .in(tone_r), .factor(win_r), .out(shaped));
    scale u_amp_scale (.clk(clk), .in(shaped), .factor(amp), .out(out));

endmodule
