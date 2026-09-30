`timescale 1ns/1ps
// The oscilloscope output: a windowed LFM chirp 150 -> 500 kHz (16000 ticks) through pulse_dds and
// the sigma-delta 1-bit DAC. Logs the sample and the output bit every tick.
// Check with: python3 host/check_sigma_delta.py
module sigma_delta_tb;

    localparam [23:0] LEN      = 24'd16000;
    localparam [31:0] WIN_STEP = 32'd268435;      // 2^32 / 16000
    localparam [31:0] F150K    = 32'd12848193;
    localparam [31:0] STEP     = 32'd122794461;   // 350 kHz over the pulse, FTW per clock x 2^16

    reg                clk = 0;
    reg                rst = 1;
    reg                sd_rst = 1;                  // held until pulse_dds' pipeline has filled
    reg                start = 0;
    wire signed [11:0] out;
    wire               active, bit_out;

    pulse_dds dut (.clk(clk), .rst(rst), .start(start), .len(LEN),
                   .ftw_start(F150K), .ftw_step(STEP), .geo(1'b0),
                   .code(16'd0), .chip_len(24'd0),
                   .win_step(WIN_STEP), .amp(13'd4096), .out(out), .active(active));
    sigma_delta sd (.clk(clk), .rst(sd_rst), .in(out), .out(bit_out));

    always #10 clk = ~clk;

    integer fd, i;
    initial begin
        fd = $fopen("sim/out/sigma_delta.txt", "w");
        repeat (5) @(negedge clk);
        rst = 0;
        repeat (10) @(negedge clk);
        sd_rst = 0;                                  // log from here: the accumulator is 0
        for (i = 0; i < 16220; i = i + 1) begin
            $fdisplay(fd, "%0d %0d", out, bit_out);
            start = (i == 10);
            @(negedge clk);
        end
        $fclose(fd);
        $display("wrote sim/out/sigma_delta.txt -- now run: python3 host/check_sigma_delta.py");
        $finish;
    end

endmodule
