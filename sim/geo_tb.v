`timescale 1ns/1ps
// A windowed geometric sweep, 2 -> 8 MHz (two octaves) over 3200 ticks. Logs every tick.
// Check with: python3 host/check_geo.py
module geo_tb;

    localparam [15:0] LEN      = 16'd3200;
    localparam [31:0] WIN_STEP = 32'd1342177;     // 2^32 / 3200
    localparam [31:0] F2MHZ    = 32'd171309237;
    localparam [31:0] RATIO    = 32'd59955509;    // (4^(32/3200) - 1) * 2^32: x4 over the pulse

    reg                clk = 0;
    reg                rst = 1;
    reg                start = 0;
    reg         [12:0] amp = 13'd4096;
    wire signed [11:0] out;
    wire               active;

    pulse_dds dut (.clk(clk), .rst(rst), .start(start), .len(LEN),
                   .ftw_start(F2MHZ), .ftw_step(RATIO), .geo(1'b1),
                   .code(16'd0), .chip_len(16'd1),
                   .win_step(WIN_STEP), .amp(amp), .out(out), .active(active));

    always #10 clk = ~clk;

    integer fd;

    task tick(input s);
        begin
            start = s;
            @(negedge clk);
            $fdisplay(fd, "%0d %0d %0d %0d %0d %0d %0d %0d", s, F2MHZ, RATIO, amp, 0, 1, out, active);
        end
    endtask

    initial begin
        $dumpfile("sim/out/geo_tb.vcd");
        $dumpvars(0, geo_tb);
        fd = $fopen("sim/out/geo_samples.txt", "w");
        $fdisplay(fd, "# len=%0d win_step=%0d", LEN, WIN_STEP);

        repeat (5) @(negedge clk);
        rst = 0;

        repeat (50) tick(0);
        tick(1);
        repeat (3400) tick(0);

        $fclose(fd);
        $display("wrote sim/out/geo_samples.txt -- now run: python3 host/check_geo.py");
        $finish;
    end

endmodule
