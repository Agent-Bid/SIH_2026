`timescale 1ns/1ps
// Testbench for pulse_dds: plays two windowed pulses and logs every tick to a file.
// host/check_pulse.py then compares the file with the Python golden model.
//
//   pulse 1: full amplitude (a second start in the middle must be ignored)
//   pulse 2: half amplitude
module pulse_dds_tb;

    localparam [31:0] FTW      = 32'd85654618;   // 1 MHz tone
    localparam [15:0] LEN      = 16'd1000;       // 1000 ticks ~ 20 us
    localparam [31:0] WIN_STEP = 32'd4294967;    // 2^32 / 1000: one window lap per pulse

    reg                clk = 0;
    reg                rst = 1;
    reg                start = 0;
    reg         [12:0] amp = 13'd4096;           // 1.0
    wire signed [11:0] out;
    wire               active;

    pulse_dds dut (.clk(clk), .rst(rst), .start(start), .len(LEN),
                   .ftw_start(FTW), .ftw_step(32'd0),
                   .code(16'd0), .chip_len(16'd1),
                   .win_step(WIN_STEP), .amp(amp), .out(out), .active(active));

    always #10 clk = ~clk;

    integer fd;

    // One tick: set start, let the clock tick, log the inputs and the outputs.
    task tick(input s);
        begin
            start = s;
            @(negedge clk);
            $fdisplay(fd, "%0d %0d %0d %0d %0d %0d %0d %0d", s, FTW, 0, amp, 0, 1, out, active);
        end
    endtask

    initial begin
        $dumpfile("sim/out/pulse_dds_tb.vcd");
        $dumpvars(0, pulse_dds_tb);

        fd = $fopen("sim/out/pulse_samples.txt", "w");
        $fdisplay(fd, "# len=%0d win_step=%0d", LEN, WIN_STEP);

        repeat (5) @(negedge clk);        // reset
        rst = 0;

        repeat (50)  tick(0);             // quiet
        tick(1);                          // pulse 1
        repeat (400) tick(0);
        tick(1);                          // start again mid-pulse: must be ignored
        repeat (700) tick(0);
        amp = 13'd2048;                   // 0.5
        tick(1);                          // pulse 2
        repeat (1100) tick(0);

        $fclose(fd);
        $display("wrote sim/out/pulse_samples.txt");
        $display("now run:  python3 host/check_pulse.py");
        $finish;
    end

endmodule
