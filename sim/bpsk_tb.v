`timescale 1ns/1ps
// One windowed BPSK pulse: 5 MHz carrier, Barker-13 code, 100 ticks per chip.
// Check with: python3 host/check_bpsk.py
module bpsk_tb;

    localparam [31:0] F5MHZ    = 32'd428273092;
    localparam [15:0] CHIP_LEN = 16'd100;
    localparam [15:0] LEN      = 16'd1300;       // 13 chips
    localparam [31:0] WIN_STEP = 32'd3303821;    // 2^32 / 1300
    localparam [15:0] BARKER13 = 16'h0A60;       // + + + + + - - + + - + - +, chip 0 = bit 0

    reg                clk = 0;
    reg                rst = 1;
    reg                start = 0;
    reg         [12:0] amp = 13'd4096;
    wire signed [11:0] out;
    wire               active;

    pulse_dds dut (.clk(clk), .rst(rst), .start(start), .len(LEN),
                   .ftw_start(F5MHZ), .ftw_step(32'd0), .geo(1'b0),
                   .code(BARKER13), .chip_len(CHIP_LEN),
                   .win_step(WIN_STEP), .amp(amp), .out(out), .active(active));

    always #10 clk = ~clk;

    integer fd;

    task tick(input s);
        begin
            start = s;
            @(negedge clk);
            $fdisplay(fd, "%0d %0d %0d %0d %0d %0d %0d %0d",
                      s, F5MHZ, 0, amp, BARKER13, CHIP_LEN, out, active);
        end
    endtask

    initial begin
        $dumpfile("sim/out/bpsk_tb.vcd");
        $dumpvars(0, bpsk_tb);
        fd = $fopen("sim/out/bpsk_samples.txt", "w");
        $fdisplay(fd, "# len=%0d win_step=%0d", LEN, WIN_STEP);

        repeat (5) @(negedge clk);
        rst = 0;

        repeat (50) tick(0);
        tick(1);
        repeat (1400) tick(0);

        $fclose(fd);
        $display("wrote sim/out/bpsk_samples.txt -- now run: python3 host/check_bpsk.py");
        $finish;
    end

endmodule
