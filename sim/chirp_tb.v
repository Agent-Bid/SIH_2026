`timescale 1ns/1ps
// Two windowed LFM chirps: 3 -> 7 MHz, then 7 -> 3 MHz (negative step). Logs every tick.
// Check with: python3 host/check_chirp.py
module chirp_tb;

    localparam [15:0] LEN      = 16'd2000;       // ~40 us
    localparam [31:0] WIN_STEP = 32'd2147484;    // 2^32 / 2000
    localparam [31:0] F3MHZ    = 32'd256963855;
    localparam [31:0] F7MHZ    = 32'd599582329;
    localparam [31:0] STEP     = 32'd171309;     // (F7MHZ - F3MHZ) / LEN

    reg                clk = 0;
    reg                rst = 1;
    reg                start = 0;
    reg         [31:0] ftw_start = F3MHZ;
    reg         [31:0] ftw_step  = STEP;
    reg         [12:0] amp = 13'd4096;
    wire signed [11:0] out;
    wire               active;

    pulse_dds dut (.clk(clk), .rst(rst), .start(start), .len(LEN),
                   .ftw_start(ftw_start), .ftw_step(ftw_step), .geo(1'b0),
                   .code(16'd0), .chip_len(16'd1),
                   .win_step(WIN_STEP), .amp(amp), .out(out), .active(active));

    always #10 clk = ~clk;

    integer fd;

    task tick(input s);
        begin
            start = s;
            @(negedge clk);
            $fdisplay(fd, "%0d %0d %0d %0d %0d %0d %0d %0d", s, ftw_start, ftw_step, amp, 0, 1, out, active);
        end
    endtask

    initial begin
        $dumpfile("sim/out/chirp_tb.vcd");
        $dumpvars(0, chirp_tb);
        fd = $fopen("sim/out/chirp_samples.txt", "w");
        $fdisplay(fd, "# len=%0d win_step=%0d", LEN, WIN_STEP);

        repeat (5) @(negedge clk);
        rst = 0;

        repeat (50) tick(0);
        tick(1);                          // up-chirp
        repeat (2200) tick(0);

        ftw_start = F7MHZ;
        ftw_step  = -STEP;                // down-chirp
        tick(1);
        repeat (2200) tick(0);

        $fclose(fd);
        $display("wrote sim/out/chirp_samples.txt -- now run: python3 host/check_chirp.py");
        $finish;
    end

endmodule
