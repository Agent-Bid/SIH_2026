`timescale 1ns/1ps
// Testbench for dds: runs a 1 MHz tone and writes every sample to a file.
// host/check_dds.py then compares the file with the Python golden model.
module dds_tb;

    localparam [31:0] FTW       = 32'd85654618;   // 1 MHz at 50.142857 MHz (27 * 13/7)
    localparam        N_SAMPLES = 4096;

    reg                clk = 0;
    reg                rst = 1;
    wire signed [11:0] sample;

    dds dut (.clk(clk), .rst(rst), .ftw(FTW), .offset(32'd0), .sample(sample));

    always #10 clk = ~clk;

    integer fd, k;

    initial begin
        $dumpfile("sim/out/dds_tb.vcd");
        $dumpvars(0, dds_tb);

        fd = $fopen("sim/out/dds_samples.txt", "w");
        $fdisplay(fd, "# ftw=%0d", FTW);        // first line tells Python the FTW

        repeat (3) @(negedge clk);             // hold reset for a few ticks
        rst = 0;

        for (k = 0; k < N_SAMPLES; k = k + 1) begin
            @(negedge clk);
            $fdisplay(fd, "%0d", sample);      // one signed number per line
        end

        $fclose(fd);
        $display("wrote %0d samples to sim/out/dds_samples.txt", N_SAMPLES);
        $display("now run:  python3 host/check_dds.py");
        $finish;
    end

endmodule
