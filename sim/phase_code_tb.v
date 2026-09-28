`timescale 1ns/1ps
// Unit test for phase_code: flip must follow the code, one bit per chip_len ticks.
module phase_code_tb;

    reg         clk = 0;
    reg         rst = 1;
    reg  [15:0] code;
    reg  [15:0] chip_len;
    wire        flip;

    phase_code dut (.clk(clk), .rst(rst), .code(code), .chip_len(chip_len), .flip(flip));

    always #10 clk = ~clk;

    integer errors = 0;
    integer k;
    reg expected;

    // Hold reset, release it, then check flip on every tick for n ticks.
    task run(input [15:0] c, input [15:0] cl, input integer n, input string what);
        integer bad;
        begin
            code = c; chip_len = cl; rst = 1;
            repeat (2) @(negedge clk);
            rst = 0;
            bad = 0;
            for (k = 0; k < n; k = k + 1) begin
                expected = c[(k / cl) % 16];
                if (flip !== expected) begin
                    if (bad == 0)
                        $display("  FAIL  %-36s tick %0d: flip = %b, expected %b (chip %0d)",
                                 what, k, flip, expected, (k / cl) % 16);
                    bad = bad + 1;
                end
                @(negedge clk);
            end
            if (bad == 0) $display("  ok    %-36s %0d ticks", what, n);
            errors = errors + bad;
        end
    endtask

    initial begin
        $dumpfile("sim/out/phase_code_tb.vcd");
        $dumpvars(0, phase_code_tb);

        run(16'h0A60, 16'd3,  13 * 3,  "Barker-13, 3 ticks per chip");
        run(16'h0A60, 16'd1,  13,      "Barker-13, 1 tick per chip");
        run(16'hB38F, 16'd5,  16 * 5,  "all 16 chips, 5 ticks per chip");
        run(16'h0001, 16'd4,  40 * 4,  "wraps back to chip 0 after 16");
        run(16'h0000, 16'd7,  50,      "code 0 never flips");

        if (errors == 0) $display("\nPASS");
        else             $display("\nFAIL: %0d wrong ticks", errors);
        $finish;
    end

endmodule
