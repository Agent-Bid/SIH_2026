`timescale 1ns/1ps
// Testbench for pulse_ctrl: checks that active stays 1 for exactly len ticks.
module pulse_ctrl_tb;

    reg        clk = 0;
    reg        rst = 1;
    reg        start = 0;
    reg [15:0] len = 5;
    wire       active;

    pulse_ctrl dut (.clk(clk), .rst(rst), .start(start), .len(len), .active(active));

    always #10 clk = ~clk;

    integer errors = 0;
    integer ticks;

    // Press start for one tick, then count how many ticks active stays 1.
    // If poke_at > 0, press start again on that tick of the pulse (it must be ignored).
    task pulse(input integer poke_at, input integer expected, input string what);
        begin
            start = 1; @(negedge clk); start = 0;
            ticks = 0;
            while (active === 1'b1 && ticks < 100000) begin
                ticks = ticks + 1;
                if (ticks == poke_at) start = 1;
                @(negedge clk);
                start = 0;
            end
            if (ticks == expected)
                $display("  ok    %-40s active for %0d ticks", what, ticks);
            else begin
                $display("  FAIL  %-40s active for %0d ticks, expected %0d", what, ticks, expected);
                errors = errors + 1;
            end
            repeat (3) @(negedge clk);    // a short gap between pulses
        end
    endtask

    initial begin
        $dumpfile("sim/out/pulse_ctrl_tb.vcd");
        $dumpvars(0, pulse_ctrl_tb);

        repeat (3) @(negedge clk);
        rst = 0;

        $display("Test 1: nothing happens without start");
        repeat (5) @(negedge clk);
        if (active === 1'b0) $display("  ok    %-40s active = 0", "idle after reset");
        else begin $display("  FAIL  %-40s active = %b", "idle after reset", active); errors = errors + 1; end

        $display("Test 2: pulse lengths");
        len = 5;   pulse(0, 5,   "len = 5");
        len = 1;   pulse(0, 1,   "len = 1 (shortest possible)");
        len = 300; pulse(0, 300, "len = 300");

        $display("Test 3: start during a pulse is ignored");
        len = 5;   pulse(2, 5,   "len = 5, start pressed again at tick 2");

        if (errors == 0) $display("\nPASS");
        else             $display("\nFAIL: %0d error(s)", errors);
        $finish;
    end

endmodule
