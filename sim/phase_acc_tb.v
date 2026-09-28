`timescale 1ns/1ps
// Testbench for phase_acc. It only ever runs in the simulator, never on the FPGA.
// It drives the inputs, checks the output after each step, and prints PASS or FAIL.
module phase_acc_tb;

    // Wires/regs that connect to the design under test ("dut")
    reg         clk = 0;
    reg         rst = 1;
    reg  [31:0] ftw = 0;
    wire [31:0] phase;

    phase_acc dut (.clk(clk), .rst(rst), .ftw(ftw), .phase(phase));

    // 50 MHz clock: flip every 10 ns, so one full tick is 20 ns
    always #10 clk = ~clk;

    integer errors = 0;

    // Compare phase with the value we expect and print the result.
    task check(input [31:0] expected, input string what);
        if (phase === expected)
            $display("  ok    %-40s phase = %h", what, phase);
        else begin
            $display("  FAIL  %-40s phase = %h, expected %h", what, phase, expected);
            errors = errors + 1;
        end
    endtask

    // We change inputs and check outputs on the FALLING edge (negedge), so we never
    // touch anything at the same moment the design reacts (the rising edge).
    initial begin
        $dumpfile("sim/out/phase_acc_tb.vcd");
        $dumpvars(0, phase_acc_tb);

        $display("Test 1: reset");
        rst = 1; ftw = 5;
        repeat (3) @(negedge clk);
        check(0, "rst=1 holds phase at 0");

        $display("Test 2: counting");
        rst = 0;
        repeat (10) @(negedge clk);
        check(50, "10 ticks of +5 = 50");

        $display("Test 3: wrap-around (3/4 of a lap per tick)");
        rst = 1; @(negedge clk);
        rst = 0; ftw = 32'hC000_0000;
        @(negedge clk); check(32'hC000_0000, "1 tick  = 0.75 lap");
        @(negedge clk); check(32'h8000_0000, "2 ticks = 1.5 laps -> wraps to 0.5");
        @(negedge clk); check(32'h4000_0000, "3 ticks = 2.25 laps -> 0.25");

        $display("Test 4: change speed mid-way (phase must not jump)");
        ftw = 100;
        @(negedge clk); check(32'h4000_0064, "0.25 lap + 100");

        $display("Test 5: long run, 1000 ticks");
        rst = 1; @(negedge clk);
        rst = 0; ftw = 32'd123456789;
        repeat (1000) @(negedge clk);
        check(32'd123456789 * 1000, "1000 x ftw, wrapped to 32 bits");

        if (errors == 0) $display("\nPASS");
        else             $display("\nFAIL: %0d error(s)", errors);
        $finish;
    end

endmodule
