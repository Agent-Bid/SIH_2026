`timescale 1ns/1ps
// Board-level test of top.v: a stand-in PLL, a fast UART and a short capture.
// The "PC" sends the frames made by host/wavegen.py (sim/top_frames.hex):
//   1. SET a down-chirp 500 -> 150 kHz, then CAPTURE
//   2. SET BPSK, Barker-7 at 250 kHz, then CAPTURE
// Everything the board sends back is written to a file.
// Check with: python3 host/capture.py --file sim/out/top_uart.txt
module top_tb;

    localparam BAUD_DIV = 8;
    localparam CAP_BITS = 11;                          // 2048 samples per capture
    localparam BYTES    = 29 + 2 * (1 << CAP_BITS);    // one capture
    localparam GROUP    = 30;                          // SET frame (27) + CAPTURE frame (3)

    reg        clk  = 0;
    reg        pc_tx = 1;
    wire [5:0] led;
    wire       uart_tx;
    wire [7:0] dac_d;
    wire       dac_clk;

    top #(.CAP_BITS(CAP_BITS), .BAUD_DIV(BAUD_DIV), .BTN_BITS(3)) dut (
        .clk(clk), .btn1(1'b1), .btn2(1'b1), .uart_rx(pc_tx), .led(led),
        .uart_tx(uart_tx), .dac_d(dac_d), .dac_clk(dac_clk));

    always #10 clk = ~clk;

    // ---- PC -> board
    reg [7:0] frames [0:2*GROUP-1];
    initial $readmemh("sim/top_frames.hex", frames);

    task send_byte(input [7:0] b);
        integer k;
        begin
            pc_tx = 0; repeat (BAUD_DIV) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                pc_tx = b[k]; repeat (BAUD_DIV) @(posedge clk);
            end
            pc_tx = 1; repeat (BAUD_DIV) @(posedge clk);
        end
    endtask

    task send_group(input integer g);
        integer k;
        for (k = 0; k < GROUP; k = k + 1)
            send_byte(frames[g * GROUP + k]);
    endtask

    // ---- board -> PC
    integer fd, i;
    integer nbytes = 0;
    reg [7:0] b;
    initial begin
        fd = $fopen("sim/out/top_uart.txt", "w");
        forever begin
            @(negedge uart_tx);
            repeat (BAUD_DIV / 2) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                repeat (BAUD_DIV) @(posedge clk);
                b[i] = uart_tx;
            end
            repeat (BAUD_DIV) @(posedge clk);
            if (uart_tx !== 1'b1) $display("  framing error at byte %0d", nbytes);
            $fdisplay(fd, "%02x", b);
            nbytes = nbytes + 1;
        end
    end

    initial begin
        $dumpfile("sim/out/top_tb.vcd");
        $dumpvars(1, top_tb);

        repeat (100) @(posedge clk);
        $display("PC: set down-chirp + capture");
        send_group(0);
        wait (nbytes == BYTES);

        $display("PC: set BPSK Barker-7 + capture");
        send_group(1);
        wait (nbytes == 2 * BYTES);

        repeat (100) @(posedge clk);
        $fclose(fd);
        $display("received %0d bytes -- now run: python3 host/capture.py --file sim/out/top_uart.txt", nbytes);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("TIMEOUT: received only %0d of %0d bytes", nbytes, 2 * BYTES);
        $finish;
    end

endmodule

// Stand-in for the Gowin rPLL (simulation only): passes the clock through, locks after 200 ns.
module rPLL #(
    parameter FCLKIN = "27", DEVICE = "", IDIV_SEL = 0, FBDIV_SEL = 0, ODIV_SEL = 8,
    parameter DYN_IDIV_SEL = "false", DYN_FBDIV_SEL = "false", DYN_ODIV_SEL = "false",
    parameter PSDA_SEL = "0000", DYN_DA_EN = "false", DUTYDA_SEL = "1000",
    parameter CLKOUT_FT_DIR = 1'b1, CLKOUTP_FT_DIR = 1'b1, CLKOUT_DLY_STEP = 0, CLKOUTP_DLY_STEP = 0,
    parameter CLKFB_SEL = "internal", CLKOUT_BYPASS = "false", CLKOUTP_BYPASS = "false",
    parameter CLKOUTD_BYPASS = "false", DYN_SDIV_SEL = 2, CLKOUTD_SRC = "CLKOUT", CLKOUTD3_SRC = "CLKOUT"
) (
    output wire       CLKOUT,
    output reg        LOCK = 1'b0,
    output wire       CLKOUTP, CLKOUTD, CLKOUTD3,
    input  wire       RESET, RESET_P, CLKIN, CLKFB,
    input  wire [5:0] FBDSEL, IDSEL, ODSEL,
    input  wire [3:0] PSDA, DUTYDA, FDLY
);
    assign CLKOUT = CLKIN;
    initial #200 LOCK = 1'b1;
endmodule
