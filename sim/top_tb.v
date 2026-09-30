`timescale 1ns/1ps
// Board-level test of top.v: a stand-in PLL, a fast UART, a short capture, and a pretend ESP32.
//   1. PC over UART: SET an LFM chirp 150 -> 500 kHz, then CAPTURE
//   2. ESP32 over SPI: a packet with a broken CRC (must be rejected), then a geometric sweep
//      150 -> 500 kHz with CS left high the whole time (the FPGA frames packets by the pause
//      after each burst, see spi_rx.v); then the PC asks for a CAPTURE
//   3. PC over UART: SET BPSK Barker-13 at 250 kHz, then CAPTURE
// The frames and the packet come from host/wavegen.py (sim/top_frames.hex, sim/top_spi.hex).
// Everything the board sends back is written to a file.
// Check with: python3 host/capture.py --file sim/out/top_uart.txt
module top_tb;

    localparam BAUD_DIV = 8;
    localparam CAP_BITS = 11;                          // 2048 samples per capture
    localparam BYTES    = 37 + 2 * (1 << CAP_BITS);    // one capture
    localparam GROUP    = 63;                          // SET frame (60) + CAPTURE frame (3)
    localparam PKT      = 57;                          // one ESP32 packet
    localparam SCK_HALF = 250;                         // ns: SPI at 2 MHz, like the ESP32

    reg        clk   = 0;
    reg        pc_tx = 1;
    reg        sck   = 0;
    reg        mosi  = 0;
    reg        cs_n  = 1;
    wire [5:0] led;
    wire       uart_tx;
    wire [7:0] dac_d;
    wire       dac_clk;

    top #(.CAP_BITS(CAP_BITS), .BAUD_DIV(BAUD_DIV), .BTN_BITS(3)) dut (
        .clk(clk), .btn1(1'b1), .btn2(1'b1), .uart_rx(pc_tx),
        .spi_sck(sck), .spi_mosi(mosi), .spi_cs_n(cs_n), .led(led),
        .uart_tx(uart_tx), .dac_d(dac_d), .dac_clk(dac_clk));

    always #10 clk = ~clk;

    // ---- PC -> board (UART)
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

    // ---- ESP32 -> board (SPI mode 0, MSB first, CS low for the whole packet)
    reg [7:0] pkt [0:PKT-1];
    initial $readmemh("sim/top_spi.hex", pkt);

    task spi_packet(input corrupt, input use_cs);
        integer k, j;
        reg [7:0] b;
        begin
            cs_n = !use_cs; #(SCK_HALF);
            for (k = 0; k < PKT; k = k + 1) begin
                b = (corrupt && k == 30) ? pkt[k] ^ 8'h01 : pkt[k];
                for (j = 7; j >= 0; j = j - 1) begin
                    mosi = b[j]; #(SCK_HALF);
                    sck  = 1;    #(SCK_HALF);
                    sck  = 0;
                end
            end
            #(SCK_HALF); cs_n = 1; #(200_000);            // 200 us pause: ends the frame
        end
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
        $display("PC: set LFM chirp + capture");
        send_group(0);
        wait (nbytes == BYTES);

        $display("ESP32: a packet with a broken CRC, then a geometric sweep; PC: capture");
        spi_packet(1, 1);
        spi_packet(0, 0);
        repeat (200) @(posedge clk);
        if (dut.spi_ok_n !== 16'd1 || dut.spi_bad_n !== 16'd1)
            $display("FAIL: SPI packets counted good %0d bad %0d, expected 1 and 1",
                     dut.spi_ok_n, dut.spi_bad_n);
        else
            $display("  SPI: the broken packet was rejected, the good one accepted");
        send_byte(8'hA5); send_byte(8'h02); send_byte(8'h02);
        wait (nbytes == 2 * BYTES);

        $display("PC: set BPSK Barker-13 + capture");
        send_group(1);
        wait (nbytes == 3 * BYTES);

        repeat (100) @(posedge clk);
        $fclose(fd);
        $display("received %0d bytes -- now run: python3 host/capture.py --file sim/out/top_uart.txt", nbytes);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("TIMEOUT: received only %0d of %0d bytes", nbytes, 3 * BYTES);
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
