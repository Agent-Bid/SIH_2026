// Tang Nano 9K top: the waveform engine, set by the ESP32 over SPI (MCU /SIH), or by the PC
// over the USB UART (host/wavegen.py), which sends the very same packet inside a UART frame.
//   S1: step through three built-in presets: LFM chirp 150 -> 500 kHz, geometric sweep
//       150 -> 500 kHz, BPSK Barker-13 at 300 kHz (the ESP32 overrides them 10 times a second)
//   S2: capture the next pulse and send it back (host/capture.py, or wavegen.py --capture)
// LEDs (on = 1): 0 BPSK, 1 armed, 2 recording or sending, 3 toggles on every good SPI packet,
//                4 pulse active, 5 geometric sweep
//
// The packet (57 bytes, SonarPacket v3 in MCU /SIH/sonar_core.h) is checked by pkt_check;
// the FPGA uses modulation (byte 5) and bytes 28-55:
//   lenClk[31:0] ftwStart[31:0] ftwStep[31:0] winStep[31:0] ampQ12[15:0] code[15:0]
//   chipLen[31:0] periodClk[31:0]
module top #(
    parameter CAP_BITS = 14,        // capture 2^14 = 16384 samples
    parameter BAUD_DIV = 435,       // 50.14 MHz / 435 = 115200 baud
    parameter BTN_BITS = 20
) (
    input  wire       clk,          // 27 MHz oscillator
    input  wire       btn1,
    input  wire       btn2,
    input  wire       uart_rx,
    input  wire       spi_sck,
    input  wire       spi_mosi,
    input  wire       spi_cs_n,
    output wire [5:0] led,
    output wire       uart_tx,
    output reg  [7:0] dac_d,
    output wire       dac_clk
);

    localparam PKT_BYTES = 57;

    // Settings: {period, chip_len, code, amp, win_step, ftw_step, ftw_start, len, mod}
    localparam SET_BITS = 32 + 24 + 16 + 13 + 32 + 32 + 32 + 24 + 2;
    localparam MOD_CW = 2'd0, MOD_LFM = 2'd1, MOD_GEO = 2'd2, MOD_BPSK = 2'd3;

    // Presets (made with host/mcu_packet.py, 13000-clock pulses every 25000 clocks)
    localparam [SET_BITS-1:0] CHIRP = {32'd25000, 24'd0,    16'h0000, 13'd4096, 32'd330382,
                                       32'd151131644, 32'd12848193, 24'd13000, MOD_LFM};
    localparam [SET_BITS-1:0] GEO   = {32'd25000, 24'd0,    16'h0000, 13'd4096, 32'd330382,
                                       32'd12755415,  32'd12848193, 24'd13000, MOD_GEO};
    localparam [SET_BITS-1:0] BPSK  = {32'd25000, 24'd1000, 16'h0A60, 13'd4096, 32'd330382,
                                       32'd0,         32'd25696386, 24'd13000, MOD_BPSK};

    wire clk50, lock;
    pll u_pll (.clk_in(clk), .clk_out(clk50), .lock(lock));

    reg [3:0] rst_cnt = 0;
    wire rst = ~rst_cnt[3];
    always @(posedge clk50)
        if (!lock)
            rst_cnt <= 0;
        else if (rst)
            rst_cnt <= rst_cnt + 1;

    wire preset_press, cap_press;
    button #(.BITS(BTN_BITS)) u_btn1 (.clk(clk50), .btn_n(btn1), .pressed(preset_press));
    button #(.BITS(BTN_BITS)) u_btn2 (.clk(clk50), .btn_n(btn2), .pressed(cap_press));

    // ---- Packets: from the ESP32 over SPI, or from the PC inside a UART frame
    wire [PKT_BYTES*8-1:0] spi_pkt;
    wire                   spi_valid, spi_bad;
    // USE_CS 0: packets are framed by the pause after each burst; the CS wire does not work on the
    // current boards (see spi_rx.v). Set USE_CS 1 when CS is connected properly.
    spi_rx #(.BYTES(PKT_BYTES), .USE_CS(0)) u_spi (.clk(clk50), .rst(rst), .sck(spi_sck), .mosi(spi_mosi),
                                       .cs_n(spi_cs_n), .packet(spi_pkt), .valid(spi_valid), .bad(spi_bad));

    wire [7:0]             rx_data;
    wire                   rx_valid;
    wire [PKT_BYTES*8-1:0] uart_pkt;
    wire                   uart_valid, capture_req;
    uart_rx #(.DIV(BAUD_DIV)) u_rx (.clk(clk50), .rst(rst), .rx(uart_rx),
                                    .data(rx_data), .valid(rx_valid));
    cmd_rx #(.BYTES(PKT_BYTES)) u_cmd (.clk(clk50), .rst(rst), .rx_data(rx_data), .rx_valid(rx_valid),
                                       .settings(uart_pkt), .settings_valid(uart_valid),
                                       .capture_req(capture_req));

    reg from_spi = 1'b0;
    always @(posedge clk50)
        if (spi_valid)
            from_spi <= 1'b1;
        else if (uart_valid)
            from_spi <= 1'b0;

    wire [PKT_BYTES*8-1:0] pkt;
    wire                   pkt_ok, pkt_bad;
    pkt_check #(.BYTES(PKT_BYTES)) u_check (.clk(clk50), .rst(rst),
                                            .packet(spi_valid ? spi_pkt : uart_pkt),
                                            .valid(spi_valid | uart_valid),
                                            .fields(pkt), .ok(pkt_ok), .bad(pkt_bad));

    wire [15:0] amp_q12 = pkt[44*8 +: 16];
    wire [SET_BITS-1:0] pkt_settings = {
        pkt[52*8 +: 32],                                   // periodClk
        pkt[48*8 +: 24],                                   // chipLen
        pkt[46*8 +: 16],                                   // code
        amp_q12 > 16'd4096 ? 13'd4096 : amp_q12[12:0],     // ampQ12
        pkt[40*8 +: 32],                                   // winStep
        pkt[36*8 +: 32],                                   // ftwStep
        pkt[32*8 +: 32],                                   // ftwStart
        pkt[28*8 +: 24],                                   // lenClk
        pkt[5*8 +: 2]                                      // modulation
    };

    // Counters for the capture header: good and bad SPI packets (bad = wrong length or CRC)
    reg [15:0] spi_ok_n = 0, spi_bad_n = 0;
    reg        spi_blink = 1'b0;
    always @(posedge clk50) begin
        if (pkt_ok && from_spi) begin
            spi_ok_n  <= spi_ok_n + 1;
            spi_blink <= ~spi_blink;
        end
        if ((pkt_bad && from_spi) || spi_bad)
            spi_bad_n <= spi_bad_n + 1;
    end

    // ---- Settings, double-buffered: new ones wait in pending until no pulse is playing
    reg  [SET_BITS-1:0] pending = CHIRP;
    reg  [SET_BITS-1:0] cur     = CHIRP;
    reg  [1:0]          preset  = 2'd0;
    reg  [31:0]         timer   = 0;
    wire                active;
    wire [31:0]         period = cur[SET_BITS-1 -: 32];
    wire                start  = (timer == 0) && (period != 0);

    always @(posedge clk50) begin
        if (pkt_ok)
            pending <= pkt_settings;
        else if (preset_press) begin
            pending <= (preset == 2'd0) ? GEO : (preset == 2'd1) ? BPSK : CHIRP;
            preset  <= (preset == 2'd2) ? 2'd0 : preset + 2'd1;
        end
        if (!active && !start)
            cur <= pending;
        timer <= (timer >= period - 1) ? 32'd0 : timer + 1;
    end

    wire [1:0]  mod;
    wire [23:0] len, chip_len;
    wire [31:0] ftw_start, ftw_step, win_step;
    wire [15:0] code;
    wire [12:0] amp;
    assign {chip_len, code, amp, win_step, ftw_step, ftw_start, len, mod} = cur[SET_BITS-33:0];
    wire geo = (mod == MOD_GEO);

    // ---- The waveform engine
    wire signed [11:0] out;
    pulse_dds u_eng (.clk(clk50), .rst(rst), .start(start), .len(len),
                     .ftw_start(ftw_start), .ftw_step(ftw_step), .geo(geo), .win_step(win_step),
                     .code(code), .chip_len(chip_len), .amp(amp),
                     .out(out), .active(active));

    // 8-bit parallel DAC: offset binary, the DAC latches in the middle of each sample
    always @(posedge clk50)
        dac_d <= {~out[11], out[10:4]};
    assign dac_clk = ~clk50;

    // ---- Capture one pulse and send it back. Header, 37 bytes (host/capture.py):
    //   A5 5A, mod, N, len, ftw_start, ftw_step, win_step, code, chip_len, amp, period,
    //   good SPI packets, bad SPI packets
    localparam [15:0] CAP_N = 1 << CAP_BITS;
    wire [295:0] header = {spi_bad_n, spi_ok_n, period, {3'b0, amp}, {8'b0, chip_len}, code,
                           win_step, ftw_step, ftw_start, {8'b0, len}, CAP_N, {6'b0, mod},
                           8'h5A, 8'hA5};

    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;
    wire       armed, recording, sending;
    capture #(.N_BITS(CAP_BITS), .HDR_BYTES(37)) u_cap (
        .clk(clk50), .rst(rst), .arm(cap_press | capture_req), .trigger(start), .sample(out),
        .header(header), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .armed(armed), .recording(recording), .sending(sending));

    uart_tx #(.DIV(BAUD_DIV)) u_tx (.clk(clk50), .rst(rst), .data(tx_data), .valid(tx_valid),
                                    .tx(uart_tx), .ready(tx_ready));

    assign led = ~{geo, active, spi_blink, recording | sending, armed, mod == MOD_BPSK};

endmodule
