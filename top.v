// Tang Nano 9K top: the waveform engine, with its settings sent by the PC over the USB UART
// (standing in for the MCU until it arrives). host/wavegen.py sends them.
//   S1: switch between two built-in presets (LFM chirp 150 -> 500 kHz / BPSK Barker-13 at 300 kHz)
//   S2: capture the next pulse and send it back (host/capture.py or wavegen.py --capture)
// LEDs (on = 1): 0 BPSK, 1 armed, 2 recording, 3 sending, 4 pulse active, 5 PLL locked
//
// Settings struct, 24 bytes little-endian (the same layout the MCU will send over SPI):
//   len[15:0]  ftw_start[47:16]  ftw_step[79:48]  win_step[111:80]
//   code[127:112]  chip_len[143:128]  amp[159:144]  period[191:160]
module top #(
    parameter CAP_BITS = 14,        // capture 2^14 = 16384 samples
    parameter BAUD_DIV = 435,       // 50.14 MHz / 435 = 115200 baud
    parameter BTN_BITS = 20
) (
    input  wire       clk,          // 27 MHz oscillator
    input  wire       btn1,
    input  wire       btn2,
    input  wire       uart_rx,
    output wire [5:0] led,
    output wire       uart_tx,
    output reg  [7:0] dac_d,
    output wire       dac_clk
);

    // Presets: {period, amp, chip_len, code, win_step, ftw_step, ftw_start, len}
    localparam [191:0] CHIRP = {32'd25000, 16'd4096, 16'd1000, 16'h0000,
                                32'd268435, 32'd1874, 32'd12848193, 16'd16000};
    localparam [191:0] BPSK  = {32'd25000, 16'd4096, 16'd1000, 16'h0A60,
                                32'd330382, 32'd0, 32'd25696386, 16'd13000};

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

    // ---- Commands from the PC
    wire [7:0]   rx_data;
    wire         rx_valid;
    wire [191:0] rx_settings;
    wire         rx_settings_valid, capture_req;
    uart_rx #(.DIV(BAUD_DIV)) u_rx (.clk(clk50), .rst(rst), .rx(uart_rx),
                                    .data(rx_data), .valid(rx_valid));
    cmd_rx u_cmd (.clk(clk50), .rst(rst), .rx_data(rx_data), .rx_valid(rx_valid),
                  .settings(rx_settings), .settings_valid(rx_settings_valid),
                  .capture_req(capture_req));

    // ---- Settings, double-buffered: new ones wait in pending until no pulse is playing
    reg  [191:0] pending = CHIRP;
    reg  [191:0] cur     = CHIRP;
    reg          preset  = 1'b0;
    reg  [31:0]  timer   = 0;
    wire         active;
    wire [31:0]  period = cur[191:160];
    wire         start  = (timer == 0) && (period != 0);

    always @(posedge clk50) begin
        if (rx_settings_valid)
            pending <= rx_settings;
        else if (preset_press) begin
            pending <= preset ? CHIRP : BPSK;
            preset  <= ~preset;
        end
        if (!active && !start)
            cur <= pending;
        timer <= (timer >= period - 1) ? 32'd0 : timer + 1;
    end

    wire [15:0] len       = cur[15:0];
    wire [31:0] ftw_start = cur[47:16];
    wire [31:0] ftw_step  = cur[79:48];
    wire [31:0] win_step  = cur[111:80];
    wire [15:0] code      = cur[127:112];
    wire [15:0] chip_len  = cur[143:128];
    wire [12:0] amp       = cur[156:144];

    // ---- The waveform engine
    wire signed [11:0] out;
    pulse_dds u_eng (.clk(clk50), .rst(rst), .start(start), .len(len),
                     .ftw_start(ftw_start), .ftw_step(ftw_step), .win_step(win_step),
                     .code(code), .chip_len(chip_len), .amp(amp),
                     .out(out), .active(active));

    // 8-bit parallel DAC: offset binary, the DAC latches in the middle of each sample
    always @(posedge clk50)
        dac_d <= {~out[11], out[10:4]};
    assign dac_clk = ~clk50;

    // ---- Capture one pulse and send it back
    localparam [15:0] CAP_N = 1 << CAP_BITS;
    wire [231:0] header = {period, {3'b0, amp}, chip_len, code, win_step, ftw_step, ftw_start,
                           len, CAP_N, 7'b0, (code != 0), 8'h5A, 8'hA5};

    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;
    wire       armed, recording, sending;
    capture #(.N_BITS(CAP_BITS), .HDR_BYTES(29)) u_cap (
        .clk(clk50), .rst(rst), .arm(cap_press | capture_req), .trigger(start), .sample(out),
        .header(header), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(tx_ready),
        .armed(armed), .recording(recording), .sending(sending));

    uart_tx #(.DIV(BAUD_DIV)) u_tx (.clk(clk50), .rst(rst), .data(tx_data), .valid(tx_valid),
                                    .tx(uart_tx), .ready(tx_ready));

    assign led = ~{lock, active, sending, recording, armed, code != 16'd0};

endmodule
