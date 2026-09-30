// SPI receiver for the ESP32's packets. Mode 0: SCK idles low, MOSI is read on SCK's rising
// edge, most significant bit first.
// Framing, two ways:
//   USE_CS = 1: cs_n low frames one packet (standard SPI).
//   USE_CS = 0: cs_n is ignored; a packet ends when SCK has been quiet for GAP clocks. The ESP32
//               sends each packet as one continuous burst (57 bytes = 228 us at 2 MHz) and then
//               nothing for ~100 ms, so the silence marks the end. Used on the current boards,
//               whose CS connection does not work (the pin only picks up noise).
// The pins are sampled with the FPGA clock (a 2-flip-flop synchronizer each, plus one more to spot
// SCK's edges), which works while SCK is well under clk / 8 (2 MHz vs 50 MHz here).
// When a frame ends after exactly BYTES bytes, packet holds them (byte 0 in bits [7:0]) and valid
// is 1 for one clock; any other length gives bad for one clock. The sync byte and CRC are checked
// later (pkt_check).
module spi_rx #(
    parameter BYTES  = 57,
    parameter USE_CS = 1,
    parameter GAP    = 5000               // clocks of silence that end a frame (100 us at 50 MHz)
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 sck,
    input  wire                 mosi,
    input  wire                 cs_n,
    output reg  [BYTES*8-1:0]   packet,
    output reg                  valid,
    output reg                  bad
);

    reg [2:0] sck_s  = 3'b000;
    reg [2:0] cs_s   = 3'b111;
    reg [1:0] mosi_s = 2'b00;
    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0], sck};
        cs_s   <= {cs_s[1:0], cs_n};
        mosi_s <= {mosi_s[0], mosi};
    end

    wire sck_rise = sck_s[1] & ~sck_s[2];

    reg [2:0]  bit_n;
    reg [6:0]  shift;
    reg [6:0]  n_bytes;                  // stops counting at 127
    reg [15:0] idle;                     // clocks since the last SCK edge (stops at GAP)

    wire started   = (n_bytes != 0) || (bit_n != 0);
    wire selected  = USE_CS ? ~cs_s[1] : 1'b1;
    // one clock at the moment the silence reaches GAP (not while it lasts, so the next packet's
    // first edge is never swallowed)
    wire frame_end = USE_CS ? (cs_s[1] & ~cs_s[2]) : (idle == GAP - 2 && !sck_rise);

    always @(posedge clk) begin
        valid <= 1'b0;
        bad   <= 1'b0;
        idle  <= sck_rise ? 16'd0 : (idle == GAP - 1 ? idle : idle + 1);
        if (rst) begin
            bit_n   <= 0;
            n_bytes <= 0;
        end
        else if (frame_end || !selected) begin
            if (frame_end && started) begin
                if (n_bytes == BYTES && bit_n == 0)
                    valid <= 1'b1;
                else
                    bad <= 1'b1;
            end
            bit_n   <= 0;
            n_bytes <= 0;
        end
        else if (sck_rise) begin
            shift <= {shift[5:0], mosi_s[1]};
            bit_n <= bit_n + 1;
            if (bit_n == 7) begin
                packet <= {shift, mosi_s[1], packet[BYTES*8-1:8]};   // byte 0 ends up in [7:0]
                if (n_bytes != 7'd127)
                    n_bytes <= n_bytes + 1;
            end
        end
    end

endmodule
