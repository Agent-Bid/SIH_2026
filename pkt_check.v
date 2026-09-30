// Checks one ESP32 packet (SonarPacket v3, see MCU /SIH/sonar_core.h): sync byte 0xA5, the
// version, and the CRC-8 (poly 0x07, init 0) over every byte but the last, one byte per clock.
// About BYTES clocks after valid, gives ok (and packet in fields) or bad, for one clock.
// A packet that arrives while one is being checked is ignored.
module pkt_check #(
    parameter BYTES   = 57,
    parameter VERSION = 3
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire [BYTES*8-1:0]   packet,
    input  wire                 valid,
    output reg  [BYTES*8-1:0]   fields,
    output reg                  ok,
    output reg                  bad
);

    function [7:0] crc_step(input [7:0] crc, input [7:0] data);
        integer b;
        reg [7:0] x;
        begin
            x = crc ^ data;
            for (b = 0; b < 8; b = b + 1)
                x = x[7] ? {x[6:0], 1'b0} ^ 8'h07 : {x[6:0], 1'b0};
            crc_step = x;
        end
    endfunction

    reg               busy = 1'b0;
    reg [6:0]         i;
    reg [7:0]         crc;
    reg [BYTES*8-1:0] rest;                           // the bytes still to go, next one in [7:0]
    wire [7:0]        crc_next = crc_step(crc, rest[7:0]);

    always @(posedge clk) begin
        ok  <= 1'b0;
        bad <= 1'b0;
        if (rst)
            busy <= 1'b0;
        else if (!busy) begin
            if (valid) begin
                fields <= packet;
                rest   <= packet;
                crc    <= 8'd0;
                i      <= 0;
                busy   <= 1'b1;
            end
        end
        else begin
            crc  <= crc_next;
            rest <= rest >> 8;
            i    <= i + 1;
            if (i == BYTES - 2) begin
                busy <= 1'b0;
                if (crc_next == fields[BYTES*8-1 -: 8] && fields[7:0] == 8'hA5 && fields[15:8] == VERSION)
                    ok <= 1'b1;
                else
                    bad <= 1'b1;
            end
        end
    end

endmodule
