// Command frames from the PC (standing in for the MCU):
//   A5, cmd, payload..., checksum      checksum = XOR of cmd and every payload byte
//   cmd 01: SET, BYTES-byte payload = one ESP32 packet (SonarPacket v3, checked by pkt_check)
//   cmd 02: CAPTURE, no payload        = capture the next pulse and send it back
//   cmd 03: STREAM ON, no payload      = capture every pulse (each one that starts after the
//                                        previous capture has been sent)
//   cmd 04: STREAM OFF, no payload
// A frame that stops arriving for TIMEOUT clocks is dropped.
module cmd_rx #(parameter BYTES = 57, parameter TIMEOUT = 50000) (
    input  wire         clk,
    input  wire         rst,
    input  wire [7:0]   rx_data,
    input  wire         rx_valid,
    output wire [BYTES*8-1:0] settings,
    output reg          settings_valid,   // one clock: a good SET frame arrived
    output reg          capture_req,      // one clock: a good CAPTURE frame arrived
    output reg          stream_on,        // one clock: STREAM ON
    output reg          stream_off        // one clock: STREAM OFF
);

    localparam SYNC = 2'd0, CMD = 2'd1, PAYLOAD = 2'd2, CHECK = 2'd3;
    localparam SET = 8'h01, CAPTURE = 8'h02, STREAM_ON = 8'h03, STREAM_OFF = 8'h04;

    reg [1:0]   state = SYNC;
    reg [7:0]   cmd;
    reg [7:0]   sum;
    reg [6:0]   n;
    reg [BYTES*8-1:0] buffer;
    assign settings = buffer;         // steady until the next frame's payload
    reg [15:0]  idle;

    always @(posedge clk) begin
        settings_valid <= 1'b0;
        capture_req    <= 1'b0;
        stream_on      <= 1'b0;
        stream_off     <= 1'b0;
        if (rst) begin
            state <= SYNC;
            idle  <= 0;
        end
        else if (!rx_valid) begin
            if (state != SYNC) begin
                idle <= idle + 1;
                if (idle == TIMEOUT)
                    state <= SYNC;
            end
        end
        else begin
            idle <= 0;
            case (state)
                SYNC:
                    if (rx_data == 8'hA5)
                        state <= CMD;
                CMD: begin
                    cmd   <= rx_data;
                    sum   <= rx_data;
                    n     <= 0;
                    state <= (rx_data == SET) ? PAYLOAD :
                             (rx_data == CAPTURE || rx_data == STREAM_ON || rx_data == STREAM_OFF) ? CHECK : SYNC;
                end
                PAYLOAD: begin
                    buffer <= {rx_data, buffer[BYTES*8-1:8]};      // byte 0 ends up in bits [7:0]
                    sum    <= sum ^ rx_data;
                    n      <= n + 1;
                    if (n == BYTES - 1)
                        state <= CHECK;
                end
                CHECK: begin
                    if (rx_data == sum) begin
                        settings_valid <= (cmd == SET);
                        capture_req    <= (cmd == CAPTURE);
                        stream_on      <= (cmd == STREAM_ON);
                        stream_off     <= (cmd == STREAM_OFF);
                    end
                    state <= SYNC;
                end
            endcase
        end
    end

endmodule
