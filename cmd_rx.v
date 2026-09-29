// Command frames from the PC (standing in for the MCU):
//   A5, cmd, payload..., checksum      checksum = XOR of cmd and every payload byte
//   cmd 01: SET, BYTES-byte payload = the settings struct (little-endian fields, see top.v)
//   cmd 02: CAPTURE, no payload        = capture the next pulse and send it back
// A frame that stops arriving for TIMEOUT clocks is dropped.
module cmd_rx #(parameter BYTES = 26, parameter TIMEOUT = 50000) (
    input  wire         clk,
    input  wire         rst,
    input  wire [7:0]   rx_data,
    input  wire         rx_valid,
    output reg  [BYTES*8-1:0] settings,
    output reg          settings_valid,   // one clock: a good SET frame arrived
    output reg          capture_req       // one clock: a good CAPTURE frame arrived
);

    localparam SYNC = 2'd0, CMD = 2'd1, PAYLOAD = 2'd2, CHECK = 2'd3;
    localparam SET = 8'h01, CAPTURE = 8'h02;

    reg [1:0]   state = SYNC;
    reg [7:0]   cmd;
    reg [7:0]   sum;
    reg [4:0]   n;
    reg [BYTES*8-1:0] buffer;
    reg [15:0]  idle;

    always @(posedge clk) begin
        settings_valid <= 1'b0;
        capture_req    <= 1'b0;
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
                    state <= (rx_data == SET) ? PAYLOAD : (rx_data == CAPTURE) ? CHECK : SYNC;
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
                        if (cmd == SET) begin
                            settings       <= buffer;
                            settings_valid <= 1'b1;
                        end
                        else
                            capture_req <= 1'b1;
                    end
                    state <= SYNC;
                end
            endcase
        end
    end

endmodule
