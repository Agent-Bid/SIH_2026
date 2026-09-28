// UART transmitter, 8N1: idle high, a start bit (0), 8 data bits LSB first, a stop bit (1).
// Every bit lasts DIV clocks (50.14 MHz / 435 = 115200 baud).
module uart_tx #(parameter DIV = 435) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       valid,    // 1 for one clock: send data (only while ready)
    output reg        tx,       // the serial line
    output wire       ready     // 1 while idle and able to take a byte
);

    localparam IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [1:0]  state = IDLE;
    reg [15:0] count;
    reg [2:0]  bit_n;
    reg [7:0]  shift;

    assign ready = (state == IDLE);

    always @(posedge clk)
        if (rst) begin
            state <= IDLE;
            tx    <= 1'b1;
        end
        else case (state)
            IDLE: begin
                tx <= 1'b1;
                if (valid) begin
                    shift <= data;
                    count <= 0;
                    tx    <= 1'b0;
                    state <= START;
                end
            end
            START:
                if (count == DIV - 1) begin
                    count <= 0;
                    bit_n <= 0;
                    tx    <= shift[0];
                    state <= DATA;
                end
                else
                    count <= count + 1;
            DATA:
                if (count == DIV - 1) begin
                    count <= 0;
                    if (bit_n == 7) begin
                        tx    <= 1'b1;
                        state <= STOP;
                    end
                    else begin
                        bit_n <= bit_n + 1;
                        tx    <= shift[bit_n + 1];
                    end
                end
                else
                    count <= count + 1;
            STOP:
                if (count == DIV - 1)
                    state <= IDLE;
                else
                    count <= count + 1;
        endcase

endmodule
