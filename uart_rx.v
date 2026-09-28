// UART receiver, 8N1. Finds the middle of the start bit, then samples every bit in its middle.
// valid is 1 for one clock when a byte with a good stop bit has arrived.
module uart_rx #(parameter DIV = 435) (
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);

    localparam IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [1:0]  sync = 2'b11;        // two flip-flops: bring the async pin into the clock domain
    wire       line = sync[1];
    reg [1:0]  state = IDLE;
    reg [15:0] count;
    reg [2:0]  bit_n;
    reg [7:0]  shift;

    always @(posedge clk) begin
        sync  <= {sync[0], rx};
        valid <= 1'b0;
        if (rst)
            state <= IDLE;
        else case (state)
            IDLE:
                if (!line) begin
                    count <= 0;
                    state <= START;
                end
            START:
                if (count == DIV / 2 - 1) begin
                    count <= 0;
                    bit_n <= 0;
                    state <= line ? IDLE : DATA;     // a glitch, not a real start bit
                end
                else
                    count <= count + 1;
            DATA:
                if (count == DIV - 1) begin
                    count <= 0;
                    shift <= {line, shift[7:1]};
                    if (bit_n == 7)
                        state <= STOP;
                    else
                        bit_n <= bit_n + 1;
                end
                else
                    count <= count + 1;
            STOP:
                if (count == DIV - 1) begin
                    state <= IDLE;
                    if (line) begin
                        data  <= shift;
                        valid <= 1'b1;
                    end
                end
                else
                    count <= count + 1;
        endcase
    end

endmodule
