// Capture buffer: after arm, records 2^N_BITS samples starting at the next trigger,
// then sends the header and the samples to a UART transmitter, one byte at a time.
// Samples go out as 16-bit little-endian (sign-extended from 12 bits). The header is
// frozen when the pulse starts, so it always describes the settings of the captured pulse.
module capture #(
    parameter N_BITS    = 14,
    parameter HDR_BYTES = 25
) (
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   arm,        // one clock: capture the next pulse
    input  wire                   trigger,    // one clock: a pulse starts now
    input  wire signed [11:0]     sample,
    input  wire [HDR_BYTES*8-1:0] header,     // byte 0 in bits [7:0]
    output reg  [7:0]             tx_data,
    output reg                    tx_valid,
    input  wire                   tx_ready,
    output wire                   armed,
    output wire                   recording,
    output wire                   sending
);

    localparam IDLE = 3'd0, ARMED = 3'd1, RECORD = 3'd2, SEND_HDR = 3'd3, SEND_DATA = 3'd4;

    reg [2:0]        state = IDLE;
    reg [N_BITS-1:0] addr;
    reg [7:0]        hdr_i;
    reg [HDR_BYTES*8-1:0] hdr;
    reg              high_byte;

    // The buffer: banks of 1024 x 12, one block RAM each (the 1K x 18 mode). Apycula 0.33
    // gets the deep, narrow block RAM modes (16K x 1, 4K x 4) wrong on the chip, so the
    // top address bits pick a bank instead. Needs N_BITS >= 11.
    localparam BANKS = 1 << (N_BITS - 10);

    reg                   wr_en = 1'b0;
    reg  [N_BITS-1:0]     wr_addr;
    reg  [11:0]           wr_data;
    reg  [N_BITS-11:0]    rd_bank;
    wire [12*BANKS-1:0]   bank_q;
    wire [11:0]           rd = bank_q[rd_bank*12 +: 12];

    genvar g;
    generate
        for (g = 0; g < BANKS; g = g + 1) begin : bank
            reg [11:0] m [0:1023];
            reg [11:0] q;
            always @(posedge clk) begin
                if (wr_en && wr_addr[N_BITS-1:10] == g)
                    m[wr_addr[9:0]] <= wr_data;
                q <= m[addr[9:0]];
            end
            assign bank_q[g*12 +: 12] = q;
        end
    endgenerate

    always @(posedge clk)
        rd_bank <= addr[N_BITS-1:10];

    assign armed     = (state == ARMED);
    assign recording = (state == RECORD);
    assign sending   = (state == SEND_HDR) || (state == SEND_DATA);

    wire can_send = tx_ready && !tx_valid;

    always @(posedge clk) begin
        tx_valid <= 1'b0;
        wr_en    <= 1'b0;
        if (rst)
            state <= IDLE;
        else case (state)
            IDLE:
                if (arm)
                    state <= ARMED;
            ARMED:
                if (trigger) begin
                    hdr     <= header;
                    wr_en   <= 1'b1;
                    wr_addr <= 0;
                    wr_data <= sample;
                    addr    <= 1;
                    state   <= RECORD;
                end
            RECORD: begin
                wr_en     <= 1'b1;
                wr_addr   <= addr;
                wr_data   <= sample;
                addr      <= addr + 1;
                if (&addr) begin
                    hdr_i <= 0;
                    state <= SEND_HDR;
                end
            end
            SEND_HDR:
                if (can_send) begin
                    tx_data  <= hdr[hdr_i*8 +: 8];
                    tx_valid <= 1'b1;
                    hdr_i    <= hdr_i + 1;
                    if (hdr_i == HDR_BYTES - 1) begin
                        high_byte <= 1'b0;
                        state     <= SEND_DATA;
                    end
                end
            SEND_DATA:
                if (can_send) begin
                    tx_valid <= 1'b1;
                    if (!high_byte) begin
                        tx_data   <= rd[7:0];
                        high_byte <= 1'b1;
                    end
                    else begin
                        tx_data   <= {{4{rd[11]}}, rd[11:8]};
                        high_byte <= 1'b0;
                        addr      <= addr + 1;
                        if (&addr)
                            state <= IDLE;
                    end
                end
            default:
                state <= IDLE;
        endcase
    end

endmodule
