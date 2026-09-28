// Debounced active-low push button: pressed is 1 for one clock per press.
// The level must hold for 2^BITS clocks (~21 ms at 50 MHz) before it counts.
module button #(parameter BITS = 20) (
    input  wire clk,
    input  wire btn_n,
    output reg  pressed
);

    reg [1:0]      sync  = 2'b11;    // two flip-flops: bring the async pin into the clock domain
    reg            level = 1'b1;     // debounced level, 1 = released
    reg [BITS-1:0] count = 0;

    always @(posedge clk) begin
        sync    <= {sync[0], btn_n};
        pressed <= 1'b0;
        if (sync[1] == level)
            count <= 0;
        else begin
            count <= count + 1;
            if (&count) begin
                level <= sync[1];
                if (!sync[1])
                    pressed <= 1'b1;
            end
        end
    end

endmodule
