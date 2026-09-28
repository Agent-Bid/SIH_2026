// Pulse controller: after a start, active is high for len clocks. Start is ignored while active.
module pulse_ctrl (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [15:0] len,
    output reg         active
);

    reg [15:0] count;

    always @(posedge clk)
        if (rst) begin
            active <= 1'b0;
            count  <= 16'd0;
        end
        else if (start && !active) begin
            active <= 1'b1;
            count  <= 16'd0;
        end
        else if (active) begin
            if (count == len - 1)
                active <= 1'b0;
            count <= count + 1;
        end

endmodule
