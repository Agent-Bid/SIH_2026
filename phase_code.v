// Phase code: flip = code[chip]; moves to the next chip every chip_len clocks.
module phase_code (
    input wire            clk,
    input wire            rst,
    input wire [15:0]     code,
    input wire [23:0]     chip_len,
    output wire           flip
);

    reg [23:0] tick;
    reg [3:0] chip;

    always @(posedge clk)
      if (rst) begin
          tick <= 24'd0;
          chip <= 4'b0;
      end
      else if (tick == chip_len - 1) begin
          tick <= 24'd0;
          chip <= chip + 4'b1;
      end
      else
          tick <= tick + 24'd1;

    assign flip = code[chip];

endmodule
