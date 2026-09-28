// 27 MHz -> 50.14 MHz (27 * 13 / 7) with the Gowin rPLL. VCO = 802 MHz.
module pll (
    input  wire clk_in,
    output wire clk_out,
    output wire lock
);

    rPLL #(
        .FCLKIN("27"), .DEVICE("GW1NR-9C"),
        .IDIV_SEL(6),                 // divide by 7
        .FBDIV_SEL(12),               // multiply by 13
        .ODIV_SEL(16),                // VCO = 50.14 MHz * 16
        .DYN_IDIV_SEL("false"), .DYN_FBDIV_SEL("false"), .DYN_ODIV_SEL("false"),
        .PSDA_SEL("0000"), .DYN_DA_EN("false"), .DUTYDA_SEL("1000"),
        .CLKOUT_FT_DIR(1'b1), .CLKOUTP_FT_DIR(1'b1),
        .CLKOUT_DLY_STEP(0), .CLKOUTP_DLY_STEP(0),
        .CLKFB_SEL("internal"),
        .CLKOUT_BYPASS("false"), .CLKOUTP_BYPASS("false"), .CLKOUTD_BYPASS("false"),
        .DYN_SDIV_SEL(2), .CLKOUTD_SRC("CLKOUT"), .CLKOUTD3_SRC("CLKOUT")
    ) u_pll (
        .CLKOUT(clk_out), .LOCK(lock), .CLKOUTP(), .CLKOUTD(), .CLKOUTD3(),
        .RESET(1'b0), .RESET_P(1'b0), .CLKIN(clk_in), .CLKFB(1'b0),
        .FBDSEL(6'b0), .IDSEL(6'b0), .ODSEL(6'b0),
        .PSDA(4'b0), .DUTYDA(4'b0), .FDLY(4'b0)
    );

endmodule
