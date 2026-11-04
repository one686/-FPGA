/************************************************************\
 **     Copyright (c) 2012-2024 Anlogic Inc.
 **  All Right Reserved.\
\************************************************************/
`timescale 1 ns / 100 fs

module sys_pll_promax_40 (
  refclk,
  reset,
  clk0_out,
  clk1_out,
  clk2_out,
  clk3_out,
  clk4_out,
  locked
);

  input refclk;
  input reset;
  output clk0_out;
  output clk1_out;
  output clk2_out;
  output clk3_out;
  output clk4_out;
  output locked;

  wire clk0_buf;
  wire clk1_buf;
  wire clk3_buf;
  wire clk4_buf;

  EG_LOGIC_BUFG bufg_feedback (
    .i(clk0_buf),
    .o(clk0_out)
  );

  EG_LOGIC_BUFG bufg_memory (
    .i(clk1_buf),
    .o(clk1_out)
  );

  EG_LOGIC_BUFG bufg_pixel (
    .i(clk3_buf),
    .o(clk3_out)
  );

  EG_LOGIC_BUFG bufg_serial (
    .i(clk4_buf),
    .o(clk4_out)
  );

  EG_PHY_PLL #(
    .DPHASE_SOURCE("DISABLE"),
    .DYNCFG("DISABLE"),
    .FIN("50.000000"),
    .FEEDBK_MODE("NORMAL"),
    .FEEDBK_PATH("CLKC0_EXT"),
    .STDBY_ENABLE("DISABLE"),
    .PLLRST_ENA("ENABLE"),
    .SYNC_ENABLE("DISABLE"),
    .GMC_GAIN(2),
    .ICP_CURRENT(24),
    .KVCO(2),
    .LPF_CAPACITOR(1),
    .LPF_RESISTOR(4),
    .REFCLK_DIV(2),
    .FBCLK_DIV(5),
    .CLKC0_ENABLE("ENABLE"),
    .CLKC0_DIV(8),
    .CLKC0_CPHASE(7),
    .CLKC0_FPHASE(0),
    .CLKC1_ENABLE("ENABLE"),
    .CLKC1_DIV(25),
    .CLKC1_CPHASE(24),
    .CLKC1_FPHASE(0),
    .CLKC2_ENABLE("ENABLE"),
    .CLKC2_DIV(25),
    .CLKC2_CPHASE(9),
    .CLKC2_FPHASE(0),
    .CLKC3_ENABLE("ENABLE"),
    .CLKC3_DIV(40),
    .CLKC3_CPHASE(39),
    .CLKC3_FPHASE(0),
    .CLKC4_ENABLE("ENABLE"),
    .CLKC4_DIV(8),
    .CLKC4_CPHASE(1),
    .CLKC4_FPHASE(0)
  ) pll_inst (
    .refclk(refclk),
    .reset(reset),
    .stdby(1'b0),
    .extlock(locked),
    .load_reg(1'b0),
    .psclk(1'b0),
    .psdown(1'b0),
    .psstep(1'b0),
    .psclksel(3'b000),
    .psdone(open),
    .dclk(1'b0),
    .dcs(1'b0),
    .dwe(1'b0),
    .di(8'b00000000),
    .daddr(6'b000000),
    .do({open, open, open, open, open, open, open, open}),
    .fbclk(clk0_out),
    .clkc({clk4_buf, clk3_buf, clk2_out, clk1_buf, clk0_buf})
  );

endmodule
