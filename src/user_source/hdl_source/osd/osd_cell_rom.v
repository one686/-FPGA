`timescale 1ns/1ps

// OSD 字符格表 ROM：1024 x 23 位 = {x0[9:0], w[4:0], src[7:0]}，地址 = ckey = {b_sel, w_alt, 格号[4:0]}
//   每个格的起始 x、宽度、字形源编号查表得到；放在 1 块 BRAM32K 中（比组合逻辑查表约省 400 个 LUT），读延迟 1 拍
//   初值 osd_cell.mif（综合）/ osd_cell.hex（仿真，ModelSim 下定义 OSD_SIM），都由 design/osd_gen.py 生成
module osd_cell_rom (
    input  wire        clk,
    input  wire [9:0]  addr,
    output wire [22:0] q
);
`ifdef OSD_SIM
    reg [22:0] mem [0:1023];
    reg [22:0] q_r;
    initial $readmemh("../src/user_source/hdl_source/osd/osd_cell.hex", mem);
    always @(posedge clk) q_r <= mem[addr];
    assign q = q_r;
`else
    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(23), .ADDR_WIDTH_A(10), .DATA_DEPTH_A(1024),
        .DATA_WIDTH_B(23), .ADDR_WIDTH_B(10), .DATA_DEPTH_B(1024),
        .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
        .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("osd_cell.mif"), .FILL_ALL("NONE")
    ) u_bram (
        .dia(23'd0), .dib(23'd0), .addra(addr), .addrb(10'd0),
        .cea(1'b1), .ceb(1'b0), .ocea(1'b0), .oceb(1'b0),
        .clka(clk), .clkb(1'b0), .wea(1'b0), .web(1'b0),
        .bea(1'b0), .beb(1'b0), .rsta(1'b0), .rstb(1'b0),
        .doa(q), .dob()
    );
`endif
endmodule
