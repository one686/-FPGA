`timescale 1ns/1ps

// OSD 字库 ROM（像素时钟域，读延迟 1 拍）。混合格式，数据宽度 16 位（一个字 2 个字节）：
//   状态栏字形：每像素只存 4 位的 g（无描边，g == o），一个字节存相邻 2 个像素
//     （偶数列在低 4 位，奇数列在高 4 位）
//     取法：地址 = base + (行*宽 + 列) >> 1，字节 b = ROM[地址]，
//           4 位值 = (行*宽 + 列) 是奇数 ? b[7:4] : b[3:0]，令 g = o = 该值
//   非压缩字形（带描边）：每像素 1 字节 [7:4] = 文字覆盖度 g，[3:0] = 文字+描边覆盖度 o
//     取法：地址 = base + 行*宽 + 列，高 4 位是 g、低 4 位是 o
// 状态栏字形 4 位压缩与逐像素 1 字节存法显示逐位相同（design/font_pack_compare.py 验证：53 个字形、整屏 OSD 0 像素差异）
//   综合：16384 x 16 位（= 32K 字节），占 8 块 BRAM32K；初值 osd_font.mif
//   仿真：ModelSim 下定义 OSD_SIM，用普通数组 + $readmemh 读同一份数据（osd_font.hex）
// 两个文件都由 design/osd_gen.py 生成。
module osd_font_rom (
    input  wire        clk,
    input  wire [13:0] addr,
    output wire [15:0] q
);
`ifdef OSD_SIM
    reg [15:0] mem [0:16383];
    reg [15:0] q_r;
    initial $readmemh("../src/user_source/hdl_source/osd/osd_font.hex", mem);
    always @(posedge clk) q_r <= mem[addr];
    assign q = q_r;
`else
    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(16), .ADDR_WIDTH_A(14), .DATA_DEPTH_A(16384),
        .DATA_WIDTH_B(16), .ADDR_WIDTH_B(14), .DATA_DEPTH_B(16384),
        .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
        .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("osd_font.mif"), .FILL_ALL("NONE")
    ) u_bram (
        .dia(16'd0), .dib(16'd0), .addra(addr), .addrb(14'd0),
        .cea(1'b1), .ceb(1'b0), .ocea(1'b0), .oceb(1'b0),
        .clka(clk), .clkb(1'b0), .wea(1'b0), .web(1'b0),
        .bea(1'b0), .beb(1'b0), .rsta(1'b0), .rstb(1'b0),
        .doa(q), .dob()
    );
`endif
endmodule
