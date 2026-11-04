`timescale 1ns/1ps

// 字幕条 RAM（5 块 BRAM32K）：存放从 TF 卡读入的 SUB.BMP 字幕条和内置默认标语
//   主 RAM 16384 x 8 位（4 块）：SD 时钟写（sub_loader 把 SUB.BMP 的像素字节按文件顺序写进来），像素时钟读
//   默认 RAM 4096 x 8 位（1 块）：只读，初值 = 默认标语"热烈欢迎"（design/make_subtitle.py --default 生成 sub_default.mif / .hex）
//   每个字节 2 个像素，高 4 位 = 偶数列；行从下往上存（和 BMP 一样），读的一方把行号反过来
//   读延迟 1 拍；use_def 和地址同拍给，选择也延迟 1 拍
//   综合：直接例化 EG_LOGIC_BRAM 32K（BRAM9K 余量不足，不能交给 TD 自动推断）；仿真：ModelSim 下定义 OSD_SIM，用数组 + $readmemh
module sub_strip_ram (
    input  wire        wclk,
    input  wire        we,
    input  wire [13:0] waddr,
    input  wire [7:0]  wdata,
    input  wire        rclk,
    input  wire        use_def,
    input  wire [13:0] raddr,
    output wire [7:0]  q
);
    wire [7:0] q_main, q_def;
    reg        def_q;
    always @(posedge rclk) def_q <= use_def;
    assign q = def_q ? q_def : q_main;
`ifdef OSD_SIM
    reg [7:0] mem [0:16383];
    reg [7:0] dmem [0:4095];
    reg [7:0] qm, qd;
    initial $readmemh("../src/user_source/hdl_source/osd/sub_default.hex", dmem);
    always @(posedge wclk) if (we) mem[waddr] <= wdata;
    always @(posedge rclk) begin
        qm <= mem[raddr];
        qd <= dmem[raddr[11:0]];
    end
    assign q_main = qm;
    assign q_def  = qd;
`else
    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(8), .ADDR_WIDTH_A(14), .DATA_DEPTH_A(16384),
        .DATA_WIDTH_B(8), .ADDR_WIDTH_B(14), .DATA_DEPTH_B(16384),
        .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"), .IMPLEMENT("32K"),
        .DEBUGGABLE("NO"), .PACKABLE("NO"), .INIT_FILE("NONE"), .FILL_ALL("NONE")
    ) u_main (
        .dia(wdata), .dib(8'd0), .addra(waddr), .addrb(raddr),
        .cea(1'b1), .ceb(1'b1), .ocea(1'b0), .oceb(1'b0),
        .clka(wclk), .clkb(rclk), .wea(we), .web(1'b0),
        .bea(1'b0), .beb(1'b0), .rsta(1'b0), .rstb(1'b0),
        .doa(), .dob(q_main)
    );
    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(8), .ADDR_WIDTH_A(12), .DATA_DEPTH_A(4096),
        .DATA_WIDTH_B(8), .ADDR_WIDTH_B(12), .DATA_DEPTH_B(4096),
        .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("32K"),
        .DEBUGGABLE("NO"), .PACKABLE("NO"), .INIT_FILE("sub_default.mif"), .FILL_ALL("NONE")
    ) u_def (
        .dia(8'd0), .dib(8'd0), .addra(raddr[11:0]), .addrb(12'd0),
        .cea(1'b1), .ceb(1'b0), .ocea(1'b0), .oceb(1'b0),
        .clka(rclk), .clkb(1'b0), .wea(1'b0), .web(1'b0),
        .bea(1'b0), .beb(1'b0), .rsta(1'b0), .rstb(1'b0),
        .doa(q_def), .dob()
    );
`endif
endmodule
