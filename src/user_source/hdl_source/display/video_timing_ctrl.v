`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// video_timing_ctrl
// 视频时序控制器(VTC)：像素时钟域内的行/场计数器加同拍组合译码。
// 输出约定：
//   x/y      : 行/场计数器本体，0 .. H_TOTAL-1 / V_TOTAL-1；de=1时即有效像素坐标
//   de       : 有效显示区标志，(x < H_ACT) && (y < V_ACT)
//   hsync    : 低有效行同步脉冲，位于 [H_ACT+H_FP, H_ACT+H_FP+H_SYNC)
//   vsync    : 低有效场同步脉冲，位于 [V_ACT+V_FP, V_ACT+V_FP+V_SYNC)
// 段顺序为 ACTIVE -> FRONT PORCH -> SYNC -> BACK PORCH（ACTIVE_LAST=1 时有效区放在最后）。
// 所有输出为计数器的同拍组合译码；整组信号统一寄存一拍由下游负责。
// 四段参数各自必须 >= 1。默认参数为 640x480@60；仿真可用缩小参数跑整帧。
// -----------------------------------------------------------------------------
module video_timing_ctrl #(
    parameter H_ACT  = 640,  // 水平有效像素
    parameter H_FP   = 16,   // 水平前肩
    parameter H_SYNC = 96,   // 水平同步脉宽
    parameter H_BP   = 48,   // 水平后肩
    parameter V_ACT  = 480,  // 垂直有效行
    parameter V_FP   = 10,   // 垂直前肩
    parameter V_SYNC = 2,    // 垂直同步脉宽
    parameter V_BP   = 33,   // 垂直后肩
    parameter ACTIVE_LAST = 0
)(
    input  wire        pclk,   // 像素时钟（25 MHz，由 PLL 从 50 MHz 生成）
    input  wire        rst_n,  // 低有效复位
    output wire        hsync,  // 低有效
    output wire        vsync,  // 低有效
    output wire        de,     // 有效显示区
    output wire [11:0] x,      // 行计数器；de=1时为有效像素列坐标
    output wire [11:0] y       // 场计数器；de=1时为有效像素行坐标
);

    localparam H_TOTAL = H_ACT + H_FP + H_SYNC + H_BP;
    localparam V_TOTAL = V_ACT + V_FP + V_SYNC + V_BP;

    // 行/场计数：x走完一行，y走一步；两个都走完回到帧起点(0,0)
    // 计数器内部只用 CW 位（640x480 时 H_TOTAL=800、V_TOTAL=525，10 位），输出高位接 0，
    //   端口仍是 12 位。高位恒为 0，综合时下游比较器随之变窄，节省资源
    localparam integer CW = (H_TOTAL > V_TOTAL) ? $clog2(H_TOTAL) : $clog2(V_TOTAL);
    reg [CW-1:0] xc, yc;
    assign x = {{(12-CW){1'b0}}, xc};
    assign y = {{(12-CW){1'b0}}, yc};
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            xc <= {CW{1'b0}};
            yc <= {CW{1'b0}};
        end else if (xc == H_TOTAL - 1) begin
            xc <= {CW{1'b0}};
            yc <= (yc == V_TOTAL - 1) ? {CW{1'b0}} : yc + 1'b1;
        end else begin
            xc <= xc + 1'b1;
        end
    end

    // 同拍组合译码：de/hsync/vsync与x/y永远对齐，不存在错拍问题
    assign de    = ACTIVE_LAST ?
                   ((x >= H_TOTAL - H_ACT) && (y >= V_TOTAL - V_ACT)) :
                   ((x < H_ACT) && (y < V_ACT));
    assign hsync = ACTIVE_LAST ?
                   ~((x >= H_FP) && (x < H_FP + H_SYNC)) :
                   ~((x >= H_ACT + H_FP) && (x < H_ACT + H_FP + H_SYNC));
    assign vsync = ACTIVE_LAST ?
                   ~((y >= V_FP) && (y < V_FP + V_SYNC)) :
                   ~((y >= V_ACT + V_FP) && (y < V_ACT + V_FP + V_SYNC));

endmodule
