`timescale 1ns/1ps

// 板上 KEY1（顶层 rst_n 引脚）一键两用，50MHz 时钟域：
//   短按（不到 LONG_MS 就松手，松手时生效）-> short_press 单拍脉冲（胶片条进/出、音乐可视化开/关）
//   按住满 LONG_MS（默认 3 秒）            -> sys_rst 拉高并一直保持到松手（松手那一下不算短按），整个系统复位
// 本模块自己不能被它产生的复位清掉，所以没有复位端：寄存器上电初值 0，0 的编码都是"没按"（pressed = 1 表示按下），
//   上电后不会误出短按或复位。消抖、计时都按 1 毫秒节拍（同 key_short_long）。
module key_reset_long #(
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_MS     = 3000
) (
    input  wire clk,
    input  wire key_n,              // 按下 = 0
    output wire short_press,
    output wire held,               // 消抖后的"按住"电平（定时关机/唤醒的任意键检测用）
    output wire sys_rst             // 1 = 复位（长按满 LONG_MS 起到松手）
);
    localparam integer MS_CYC = CLK_HZ / 1000;
    localparam integer PW = $clog2(MS_CYC + 1);
    localparam integer DW = $clog2(DEBOUNCE_MS + 1), HW = $clog2(LONG_MS + 1);

    reg          p0 = 1'b0, p1 = 1'b0, pressed = 1'b0;   // 同步后的"按下"、消抖后的"按下"
    reg [PW-1:0] pcnt = 0;
    reg [DW-1:0] dcnt = 0;
    reg [HW-1:0] hcnt = 0;
    reg          sp_r = 1'b0, rst_r = 1'b0;
    assign short_press = sp_r;
    assign sys_rst     = rst_r;
    assign held        = pressed;
    wire ms   = (pcnt == MS_CYC - 1);
    wire flip = (p1 != pressed) && ms && (dcnt >= DEBOUNCE_MS - 1);

    always @(posedge clk) begin
        p0 <= !key_n; p1 <= p0;
        sp_r <= 1'b0;
        pcnt <= ms ? {PW{1'b0}} : pcnt + 1'b1;
        if (p1 == pressed)
            dcnt <= 0;
        else if (flip) begin
            dcnt    <= 0;
            pressed <= p1;
            hcnt    <= 0;
            if (!p1) begin                              // 松手
                if (!rst_r) sp_r <= 1'b1;         // 没到长按 = 短按
                rst_r <= 1'b0;
            end
        end else if (ms)
            dcnt <= dcnt + 1'b1;
        if (pressed && !flip && ms && !rst_r) begin
            if (hcnt >= LONG_MS - 1) rst_r <= 1'b1;
            else                     hcnt <= hcnt + 1'b1;
        end
    end
endmodule
