`timescale 1ns/1ps

// 按键短按 / 长按 / 按住连发检测（低电平按下，单时钟域）
//   按下消抖后开始计时：
//     按住满 LONG_MS      -> long_press 单拍脉冲，之后每 REPEAT_MS 再来一个（按住连发）
//     不到 LONG_MS 就松开 -> 松开时 short_press 单拍脉冲
//     长按过的这一次，松开时不再出短按
//   press = 消抖后按下那一拍的单拍脉冲（不需要区分长短按时用，按下立即响应）
//   held = 消抖后的"按住"电平；long_active = 本次按住已经触发过长按（松开清零）
//   REPEAT=0 时长按只触发一次（按住不连发）。
// 短按在"松开时"生效，这样同一个键才能区分短按和长按（按下即生效的话，长按前会先多出一次短按）。
// EXT_MS = 1 时不用内部分频器，改用外部 1 ms 节拍 ms_in：同一时钟域、同一复位的多个按键共用一个分频器以节省 LUT。
module key_short_long #(
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_MS     = 1000,
    parameter integer REPEAT_MS   = 500,
    parameter integer REPEAT      = 1,
    parameter integer EXT_MS      = 0
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ms_in,              // EXT_MS = 1 时的 1 ms 节拍（单拍），否则不用
    input  wire key_n,              // 按下 = 0
    output reg  press,
    output reg  short_press,
    output reg  long_press,
    output wire held,
    output reg  long_active
);
    // 资源优化：先分频出 1ms 节拍，消抖和按住计时都按毫秒数（计数器位数少很多）
    localparam integer MS_CYC = CLK_HZ / 1000;
    localparam integer PW = $clog2(MS_CYC + 1);
    localparam integer HMAX = (LONG_MS > REPEAT_MS) ? LONG_MS : REPEAT_MS;
    localparam integer DW = $clog2(DEBOUNCE_MS + 1), HW = $clog2(HMAX + 1);

    reg s0, s1, stable;
    reg [PW-1:0] pcnt;
    reg [DW-1:0] dcnt;
    reg [HW-1:0] hcnt;
    wire ms = EXT_MS ? ms_in : (pcnt == MS_CYC - 1);
    assign held = !stable;
    wire flip = (s1 != stable) && ms && (dcnt >= DEBOUNCE_MS - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0 <= 1'b1; s1 <= 1'b1; stable <= 1'b1;
            pcnt <= 0; dcnt <= 0; hcnt <= 0;
            press <= 1'b0; short_press <= 1'b0; long_press <= 1'b0; long_active <= 1'b0;
        end else begin
            s0 <= key_n; s1 <= s0;
            press       <= 1'b0;
            short_press <= 1'b0;
            long_press  <= 1'b0;
            if (!EXT_MS) pcnt <= ms ? {PW{1'b0}} : pcnt + 1'b1;
            // 消抖：输入与稳定值不同且持续 DEBOUNCE_MS 毫秒才翻转（中途变回去就重新计）
            if (s1 == stable)
                dcnt <= 0;
            else if (flip) begin
                dcnt <= 0;
                stable <= s1;
                if (!s1) press <= 1'b1;          // 按下
                if (s1 && !long_active)          // 松开：没长按过 = 短按
                    short_press <= 1'b1;
                if (s1) long_active <= 1'b0;
                hcnt <= 0;
            end else if (ms)
                dcnt <= dcnt + 1'b1;
            // 按住计时（翻转这一拍不计，避免松开同拍又触发长按）
            if (!stable && !flip && ms) begin
                if (!long_active) begin
                    if (hcnt >= LONG_MS - 1) begin
                        long_press <= 1'b1; long_active <= 1'b1; hcnt <= 0;
                    end else
                        hcnt <= hcnt + 1'b1;
                end else if (REPEAT) begin
                    if (hcnt >= REPEAT_MS - 1) begin
                        long_press <= 1'b1; hcnt <= 0;
                    end else
                        hcnt <= hcnt + 1'b1;
                end
            end
        end
    end
endmodule
