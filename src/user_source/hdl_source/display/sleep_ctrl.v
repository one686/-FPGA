`timescale 1ns/1ps

// 定时关机（像素时钟域）
//   菜单第 6 行选档位 slv：0 关 / 1 1 分 / 2 15 分 / 3 30 分 / 4 60 分 / 5 90 分（slv 由 media_display_path 的菜单给出）。
//   档位一变就从头计（rem = 该档时长）；菜单开着时不走；每秒（sec_tick，osd_runtime 的 1 秒节拍）减 1。
//   剩 00:10 ~ 00:01：warn = 1（画面中间提示框）。这时按任意键 = 重新从头计，这一下按键被吞掉。
//   到 00:00：ST_FADE 画面亮度 f_sl 每帧减 FADE_STEP（64 帧 ≈ 1 秒）、声音暂停、状态条/字幕等淡出（sleep = 1）
//     -> ST_OFF：tmds_off = 1（HDMI 发送端保持复位，没有 TMDS 时钟，显示器"无信号"后待机），LED4 慢闪
//     -> 按任意键：ST_WAKE 打开 HDMI，黑屏等 WAKE_HOLD 帧（显示器重新出图）-> ST_FIN 淡入、声音继续 -> 回到 ST_RUN；
//        唤醒时档位自动变回"关"（slv_clr）。淡出过程中按键也直接淡入回来。
//   按键屏蔽 mask：提示框出现时、淡出 / 关着 / 唤醒 / 淡入时，以及吞掉的那一下按键松手后 REL_FRAMES 帧内为 1，
//     这期间各按键的正常功能都不响应（KEY1 长按 3 秒复位不受影响）。
//   key_press / key_held：4 个键"任意一个按下"经一个消抖器后的单拍 / 电平。
module sleep_ctrl #(
    parameter integer TICKS      = 1,          // 几个 sec_tick 算 1 秒（仿真加速：tb 里运行时间每帧走 60 秒，这里可以设 60）
    parameter [15:0]  LV1_BCD    = 16'h0100,   // 1 分档的时长（仿真可改短）
    parameter integer FADE_STEP  = 4,          // 淡出 / 淡入每帧步长（256 / 4 = 64 帧）
    parameter integer WAKE_HOLD  = 60,         // 唤醒后黑屏等待的帧数（约 1 秒，显示器重新识别信号）
    parameter integer REL_FRAMES = 6           // 吞掉的按键松手后再屏蔽的帧数（约 0.1 秒）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        sec_tick,
    input  wire        menu_open,
    input  wire [2:0]  slv,
    input  wire        key_press,
    input  wire        key_held,
    output reg  [15:0] rem,          // 剩余时间 BCD m1 m0 s1 s0
    output wire        son,          // 显示剩余时间
    output wire        warn,         // 最后 10 秒提示框
    output wire        sleep,        // 状态条、字幕、提示行、频谱淡出
    output reg  [8:0]  f_sl,         // 画面亮度系数（256 = 原样）
    output wire        tmds_off,
    output wire        audio_pause,
    output wire        mask,
    output reg         slv_clr,      // 单拍：档位回到"关"
    output wire        off,          // 关着（LED 指示用）
    output reg         blink         // 关着时每秒翻转一次
);
    localparam [2:0] ST_RUN = 3'd0, ST_FADE = 3'd1, ST_OFF = 3'd2, ST_WAKE = 3'd3, ST_FIN = 3'd4;
    reg [2:0]  st;
    reg [2:0]  slv_q;
    reg        hold;                 // 吞掉一下按键：等松手
    reg [2:0]  rel_cnt;
    reg [6:0]  wcnt;

    function [15:0] preset;
        input [2:0] l;
        begin
            case (l)
                3'd1: preset = LV1_BCD;   3'd2: preset = 16'h1500;  3'd3: preset = 16'h3000;
                3'd4: preset = 16'h6000;  3'd5: preset = 16'h9000;  default: preset = 16'h0000;
            endcase
        end
    endfunction
    function [15:0] bcd_dec;         // mm:ss 减 1 秒（t != 0）
        input [15:0] t;
        begin
            bcd_dec = t;
            if (t[3:0] != 4'd0) bcd_dec[3:0] = t[3:0] - 4'd1;
            else begin
                bcd_dec[3:0] = 4'd9;
                if (t[7:4] != 4'd0) bcd_dec[7:4] = t[7:4] - 4'd1;
                else begin
                    bcd_dec[7:4] = 4'd5;
                    if (t[11:8] != 4'd0) bcd_dec[11:8] = t[11:8] - 4'd1;
                    else begin
                        bcd_dec[11:8] = 4'd9;
                        bcd_dec[15:12] = t[15:12] - 4'd1;
                    end
                end
            end
        end
    endfunction

    // 1 秒节拍（TICKS 个 sec_tick）
    wire tick;
    generate
        if (TICKS <= 1) begin : g_t1
            assign tick = sec_tick;
        end else begin : g_tn
            reg [$clog2(TICKS)-1:0] tc;
            always @(posedge clk or negedge rst_n)
                if (!rst_n) tc <= 0;
                else if (sec_tick) tc <= (tc == TICKS - 1) ? 0 : tc + 1'b1;
            assign tick = sec_tick && (tc == TICKS - 1);
        end
    endgenerate

    wire run = (st == ST_RUN);
    assign son   = run && (slv != 3'd0);
    assign warn  = son && (rem[15:8] == 8'h00) && (rem[7:0] != 8'h00) && (rem[7:0] <= 8'h10);
    assign sleep = (st == ST_FADE) || (st == ST_OFF) || (st == ST_WAKE);
    assign tmds_off    = (st == ST_OFF);
    assign audio_pause = sleep;
    assign off  = (st == ST_OFF);
    assign mask = warn || !run || hold;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= ST_RUN; slv_q <= 3'd0; rem <= 16'd0; f_sl <= 9'd256;
            hold <= 1'b0; rel_cnt <= 3'd0; wcnt <= 7'd0; slv_clr <= 1'b0; blink <= 1'b0;
        end else begin
            slv_clr <= 1'b0;
            slv_q <= slv;
            if (sec_tick) blink <= off ? ~blink : 1'b0;
            // 吞掉的按键：松手后再等 REL_FRAMES 帧
            if (hold) begin
                if (key_held) rel_cnt <= 3'd0;
                else if (frame_start) begin
                    if (rel_cnt == REL_FRAMES - 1) hold <= 1'b0;
                    else rel_cnt <= rel_cnt + 3'd1;
                end
            end
            case (st)
                ST_RUN: begin
                    if (slv != slv_q)                              // 档位变了：从头计（变成"关"就停）
                        rem <= preset(slv);
                    else if (warn && key_press) begin              // 提示时按任意键：从头计，吞掉这一下
                        rem <= preset(slv); hold <= 1'b1; rel_cnt <= 3'd0;
                    end else if (son && !menu_open) begin
                        if (rem == 16'd0) st <= ST_FADE;
                        else if (tick) rem <= bcd_dec(rem);
                    end
                end
                ST_FADE: begin
                    if (key_press) begin                          // 淡出时按键：直接淡入回来
                        st <= ST_FIN; slv_clr <= 1'b1; hold <= 1'b1; rel_cnt <= 3'd0;
                    end else if (frame_start) begin
                        if (f_sl <= FADE_STEP) begin f_sl <= 9'd0; st <= ST_OFF; end
                        else f_sl <= f_sl - FADE_STEP;
                    end
                end
                ST_OFF: begin
                    if (key_press) begin
                        st <= ST_WAKE; slv_clr <= 1'b1; hold <= 1'b1; rel_cnt <= 3'd0; wcnt <= 7'd0;
                    end
                end
                ST_WAKE: begin
                    if (frame_start) begin
                        if (wcnt >= WAKE_HOLD - 1) st <= ST_FIN;
                        else wcnt <= wcnt + 7'd1;
                    end
                end
                default: begin                                    // ST_FIN 淡入
                    if (frame_start) begin
                        if (f_sl >= 9'd256 - FADE_STEP) begin f_sl <= 9'd256; st <= ST_RUN; end
                        else f_sl <= f_sl + FADE_STEP;
                    end
                end
            endcase
        end
    end
endmodule
