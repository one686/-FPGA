`timescale 1ns/1ps

// 全屏音乐可视化（video_clk 域，25MHz）：64 根彩虹频谱柱 + 倒影 + 峰值帽 + 波形线，叠在 OSD 和音浪层之上
//   音乐层短按板上 KEY1 开 / 关（上层给 viz 电平），开关时按帧淡入淡出
//   画面和 design/fullscreen_gen.py 逐位一致（那里有完整说明），这里只记实现：
//   - 64 根频谱柱：柱高 / 峰值帽从 audio_spectrum 显示表的第二个读口取（fs_ra / fs_rq），不另算频谱
//       每 20 列是引擎的一根 i：前 10 列 = 第 i 根，后 10 列 = (第 i 根 + 第 i+1 根) >> 1
//       cur = 第 i 根（行首和每 20 列的最后一列装入），整段读口地址 = i + 1（第 31 根读自己），读出来就是"下一根"
//   - 彩虹色：一个组合函数（6 段 x 32，(217*f)>>5 的斜坡），柱子和波形线共用（帧起点后一拍算波形线的颜色并锁存）
//   - 波形线：1024 x 8 乒乓缓冲（1 块 BRAM9K）：帧起点换缓冲，写这一帧前 512 个样本，显示上一帧的；第 x 列读第 x*4//5 个
//   - 淡入淡出 f 0~16；透明度 a * f >> 4，混合 (bg*16 + (色 - bg)*a) >> 4（f = 0 时输出 = 输入，逐位不变）
// 流水线：raw_x/raw_y 比 rgb_in 早 2 拍（和 audio_spectrum 的输入同一组坐标）。
//   第 0 拍 按坐标发读地址（显示表 B 口、波形缓冲）、算柱 / 帽 / 倒影的命中和颜色；第 1 拍 波形命中、选色、乘淡化；
//   第 2 拍 和 rgb_in 混合并寄存 -> 输出比输入晚 1 拍（hs/vs/de 一起晚 1 拍）。
module fullscreen_viz #(
    parameter integer H_BLANK = 160,
    parameter integer V_BLANK = 45,
    parameter integer FADE_STEP = 1         // 淡入淡出每帧走几（16 = 不透明）；仿真可改大，覆盖不透明的画面
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        viz,                 // 1 = 全屏模式（本时钟域电平）
    input  wire        pcm_valid,
    input  wire signed [15:0] pcm_left,
    input  wire signed [15:0] pcm_right,
    input  wire [11:0] raw_x,               // 比 rgb_in 早 2 拍
    input  wire [11:0] raw_y,
    output wire [4:0]  fs_ra,               // audio_spectrum 显示表 B 口
    input  wire [17:0] fs_rq,               //   {保持计数, 峰值, 柱高}，读延迟 1 拍
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    output reg         hs_out,
    output reg         vs_out,
    output reg         de_out,
    output reg  [23:0] rgb_out
);
    localparam integer BAR_BASE = 440, WAVE_Y = 200, WAVE_LIM = 60, WAVE_N = 512, A_BAR = 13;
    localparam [7:0] HUE_N = 8'd192;

    // ---------------- 彩虹色：色相 0~191 ----------------
    function [23:0] rainbow;
        input [7:0] h;
        reg   [12:0] m;
        reg   [7:0]  t, q;
        begin
            m = h[4:0] * 8'd217;
            t = 8'd38 + m[12:5];
            q = 8'd255 - m[12:5];
            case (h[7:5])
                3'd0:    rainbow = {8'd255, t, 8'd38};
                3'd1:    rainbow = {q, 8'd255, 8'd38};
                3'd2:    rainbow = {8'd38, 8'd255, t};
                3'd3:    rainbow = {8'd38, q, 8'd255};
                3'd4:    rainbow = {t, 8'd38, 8'd255};
                default: rainbow = {8'd255, 8'd38, q};
            endcase
        end
    endfunction
    function [7:0] hue_add;                  // (h + n) mod 192，n < 192
        input [7:0] h;
        input [7:0] n;
        begin
            hue_add = ({1'b0, h} + {1'b0, n} >= {1'b0, HUE_N}) ? h + n - HUE_N : h + n;
        end
    endfunction

    // ---------------- 帧级：淡化、颜色滚动、波形缓冲 ----------------
    reg  [4:0]  f;                           // 0~16
    reg  [7:0]  scroll;                      // 0~191
    reg         fsq;                         // 帧起点后一拍：算波形线颜色
    reg  [23:0] wave_rgb;
    reg         wb;                          // 正在写的缓冲；显示 ~wb
    reg  [9:0]  widx;
    wire signed [16:0] msum = pcm_left + pcm_right;
    wire signed [15:0] mono = msum[16:1];
    wire signed [7:0]  m8   = mono[15:8];                                   // mono >>> 8
    wire signed [7:0]  wv   = (m8 > WAVE_LIM) ? WAVE_LIM : (m8 < -WAVE_LIM) ? -WAVE_LIM : m8;
    reg  [7:0]  wmem [0:1023];
    reg         w_we;
    reg  [9:0]  w_wa;
    reg  [7:0]  w_wd;
    integer ii;
    initial for (ii = 0; ii < 1024; ii = ii + 1) wmem[ii] = 8'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f <= 5'd0; scroll <= 8'd0; fsq <= 1'b0; wave_rgb <= 24'd0;
            wb <= 1'b0; widx <= 10'd0; w_we <= 1'b0; w_wa <= 10'd0; w_wd <= 8'd0;
        end else begin
            fsq  <= frame_start;
            w_we <= 1'b0;
            if (frame_start) begin
                f      <= viz ? ((f + FADE_STEP >= 16) ? 5'd16 : f + FADE_STEP) : ((f <= FADE_STEP) ? 5'd0 : f - FADE_STEP);
                scroll <= hue_add(scroll, 8'd3);
                wb     <= ~wb;
                widx   <= pcm_valid ? 10'd1 : 10'd0;                        // 同一拍的样本算进新一帧
                if (pcm_valid) begin w_we <= 1'b1; w_wa <= {~wb, 9'd0}; w_wd <= wv; end
            end else if (pcm_valid && widx < WAVE_N) begin
                w_we <= 1'b1; w_wa <= {wb, widx[8:0]}; w_wd <= wv;
                widx <= widx + 10'd1;
            end
            if (fsq) wave_rgb <= rainbow(hue_add(scroll, 8'd96));
        end
    end

    // ---------------- 第 0 拍：坐标 ----------------
    wire signed [12:0] lx  = $signed({1'b0, raw_x}) - H_BLANK;
    wire signed [12:0] row = $signed({1'b0, raw_y}) - V_BLANK;
    wire        act_x = (lx >= 0) && (lx < 640);
    wire        act_y = (row >= 0) && (row < 480);
    wire        lx_m1 = (lx == -13'sd1);                                  // 行首前一拍
    reg  [4:0]  p;                                                        // 20 列一段里的第几列
    reg  [4:0]  i;                                                        // 第几段 = 引擎第几根
    reg  [7:0]  hue_c;                                                    // 当前这一根的色相
    reg  [17:0] cur;                                                      // 第 i 根
    reg  [8:0]  ws;                                                       // 波形样本号 = x*4//5
    reg  [2:0]  wrem;
    assign fs_ra = lx_m1 ? 5'd1 : !act_x ? 5'd0 : (i == 5'd31) ? 5'd31 : i + 5'd1;
    wire        odd   = (p >= 5'd10);
    wire [4:0]  px    = odd ? p - 5'd10 : p;
    wire        in_bx = act_x && (px >= 5'd1) && (px <= 5'd8);
    wire [6:0]  bsum  = {1'b0, cur[5:0]}  + {1'b0, fs_rq[5:0]};
    wire [6:0]  csum  = {1'b0, cur[11:6]} + {1'b0, fs_rq[11:6]};
    wire [5:0]  bar_v = odd ? bsum[6:1] : cur[5:0];
    wire [5:0]  cap_v = odd ? csum[6:1] : cur[11:6];
    wire signed [12:0] d = BAR_BASE - 1 - row;                            // 离底边第几行（底边上一行 = 0）
    wire signed [12:0] r = row - (BAR_BASE + 1);                          // 倒影第几行
    wire [6:0]  d8    = d[9:3];
    wire        d_ok  = !d[12] && (d < 13'sd480);
    wire        b_hit = act_y && in_bx && d_ok && (d8 < {1'b0, bar_v});
    wire        c_hit = act_y && in_bx && d_ok && (cap_v != 6'd0) && (d8 == {1'b0, cap_v}) &&
                        (d[2:0] == 3'd1 || d[2:0] == 3'd2);
    wire        r_hit = act_y && in_bx && (r >= 0) && (r < 32) && ({3'd0, r[4:2]} < bar_v);
    wire [1:0]  lvl   = (d < 13'sd80) ? 2'd2 : (d < 13'sd160) ? 2'd1 : 2'd0;
    wire [23:0] rb    = rainbow(hue_c);
    reg  [7:0]  wq;
    always @(posedge clk) begin
        if (w_we) wmem[w_wa] <= w_wd;
        wq <= wmem[{~wb, ws}];
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p <= 5'd0; i <= 5'd0; hue_c <= 8'd0; cur <= 18'd0; ws <= 9'd0; wrem <= 3'd0;
        end else begin
            if (lx_m1) begin
                p <= 5'd0; i <= 5'd0; hue_c <= scroll; cur <= fs_rq; ws <= 9'd0; wrem <= 3'd0;
            end else if (act_x) begin
                if (p == 5'd19) begin p <= 5'd0; i <= i + 5'd1; cur <= fs_rq; end
                else p <= p + 5'd1;
                if (p == 5'd9 || p == 5'd19) hue_c <= hue_add(hue_c, 8'd3);
                if (wrem >= 3'd1) begin ws <= ws + 9'd1; wrem <= wrem - 3'd1; end   // rem + 4 >= 5
                else wrem <= wrem + 3'd4;
            end
        end
    end

    // ---------------- 第 1 拍：波形、选色、淡化 ----------------
    reg         s1_act, s1_first, s1_b, s1_c, s1_r;
    reg  [1:0]  s1_lvl;
    reg  [2:0]  s1_ra;
    reg  [23:0] s1_rb;
    reg  signed [12:0] s1_w;
    reg  signed [7:0]  wprev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_act <= 1'b0; s1_first <= 1'b0; s1_b <= 1'b0; s1_c <= 1'b0; s1_r <= 1'b0;
            s1_lvl <= 2'd0; s1_ra <= 3'd0; s1_rb <= 24'd0; s1_w <= 13'sd0; wprev <= 8'sd0;
        end else begin
            s1_act <= act_x && act_y; s1_first <= (lx == 13'sd0);
            s1_b <= b_hit; s1_c <= c_hit; s1_r <= r_hit; s1_lvl <= lvl;
            s1_ra <= 3'd7 - r[4:2]; s1_rb <= rb;
            s1_w <= WAVE_Y - row;
            wprev <= $signed(wq);
        end
    end
    wire signed [7:0]  va = $signed(wq);
    wire signed [7:0]  vb = s1_first ? va : wprev;
    wire signed [7:0]  vlo = (va < vb) ? va : vb;
    wire signed [7:0]  vhi = (va < vb) ? vb : va;
    wire        w_hit = s1_act && (s1_w >= vlo) && (s1_w <= vhi + 13'sd1);
    function [7:0] shd;
        input [7:0] c;
        input [1:0] l;
        shd = (l == 2'd2) ? c - {2'b00, c[7:2]} : (l == 2'd1) ? c - {3'b000, c[7:3]} : c;
    endfunction
    reg  [23:0] lay;
    reg  [4:0]  a0;
    always @* begin
        lay = 24'd0; a0 = 5'd0;
        if (w_hit)     begin lay = wave_rgb; a0 = 5'd16; end
        else if (s1_c) begin lay = {8'd240, 8'd240, 8'd255}; a0 = 5'd16; end
        else if (s1_b) begin lay = {shd(s1_rb[23:16], s1_lvl), shd(s1_rb[15:8], s1_lvl), shd(s1_rb[7:0], s1_lvl)}; a0 = A_BAR; end
        else if (s1_r) begin lay = {1'b0, s1_rb[23:17], 1'b0, s1_rb[15:9], 1'b0, s1_rb[7:1]}; a0 = {2'b00, s1_ra}; end
    end
    wire [9:0]  af = a0 * f;
    reg  [23:0] s2_lay;
    reg  [4:0]  s2_a;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_lay <= 24'd0; s2_a <= 5'd0;
        end else begin
            s2_lay <= lay; s2_a <= af[8:4];
        end
    end

    // ---------------- 第 2 拍：和 rgb_in 混合，寄存输出 ----------------
    wire signed [5:0]  as = $signed({1'b0, s2_a});
    wire signed [8:0]  er = $signed({1'b0, s2_lay[23:16]}) - $signed({1'b0, rgb_in[23:16]});
    wire signed [8:0]  eg = $signed({1'b0, s2_lay[15:8]})  - $signed({1'b0, rgb_in[15:8]});
    wire signed [8:0]  eb = $signed({1'b0, s2_lay[7:0]})   - $signed({1'b0, rgb_in[7:0]});
    wire signed [13:0] mr = $signed({2'b0, rgb_in[23:16], 4'd0}) + er * as;
    wire signed [13:0] mg = $signed({2'b0, rgb_in[15:8], 4'd0})  + eg * as;
    wire signed [13:0] mb = $signed({2'b0, rgb_in[7:0], 4'd0})   + eb * as;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_out <= 1'b1; vs_out <= 1'b1; de_out <= 1'b0; rgb_out <= 24'd0;
        end else begin
            hs_out <= hs_in; vs_out <= vs_in; de_out <= de_in;
            rgb_out <= {mr[11:4], mg[11:4], mb[11:4]};
        end
    end
endmodule
