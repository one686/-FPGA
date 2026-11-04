`timescale 1ns/1ps

// 图片转场控制器（像素时钟域）
//
// 原理：旧图和新图都已在 SDRAM 的整帧缓存块（bank 0~5）里。转场时每个像素只从"旧图"或"新图"其中一张取
// （由本模块在帧起点给出的几何参数决定，media_frame_reader 按行内分段读），所以每帧读的总量
// 仍是一张图，SDRAM 带宽不增加；像素再乘一个亮度系数 f_pix，实现淡入淡出和边缘阴影。
//
// 拨码 SW3 SW2 SW1（sw[2:0]，SW1 为最低位）选择转场，在每次转场开始时锁存：
//   0 不转场（直接切换）
//   1 淡入淡出（旧图渐暗到黑 -> 新图渐亮）
//   2 擦除   新图盖过旧图，从上往下 / 从下往上 每次轮换，边缘带柔和阴影
//   3 推移   新图从左边滑进来，把旧图往右推出屏幕，边缘带阴影
//   4 百叶窗 横向（SLAT 行一片），翻开处带阴影
//   5 中心展开 新图从屏幕中心以矩形向外展开，外沿带阴影
//   6 依次轮换以上 5 种效果
//   7 不转场（同 0）
//
// 几何参数（给 reader，帧起点更新，一帧内不变 -> 不撕裂）：
//   g_mode 0 矩形：行在 [r0,r1) 且列在 [c0,c1) 读新图（擦除/推移/淡入后半程/中心展开/静止=空矩形）
//          1 横百叶窗：行 % SLAT < v 读新图
//   new_dx/old_dx（11 位补码）：新/旧图的源列 = 屏幕列 + 偏移（推移用）；new_dy/old_dy 保留为 0
//
// 时序：进度 p 在帧起点更新，按缓动曲线 e = 256*(3t^2 - 2t^3) 映射（慢起-加速-慢停），
// TRANS_FRAMES 帧完成（默认 60 帧 = 1 秒）。转场进行中再按键：当前转场先放完，再转场到最后选中的那张（中间经过的图不逐张播放）。
// 上电第一张图（拨码为 1~6 时）：TRANS_FRAMES/2 帧从黑淡入。
//
// f_pix：提前一拍按"下一个像素"的坐标算好并寄存，显示通路直接用。
//   淡入淡出时 = 整帧亮度；其他效果 = 阴影系数（只作用在旧图像素上），不转场时恒为 256。
//
// 胶片条：画面 = {bank, 胶片条开没开}，strip_in 是要显示的胶片条状态（和 target 同一组同步）。
//   胶片条开 / 关（含 KEY1 取消浏览、换回进入前那张的情况）一律用淡出淡入（不管拨码），时长是普通转场的一半（p 每帧 +2）：
//     前半程旧画面渐暗到黑，后半程新画面从黑里渐亮（每帧只显示其中一种画面，SDRAM 带宽和平时一样）。
//   胶片条开着时换 bank（浏览）直接切换，不播转场。
//   strip_vis = 本帧显示胶片条；strip_f = 胶片条上 OSD 文字的淡化系数（= 淡入淡出的整帧亮度，不显示时 0）。
module video_transition #(
    parameter integer H_ACT        = 640,
    parameter integer V_ACT        = 480,
    parameter integer H_BLANK      = 160,
    parameter integer V_BLANK      = 45,
    parameter integer TRANS_FRAMES = 60,   // 转场帧数（60 帧 = 1 秒），要求 2 ~ 160
    parameter integer SLAT         = 40,   // 横百叶窗每片的行数
    parameter integer WIPE_SH      = 32,   // 擦除/推移/中心展开的阴影宽度（像素）
    parameter integer WIPE_K       = 3,    // 阴影最深处变暗 WIPE_SH*WIPE_K/256（默认 37%）
    parameter integer BLIND_SH     = 8,
    parameter integer BLIND_K      = 6
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire [11:0] raw_x,
    input  wire [11:0] raw_y,
    input  wire [2:0]  target,          // 要显示的 bank（已同步、去抖；0~5）
    input  wire        vis_in,          // 首图已提交（已同步、去抖）
    input  wire [2:0]  sw_async,        // 拨码开关（异步）
    input  wire        strip_in,        // 要显示的胶片条状态（已同步、去抖）
    // 帧参数（帧起点更新，一帧内不变）
    output reg         visible,
    output wire [2:0]  rd_old,          // 旧图 bank（静止时 = 当前图）
    output wire [2:0]  rd_new,          // 新图 bank
    output reg  [1:0]  g_mode,
    output reg  [9:0]  g_r0, g_r1, g_c0, g_c1, g_v,
    output reg  [10:0] g_new_dy, g_old_dy, g_new_dx, g_old_dx,
    // 像素参数（提前一拍寄存）
    output reg  [8:0]  f_pix,           // 本像素亮度系数 0~256（256 = 原样）
    // 状态
    output wire        trans_active,
    output wire [2:0]  trans_kind,
    output wire [1:0]  trans_var,
    output wire        strip_vis,       // 本帧显示胶片条
    output wire [8:0]  strip_f
);
    localparam [1:0] ST_IDLE = 2'd0, ST_TRANS = 2'd1, ST_INTRO = 2'd2;
    localparam [2:0] K_FADE = 3'd1, K_WIPE = 3'd2, K_PUSH = 3'd3, K_BLIND = 3'd4, K_CENTER = 3'd5;
    localparam [1:0] G_RECT = 2'd0, G_HBLIND = 2'd1;
    localparam integer HALF_W = H_ACT / 2, HALF_H = V_ACT / 2;

    // ---------------- 缓动曲线表（综合时按参数算成常数） ----------------
    function [8:0] ease_c;
        input integer p;
        input integer n;
        integer num, den;
        begin
            num = p * p * (3 * n - 2 * p);
            den = n * n * n;
            ease_c = (256 * num + den / 2) / den;
        end
    endfunction
    wire [8:0] ease_tab [0:TRANS_FRAMES];
    genvar gi;
    generate
        for (gi = 0; gi <= TRANS_FRAMES; gi = gi + 1) begin : g_ease
            assign ease_tab[gi] = ease_c(gi, TRANS_FRAMES);
        end
    endgenerate

    // ---------------- 拨码同步 ----------------
    reg [2:0] sw_m, sw_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sw_m <= 3'd0; sw_s <= 3'd0; end
        else begin sw_m <= sw_async; sw_s <= sw_m; end
    end
    wire no_trans = (sw_s == 3'd0) || (sw_s == 3'd7);

    // ---------------- 帧级状态 ----------------
    reg [1:0] state;
    reg [7:0] p;
    reg [2:0] cur, old_b, new_b;
    reg [2:0] kind;
    reg [1:0] var_r;            // 擦除：1 从上往下  3 从下往上；其他效果恒 0
    reg       wipe_up;          // 下一次擦除的方向：0 从上往下，1 从下往上（每次擦除后翻转）
    reg [2:0] seq_idx;          // 轮换模式：0..4 -> 效果 1..5
    reg       cur_strip, old_strip, new_strip;   // 当前 / 转场前 / 转场后的胶片条状态
    reg       sfade;            // 这次转场是胶片条开关（淡出淡入，p 每帧 +2）

    wire [2:0] pick_kind = (sw_s == 3'd6) ? seq_idx + 3'd1 : sw_s;
    wire [1:0] pick_var  = (pick_kind == K_WIPE) ? {wipe_up, 1'b1} : 2'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; p <= 8'd0; cur <= 3'd0; old_b <= 3'd0; new_b <= 3'd0;
            kind <= K_FADE; var_r <= 2'd0; wipe_up <= 1'b0; seq_idx <= 3'd0; visible <= 1'b0;
            cur_strip <= 1'b0; old_strip <= 1'b0; new_strip <= 1'b0; sfade <= 1'b0;
        end else if (frame_start) begin
            visible <= vis_in;
            if (!vis_in) begin
                state <= ST_IDLE;
                cur <= target; cur_strip <= strip_in;
            end else if (!visible) begin                    // 首图出现
                cur <= target; cur_strip <= strip_in;
                if (!no_trans) begin state <= ST_INTRO; p <= 8'd1; end
                else state <= ST_IDLE;
            end else begin
                case (state)
                    ST_IDLE: if (strip_in != cur_strip) begin   // 胶片条开 / 关 -> 淡出淡入
                        state <= ST_TRANS; p <= 8'd2; sfade <= 1'b1;
                        old_b <= cur; new_b <= target; old_strip <= cur_strip; new_strip <= strip_in;
                        kind <= K_FADE; var_r <= 2'd0;
                    end else if (target != cur) begin
                        if (no_trans || cur_strip) begin
                            cur <= target;                  // 直接切换（胶片条开着时浏览也直接切换）
                        end else begin
                            sfade <= 1'b0; old_strip <= cur_strip; new_strip <= cur_strip;
                            state <= ST_TRANS; p <= 8'd1;
                            old_b <= cur; new_b <= target;
                            kind <= pick_kind; var_r <= pick_var;
                            if (pick_kind == K_WIPE) wipe_up <= ~wipe_up;
                            if (sw_s == 3'd6) seq_idx <= (seq_idx == 3'd4) ? 3'd0 : seq_idx + 3'd1;
                        end
                    end
                    ST_TRANS: if (p >= TRANS_FRAMES) begin
                        state <= ST_IDLE; cur <= new_b; cur_strip <= new_strip;
                    end else
                        p <= p + (sfade ? 8'd2 : 8'd1);
                    default: if ({p, 1'b0} >= TRANS_FRAMES) state <= ST_IDLE;   // ST_INTRO
                             else p <= p + 8'd1;
                endcase
            end
        end
    end

    // ---------------- 帧参数（组合，基于帧起点更新过的寄存器，一帧内稳定） ----------------
    // e × 常数：用移位相加实现（常数的每个 1 位加一次），不占 DSP
    function [19:0] cmul;
        input [8:0] a;
        input integer k;
        integer i;
        begin
            cmul = 20'd0;
            for (i = 0; i < 11; i = i + 1)
                if ((k >> i) & 1) cmul = cmul + ({11'd0, a} << i);
        end
    endfunction
    wire [8:0]  e   = ease_tab[(p > TRANS_FRAMES) ? TRANS_FRAMES : p];
    wire [8:0]  e2  = ease_tab[({p, 1'b0} > TRANS_FRAMES) ? TRANS_FRAMES : {p, 1'b0}];
    wire [19:0] m_bh = cmul(e, V_ACT);
    wire [19:0] m_bw = cmul(e, H_ACT);
    wire [19:0] m_bs = cmul(e, SLAT);
    wire [19:0] m_hh = cmul(e, HALF_H);
    wire [19:0] m_hw = cmul(e, HALF_W);
    wire [9:0]  bh  = m_bh[17:8];                   // 上下擦除的边界行（0~V_ACT）
    wire [9:0]  bwx = m_bw[17:8];                   // 推移的边界列（0~H_ACT）
    wire [9:0]  bs  = m_bs[17:8];                   // 百叶窗每片已翻开的行数（0~SLAT）
    wire [9:0]  hh  = m_hh[17:8];                   // 中心矩形半高 / 半宽
    wire [9:0]  hw  = m_hw[17:8];
    wire        trans = (state == ST_TRANS);
    reg  [8:0]  alpha_f;                            // 整帧亮度（淡入淡出）
    wire [9:0]  e_x2   = {e, 1'b0};
    wire [9:0]  fa_out = 10'd256 - e_x2;            // e<128  时 256..2
    wire [9:0]  fa_in  = e_x2 - 10'd256;            // e>=128 时 0..256
    wire [10:0] bwx11 = {1'b0, bwx};

    assign rd_old = trans ? old_b : cur;
    assign rd_new = new_b;
    assign trans_active = (state != ST_IDLE);
    assign trans_kind = trans ? kind : 3'd0;
    assign trans_var  = var_r;
    // 胶片条（淡出淡入前半程是旧画面，后半程是新画面）
    assign strip_vis  = trans ? ((kind == K_FADE && e >= 9'd128) ? new_strip : old_strip) : cur_strip;

    always @* begin
        g_mode = G_RECT; g_r0 = 10'd0; g_r1 = 10'd0; g_c0 = 10'd0; g_c1 = 10'd0; g_v = 10'd0;
        g_new_dy = 11'd0; g_old_dy = 11'd0; g_new_dx = 11'd0; g_old_dx = 11'd0;
        alpha_f = 9'd256;
        if (state == ST_INTRO)
            alpha_f = e2;
        else if (trans) begin
            case (kind)
                K_FADE: begin                             // 前半程旧图 256->0，后半程新图 0->256
                    if (e < 9'd128) alpha_f = fa_out[8:0];
                    else begin g_r1 = V_ACT; g_c1 = H_ACT; alpha_f = fa_in[8:0]; end
                end
                K_WIPE: begin
                    g_c1 = H_ACT;
                    if (var_r[1]) begin g_r0 = V_ACT - bh; g_r1 = V_ACT; end      // 从下往上
                    else          begin g_r1 = bh; end                              // 从上往下
                end
                K_PUSH: begin                             // 从左往右推：新图右半先露出，旧图右移
                    g_r1 = V_ACT; g_c1 = bwx;
                    g_new_dx = H_ACT - bwx11; g_old_dx = 11'd0 - bwx11;
                end
                K_BLIND: begin g_mode = G_HBLIND; g_v = bs; end
                default: begin                            // K_CENTER
                    g_r0 = HALF_H - hh; g_r1 = HALF_H + hh;
                    g_c0 = HALF_W - hw; g_c1 = HALF_W + hw;
                end
            endcase
        end
    end

    // ---------------- 像素参数：下一个像素的坐标 ----------------
    wire [11:0] xn   = raw_x + 12'd1;
    wire [11:0] cn12 = xn - H_BLANK;
    wire [11:0] yy12 = raw_y - V_BLANK;
    wire [10:0] c    = {1'b0, cn12[9:0]};            // 列（只在有效区内有意义）
    wire [10:0] y    = {1'b0, yy12[9:0]};            // 行
    reg  [9:0]  ymod;                                // 行 % SLAT（每行开头更新）
    wire [9:0]  ymod_now = (raw_y == V_BLANK) ? 10'd0 : (ymod == SLAT - 1) ? 10'd0 : ymod + 10'd1;

    // 阴影：新图边缘外 WIPE_SH 像素内的旧图逐渐变暗（像新图投下的影子），系数 256 - K * (SH - 距离)
    // 阴影曲线做成常数表（综合时算好），避免 K*(SH-dd) 被 TD 推成 3 个 DSP。
    //   表的下标取距离的低 log2(SH) 位，所以 WIPE_SH / BLIND_SH 必须是 2 的幂（默认 32 / 8）
    localparam integer WSW = $clog2(WIPE_SH), BSW = $clog2(BLIND_SH);
`ifdef OSD_SIM
    initial begin
        if ((1 << WSW) != WIPE_SH || (1 << BSW) != BLIND_SH) begin
            $display("[ASSERT] %m: WIPE_SH / BLIND_SH 必须是 2 的幂");
            $finish;
        end
    end
`endif
    wire [8:0] wsh_t [0:WIPE_SH-1];
    wire [8:0] bsh_t [0:BLIND_SH-1];
    generate
        for (gi = 0; gi < WIPE_SH; gi = gi + 1) begin : g_wsh
            assign wsh_t[gi] = 256 - WIPE_K * (WIPE_SH - gi);
        end
        for (gi = 0; gi < BLIND_SH; gi = gi + 1) begin : g_bsh
            assign bsh_t[gi] = 256 - BLIND_K * (BLIND_SH - gi);
        end
    endgenerate

    // 擦除、推移、中心展开共用一套"到新图矩形 [g_r0,g_r1) x [g_c0,g_c1) 的切比雪夫距离"：
    //   擦除只看行（从上往下：矩形 [0,bh) 行，距离 = y - bh；从下往上：[480-bh,480)，距离 = 480-1-bh-y），
    //   推移只看列（[0,bwx) 列，距离 = c - bwx），中心展开行列都看。
    //   y >= 480 只出现在场消隐（y = 979~1023），那时距离 >= 499，系数仍是 256。
    // 两个距离都 < WIPE_SH 时才查表（只比低 WSW 位取较大值），否则较大的那个 >= WIPE_SH，系数 256
    wire sh_en  = trans && (((kind == K_WIPE) && bh != 0 && bh != V_ACT) ||
                            ((kind == K_PUSH) && bwx != 0 && bwx != H_ACT) ||
                            ((kind == K_CENTER) && hh != 0 && hw != 0));
    wire row_en = (kind != K_PUSH);
    wire col_en = (kind != K_WIPE);
    reg  [8:0]  shade;
    reg  [10:0] dr, dc;
    reg         out_r, out_c;
    always @* begin
        shade = 9'd256; dr = 11'd0; dc = 11'd0; out_r = 1'b0; out_c = 1'b0;
        if (trans && kind == K_BLIND) begin
            if (bs != 0 && bs != SLAT && ymod >= bs && ymod - bs < BLIND_SH) shade = bsh_t[ymod[BSW-1:0] - bs[BSW-1:0]];
        end else if (sh_en) begin
            out_r = row_en && ((y < g_r0) || (y >= g_r1));
            out_c = col_en && ((c < g_c0) || (c >= g_c1));
            dr = !out_r ? 11'd0 : (y < g_r0) ? g_r0 - 11'd1 - y : y - g_r1;
            dc = !out_c ? 11'd0 : (c < g_c0) ? g_c0 - 11'd1 - c : c - g_c1;
            if ((out_r || out_c) && dr < WIPE_SH && dc < WIPE_SH)
                shade = wsh_t[(dr[WSW-1:0] > dc[WSW-1:0]) ? dr[WSW-1:0] : dc[WSW-1:0]];
        end
    end
    wire [8:0] f_next = (state == ST_INTRO || (trans && kind == K_FADE)) ? alpha_f : shade;
    assign strip_f = !strip_vis ? 9'd0 : (trans && kind == K_FADE) ? alpha_f : 9'd256;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ymod <= 10'd0; f_pix <= 9'd256;
        end else begin
            f_pix <= f_next;
            if (raw_x == 12'd0) ymod <= ymod_now;
        end
    end
endmodule
