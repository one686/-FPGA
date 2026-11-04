`timescale 1ns/1ps

// OSD 引擎（像素时钟域）：状态条、滚动字幕、画面调节菜单、提示行、胶片条文字、定时关机提示，
//   抗锯齿字库 + 精确乘法 Alpha 混合，叠加在背景视频上。
//
// 坐标：raw_x/raw_y 来自 video_timing_ctrl（ACTIVE_LAST=1，有效区 x∈[H_BLANK,H_BLANK+640)，
//       y∈[V_BLANK,V_BLANK+480)）。引擎比当前像素提前 LEAD=5 个像素计算，流水如下：
//   周期 t   ：S0 组合逻辑——判断区域、字符格、底板位置、字形号/行/列、颜色/透明度/淡化系数
//   周期 t+1 ：S0B 寄存器——寄存 S0 的结果（S0 区域判断较长，单独切一级保证时序）
//   周期 t+2 ：S0R 寄存器——算出字库地址
//   周期 t+3 ：S1 取字库像素 g/o，乘出 文字色×g、底板×(1-o)、底板透明度×(1-o)
//   周期 t+4 ：S2 合成 C/A，乘淡入淡出系数 f
//   周期 t+5 ：S3 寄存结果，与当前背景像素做最后一次混合（组合逻辑，下一拍进 rgb_video_adapter）
// 没有 OSD 的像素 C=0、A=0，out = bg 逐位不变。混合公式见 design/osd_gen.py 文件头。
//
// 显示行（括号内为 b_sel 编号）：
//   左侧状态行 A(0) 常显；C(2) 读卡错误、D(3) 坏图数、E(4) 音乐错误，有内容时淡入，按出现顺序自动排槽（band_s）。
//   右侧行只占 x >= SPLIT_X：M(5) 音乐信息 / P(6) "♪ 琶音"（二选一，固定在 A 行下面）；T(7) 提示行（y=T_Y）；
//     B(1) "关机 mm:ss"（固定在左侧第 3 个槽的 y，不参加排槽；淡化 = f_band[1] x f_stat，受 KEY3 显示模式影响）。
//     右侧行和左侧行同一高度时按 x 分开（左侧行的底板不会超过 SPLIT_X），扫到 SPLIT_X 前一拍字符格指针清零。
//   提示行 T：SW4 层 / 暂停 / 循环变化或恢复默认时显示"图片模式/音乐模式/暂停播放/继续播放/单曲循环/列表循环/恢复默认"
//     T_FRAMES 帧再淡出；不受 KEY3 显示模式影响（OSD 关着时拨 SW4 也能看到切到了哪一层）。
//     M/P 行开头的 ♪ 暂停时换成 ⏸，单曲循环时 M 行尾显示"单"。
//   画面调节菜单（8~13）：左侧 y=MY0~MY1 圆角半透明面板，6 行"亮度 / 对比度 / 饱和度 / 色温 / 轮播 / 定时"，
//     当前行有高亮条 + 金色字；前 4 行有 11 格进度条（档位 0~10 亮 1~11 格），"定时"行 5 格（位置查 sl_seg）。
//     不受 KEY3 显示模式影响，开关时淡入淡出（MENU_STEP）。区域颜色是 osd_gen.py 预先合成好的常数（menu_cp/menu_ap）。
//     状态条"自动 N秒"随轮播周期变化（st_per_s）；随音乐档（st_per_s = 0）显示"自动 ♪秒"。
//   胶片条文字（14 / 15）：14 = "n/N"（和状态条同样的字段），15 = 每格中间"加载中 / 坏图片"（st_sst 给每格状态；
//     标签格的源码 = LBL_BASE(192) + 格号*4 + 第几个字）。只写字，底板、缩略图、金框由 filmstrip 画。
//     这两行在 row_rom 的高 7 位（单独记行内第几行），b_hit 最优先；淡化系数 st_sf 跟着胶片条淡出淡入。
//     st_hide = 1 时字幕淡出。
//   定时关机最后 10 秒提示框：x [WX0, WX1)、y [WY0, WY1) 圆角底板 + 10 格进度条（亮 w_sec 格）+ 两行字；两行字借用菜单
//     第 2、3 行的 y 和 b_sel（9、10），字符格表用"第二套"（ckey 第 5 位 = 1）。菜单开着时不画（两者不会同时出现），
//     不受 KEY3 显示模式影响。定时关机状态由 media_display_path 的 sleep_ctrl 给，帧起点锁存。
//   st_sleep = 1（关机淡出 / 关着 / 唤醒等待）：状态条、字幕、提示行的淡化目标都变成 0。
// 字形编号 7 位（127 = 空白），字段编码 128 起（FIELD_BASE）。
// 帧起点（frame_start，场同步开始，距有效区 35 行）依次：锁存状态 -> 更新淡入淡出/滚动/字段字形
// -> 排列状态行位置、算有效淡化系数、字幕起始格。一帧之内这些量都不变，所以不会撕裂。
module osd_engine #(
    parameter integer H_BLANK    = 160,
    parameter integer V_BLANK    = 45,
    parameter integer SUB_X_INIT = 640,    // 字幕起始 x（仿真时可改，上板用默认）
    parameter integer SUB_SPEED_P = 1,     // 每帧滚动像素数
    parameter integer TOAST_FRAMES_P = 0   // 提示行显示帧数（0 = 用 osd_layout.vh 的 TOAST_FRAMES；仿真可改小）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [11:0] raw_x,
    input  wire [11:0] raw_y,
    input  wire        frame_start,
    // 状态（已同步到本时钟域，帧起点锁存）
    input  wire [6:0]  st_idx,       // 当前第几张（1~99，0=无）
    input  wire [6:0]  st_live,      // 总张数（好图）
    input  wire        st_auto,
    input  wire        st_loading,
    input  wire [3:0]  st_err,       // 读卡错误码（0=无）
    input  wire [6:0]  st_bad,       // 坏图数
    input  wire [7:0]  st_vol,       // 音量 0~255
    input  wire [23:0] st_time,      // 运行时间 BCD：h1 h0 m1 m0 s1 s0
    // 音乐
    input  wire        st_wav,       // 1 = 正在用 WAV（显示 M 行），0 = 琶音（显示 P 行）
    input  wire [6:0]  st_midx,      // 第几首（1 起）
    input  wire [6:0]  st_mcnt,      // 共几首
    input  wire        st_mknown,    // 总时长已算出（否则时间显示 --:--）
    input  wire [15:0] st_mtot,      // 总时长 BCD m1 m0 s1 s0
    input  wire [2:0]  st_merr,      // 音乐错误码（0 = 无）
    input  wire [15:0] st_mel,       // 已播放 BCD m1 m0 s1 s0
    // 按键层 / 播放状态
    input  wire        st_layer,     // SW4：0 图片层 1 音乐层
    input  wire        st_pause,     // 暂停
    input  wire        st_loop,      // 单曲循环
    // 画面调节菜单
    input  wire [6:0]  st_per_s,     // 轮播周期（秒），显示在状态条上；0 = 随音乐（显示 ♪）
    input  wire        st_menu,      // 菜单打开
    input  wire [2:0]  st_msel,      // 当前行 0~5
    input  wire [15:0] st_mlv,       // 4 个档位 {色温, 饱和度, 对比度, 亮度}，各 0~10（5 = 不调）
    input  wire        st_rst,       // 每恢复默认一次翻转一次（提示"恢复默认"）
    // 胶片条（帧起点更新，一帧内不变）
    input  wire        st_strip,     // 本帧显示胶片条
    input  wire [8:0]  st_sf,        // 胶片条文字的淡化系数
    input  wire [9:0]  st_sst,       // 5 格状态（[1:0] = 第 0 格）：0 无 1 缩略图 2 加载中 3 坏图
    input  wire        st_hide,      // 胶片条开着或还看得见：字幕淡出
    // 定时关机
    input  wire [2:0]  st_slv,       // 菜单第 6 行的档位 0~5（关 1 15 30 60 90 分）
    input  wire [15:0] st_srem,      // 剩余时间 BCD m1 m0 s1 s0
    input  wire        st_son,       // 显示右侧"关机 mm:ss"
    input  wire        st_warn,      // 显示最后 10 秒提示框
    input  wire        st_sleep,     // 关机淡出 / 关着：状态条、字幕、提示行淡出
    input  wire        mode_next,    // KEY3 单拍脉冲：切换显示模式
    output reg  [1:0]  mode,         // 0 全部  1 只字幕  2 只状态  3 全关
    // 字幕条 RAM 写口（SD 时钟域，sd_card_bmp 的 sub_loader 给）和结果
    //   sub_ok 在本模块里同步 2 拍、帧起点锁存；sub_w / sub_top 只在 sub_ok = 1 时用（那时已经不变）
    input  wire        sub_wclk,
    input  wire        sub_we,
    input  wire [13:0] sub_waddr,
    input  wire [7:0]  sub_wdata,
    input  wire        sub_ok_async,
    input  wire [9:0]  sub_w,
    input  wire [13:0] sub_top,
    input  wire [23:0] bg_rgb,       // 当前像素（与 raw_x 同拍）
    output wire [23:0] out_rgb
);
    `include "osd_layout.vh"

    localparam integer LEAD = 5;
    localparam [7:0]   T_FRAMES = (TOAST_FRAMES_P != 0) ? TOAST_FRAMES_P : TOAST_FRAMES;
    localparam [6:0]   G_BLANK = 7'd127;

    // ======================== 帧级状态 ========================
    reg [6:0]  s_idx;  reg [6:0] s_live; reg s_auto; reg s_loading;
    reg [3:0]  s_err;  reg [6:0] s_bad;  reg [7:0] s_vol; reg [23:0] s_time;
    reg [6:0]  s_per;  reg s_menu; reg [2:0] s_msel; reg [15:0] s_mlv;   // 菜单
    reg [2:0]  s_slv;  reg [15:0] s_rem; reg s_son, s_warn, s_sleep;        // 定时关机
    reg [8:0]  f_w;                          // 提示框淡化
    reg        wvis;
    reg [3:0]  w_sec;                        // 提示框亮几格（= 剩余秒数 1~10，不显示时冻结）
    // 音乐状态在帧起点直接用输入（上游已同步，且只在这里取一次），不另存一份，省寄存器
    wire       s_wav = st_wav;  wire [6:0] s_midx = st_midx, s_mcnt = st_mcnt; wire s_mknown = st_mknown;
    wire [15:0] s_mtot = st_mtot, s_mel = st_mel; wire [2:0] s_merr = st_merr;
    reg [2:0]  fs_q;                         // 帧起点后的第 1/2 拍
    reg [8:0]  f_stat, f_sub;
    reg [8:0]  f_band [1:4];                 // B C D E
    reg [8:0]  f_eff  [0:7];                 // A B C D E M P T 的有效淡化系数
    reg [8:0]  f_sub_eff;
    reg [2:0]  band_s [0:4];                 // 左侧行 A~E 排在第几个槽（槽 k 的 y = 8 + k*SLOT_PITCH；M/P 固定在 R_Y）
    reg [7:0]  band_occ;
    // 提示行
    reg        q_layer, q_pause, q_loop, q_rst;   // 上一帧的值
    reg [7:0]  t_cnt;                        // 提示还要显示几帧
    reg [8:0]  f_t;
    // 菜单
    reg [8:0]  f_menu;
    reg        mvis;
    reg signed [11:0] xs;                    // 字幕起始 x
    reg [9:0]  sub_col0;                     // xs<=0 时第 0 列是字幕条的第几列（= -xs）
    // 字幕条（卡里的 SUB.BMP，没有就用默认标语"热烈欢迎"，宽 SUB_DEF_W）
    localparam [9:0]  SUB_DEF_W   = 10'd144;
    localparam [13:0] SUB_DEF_TOP = 14'd2808;   // 39 x 72：画面第 0 行 = 文件最后一行
    reg  [1:0] subok_sy;
    reg        s_subok;                      // 本帧用卡里的字幕
    wire [9:0]  sub_wf  = s_subok ? sub_w   : SUB_DEF_W;
    wire [13:0] sub_tpf = s_subok ? sub_top : SUB_DEF_TOP;
    wire [8:0]  sub_wb  = sub_wf[9:1];       // 每行字节数
    // 字段表：64x7 分布式 RAM（异步读），比 47 个并行寄存器 + 47 选 1 省 LUT。
    //   帧起点后第 2 拍起每拍写一个字段（fw_k = 0~46，约 47 拍，都在场消隐里）。字段的值只取帧级量：
    //   s_*（frame_start 锁存）和 w_* / q_pause / q_loop（第 1 拍 fs_q[0] 锁存），所以一帧之内内容一致。
    //   RAM 不能复位：复位后第一次帧更新（fw_init = 1）把"有条件才写"的字段也写一遍（条件不成立时写空白），
    //   等效于复位清成空白。在这之前所有 OSD 的淡化系数都是 0，字段内容不影响画面。
    reg [6:0]  fld [0:63];                   // 字段 -> 字形号（下标 = 字段编码 - FIELD_BASE）
    // synthesis translate_off
    integer fi;
    initial for (fi = 0; fi < 64; fi = fi + 1) fld[fi] = 7'd0;     // 仿真：和上板一样（无初值的分布式 RAM 上电为 0），不用 X
    // synthesis translate_on
    reg        fw_run, fw_init;
    reg [5:0]  fw_k;
    reg [6:0]  w_midx, w_mcnt;               // 第 1 拍锁存的音乐状态
    reg        w_mknown;
    reg [15:0] w_mel, w_mtot;
    reg [2:0]  w_merr;
    reg        w_tev;                        // 第 1 拍有没有提示事件、是哪一种
    reg [2:0]  w_tkind;
    integer i;

    wire stat_on = ((mode == 2'd0) || (mode == 2'd2)) && !s_sleep;                // 关机时淡出
    wire sub_on  = ((mode == 2'd0) || (mode == 2'd1)) && !st_hide && !s_sleep;   // 胶片条显示时字幕淡出
    // 各行淡入目标：A 常显，B = 右侧"关机 mm:ss"，C/D/E = 读卡错误 / 坏图 / 音乐错误
    wire [4:0] tgt = {s_merr != 0, s_bad != 0, s_err != 0, s_son, 1'b1};   // E D C B A

    function [8:0] step;
        input [8:0] v;
        input       up;
        input [8:0] st;
        begin
            if (up) step = (v + st > 9'd256) ? 9'd256 : v + st;
            else    step = (v < st) ? 9'd0 : v - st;
        end
    endfunction

    // 数字 d（0~9）的字形号。osd_gen.py 保证 G_DIGIT0 是 16 的倍数，所以直接拼位，不用加法器
    function [6:0] dg;
        input [3:0] d;
        dg = {G_DIGIT0[6:4], d};
    endfunction
    wire [14:0] pct_mul = s_vol * 7'd101;
    wire [6:0]  pct = pct_mul[14:8];
    wire [11:0] neg_xs = -xs;
    // 两位数拆十位/个位（v < 100）：查表 dec_tg / dec_og（osd_layout.vh）直接给出十位 / 个位数字的字形号
    //   （= G_DIGIT0 + 数字），省掉乘常数和减法。字段串行写，6 个两位数（第几张 / 共几张 / 轮播秒数 / 坏图数 /
    //   第几首 / 共几首）共用一份查表，输入按正在写的字段 fw_k 选
    reg  [6:0] w2_v;
    always @* begin
        case (fw_k)
            F_IDX1-FIELD_BASE, F_IDX0-FIELD_BASE: w2_v = s_idx;
            F_TOT1-FIELD_BASE, F_TOT0-FIELD_BASE: w2_v = s_live;
            F_PT-FIELD_BASE,   F_PO-FIELD_BASE:   w2_v = s_per;        // 轮播周期
            F_BAD1-FIELD_BASE, F_BAD0-FIELD_BASE: w2_v = s_bad;
            F_SI1-FIELD_BASE,  F_SI0-FIELD_BASE:  w2_v = w_midx;
            default:                              w2_v = w_mcnt;       // SC1 / SC0
        endcase
    end
    wire [6:0] w2_tg = dec_tg(w2_v), w2_og = dec_og(w2_v);
    // 提示：哪一项变了（同时变按 SW4 > 暂停 > 循环 > 恢复默认），
    //   提示种类 0~6 = 图片模式 音乐模式 暂停播放 继续播放 单曲循环 列表循环 恢复默认
    wire       t_ev_l = (st_layer != q_layer), t_ev_p = (st_pause != q_pause), t_ev_o = (st_loop != q_loop);
    wire       t_ev_r = (st_rst != q_rst);
    wire       t_ev = t_ev_l || t_ev_p || t_ev_o || t_ev_r;
    wire [2:0] t_kind = t_ev_l ? {2'b00, st_layer} : t_ev_p ? (st_pause ? 3'd2 : 3'd3) :
                        t_ev_o ? (st_loop ? 3'd4 : 3'd5) : 3'd6;
    // 菜单数值：档位 0~10 -> 符号（+ / - / 空）和一位数字
    function [6:0] lv_sign;
        input [3:0] lv;
        begin
            lv_sign = (lv > 4'd5) ? G_PLUS : (lv < 4'd5) ? G_DASH : G_BLANK;
        end
    endfunction
    function [6:0] lv_digit;
        input [3:0] lv;
        begin
            lv_digit = dg((lv >= 4'd5) ? lv - 4'd5 : 4'd5 - lv);
        end
    endfunction


    reg [2:0] s_acc;
    // 乘积必须先放进足够宽的线再移位：写成 (f_band[i]*f_stat)>>8 赋给 9 位寄存器时，
    // Verilog 按左边 9 位宽度算乘法，高位先被截掉（B/C/D 行会变成完全透明）
    wire [17:0] f_band_mul [1:4];
    assign f_band_mul[1] = f_band[1] * f_stat;
    assign f_band_mul[2] = f_band[2] * f_stat;
    assign f_band_mul[3] = f_band[3] * f_stat;
    assign f_band_mul[4] = f_band[4] * f_stat;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode <= 2'd0;
            fs_q <= 3'd0;
            s_idx <= 0; s_live <= 0; s_auto <= 0; s_loading <= 0;
            s_err <= 0; s_bad <= 0; s_vol <= 0; s_time <= 0;
            f_stat <= 0; f_sub <= 0; f_sub_eff <= 0;
            for (i = 1; i <= 4; i = i + 1) f_band[i] <= 0;
            for (i = 0; i < 8; i = i + 1) f_eff[i] <= 0;
            for (i = 0; i < 5; i = i + 1) band_s[i] <= 3'd0;
            band_occ <= 8'b00000001;
            q_layer <= 1'b0; q_pause <= 1'b0; q_loop <= 1'b0; q_rst <= 1'b0; t_cnt <= 8'd0; f_t <= 9'd0;
            s_per <= 7'd3; s_menu <= 1'b0; s_msel <= 3'd0; s_mlv <= 16'h5555; f_menu <= 9'd0; mvis <= 1'b0;
            s_slv <= 3'd0; s_rem <= 16'd0; s_son <= 1'b0; s_warn <= 1'b0; s_sleep <= 1'b0;
            f_w <= 9'd0; wvis <= 1'b0; w_sec <= 4'd0;
            xs <= SUB_X_INIT;
            sub_col0 <= 0;
            subok_sy <= 2'b00; s_subok <= 1'b0;
            fw_run <= 1'b0; fw_init <= 1'b1; fw_k <= 6'd0;
            w_midx <= 0; w_mcnt <= 0; w_mknown <= 0; w_mel <= 0; w_mtot <= 0; w_merr <= 0;
            w_tev <= 1'b0; w_tkind <= 3'd0;
        end else begin
            if (mode_next)
                mode <= mode + 2'd1;
            fs_q <= {fs_q[1:0], frame_start};
            subok_sy <= {subok_sy[0], sub_ok_async};

            if (frame_start) begin                       // 第 0 拍：锁存状态
                s_idx <= st_idx; s_live <= st_live; s_auto <= st_auto; s_loading <= st_loading;
                s_err <= st_err; s_bad <= st_bad; s_vol <= st_vol; s_time <= st_time;
                s_per <= st_per_s; s_menu <= st_menu; s_msel <= st_msel; s_mlv <= st_mlv;
                s_slv <= st_slv; s_rem <= st_srem; s_son <= st_son; s_warn <= st_warn; s_sleep <= st_sleep;
                s_subok <= subok_sy[1];
            end
            if (fs_q[0]) begin                           // 第 1 拍：淡入淡出、滚动、字段字形
                f_stat <= step(f_stat, stat_on, GROUP_STEP);
                f_sub  <= step(f_sub, sub_on, GROUP_STEP);
                for (i = 1; i <= 4; i = i + 1)
                    f_band[i] <= step(f_band[i], tgt[i], BAND_STEP);
                if (xs - SUB_SPEED_P <= -$signed({2'b00, sub_wf})) xs <= 12'sd640;
                else                                 xs <= xs - SUB_SPEED_P;
                // 字段从下一拍起串行写（见 fw_k），这里先锁存这一拍的输入
                w_midx <= st_midx; w_mcnt <= st_mcnt; w_mknown <= st_mknown;
                w_mel <= st_mel; w_mtot <= st_mtot; w_merr <= st_merr;
                w_tev <= t_ev; w_tkind <= t_kind;
                fw_run <= 1'b1; fw_k <= 6'd0;
                q_layer <= st_layer; q_pause <= st_pause; q_loop <= st_loop; q_rst <= st_rst;
                // 菜单
                f_menu <= step(f_menu, s_menu, MENU_STEP);
                f_t <= step(f_t, (t_cnt != 8'd0) && !s_sleep, BAND_STEP);
                f_w <= step(f_w, s_warn, BAND_STEP);                    // 关机提示框
                if (s_warn) w_sec <= s_rem[4] ? 4'd10 : s_rem[3:0];       //   剩余 00:01~00:10 秒
                if (t_ev)
                    t_cnt <= T_FRAMES;
                else if (t_cnt != 8'd0)
                    t_cnt <= t_cnt - 8'd1;
            end
            if (fw_run) begin                            // 第 2 拍起每拍写一个字段
                fw_k <= fw_k + 6'd1;
                if (fw_k == N_FIELDS - 1) begin
                    fw_run <= 1'b0;
                    fw_init <= 1'b0;
                end
            end
            if (fs_q[1]) begin                           // 第 2 拍：排列状态行、有效系数、字幕起始格
                // 只记槽号（3 位），不存每行的 10 位 y（省 LUT）
                s_acc = 3'd1;
                band_s[0] <= 3'd0;
                band_occ[0] <= 1'b1;
                band_occ[1] <= tgt[1] || (f_band[1] != 0);   // B 在右侧固定位置，不排槽
                for (i = 2; i <= 4; i = i + 1) begin
                    if (tgt[i] || f_band[i] != 0) begin
                        band_s[i] <= s_acc;
                        band_occ[i] <= 1'b1;
                        s_acc = s_acc + 3'd1;
                    end else
                        band_occ[i] <= 1'b0;
                end
                band_occ[5] <= s_wav;
                band_occ[6] <= !s_wav;
                band_occ[7] <= (t_cnt != 8'd0) || (f_t != 9'd0);
                mvis <= s_menu || (f_menu != 9'd0);
                wvis <= s_warn || (f_w != 9'd0);
                f_eff[0] <= f_stat;
                f_eff[5] <= f_stat;
                f_eff[6] <= f_stat;
                f_eff[7] <= f_t;
                for (i = 1; i <= 4; i = i + 1)
                    f_eff[i] <= f_band_mul[i] >> 8;
                f_sub_eff <= f_sub;
                sub_col0 <= neg_xs[9:0];
            end
        end
    end

    // 字段表写口：fw_k 号字段的字形号 fw_d；fw_hold = 这个字段这一帧保持不更新
    //   分两级（省 LUT）：大多数字段是"数字"，先按 fw_k 选出 4 位数字 fw_n（数字字形号 = {G_DIGIT0 高 3 位, 数字}，
    //   dec_tg / dec_og 的低 4 位就是数字），再按 fw_c（数字 / 空白 / 横杠 / 其他字）拼出字形号；"其他字"另用一个小选择。
    localparam [1:0] FC_DIG = 2'd0, FC_BLK = 2'd1, FC_DSH = 2'd2, FC_OTH = 2'd3;
    reg  [6:0] fw_d;
    reg        fw_hold;
    reg  [3:0] fw_n;
    reg  [1:0] fw_c;
    reg  [6:0] fw_o;
    reg  [3:0] fw_mlv;                       // 菜单档位（4 行共用一份 lv_sign / lv_digit）
    reg  [7:0] fw_smin;                      // 定时档位 -> 分钟数 BCD（0 = 关）
    always @* begin
        case (s_slv)
            3'd1: fw_smin = 8'h01;  3'd2: fw_smin = 8'h15;  3'd3: fw_smin = 8'h30;
            3'd4: fw_smin = 8'h60;  3'd5: fw_smin = 8'h90;  default: fw_smin = 8'h00;
        endcase
    end
    wire [3:0] w2_t = w2_tg[3:0], w2_o = w2_og[3:0];
    wire [3:0] fw_mdg = (fw_mlv >= 4'd5) ? fw_mlv - 4'd5 : 4'd5 - fw_mlv;
    always @* begin
        case (fw_k)
            F_MS1-FIELD_BASE, F_MD1-FIELD_BASE: fw_mlv = s_mlv[7:4];
            F_MS2-FIELD_BASE, F_MD2-FIELD_BASE: fw_mlv = s_mlv[11:8];
            F_MS3-FIELD_BASE, F_MD3-FIELD_BASE: fw_mlv = s_mlv[15:12];
            default:                            fw_mlv = s_mlv[3:0];
        endcase
    end
    always @* begin
        fw_hold = 1'b0; fw_n = 4'd0; fw_c = FC_BLK; fw_o = G_BLANK;
        case (fw_k)
            // 第几张右对齐（十位格不足时空），共几张左对齐（个位格不足时空）；没有图时 "-/-"
            F_IDX1-FIELD_BASE: begin fw_n = w2_t; fw_c = (s_live == 0 || s_idx < 7'd10) ? FC_BLK : FC_DIG; end
            F_IDX0-FIELD_BASE: begin fw_n = w2_o; fw_c = (s_live == 0) ? FC_DSH : FC_DIG; end
            F_TOT1-FIELD_BASE: begin fw_n = (s_live >= 7'd10) ? w2_t : w2_o; fw_c = (s_live == 0) ? FC_DSH : FC_DIG; end
            F_TOT0-FIELD_BASE: begin fw_n = w2_o; fw_c = (s_live >= 7'd10) ? FC_DIG : FC_BLK; end
            F_MA-FIELD_BASE:   begin fw_c = FC_OTH; fw_o = s_auto ? G_ZI : G_SHOU; end
            F_MB-FIELD_BASE:   begin fw_c = FC_OTH; fw_o = G_DONG; end
            F_PT-FIELD_BASE:   begin fw_n = w2_t; fw_c = (s_per >= 7'd10) ? FC_DIG : FC_BLK; end
            F_PO-FIELD_BASE:   begin fw_n = w2_o; fw_c = (s_per == 7'd0) ? FC_OTH : FC_DIG; fw_o = G_NOTE; end   // 0 = 随音乐：♪
            F_VH-FIELD_BASE:   begin fw_n = 4'd1; fw_c = (pct >= 100) ? FC_DIG : FC_BLK; end
            F_VT-FIELD_BASE:   begin fw_n = pct_tens(pct); fw_c = (pct >= 10) ? FC_DIG : FC_BLK; end
            F_VO-FIELD_BASE:   begin fw_n = pct_ones(pct); fw_c = FC_DIG; end
            F_H1-FIELD_BASE:   begin fw_n = s_time[23:20]; fw_c = FC_DIG; end
            F_H0-FIELD_BASE:   begin fw_n = s_time[19:16]; fw_c = FC_DIG; end
            F_N1-FIELD_BASE:   begin fw_n = s_time[15:12]; fw_c = FC_DIG; end
            F_N0-FIELD_BASE:   begin fw_n = s_time[11:8];  fw_c = FC_DIG; end
            F_S1-FIELD_BASE:   begin fw_n = s_time[7:4];   fw_c = FC_DIG; end
            F_S0-FIELD_BASE:   begin fw_n = s_time[3:0];   fw_c = FC_DIG; end
            // C/D/E 行的字段只在该行"应当显示"时更新；淡出期间保持消失前的内容，
            // 不会出现"读卡错误 0"这种过渡画面
            F_ERR-FIELD_BASE:  begin fw_n = s_err; fw_c = FC_DIG; fw_hold = (s_err == 0); end
            F_BAD1-FIELD_BASE: begin fw_n = (s_bad >= 7'd10) ? w2_t : w2_o; fw_c = FC_DIG; fw_hold = (s_bad == 0); end
            F_BAD0-FIELD_BASE: begin fw_n = w2_o; fw_c = (s_bad >= 7'd10) ? FC_DIG : FC_BLK; fw_hold = (s_bad == 0); end
            // 音乐
            F_SI1-FIELD_BASE:  begin fw_n = w2_t; fw_c = (w_midx >= 7'd10) ? FC_DIG : FC_BLK; end
            F_SI0-FIELD_BASE:  begin fw_n = w2_o; fw_c = FC_DIG; end
            F_SC1-FIELD_BASE:  begin fw_n = (w_mcnt >= 7'd10) ? w2_t : w2_o; fw_c = FC_DIG; end
            F_SC0-FIELD_BASE:  begin fw_n = w2_o; fw_c = (w_mcnt >= 7'd10) ? FC_DIG : FC_BLK; end
            F_EM1-FIELD_BASE:  begin fw_n = w_mel[15:12];  fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_EM0-FIELD_BASE:  begin fw_n = w_mel[11:8];   fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_ES1-FIELD_BASE:  begin fw_n = w_mel[7:4];    fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_ES0-FIELD_BASE:  begin fw_n = w_mel[3:0];    fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_TM1-FIELD_BASE:  begin fw_n = w_mtot[15:12]; fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_TM0-FIELD_BASE:  begin fw_n = w_mtot[11:8];  fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_TS1-FIELD_BASE:  begin fw_n = w_mtot[7:4];   fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_TS0-FIELD_BASE:  begin fw_n = w_mtot[3:0];   fw_c = w_mknown ? FC_DIG : FC_DSH; end
            F_ME-FIELD_BASE:   begin fw_n = {1'b0, w_merr}; fw_c = FC_DIG; fw_hold = (w_merr == 0); end
            // 暂停 / 循环标志与提示行（q_pause / q_loop 就是第 1 拍锁存的 st_pause / st_loop）
            F_MI-FIELD_BASE:   begin fw_c = FC_OTH; fw_o = q_pause ? G_PAUSE : G_NOTE; end
            F_LP-FIELD_BASE:   begin fw_c = q_loop ? FC_OTH : FC_BLK; fw_o = G_DAN; end
            F_T1-FIELD_BASE:   begin fw_c = FC_OTH; fw_o = toast_gid({w_tkind, 2'd0}); fw_hold = !w_tev; end
            F_T2-FIELD_BASE:   begin fw_c = FC_OTH; fw_o = toast_gid({w_tkind, 2'd1}); fw_hold = !w_tev; end
            F_T3-FIELD_BASE:   begin fw_c = FC_OTH; fw_o = toast_gid({w_tkind, 2'd2}); fw_hold = !w_tev; end
            F_T4-FIELD_BASE:   begin fw_c = FC_OTH; fw_o = toast_gid({w_tkind, 2'd3}); fw_hold = !w_tev; end
            // 菜单：符号（+ / - / 空）和一位数字
            F_MS0-FIELD_BASE, F_MS1-FIELD_BASE, F_MS2-FIELD_BASE, F_MS3-FIELD_BASE:
                begin fw_c = (fw_mlv > 4'd5) ? FC_OTH : (fw_mlv < 4'd5) ? FC_DSH : FC_BLK; fw_o = G_PLUS; end
            F_MD0-FIELD_BASE, F_MD1-FIELD_BASE, F_MD2-FIELD_BASE, F_MD3-FIELD_BASE:
                begin fw_n = fw_mdg; fw_c = FC_DIG; end
            // 定时关机：剩余时间（显示时才更新）、菜单档位（右对齐，"关"在"分"的位置）、提示框秒数
            F_ZM1-FIELD_BASE:  begin fw_n = s_rem[15:12]; fw_c = FC_DIG; fw_hold = !s_son; end
            F_ZM0-FIELD_BASE:  begin fw_n = s_rem[11:8];  fw_c = FC_DIG; fw_hold = !s_son; end
            F_ZS1-FIELD_BASE:  begin fw_n = s_rem[7:4];   fw_c = FC_DIG; fw_hold = !s_son; end
            F_ZS0-FIELD_BASE:  begin fw_n = s_rem[3:0];   fw_c = FC_DIG; fw_hold = !s_son; end
            F_SVA-FIELD_BASE:  begin fw_n = fw_smin[7:4]; fw_c = (fw_smin[7:4] != 4'd0) ? FC_DIG : FC_BLK; end
            F_SVB-FIELD_BASE:  begin fw_n = fw_smin[3:0]; fw_c = (fw_smin != 8'd0) ? FC_DIG : FC_BLK; end
            F_SVC-FIELD_BASE:  begin fw_c = FC_OTH; fw_o = (fw_smin != 8'd0) ? G_FEN : G_GUAN; end
            F_WS1-FIELD_BASE:  begin fw_n = s_rem[7:4]; fw_c = (s_rem[7:4] != 4'd0) ? FC_DIG : FC_BLK; fw_hold = !s_warn; end
            F_WS0-FIELD_BASE:  begin fw_n = s_rem[3:0]; fw_c = FC_DIG; fw_hold = !s_warn; end
            default: ;
        endcase
        case (fw_c)
            FC_DIG:  fw_d = dg(fw_n);
            FC_BLK:  fw_d = G_BLANK;
            FC_DSH:  fw_d = G_DASH;
            default: fw_d = fw_o;
        endcase
    end
    // 不带复位、单写口、异步读：TD 推成分布式 RAM。复位后第一次更新时，不更新的字段写空白
    always @(posedge clk)
        if (fw_run && (!fw_hold || fw_init))
            fld[fw_k] <= fw_hold ? G_BLANK : fw_d;

    // ======================== S0：区域、字符格、地址 ========================
    wire signed [12:0] lx = $signed({1'b0, raw_x}) - (H_BLANK - LEAD);
    wire signed [12:0] ayw = $signed({1'b0, raw_y}) - V_BLANK;
    wire        x_ok = (lx >= 0) && (lx < 640);
    wire        y_ok = (ayw >= 0) && (ayw < 480);
    wire [9:0]  ax = lx[9:0];
    wire [9:0]  ay = ayw[9:0];
    wire        line_start = (raw_x == 12'd0);

    // 只和行号有关的判断查按行的表 row_rom（osd_gen.py 生成，TD 推成 BRAM ROM），寄存一拍（省 LUT）。
    //   行号在 raw_x == 0 那拍变，表的输出只在这一拍是上一行的（有效区外；字符格 / 字幕扫描这拍正好复位），不影响结果。
    //   有效区外的行（y_ok = 0）只取 ay 低 9 位会查到别的行，S0 的输出都被 x_ok && y_ok 挡住。
    reg  [34:0] rowq;
    always @(posedge clk)
        rowq <= row_rom(ay[8:0]);
    wire       sl_hit     = rowq[27];
    wire [2:0] sl_k       = rowq[26:24];
    wire       r_row      = rowq[23];
    wire       t_row      = rowq[22];
    wire       m_yin      = rowq[21];
    wire       m_row_ok   = rowq[20];
    wire [2:0] m_r        = rowq[19:17];
    wire       h_ny       = rowq[16];      // 菜单高亮条：min(m_ry, BAND_H-1-m_ry) < PANEL_R
    wire [2:0] h_dy       = rowq[15:13];   //   它的低 3 位
    wire       m_segy     = rowq[12];      // 进度条的 y 条件（含 m_row_ok && m_r < 4）
    wire [3:0] mny        = rowq[11:8];    // 菜单面板上下圆角 near3(ay - MY0, MY1 - 1 - ay)
    wire       sub_band   = rowq[7];
    wire       sub_row_ok = rowq[6];
    wire [5:0] roff       = rowq[5:0];     // 行内第几行（槽 / 音乐行 / 提示行 / 菜单行 / 字幕行）
    wire       s_nrow     = rowq[34];      // 胶片条 "n/N" 行
    wire       s_lrow     = rowq[33];      //   胶片条标签行
    wire [4:0] soff       = rowq[32:28];   //   这两行的行内第几行
    // 状态行命中：右侧音乐行（x >= SPLIT_X）优先，否则左侧 A~E
    // 菜单行：第几行（ay 落在哪一行的 [行首, 行首+BAND_H)）
    wire       m_xin = (ax >= MX0) && (ax < MX1);
    // 左侧行只可能排在 5 个槽上，先求出 ay 落在第几个槽（sl_k）、槽内第几行（roff），
    //   再看哪一行占着这个槽（3 位比较，比每行比 y 范围省 LUT）。已占用的行槽号互不相同。
    // 关机提示框的 x / y 范围（菜单那样的圆角：near3 左右 + 上下）
    wire [10:0] wdl = {1'b0, ax} - WX0, wdr = (WX1 - 1) - {1'b0, ax};
    wire [10:0] wdt = {1'b0, ay} - WY0, wdb = (WY1 - 1) - {1'b0, ay};
    wire        w_xin = !wdl[10] && !wdr[10];
    wire        w_yin = !wdt[10] && !wdb[10];
    reg        b_hit;
    reg [3:0]  b_sel;
    reg [4:0]  ry;
    reg        w_alt;                    // 关机提示框的两行字（b_sel 9 / 10 的第二套字符格）
    always @* begin
        b_hit = 1'b0; b_sel = 4'd0; ry = 5'd0; w_alt = 1'b0;
        for (i = 4; i >= 0; i = i - 1)
            if (i != 1 && band_occ[i] && sl_hit && band_s[i] == sl_k) begin
                b_hit = 1'b1; b_sel = i; ry = roff[4:0];
            end
        if (r_row && ax >= SPLIT_X) begin
            b_hit = 1'b1; b_sel = band_occ[5] ? 4'd5 : 4'd6; ry = roff[4:0];
        end
        if (t_row && ax >= SPLIT_X && band_occ[7]) begin
            b_hit = 1'b1; b_sel = 4'd7; ry = roff[4:0];
        end
        if (sl_hit && sl_k == 3'd3 && ax >= SPLIT_X && band_occ[1]) begin   // 右侧"关机 mm:ss"
            b_hit = 1'b1; b_sel = 4'd1; ry = roff[4:0];
        end
        if (mvis && m_row_ok && m_xin) begin           // 菜单行（与左侧状态行不重叠）
            b_hit = 1'b1; b_sel = 4'd8 + m_r; ry = roff[4:0];
        end
        if (!mvis && wvis && m_row_ok && (m_r == 3'd1 || m_r == 3'd2) && w_xin) begin   // 关机提示框两行
            b_hit = 1'b1; b_sel = 4'd8 + m_r; ry = roff[4:0]; w_alt = 1'b1;
        end
        if (st_strip && (s_nrow || s_lrow)) begin      // 胶片条两行：整行归它，最优先
            b_hit = 1'b1; b_sel = s_nrow ? 4'd14 : 4'd15; ry = soff; w_alt = 1'b0;
        end
    end
    wire s_band = (b_sel[3:1] == 3'b111);              // b_sel 14 / 15
    wire [5:0] sub_row = roff;
    wire [4:0] trow = ry - TEXT_DY;
    wire trow_ok = (ry >= TEXT_DY) && (ry < TEXT_DY + ST_H);

    // 圆角：离左右（上下）边的距离 dl、dr（都 >= 0 时才有意义）里较小的那个是否 < PANEL_R，以及它的低 3 位
    //   不做 10 位的 min(dl, dr)（省 LUT），分别看 dl < PANEL_R、dr < PANEL_R，两边都近时才在 3 位上取较小值
    //   （PANEL_R <= 8），与"先取较小值再和 PANEL_R 比"等价。
    function [3:0] near3;               // {较小的距离 < PANEL_R, 较小的距离低 3 位}
        input [10:0] dl;
        input [10:0] dr;
        reg nl, nr;
        begin
            nl = (dl < PANEL_R);
            nr = (dr < PANEL_R);
            near3[3]   = nl || nr;
            near3[2:0] = (nl && nr) ? ((dl[2:0] < dr[2:0]) ? dl[2:0] : dr[2:0]) : nl ? dl[2:0] : dr[2:0];
        end
    endfunction

    // 圆角底板（在不在 [x0, x1) 直接用 11 位减法的借位判断）
    function panel_in;
        input [9:0] x;
        input [9:0] x0;
        input [9:0] x1;
        input [4:0] yy;
        reg [10:0] dl, dr;
        reg [3:0]  nx;
        reg [4:0]  dy;
        begin
            dl = {1'b0, x} - {1'b0, x0};
            dr = {1'b0, x1} - 11'd1 - {1'b0, x};
            nx = near3(dl, dr);
            if (dl[10] || dr[10]) panel_in = 1'b0;            // x < x0 或 x >= x1
            else begin
                dy = (yy < BAND_H - 1 - yy) ? yy : BAND_H - 1 - yy;
                if (nx[3] && dy < PANEL_R) panel_in = corner_in({nx[2:0], dy[2:0]});
                else panel_in = 1'b1;
            end
        end
    endfunction
    wire p_hit = b_hit && (panel_in(ax, panel_x0({b_sel, 1'b0}), panel_x1({b_sel, 1'b0}), ry) ||
                           panel_in(ax, panel_x0({b_sel, 1'b1}), panel_x1({b_sel, 1'b1}), ry));

    // 菜单区域：0 无 1 面板 2 面板+高亮 3 灭格 4 灭格(高亮) 5 亮格 6 亮格(当前行)
    //   （距离只在面板/高亮条范围内用，那时都 >= 0）
    wire [10:0] mdl = {1'b0, ax} - MX0, mdr = (MX1 - 1) - {1'b0, ax};
    wire [3:0]  mnx = near3(mdl, mdr);            // 上下方向的 mny 在 row_rom 里
    wire        m_pan = mvis && m_xin && m_yin && !(mnx[3] && mny[3] && !corner_in({mnx[2:0], mny[2:0]}));
    wire        m_sel = m_row_ok && (m_r == s_msel);
    wire [10:0] hdl = {1'b0, ax} - (MX0 + MHL_M), hdr = (MX1 - MHL_M - 1) - {1'b0, ax};
    wire [3:0]  hnx = near3(hdl, hdr);
    wire        m_hl = m_sel && (ax >= MX0 + MHL_M) && (ax < MX1 - MHL_M) &&
                       !(hnx[3] && h_ny && !corner_in({hnx[2:0], h_dy}));   // 上下圆角在 row_rom 里
    wire [9:0] bo = ax - MBAR_X0;
    wire [9:0] so = ax - SBAR_X0;                 // 第 6 行"定时"的 5 格（位置查 sl_seg）
    wire [3:0] s_sg = sl_seg(so[6:0]);
    wire       m_r5 = (m_r == 3'd5);
    wire       m_seg = m_segy && (m_r5 ? ((ax >= SBAR_X0) && (so < 10'd70) && s_sg[3])
                                       : ((ax >= MBAR_X0) && (ax < MBAR_X0 + MSEG_N * MSEG_P) && (bo[2:0] < MSEG_W)));
    wire [3:0] m_lv = (m_r == 3'd0) ? s_mlv[3:0] : (m_r == 3'd1) ? s_mlv[7:4] :
                      (m_r == 3'd2) ? s_mlv[11:8] : s_mlv[15:12];
    wire       m_on = m_r5 ? (s_sg[2:0] < s_slv) : (bo[6:3] <= m_lv);
    wire [2:0] m_code = !m_pan ? 3'd0 :
                        m_seg ? (m_on ? (m_sel ? 3'd6 : 3'd5) : (m_hl ? 3'd4 : 3'd3)) :
                        m_hl  ? 3'd2 : 3'd1;
    // 关机提示框区域：0 无 1 底板 2 灭格 3 亮格（底板号 cpi = 16 + 区域码）
    wire [3:0] wnx = near3(wdl, wdr), wny = near3(wdt, wdb);
    wire       w_pan = wvis && !mvis && w_xin && w_yin && !(wnx[3] && wny[3] && !corner_in({wnx[2:0], wny[2:0]}));
    wire [9:0] wbx = ax - WBAR_X0;
    wire       w_bar = (ay >= WBAR_Y0) && (ay < WBAR_Y1) && (ax >= WBAR_X0) && (wbx < WSEG_N * 16) && (wbx[3:0] < WSEG_W);
    wire [4:0] w_cpi = !w_pan ? 5'd0 : w_bar ? ((wbx[7:4] < w_sec) ? 5'd19 : 5'd18) : 5'd17;

    // 状态行字符格扫描（每行在 raw_x==0 复位）
    reg [5:0] c_ptr, c_col;
    reg       c_in;
    wire [9:0] ckey = {b_sel, w_alt, c_ptr[4:0]};   // 第 5 位选第二套（关机提示框）；每行最多 31 格
    wire       c_split = (r_row || t_row || (sl_hit && sl_k == 3'd3 && band_occ[1])) && x_ok && (ax == SPLIT_X - 1);   // 进入右侧行前清零字符格指针
    // 字符格表 {x0, w, src} 放在一块 BRAM32K 同步 ROM（osd_cell.mif / .hex，design/osd_gen.py 生成，比组合查表省约 350~400 LUT）。
    //   ROM 读延迟 1 拍，所以地址用"下一拍的 ckey"：
    //     b_sel_n / w_alt_n = 上面 b_sel / w_alt 那段判断，其中和 x 有关的条件改用 ax + 1（下一个像素）；
    //     c_ptr_n = c_ptr 寄存器的下一状态（和下面 always 块完全同一个式子）。
    //   成立的前提：有效行里 raw_x 每拍加 1，rowq（行表）只在 raw_x == 0 后一拍变，帧级量（band_occ、mvis、wvis、st_strip）在消隐里变。
    //   只有每行 raw_x = 0、1 两拍预测不准，这两拍 x_ok = 0、c_in = 0（line_start 刚清），c_act = 0，用不到表的值。
    //   修改上面 b_sel / w_alt 的判断（优先级、范围），b_sel_n / w_alt_n 必须一起改；OSD_SIM 下有断言检查两者一致。
    wire [9:0]  ax_n = ax + 10'd1;
    wire [10:0] wdl_n = {1'b0, ax_n} - WX0, wdr_n = (WX1 - 1) - {1'b0, ax_n};
    wire        w_xin_n = !wdl_n[10] && !wdr_n[10];
    wire        m_xin_n = (ax_n >= MX0) && (ax_n < MX1);
    reg  [3:0]  b_sel_n;
    reg         w_alt_n;
    always @* begin
        b_sel_n = 4'd0; w_alt_n = 1'b0;
        for (i = 4; i >= 0; i = i - 1)
            if (i != 1 && band_occ[i] && sl_hit && band_s[i] == sl_k) b_sel_n = i;
        if (r_row && ax_n >= SPLIT_X) b_sel_n = band_occ[5] ? 4'd5 : 4'd6;
        if (t_row && ax_n >= SPLIT_X && band_occ[7]) b_sel_n = 4'd7;
        if (sl_hit && sl_k == 3'd3 && ax_n >= SPLIT_X && band_occ[1]) b_sel_n = 4'd1;
        if (mvis && m_row_ok && m_xin_n) b_sel_n = 4'd8 + m_r;
        if (!mvis && wvis && m_row_ok && (m_r == 3'd1 || m_r == 3'd2) && w_xin_n) begin
            b_sel_n = 4'd8 + m_r; w_alt_n = 1'b1;
        end
        if (st_strip && (s_nrow || s_lrow)) begin
            b_sel_n = s_nrow ? 4'd14 : 4'd15; w_alt_n = 1'b0;
        end
    end
    wire [22:0] cq;                                  // 本拍 ckey 的 {x0, w, src}
    wire [9:0] c_x0 = cq[22:13];
    wire [4:0] c_w  = cq[12:8];
    wire [7:0] c_src = cq[7:0];
    wire c_start = !c_in && x_ok && (ax == c_x0);
    wire c_act = b_hit && (c_in || c_start);
    wire [5:0] c_colnow = c_in ? c_col : 6'd0;
    // 标签格（c_src = 192 + 格号*4 + 第几个字）按这一格的状态取"加载中" / "坏图片"的字，别的状态空
    reg  [1:0] l_st;
    always @* begin
        case (c_src[4:2])
            3'd0: l_st = st_sst[1:0];  3'd1: l_st = st_sst[3:2];  3'd2: l_st = st_sst[5:4];
            3'd3: l_st = st_sst[7:6];  default: l_st = st_sst[9:8];
        endcase
    end
    wire [6:0] l_gid = (l_st == 2'd2) ? lbl_load(c_src[1:0]) : (l_st == 2'd3) ? lbl_bad(c_src[1:0]) : G_BLANK;
    wire [6:0] c_gid = (c_src < FIELD_BASE) ? c_src[6:0] : (c_src[7:6] == 2'b11) ? l_gid : fld[c_src[5:0]];
    wire [5:0] c_ptr_n = (line_start || c_split) ? 6'd0 :
                         (c_act && (c_colnow == c_w - 1)) ? c_ptr + 6'd1 : c_ptr;
    wire [9:0] ckey_n = {b_sel_n, w_alt_n, c_ptr_n[4:0]};
    osd_cell_rom u_cell (.clk(clk), .addr(ckey_n), .q(cq));
`ifdef OSD_SIM
    reg [9:0] ckey_q;
    always @(posedge clk) begin
        ckey_q <= ckey_n;
        if (rst_n && x_ok && y_ok && ckey_q != ckey)
            $display("[ASSERT] %t osd_engine: 字符格表预测地址 %h != 实际 ckey %h（b_sel_n / w_alt_n 和 b_sel / w_alt 不一致）",
                     $time, ckey_q, ckey);
    end
`endif
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c_ptr <= 0; c_col <= 0; c_in <= 1'b0;
        end else if (line_start || c_split) begin
            c_ptr <= 0; c_col <= 0; c_in <= 1'b0;
        end else if (c_act) begin
            if (c_colnow == c_w - 1) begin
                c_in <= 1'b0; c_ptr <= c_ptr + 6'd1;
            end else begin
                c_in <= 1'b1; c_col <= c_colnow + 6'd1;
            end
        end
    end

    // 字幕扫描（每行在 raw_x==0 复位）。整条字幕一个列号 s_col（0 ~ 宽-1）
    //   s_rb = 本行在字幕条 RAM 里的起始地址：raw_x==0 那拍 rowq 还是上一行的，上一行是字幕行就减一行，否则回到第 0 行（top）
    reg [9:0]  s_col;
    reg        s_in;
    reg [13:0] s_rb;
    // xs > 0 时"lx == xs"等价于 xs < 1024 且 ax == xs 的低 10 位（x_ok 时 lx = ax），省掉 13 位减法
    wire s_start_neg = !s_in && x_ok && (ax == 10'd0) && (xs <= 0);
    wire s_start_pos = !s_in && x_ok && (xs > 0) && (xs[11:10] == 2'b00) && (ax == xs[9:0]);
    wire s_act = sub_row_ok && x_ok && (s_in || s_start_neg || s_start_pos);
    wire [9:0] s_colnow  = s_in ? s_col  : (s_start_neg ? sub_col0 : 10'd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_col <= 0; s_in <= 1'b0; s_rb <= 0;
        end else if (line_start) begin
            s_col <= 0; s_in <= 1'b0;
            s_rb  <= (sub_band && sub_row_ok) ? s_rb - {5'd0, sub_wb} : sub_tpf;
        end else if (s_act) begin
            if (s_colnow == sub_wf - 10'd1) begin
                s_in <= 1'b0;                                // 本行结束（ax 只增不减，本行不会再开始）
            end else begin
                s_in <= 1'b1; s_col <= s_colnow + 10'd1;
            end
        end
    end

    // 本像素用哪个字形、哪一行哪一列；颜色与底板
    reg        txt_v;
    reg [6:0]  gid;
    reg [5:0]  grow, gcol;
    reg        sub_v;                            // 这个像素取字幕条 RAM（不查字库）
    // S0 只选"颜色号"（tci / cpi），S0B 用颜色号查 tc_rom / cpap_rom（osd_gen.py 生成），
    //   省掉各张小色表和 24 位宽的多路选择（省 LUT）。
    reg [5:0]  tci;                              // 文字色号：0 无，1~8 状态行，9/10 菜单（普通/当前行），16+ 字幕渐变
    reg [4:0]  cpi;                              // 底板号：0 无，1~8 状态行，9~15 菜单区域码，16 字幕底板
    reg [8:0]  f0;
    always @* begin
        txt_v = 1'b0; gid = 0; grow = 0; gcol = 0; sub_v = 1'b0;
        tci = 6'd0; cpi = 5'd0; f0 = 9'd0;
        if (x_ok && y_ok) begin
            if (b_hit) begin
                if (s_band) begin                      // 胶片条两行：只有字，没有底板
                    f0 = st_sf;
                    tci = b_sel[0] ? 6'd12 : 6'd11;    // b_sel 14 = "n/N"、15 = 标签
                end else if (w_alt) begin              // 关机提示框两行
                    f0 = f_w;
                    tci = m_r[0] ? 6'd13 : 6'd14;
                    cpi = w_cpi;
                end else if (b_sel[3]) begin           // 菜单行
                    f0 = f_menu;
                    tci = m_sel ? 6'd10 : 6'd9;
                    cpi = 5'd9 + {2'b0, m_code};
                end else begin
                    f0 = f_eff[b_sel[2:0]];
                    tci = 6'd1 + {3'b0, b_sel[2:0]};
                    if (p_hit) cpi = 5'd1 + {2'b0, b_sel[2:0]};
                end
                if (c_act && trow_ok && c_gid != G_BLANK) begin
                    txt_v = 1'b1; gid = c_gid; grow = {1'b0, trow}; gcol = c_colnow;
                end
            end else if (m_pan) begin                  // 菜单面板（行与行之间、上下留边）
                f0 = f_menu;
                cpi = 5'd9 + {2'b0, m_code};
            end else if (w_pan) begin                  // 关机提示框底板、进度条
                f0 = f_w;
                cpi = w_cpi;
            end else if (sub_band) begin
                f0 = f_sub_eff;
                cpi = 5'd16;
                if (sub_row_ok) tci = 6'd16 + sub_row;
                if (s_act) begin
                    txt_v = 1'b1; sub_v = 1'b1;
                end
            end
        end
    end
    // ---- S0B：寄存 S0 的结果（S0 组合逻辑较长，单独切一级） ----
    reg        txt_vb;
    reg        sub_vb;
    reg [9:0]  scol_b;
    reg [6:0]  gidb;
    reg [5:0]  growb, gcolb;
    reg [23:0] tcb, cpb;
    reg [8:0]  apb, fb;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txt_vb <= 0; gidb <= 0; growb <= 0; gcolb <= 0; fb <= 0;
            sub_vb <= 1'b0; scol_b <= 10'd0;
        end else begin
            txt_vb <= txt_v; gidb <= gid; growb <= grow; gcolb <= gcol;
            fb <= f0;
            sub_vb <= sub_v; scol_b <= s_colnow;
        end
    end
    // 查表（不带复位，TD 才能推成 BRAM ROM）：S0 给号，S0B 出颜色
    //   字形表也在这一拍查：S0 的字形号 gid 直接作 ROM 地址
    reg [15:0] gbase_b;
    reg [5:0]  gw_b;
    reg        gpk_b;
    always @(posedge clk) begin
        tcb         <= tc_rom(tci);             // 颜色表很小，TD 用 LUT 做（单独放 BRAM 要 4 块，只省 40 多个 LUT，不划算）
        {cpb, apb}  <= cpap_rom(cpi);
        gbase_b     <= glyph_base(gid);         // 字形表 TD 推成 BRAM ROM（1 块 BRAM9K）
        gw_b        <= glyph_w(gid);
        gpk_b       <= glyph_packed(gid);
    end

    // ---- S0R：算出字库地址并寄存 ----
    // 像素序号 n = 行*宽 + 列（整个字形连续编号）
    //   压缩字形：字节地址 = base + n>>1，n 是奇数取高 4 位，否则低 4 位，g = o
    //   非压缩字形：字节地址 = base + n，高 4 位是 g、低 4 位是 o
    // ROM 一行 16 位 = 2 个字节：行地址 = 字节地址>>1，字节地址是奇数取高字节
    wire [15:0] npix = growb * gw_b + gcolb;
    wire        pk0  = gpk_b;
    wire [15:0] baddr = gbase_b + (pk0 ? {1'b0, npix[15:1]} : npix);
    reg  [13:0] rom_addr_q;
    reg         packed_q, odd_q, hib_q;
    reg  [13:0] sub_addr_q;                      // 字幕条 RAM 地址 = 本行起始 + 列号/2
    reg         sub_vq, sodd_q;
    reg        txt_vq;
    reg [23:0] tcq, cpq;
    reg [8:0]  apq, fq;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rom_addr_q <= 0; packed_q <= 1'b0; odd_q <= 1'b0; hib_q <= 1'b0;
            sub_addr_q <= 0; sub_vq <= 1'b0; sodd_q <= 1'b0;
            txt_vq <= 0; tcq <= 0; cpq <= 0; apq <= 0; fq <= 0;
        end else begin
            rom_addr_q <= baddr[14:1];
            hib_q      <= baddr[0];
            packed_q   <= pk0;
            odd_q      <= npix[0];
            sub_addr_q <= s_rb + {5'd0, scol_b[9:1]};
            sub_vq     <= sub_vb;
            sodd_q     <= scol_b[0];
            txt_vq <= txt_vb; tcq <= tcb; cpq <= cpb; apq <= apb; fq <= fb;
        end
    end

    // ======================== 字库 ROM（读延迟 1 拍） ========================
    wire [15:0] rom_q;
    osd_font_rom u_rom (.clk(clk), .addr(rom_addr_q), .q(rom_q));
    // 字幕条 RAM（读延迟 1 拍，和字库同一拍出数据）
    wire [7:0] sub_q;
    sub_strip_ram u_sub_ram (
        .wclk(sub_wclk), .we(sub_we), .waddr(sub_waddr), .wdata(sub_wdata),
        .rclk(clk), .use_def(!s_subok), .raddr(sub_addr_q), .q(sub_q));

    // ======================== S1：乘法 ========================
    reg        txt_v1;
    reg [23:0] tc1, cp1;
    reg [8:0]  ap1, f1;
    reg        pk_b, odd_b, hib_b;
    reg        sub_v1, sodd_1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txt_v1 <= 0; tc1 <= 0; cp1 <= 0; ap1 <= 0; f1 <= 0;
            pk_b <= 0; odd_b <= 0; hib_b <= 0;
            sub_v1 <= 1'b0; sodd_1 <= 1'b0;
        end else begin
            sub_v1 <= sub_vq; sodd_1 <= sodd_q;
            txt_v1 <= txt_vq; tc1 <= tcq; cp1 <= cpq; ap1 <= apq; f1 <= fq;
            pk_b <= packed_q; odd_b <= odd_q; hib_b <= hib_q;
        end
    end
    function [8:0] a9;
        input [3:0] v;
        begin
            a9 = v * 5'd17 + (v == 4'd15);
        end
    endfunction
    // 先按字节地址奇偶取字节；压缩字形再按像素奇偶取半个字节（g = o），非压缩字形高 4 位 g、低 4 位 o
    wire [7:0] rom_b  = hib_b ? rom_q[15:8] : rom_q[7:0];
    wire [3:0] pk_nib = odd_b ? rom_b[7:4] : rom_b[3:0];
    // 字幕条像素 0~15 -> {g, o}，固定调色板（和 design/make_subtitle.py 的 FIXED_PAL 一样，改一处必须改另一处）
    function [7:0] sub_pal;
        input [3:0] k;
        case (k)
            4'd0:  sub_pal = 8'h00;  4'd1:  sub_pal = 8'hFF;  4'd2:  sub_pal = 8'h0F;  4'd3:  sub_pal = 8'h01;
            4'd4:  sub_pal = 8'h02;  4'd5:  sub_pal = 8'h03;  4'd6:  sub_pal = 8'h06;  4'd7:  sub_pal = 8'h05;
            4'd8:  sub_pal = 8'h04;  4'd9:  sub_pal = 8'h08;  4'd10: sub_pal = 8'hCF;  4'd11: sub_pal = 8'h1F;
            4'd12: sub_pal = 8'h7F;  4'd13: sub_pal = 8'h4F;  4'd14: sub_pal = 8'h0C;  default: sub_pal = 8'h2F;
        endcase
    endfunction
    wire [7:0] sub_go = sub_pal(sodd_1 ? sub_q[3:0] : sub_q[7:4]);   // 偶数列在高 4 位
    wire [3:0] g4 = txt_v1 ? (sub_v1 ? sub_go[7:4] : pk_b ? pk_nib : rom_b[7:4]) : 4'd0;
    wire [3:0] o4 = txt_v1 ? (sub_v1 ? sub_go[3:0] : pk_b ? pk_nib : rom_b[3:0]) : 4'd0;
    wire [8:0] g9 = a9(g4);
    wire [8:0] o9 = a9(o4);
    wire [8:0] o9c = 9'd256 - o9;
    wire [16:0] mt_r = tc1[23:16] * g9, mt_g = tc1[15:8] * g9, mt_b = tc1[7:0] * g9;
    wire [16:0] mp_r = cp1[23:16] * o9c, mp_g = cp1[15:8] * o9c, mp_b = cp1[7:0] * o9c;
    wire [17:0] ma   = ap1 * o9c;

    reg [8:0] c2_r, c2_g, c2_b, a2, f2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c2_r <= 0; c2_g <= 0; c2_b <= 0; a2 <= 0; f2 <= 0;
        end else begin
            c2_r <= mt_r[16:8] + mp_r[16:8];
            c2_g <= mt_g[16:8] + mp_g[16:8];
            c2_b <= mt_b[16:8] + mp_b[16:8];
            a2   <= o9 + ma[16:8];
            f2   <= f1;
        end
    end

    // ======================== S2：淡入淡出 ========================
    wire [17:0] mf_r = c2_r * f2, mf_g = c2_g * f2, mf_b = c2_b * f2, mf_a = a2 * f2;
    reg [8:0] c3_r, c3_g, c3_b, a3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            c3_r <= 0; c3_g <= 0; c3_b <= 0; a3 <= 0;
        end else begin
            c3_r <= mf_r[16:8]; c3_g <= mf_g[16:8]; c3_b <= mf_b[16:8]; a3 <= mf_a[16:8];
        end
    end

    // ======================== S3：与背景混合 ========================
    wire [8:0]  a3c = 9'd256 - a3;
    wire [16:0] mb_r = bg_rgb[23:16] * a3c, mb_g = bg_rgb[15:8] * a3c, mb_b = bg_rgb[7:0] * a3c;
    wire [9:0]  o_r = c3_r + mb_r[16:8], o_g = c3_g + mb_g[16:8], o_b = c3_b + mb_b[16:8];
    assign out_rgb = {(o_r > 10'd255) ? 8'd255 : o_r[7:0],
                      (o_g > 10'd255) ? 8'd255 : o_g[7:0],
                      (o_b > 10'd255) ? 8'd255 : o_b[7:0]};
endmodule
