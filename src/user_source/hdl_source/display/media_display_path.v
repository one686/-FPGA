`timescale 1ns/1ps

// 显示通路（像素时钟域 pix_clk 25MHz，SDRAM 读侧在 mem_clk 40MHz）
//
//   video_timing_ctrl (ACTIVE_LAST=1，有效区在后；HDMI 音频侧 hdmi_video_x_align 按此时序对齐行坐标)
//     -> media_frame_reader (mem_clk 发 SDRAM 读 burst，异步FIFO+预取 送到 pix_clk)
//     -> 转场亮度系数乘法 -> color_adjust 查表调色 -> osd_engine -> audio_spectrum -> fullscreen_viz
//     -> rgb_video_adapter (hs/vs/de/rgb 同拍寄存一拍，消隐期 rgb=0)
//
// 显示哪块整帧缓存（bank 0~5）由 sd_card_bmp 的 disp_buf_idx 决定（sd_card_clk 域）。
//   {display_valid, bank 号, strip_open} 一起两级同步、连续两拍一致才采用（去抖），由转场控制器在帧起点同一拍锁存，
//   切换不会撕裂；首图提交前黑屏也按帧生效——否则首图在某帧中间提交时，该帧顶部会露出几行旧数据
//   （BMP 自下而上存储，最上面几行是最后写进 SDRAM 的）。
//
// 转场（video_transition）：拨码 SW3..SW1 选效果，0 = 直接切换。控制器在帧起点给 reader 几何参数
//   （行内分段读新/旧图），并逐像素给出亮度系数 f_pix（淡入淡出 + 边缘阴影），像素乘 f_pix/256。
//   f_pix=256 时 (x*256)>>8 = x，原样输出。
// 查表调色（color_adjust：亮度/对比度/色温 + 饱和度）只作用于图片，不影响 OSD 和频谱；
//   带来 COLOR_LAT=3 拍延迟，OSD、频谱用的坐标和 hs/vs/de 一起延迟（c_*），画面对齐。
// OSD（osd_engine）：状态条、滚动字幕（SUB.BMP 或内置标语）、菜单、提示行，抗锯齿字库 + 精确乘法混合。
// 音频可视化：audio_spectrum 画频谱柱 + VU 表小面板；fullscreen_viz 为全屏音乐可视化层（音乐层短按 KEY1 开/关）。
// OSD_ENABLE=0 时直通（小画面的系统仿真用）。
//
// 画面调节菜单（本模块内，像素时钟域）：
//   图片层（SW4=0）长按 KEY3 1 秒 = 打开菜单；菜单里 KEY3 短按 = 下一项
//   （亮度 / 对比度 / 饱和度 / 色温 / 轮播周期 / 定时关机），KEY3 长按 1 秒 = 恢复默认（四项调色回中间档，
//   轮播周期和定时不动；OSD 提示"恢复默认"）；KEY2 = +1、KEY4 = -1（按下立刻生效，按住 0.5 秒后每 0.15 秒再一档，到头停住）。
//   5 秒（MENU_CLOSE_FRAMES 帧）没有按键自动关闭；拨 SW4 到音乐层或胶片条打开时立刻关闭。
//   每个键"这一次按下归谁"按按下那一刻菜单开没开决定；菜单开着时 sd_card_bmp 不响应 KEY2/KEY4（menu_open 送过去）。
//
// 胶片条（STRIP_EN = 1；见 filmstrip.v、strip_layout.vh）：
//   sd_card_bmp 给 strip_open 和格表信息 strip_info；转场控制器决定本帧是否显示胶片条（开 / 关都是淡出淡入）；
//   filmstrip 算 5 格放哪张图、逐像素画底板 / 金框 / 占位 / 箭头，reader 在胶片条行只读缩略图。
//   大图压暗、底板颜色都复用转场乘法器（像素 x 亮度系数），淡出淡入时整条一起变暗变亮。
//   KEY3（图片层）：按下那一刻胶片条开着 -> 短按 = 确认（strip_cfm_tgl 翻转一次，sd_card_bmp 收起胶片条），长按不响应；
//   胶片条开着时不能开菜单；胶片条开着或还看得见时隐藏频谱和字幕。
//
// 定时关机（sleep_ctrl.v）：菜单第 6 项（msel 5，档位 slv 0~5），倒计时、最后 10 秒提示框、
//   时间到画面淡出（与转场亮度系数取较小值，不另加乘法器）、OSD / 频谱 / 全屏层淡出、声音暂停 -> 关 HDMI（sl_tmds_off）；
//   任意键唤醒。sl_mask = 1 时本模块的 KEY2/3/4 事件都不响应（顶层另外屏蔽 sd_card_bmp 的 KEY1 短按、KEY2、KEY4）。
//   提示框出现时全屏层退出（否则会被全屏层盖住）。
// video_error（像素域，粘滞，数码管显示用）：
//   0=正常  1=帧超时(上一帧没读完新帧就来了)  2=读FIFO溢出  3=显示下溢(像素断供)
//   4=短帧(返回数据不够一帧)
module media_display_path #(
    parameter integer IMAGE_WORDS = 307200,
    parameter integer RGB_IN_HIGH = 1,     // SDRAM 中像素字格式 {RGB,8'h00}
    // 视频时序（默认 640x480@60）。只有系统仿真会改小，上板保持默认，IMAGE_WORDS 须等于 H_ACT*V_ACT
    parameter integer H_ACT = 640, parameter integer H_FP = 16,
    parameter integer H_SYNC = 96, parameter integer H_BP = 48,
    parameter integer V_ACT = 480, parameter integer V_FP = 10,
    parameter integer V_SYNC = 2,  parameter integer V_BP = 33,
    // OSD
    parameter integer OSD_ENABLE = 1,
    parameter integer OSD_SUB_X_INIT = 640,      // 字幕起始位置/速度（仿真时可改）
    parameter integer OSD_SUB_SPEED = 1,
    parameter integer OSD_TOAST_FRAMES = 0,      // 提示行显示帧数（0 = 默认 120 帧 = 2 秒）
    parameter integer OSD_SEC_CYCLES = 25_000_000,
    parameter integer OSD_DEBOUNCE = 500_000,      // 不再使用，保留参数为兼容例化
    parameter integer KEY_CLK_HZ = 25_000_000,     // 本模块按键（KEY2/3/4）计时用的时钟频率（仿真可改小来加速）
    // 查表调色
    parameter integer COLOR_ENABLE = 1,          // 0 = 不插调色（64x48 小画面的仿真用：场消隐太短，来不及算表）
    parameter integer COLOR_STEP = 8,            // 调色平滑：每帧增益最多变 8/256（约 0.3 秒走完一个满档）
    parameter integer MENU_CLOSE_FRAMES = 300,
    parameter integer FULL_FADE_STEP = 1,        // 全屏可视化淡入淡出每帧步长（仿真可改大）；MENU_CLOSE_FRAMES：菜单无操作自动关闭，300 帧 = 5 秒（最大 511）
    // 转场
    parameter integer TRANS_FRAMES = 60,         // 转场时长（帧），60 帧 = 1 秒
    parameter integer TRANS_SLAT = 40,           // 横百叶窗每片行数（须整除 V_ACT 才整齐）
    parameter integer TRANS_WIPE_SH = 32,        // 擦除/推移/中心展开阴影宽度（像素，8 的倍数）
    // 胶片条（几何在 strip_layout.vh，按 640x480 定；小画面的仿真设 0）
    parameter integer STRIP_EN = 1,
    parameter integer THUMB_BASE = 1843200,      // 第 k 张图的缩略图 = THUMB_BASE + k x THUMB_WORDS（须与写侧一致）
    parameter integer THUMB_WORDS = 10752,
    // 定时关机（见 sleep_ctrl.v；仿真可改小）
    parameter integer SL_TICKS = 1,
    parameter [15:0]  SL_LV1_BCD = 16'h0100,
    parameter integer SL_FADE_STEP = 4,
    parameter integer SL_WAKE_HOLD = 60,
    parameter integer SL_REL_FRAMES = 6,
    parameter integer PIC_MAX = 12               // 图片最多几张（= sd_card_bmp 的 BMP_MAX，决定 strip_info 位宽）
) (
    input  wire        mem_clk,
    input  wire        mem_rst_n,
    input  wire        controller_init_done,   // mem_clk 域
    output wire        app_rd_en,
    output wire [20:0] app_rd_addr,
    input  wire        sdram_rd_valid,
    input  wire [31:0] sdram_rd_data,
    input  wire        wr_busy,                // mem_clk 域，写侧忙
    output wire        rd_busy,                // mem_clk 域，读侧忙

    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire [2:0]  display_index_async,    // sd_card_clk 域 disp_buf_idx（bank 0~5）
    input  wire        display_valid_async,    // sd_card_clk 域，首图提交前黑屏
    output wire        video_hs,
    output wire        video_vs,
    output wire        video_de,
    output wire [23:0] video_rgb,
    output wire        frame_start,
    output reg  [3:0]  video_error,
    // OSD：状态（sd_card_clk 域）、KEY3
    input  wire [26:0] osd_status_async,       // {idx[6:0], live[6:0], auto, loading, err[3:0], bad[6:0]}（计数可显示两位数）
    input  wire        osd_key_n,              // KEY3，按下=0。图片层 短按(松手) = OSD 显示模式 / 菜单里下一项，长按 = 打开菜单 / 菜单里恢复默认；
                                               //   音乐层 短按 = 音量+，长按 = 音量-
    input  wire        menu_inc_key_n,         // 板上 KEY2（菜单里 +1），按下=0
    input  wire        menu_dec_key_n,         // 板上 KEY4（菜单里 -1），按下=0
    output wire [1:0]  osd_mode,
    // 音乐状态（sd_card_clk 域）、已播放时间（本时钟域）、音量
    input  wire [36:0] mus_status_async,       // {pause, loop, wav, idx[6:0], cnt[6:0], tot_known, tot_bcd[15:0], err[2:0]}
    input  wire [15:0] mus_elapsed,            // 已播放 m1 m0 s1 s0（BCD）
    output reg  [8:0]  vol_gain,               // 音量增益 0~256（给音源）
    // 转场
    input  wire [2:0]  trans_sw_async,         // 拨码 {SW3,SW2,SW1}
    input  wire        layer_async,            // SW4（0 = 图片层，1 = 音乐层）
    // 音频可视化用的样本（本时钟域，实际送进 HDMI 的样本）
    input  wire        wave_pcm_valid,
    input  wire signed [15:0] wave_pcm_left,
    input  wire signed [15:0] wave_pcm_right,
    // 菜单状态（给 sd_card_bmp）
    output reg         menu_open,              // 1 = 菜单开着（sd_card_bmp 不响应 KEY2/KEY4）
    output reg  [3:0]  per_lvl,                // 轮播周期档位 0~8（2 3 5 8 10 15 30 60 秒 / 8 = 随音乐），默认 1 = 3 秒
    output wire        trans_active,
    // 胶片条
    input  wire        strip_open_async,       // sd_card_clk 域：胶片条开着
    input  wire [3*PIC_MAX+8:0] strip_info_async, // sd_card_clk 域：{缩略图翻转, 有缩略图, 坏图, 张数, 焦点}（3 x PIC_MAX + 9 位）
    output reg  [5:0]  bank_busy,              // 本时钟域，本帧 / 上一帧读过的 bank（给 sd_card_bmp，换入新图时不覆盖这些）
    output reg         strip_cfm_tgl,          // 本时钟域：胶片条里 KEY3 短按一次翻转一次（确认）
    // 全屏音乐可视化（sd_card_clk 域电平：音乐层短按 KEY1 进 / 出，回图片层自动退出）
    input  wire        viz_full_async,
    // 定时关机
    input  wire        key1_held_async,        // 板上 KEY1 消抖后的"按住"（50 MHz 域，顶层 key_reset_long 给；定时关机的任意键）
    output wire        sl_mask,                // 本时钟域：屏蔽按键（顶层同步后屏蔽 sd_card_bmp 的按键）
    output wire        sl_tmds_off,            // 本时钟域：关 HDMI 发送端
    output wire        sl_apause,              // 本时钟域：声音暂停
    output wire        sl_sleep,               // 本时钟域：关机淡出 / 关着 / 唤醒等待（sd_card_bmp 停自动轮播）
    output wire        sl_off,                 // 关着（LED）
    output wire        sl_blink,
    // 卡上 SUB.BMP 字幕：sd_card_bmp 来的字幕条 RAM 写口（sub_wclk 域）和读取结果，直接给 osd_engine
    input  wire        sub_wclk,
    input  wire        sub_we,
    input  wire [13:0] sub_waddr,
    input  wire [7:0]  sub_wdata,
    input  wire        sub_ok_async,
    input  wire [9:0]  sub_w,
    input  wire [13:0] sub_top
);
    // 定时关机：sleep_ctrl 例化在 OSD 段里（需要 osd_runtime 的 1 秒节拍），这里是它和菜单、画面之间的连线
    wire [15:0] sl_rem;
    wire        sl_son, sl_warn, sl_clr;
    wire [8:0]  sl_f;
    `include "strip_layout.vh"

    // ---------------- 跨时钟域 ----------------
    wire [2:0] index_s;
    reg  [2:0] index_s_q;
    reg  [2:0] active_index;       // 连续两拍一致才采用，避免跨域采到中间值
    wire       display_valid_pix;
    reg        display_valid_q;
    reg        display_valid_f;    // 与 active_index 一起去抖后的 display_valid
    wire       strip_pix;          // 胶片条开着（和 bank 号同一组去抖）
    reg        strip_q, strip_tgt;
    wire       strip_vis;          // 本帧显示胶片条（转场控制器）
    wire [8:0] strip_f;
    wire       frame_visible;      // 本帧是否显示（转场控制器在帧起点锁存）
    wire [2:0] tr_old, tr_new;
    wire [1:0] tr_gmode;
    wire [2:0] tr_kind;            // 转场种类（0 = 没在转场）
    wire [9:0] tr_r0, tr_r1, tr_c0, tr_c1, tr_v;
    wire [10:0] tr_ndy, tr_ody, tr_ndx, tr_odx;
    wire [8:0] tr_f;
    wire       init_done_pix;

    sync_2ff u_index_sync0 (.clk(pix_clk), .rst_n(pix_rst_n),
                            .async_in(display_index_async[0]), .sync_out(index_s[0]));
    sync_2ff u_index_sync1 (.clk(pix_clk), .rst_n(pix_rst_n),
                            .async_in(display_index_async[1]), .sync_out(index_s[1]));
    sync_2ff u_index_sync2 (.clk(pix_clk), .rst_n(pix_rst_n),
                            .async_in(display_index_async[2]), .sync_out(index_s[2]));
    sync_2ff u_valid_sync  (.clk(pix_clk), .rst_n(pix_rst_n),
                            .async_in(display_valid_async), .sync_out(display_valid_pix));
    sync_2ff u_init_sync   (.clk(pix_clk), .rst_n(pix_rst_n),
                            .async_in(controller_init_done), .sync_out(init_done_pix));
    sync_2ff u_strip_sync  (.clk(pix_clk), .rst_n(pix_rst_n),
                            .async_in(STRIP_EN ? strip_open_async : 1'b0), .sync_out(strip_pix));
    wire [3*PIC_MAX+8:0] strip_info;
    osd_status_sync #(.N(3*PIC_MAX+9)) u_sinfo_sync (.clk(pix_clk), .rst_n(pix_rst_n),
                                            .async_in(strip_info_async), .sync_out(strip_info));

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            index_s_q       <= 3'd0;
            active_index    <= 3'd0;
            display_valid_q <= 1'b0;
            display_valid_f <= 1'b0;
            strip_q         <= 1'b0;
            strip_tgt       <= 1'b0;
        end else begin
            index_s_q       <= index_s;
            display_valid_q <= display_valid_pix;
            strip_q         <= strip_pix;
            if (index_s == index_s_q && display_valid_pix == display_valid_q && strip_pix == strip_q) begin
                active_index    <= index_s;
                display_valid_f <= display_valid_pix;
                strip_tgt       <= strip_pix;
            end
        end
    end

    // ---------------- 视频时序 ----------------
    wire        raw_hs, raw_vs, raw_de;
    wire [11:0] raw_x, raw_y;
    reg         raw_vs_q;

    video_timing_ctrl #(
        .H_ACT(H_ACT), .H_FP(H_FP), .H_SYNC(H_SYNC), .H_BP(H_BP),
        .V_ACT(V_ACT), .V_FP(V_FP), .V_SYNC(V_SYNC), .V_BP(V_BP),
        .ACTIVE_LAST(1)
    ) u_timing (
        .pclk(pix_clk), .rst_n(pix_rst_n),
        .hsync(raw_hs), .vsync(raw_vs), .de(raw_de),
        .x(raw_x), .y(raw_y)
    );

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n)
            raw_vs_q <= 1'b1;
        else
            raw_vs_q <= raw_vs;
    end
    assign frame_start = raw_vs_q && !raw_vs;      // 场同步开始，距有效区还有 35 行

    // ---------------- 转场控制器 ----------------
    video_transition #(
        .H_ACT(H_ACT), .V_ACT(V_ACT), .H_BLANK(H_FP + H_SYNC + H_BP), .V_BLANK(V_FP + V_SYNC + V_BP),
        .TRANS_FRAMES(TRANS_FRAMES), .SLAT(TRANS_SLAT), .WIPE_SH(TRANS_WIPE_SH)
    ) u_transition (
        .clk(pix_clk), .rst_n(pix_rst_n), .frame_start(frame_start),
        .raw_x(raw_x), .raw_y(raw_y),
        .target(active_index), .vis_in(display_valid_f), .sw_async(trans_sw_async), .strip_in(strip_tgt),
        .visible(frame_visible),
        .rd_old(tr_old), .rd_new(tr_new), .g_mode(tr_gmode),
        .g_r0(tr_r0), .g_r1(tr_r1), .g_c0(tr_c0), .g_c1(tr_c1), .g_v(tr_v),
        .g_new_dy(tr_ndy), .g_old_dy(tr_ody), .g_new_dx(tr_ndx), .g_old_dx(tr_odx),
        .f_pix(tr_f), .trans_active(trans_active), .trans_kind(tr_kind), .trans_var(),
        .strip_vis(strip_vis), .strip_f(strip_f)
    );

    // ---------------- 胶片条：格表 + 逐像素 ----------------
    wire [29:0] sp_slots;
    wire [9:0]  sp_sstate;
    wire [20:0] sp_words;
    wire        sp_take, sp_paint, sp_dim;
    wire [23:0] sp_rgb;
    filmstrip #(.H_BLANK(H_FP + H_SYNC + H_BP), .V_BLANK(V_FP + V_SYNC + V_BP), .H_ACT(H_ACT), .V_ACT(V_ACT), .NP(PIC_MAX)) u_strip (
        .clk(pix_clk), .rst_n(pix_rst_n), .frame_start(frame_start),
        .raw_x(raw_x), .raw_y(raw_y), .info(strip_info), .strip_vis(strip_vis),
        .slots(sp_slots), .sstate(sp_sstate), .frame_words(sp_words),
        .take(sp_take), .paint(sp_paint), .paint_rgb(sp_rgb), .dim(sp_dim)
    );
    wire strip_hide = strip_tgt || strip_vis;      // 胶片条开着或还看得见：藏频谱和字幕

    // 正在读的 bank 报给 sd_card_bmp（6 块整帧缓存按需换入，换入时不能覆盖正在显示的块）。
    //   转场中再按键时要"先放完当前转场再转到最后那张"，屏幕上的两张不一定是 sd_card_bmp 记的当前图和上一张，
    //   所以按实际读的报：本帧用到的 bank（转场旧图 / 新图、去抖后的目标）在一帧里累计，帧起点存成"上一帧"，
    //   报 本帧 | 上一帧，这样帧起点前后读侧还在读上一帧最后几个 burst 时也算在里面；
    //   寄存输出，每位按帧保持，sd 侧每位单独两级同步即可。
    wire [5:0] bb_now = (6'd1 << tr_old) | (trans_active ? (6'd1 << tr_new) : 6'd0) | (6'd1 << active_index);
    reg  [5:0] bb_cur, bb_last;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            bb_cur <= 6'd0; bb_last <= 6'd0; bank_busy <= 6'd0;
        end else begin
            if (frame_start) begin
                bb_last <= bb_cur;
                bb_cur  <= bb_now;
            end else
                bb_cur  <= bb_cur | bb_now;
            bank_busy <= bb_cur | bb_last;
        end
    end
    // ---------------- 帧读取器 ----------------
    // SDRAM 初始化完成前不启动读，也不统计下溢（否则上电第一帧必然报下溢）
    wire        reader_frame_start   = frame_start && init_done_pix;
    wire        reader_pixel_consume = raw_de && init_done_pix && sp_take;   // 胶片条行里只在缩略图上取像素
    wire [23:0] reader_rgb;
    wire        reader_pixel_valid;
    wire        reader_underflow;
    wire [7:0]  reader_error;

    media_frame_reader #(
        .ADDR_WIDTH(21), .IMAGE_WORDS(IMAGE_WORDS),
        .FIFO_DEPTH(512), .FIFO_MARGIN(16),
        .BURST_SIZE(128), .BURST_GAP_CYCLES(12), .MAX_INFLIGHT_BURSTS(2),
        .DIAG_RETURN_PATTERN(0), .DIAG_SYNTH_STREAM(0),
        .RGB_IN_HIGH(RGB_IN_HIGH), .LINE_WORDS(H_ACT), .SLAT(TRANS_SLAT),
        .STRIP_EN(STRIP_EN), .STRIP_Y0(SP_Y0), .STRIP_FY0(SP_FY0), .STRIP_SY0(SP_SY0), .STRIP_SH(SP_SH),
        .THUMB_W(SP_TW), .THUMB_SW(SP_SW), .THUMB_H(SP_TH), .THUMB_BASE(THUMB_BASE), .THUMB_WORDS(THUMB_WORDS)
    ) u_reader (
        .mem_clk(mem_clk), .mem_rst_n(mem_rst_n),
        .controller_init_done(controller_init_done),
        .controller_busy(1'b0), .refresh_active(1'b0),
        .app_rd_en(app_rd_en), .app_rd_addr(app_rd_addr),
        .sdram_rd_valid(sdram_rd_valid), .sdram_rd_data(sdram_rd_data),
        .wr_busy(wr_busy), .rd_busy(rd_busy),
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n),
        .frame_start(reader_frame_start), .active_index(tr_old),
        .trans_new_index(tr_new), .trans_gmode(tr_gmode),
        .trans_r0(tr_r0), .trans_r1(tr_r1), .trans_c0(tr_c0), .trans_c1(tr_c1), .trans_v(tr_v),
        .trans_new_dy(tr_ndy), .trans_old_dy(tr_ody), .trans_new_dx(tr_ndx), .trans_old_dx(tr_odx),
        .strip_on(strip_vis), .strip_slots(sp_slots), .frame_words(sp_words),
        .pixel_consume(reader_pixel_consume),
        .pixel_rgb(reader_rgb), .pixel_valid(reader_pixel_valid),
        .underflow(reader_underflow),
        .reader_busy(), .frame_read_done(), .reader_index(),
        .reader_error(reader_error)
    );

    // ---------------- 输出（经转场亮度系数） ----------------
    // 本帧是否显示、显示哪个 bank 都由转场控制器在帧起点同一拍锁存（保证首图整帧出现）
    // 胶片条里的底板 / 金框 / 占位 / 箭头换成常数颜色；胶片条上面的大图亮度再乘 SP_DIM/256（压暗约 30%）
    wire [16:0] tr_fd = tr_f * SP_DIM;
    wire [8:0]  px_f0 = sp_dim ? tr_fd[16:8] : tr_f;
    wire [8:0]  px_f  = (sl_f < px_f0) ? sl_f : px_f0;    // 关机淡出 / 唤醒淡入（平时 sl_f = 256，结果不变）
    wire [23:0] px_in = sp_paint ? sp_rgb : reader_rgb;
    wire [16:0] tr_mr = px_in[23:16] * px_f;
    wire [16:0] tr_mg = px_in[15:8]  * px_f;
    wire [16:0] tr_mb = px_in[7:0]   * px_f;
    wire [23:0] shown_rgb = (frame_visible && (reader_pixel_valid || sp_paint)) ?
                            {tr_mr[15:8], tr_mg[15:8], tr_mb[15:8]} : 24'h000000;

    // ---------------- KEY3（按 SW4 分层，按"按下那一刻"的层算） ----------------
    //   图片层：菜单关着 短按（松手生效，以便与长按区分）= OSD 显示模式，长按 1 秒 = 打开菜单
    //           菜单开着 短按 = 下一项，长按 1 秒 = 恢复默认（按住不连发）——看按下那一刻菜单开没开
    //           胶片条开着 短按 = 确认
    //   音乐层：短按 = 音量 +1 档，长按 1 秒 = 音量 -1 档（按住每 0.5 秒再降一档）
    // 音量 9 档：0 / 12.5% / 25% / ... / 100%（增益 = 档位 x 32），到头不再变；上电 100%
    wire       layer_s;
    wire       k3_press, k3_short, k3_long;   // 经 sl_mask 屏蔽后的事件
    reg        k3_lay_q, k3_men_q, k3_first, k3_str_q;
    reg  [3:0] vol_lvl;
    wire [7:0] vol_osd = (vol_lvl == 4'd8) ? 8'd255 : {vol_lvl[2:0], 5'd0};   // OSD 显示用（255 显示 100%）
    sync_2ff u_layer_sync (.clk(pix_clk), .rst_n(pix_rst_n), .async_in(layer_async), .sync_out(layer_s));
    // 本时钟域的 3 个按键共用一个 1 ms 分频器（省 LUT；节拍与 key_short_long 内部分频器相同）
    localparam integer KMS_CYC = KEY_CLK_HZ / 1000;
    reg  [$clog2(KMS_CYC + 1)-1:0] kms_cnt;
    wire       kms = (kms_cnt == KMS_CYC - 1);
    always @(posedge pix_clk or negedge pix_rst_n)
        if (!pix_rst_n) kms_cnt <= 0;
        else            kms_cnt <= kms ? 0 : kms_cnt + 1'b1;
    wire       k3_press_r, k3_short_r, k3_long_r, k3_held;
    key_short_long #(.CLK_HZ(KEY_CLK_HZ), .DEBOUNCE_MS(20), .LONG_MS(1000), .REPEAT_MS(500), .EXT_MS(1)) u_key3 (
        .clk(pix_clk), .rst_n(pix_rst_n), .ms_in(kms), .key_n(osd_key_n),
        .press(k3_press_r), .short_press(k3_short_r), .long_press(k3_long_r), .held(k3_held), .long_active()
    );
    // 定时关机屏蔽按键时，本模块的按键事件都不算
    assign k3_press = k3_press_r && !sl_mask;
    assign k3_short = k3_short_r && !sl_mask;
    assign k3_long  = k3_long_r  && !sl_mask;
    wire k3_lay    = k3_press ? layer_s : k3_lay_q;
    wire k3_men    = k3_press ? menu_open : k3_men_q;    // 这次按下时菜单开没开
    wire k3_1st    = k3_press ? 1'b1 : k3_first;         // 这次按下还没出过长按
    wire k3_str    = k3_press ? strip_tgt : k3_str_q;    // 这次按下时胶片条开没开
    wire key_press = k3_short && !k3_lay && !k3_men && !k3_str;   // OSD 模式（松手生效）
    wire strip_cfm = k3_short && !k3_lay && k3_str;      // 胶片条里短按 = 确认
    wire vol_up    = k3_short && k3_lay;
    wire vol_dn    = k3_long  && k3_lay;
    // 菜单
    wire menu_on   = k3_long  && !k3_lay && !k3_men && k3_1st && !k3_str; // 打开菜单（胶片条里不能开）
    wire menu_rst  = k3_long  && !k3_lay &&  k3_men && k3_1st && menu_open; // 恢复默认
    wire menu_next = k3_short && !k3_lay &&  k3_men && menu_open;          // 下一项
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            k3_lay_q <= 1'b0; k3_men_q <= 1'b0; k3_first <= 1'b0; k3_str_q <= 1'b0;
            vol_lvl <= 4'd8; vol_gain <= 9'd256;
            strip_cfm_tgl <= 1'b0;
        end else begin
            k3_lay_q <= k3_lay;
            k3_men_q <= k3_men;
            k3_first <= k3_1st && !k3_long;
            k3_str_q <= k3_str;
            if (strip_cfm) strip_cfm_tgl <= ~strip_cfm_tgl;
            if (vol_up && vol_lvl != 4'd8) vol_lvl <= vol_lvl + 4'd1;
            if (vol_dn && vol_lvl != 4'd0) vol_lvl <= vol_lvl - 4'd1;
            vol_gain <= {vol_lvl, 5'd0};
        end
    end

    // ---------------- 画面调节菜单 ----------------
    // KEY2 = +1、KEY4 = -1：按下立刻一档，按住 0.5 秒后每 0.15 秒再一档；只在"按下那一刻菜单开着"时算数
    wire k2_press_r, k2_long_r, k4_press_r, k4_long_r, k2_held, k4_held;
    key_short_long #(.CLK_HZ(KEY_CLK_HZ), .DEBOUNCE_MS(20), .LONG_MS(500), .REPEAT_MS(150), .EXT_MS(1)) u_key_inc (
        .clk(pix_clk), .rst_n(pix_rst_n), .ms_in(kms), .key_n(menu_inc_key_n),
        .press(k2_press_r), .short_press(), .long_press(k2_long_r), .held(k2_held), .long_active()
    );
    key_short_long #(.CLK_HZ(KEY_CLK_HZ), .DEBOUNCE_MS(20), .LONG_MS(500), .REPEAT_MS(150), .EXT_MS(1)) u_key_dec (
        .clk(pix_clk), .rst_n(pix_rst_n), .ms_in(kms), .key_n(menu_dec_key_n),
        .press(k4_press_r), .short_press(), .long_press(k4_long_r), .held(k4_held), .long_active()
    );
    // 定时关机的"任意键" = 已经消抖过的 KEY2/3/4 + KEY1（复用现有消抖结果，省 LUT）
    wire k1_held;
    reg  k1_held_q;
    sync_2ff u_k1_sync (.clk(pix_clk), .rst_n(pix_rst_n), .async_in(key1_held_async), .sync_out(k1_held));
    always @(posedge pix_clk or negedge pix_rst_n)
        if (!pix_rst_n) k1_held_q <= 1'b0;
        else            k1_held_q <= k1_held;
    wire any_press = k3_press_r || k2_press_r || k4_press_r || (k1_held && !k1_held_q);
    wire any_held  = k3_held || k2_held || k4_held || k1_held;
    wire k2_press = k2_press_r && !sl_mask, k2_long = k2_long_r && !sl_mask;
    wire k4_press = k4_press_r && !sl_mask, k4_long = k4_long_r && !sl_mask;
    reg  k2_men_q, k4_men_q;
    wire k2_men = k2_press ? menu_open : k2_men_q;
    wire k4_men = k4_press ? menu_open : k4_men_q;
    wire m_inc  = (k2_press || k2_long) && k2_men && menu_open;
    wire m_dec  = (k4_press || k4_long) && k4_men && menu_open;
    reg  [2:0] msel;                           // 0 亮度 1 对比度 2 饱和度 3 色温 4 轮播 5 定时
    reg  [2:0] slv;                            // 定时档位 0~5（关 1 15 30 60 90 分）
    reg  [3:0] lv_b, lv_k, lv_s, lv_t;         // 0~10，5 = 不调
    reg        rst_tgl;                        // 每恢复默认一次翻转（OSD 提示）
    reg  [8:0] idle;                           // 无操作的帧数（9 位，最多 511 帧）
    wire       m_act = menu_on || menu_rst || menu_next || m_inc || m_dec;
    function [3:0] lv_step;                    // 档位 +1 / -1，到头停住
        input [3:0] v;
        input       up;
        begin
            if (up) lv_step = (v == 4'd10) ? v : v + 4'd1;
            else    lv_step = (v == 4'd0)  ? v : v - 4'd1;
        end
    endfunction
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            menu_open <= 1'b0; msel <= 3'd0; idle <= 9'd0; rst_tgl <= 1'b0;
            lv_b <= 4'd5; lv_k <= 4'd5; lv_s <= 4'd5; lv_t <= 4'd5; per_lvl <= 4'd1;
            k2_men_q <= 1'b0; k4_men_q <= 1'b0; slv <= 3'd0;
        end else begin
            k2_men_q <= k2_men;
            k4_men_q <= k4_men;
            if (menu_on) begin
                menu_open <= 1'b1; msel <= 3'd0;
            end
            if (menu_next) msel <= (msel == 3'd5) ? 3'd0 : msel + 3'd1;
            if (menu_rst) begin
                lv_b <= 4'd5; lv_k <= 4'd5; lv_s <= 4'd5; lv_t <= 4'd5;
                rst_tgl <= ~rst_tgl;
            end else if (m_inc != m_dec) begin         // 同时按两个键：都不算
                case (msel)
                    3'd0: lv_b <= lv_step(lv_b, m_inc);
                    3'd1: lv_k <= lv_step(lv_k, m_inc);
                    3'd2: lv_s <= lv_step(lv_s, m_inc);
                    3'd3: lv_t <= lv_step(lv_t, m_inc);
                    3'd4: per_lvl <= m_inc ? ((per_lvl == 4'd8) ? per_lvl : per_lvl + 4'd1)
                                           : ((per_lvl == 4'd0) ? per_lvl : per_lvl - 4'd1);
                    default: slv <= m_inc ? ((slv == 3'd5) ? slv : slv + 3'd1)       // 定时（恢复默认不动它）
                                          : ((slv == 3'd0) ? slv : slv - 3'd1);
                endcase
            end
            if (sl_clr) slv <= 3'd0;                   // 唤醒后定时变回"关"
            // 自动关闭：每帧起点计一帧，有按键操作清零；拨到音乐层立刻关
            if (m_act)
                idle <= 9'd0;
            else if (frame_start && menu_open) begin
                if (idle == MENU_CLOSE_FRAMES - 1) begin
                    menu_open <= 1'b0; idle <= 9'd0;
                end else
                    idle <= idle + 9'd1;
            end
            if (layer_s || strip_tgt) begin           // 胶片条开着也关菜单
                menu_open <= 1'b0; idle <= 9'd0;
            end
        end
    end
    function [6:0] period_sec;                 // 档位 -> 秒（须与 osd_gen.py 的 PERIODS 一致）；8 随音乐 -> 0（OSD 显示 ♪）
        input [3:0] l;
        begin
            case (l)
                4'd0: period_sec = 7'd2;   4'd1: period_sec = 7'd3;   4'd2: period_sec = 7'd5;
                4'd3: period_sec = 7'd8;   4'd4: period_sec = 7'd10;  4'd5: period_sec = 7'd15;
                4'd6: period_sec = 7'd30;  4'd7: period_sec = 7'd60;  default: period_sec = 7'd0;
            endcase
        end
    endfunction

    // ---------------- 查表调色（亮度 / 对比度 / 色温 + 饱和度） ----------------
    wire [23:0] color_rgb;                     // 调色后的图片（比 shown_rgb 晚 COLOR_LAT 拍）
    wire [11:0] c_x, c_y;                      // 与 color_rgb 对齐的坐标、同步
    wire        c_hs, c_vs, c_de;
    generate
        if (COLOR_ENABLE) begin : g_color
            color_adjust #(.STEP(COLOR_STEP)) u_color (
                .clk(pix_clk), .rst_n(pix_rst_n), .frame_start(frame_start),
                .lv_b(lv_b), .lv_k(lv_k), .lv_t(lv_t), .lv_s(lv_s),
                .rgb_in(shown_rgb), .rgb_out(color_rgb),
                .cur_gb(), .cur_gk(), .cur_gr(), .cur_gbl(), .cur_gs()
            );
            // 坐标、同步延迟 COLOR_LAT = 3 拍
            reg [11:0] x1, x2, x3, y1, y2, y3;
            reg        hs1, hs2, hs3, vs1, vs2, vs3, de1, de2, de3;
            always @(posedge pix_clk or negedge pix_rst_n) begin
                if (!pix_rst_n) begin
                    x1 <= 0; x2 <= 0; x3 <= 0; y1 <= 0; y2 <= 0; y3 <= 0;
                    hs1 <= 1'b1; hs2 <= 1'b1; hs3 <= 1'b1; vs1 <= 1'b1; vs2 <= 1'b1; vs3 <= 1'b1;
                    de1 <= 1'b0; de2 <= 1'b0; de3 <= 1'b0;
                end else begin
                    x1 <= raw_x;  x2 <= x1;  x3 <= x2;  y1 <= raw_y;  y2 <= y1;  y3 <= y2;
                    hs1 <= raw_hs; hs2 <= hs1; hs3 <= hs2; vs1 <= raw_vs; vs2 <= vs1; vs3 <= vs2;
                    de1 <= raw_de; de2 <= de1; de3 <= de2;
                end
            end
            assign c_x = x3; assign c_y = y3; assign c_hs = hs3; assign c_vs = vs3; assign c_de = de3;
        end else begin : g_no_color
            assign color_rgb = shown_rgb;
            assign c_x = raw_x; assign c_y = raw_y; assign c_hs = raw_hs; assign c_vs = raw_vs; assign c_de = raw_de;
        end
    endgenerate

    // ---------------- OSD + 音频可视化 ----------------
    wire [23:0] osd_rgb;
    wire        out_hs, out_vs, out_de;        // 叠加频谱和全屏层后的视频（hs/vs/de 随像素一起延迟）
    wire [23:0] out_rgb;
    generate
        if (OSD_ENABLE) begin : g_osd
            wire [26:0] st;
            wire [36:0] ms;
            wire [23:0] runtime_bcd;
            osd_status_sync #(.N(27)) u_st_sync (.clk(pix_clk), .rst_n(pix_rst_n),
                                                 .async_in(osd_status_async), .sync_out(st));
            osd_status_sync #(.N(37)) u_ms_sync (.clk(pix_clk), .rst_n(pix_rst_n),
                                                 .async_in(mus_status_async), .sync_out(ms));
            wire        sec_tick;
            osd_runtime #(.SEC_CYCLES(OSD_SEC_CYCLES)) u_runtime (.clk(pix_clk), .rst_n(pix_rst_n),
                                                                  .bcd(runtime_bcd), .sec_tick(sec_tick));
            sleep_ctrl #(
                .TICKS(SL_TICKS), .LV1_BCD(SL_LV1_BCD), .FADE_STEP(SL_FADE_STEP),
                .WAKE_HOLD(SL_WAKE_HOLD), .REL_FRAMES(SL_REL_FRAMES)
            ) u_sleep (
                .clk(pix_clk), .rst_n(pix_rst_n), .frame_start(frame_start), .sec_tick(sec_tick),
                .menu_open(menu_open), .slv(slv), .key_press(any_press), .key_held(any_held),
                .rem(sl_rem), .son(sl_son), .warn(sl_warn), .sleep(sl_sleep), .f_sl(sl_f),
                .tmds_off(sl_tmds_off), .audio_pause(sl_apause), .mask(sl_mask), .slv_clr(sl_clr),
                .off(sl_off), .blink(sl_blink)
            );
            wire [8:0] st_sf_eff = (sl_f < strip_f) ? sl_f : strip_f;   // 胶片条上的字跟着画面一起淡出
            osd_engine #(
                .H_BLANK(H_FP + H_SYNC + H_BP), .V_BLANK(V_FP + V_SYNC + V_BP),
                .SUB_X_INIT(OSD_SUB_X_INIT), .SUB_SPEED_P(OSD_SUB_SPEED),
                .TOAST_FRAMES_P(OSD_TOAST_FRAMES)
            ) u_osd (
                .clk(pix_clk), .rst_n(pix_rst_n),
                .raw_x(c_x), .raw_y(c_y), .frame_start(frame_start),
                .st_idx(st[26:20]), .st_live(st[19:13]), .st_auto(st[12]), .st_loading(st[11]),
                .st_err(st[10:7]), .st_bad(st[6:0]),
                .st_vol(vol_osd), .st_time(runtime_bcd),
                .st_wav(ms[34]), .st_midx(ms[33:27]), .st_mcnt(ms[26:20]), .st_mknown(ms[19]),
                .st_mtot(ms[18:3]), .st_merr(ms[2:0]), .st_mel(mus_elapsed),
                .st_layer(layer_s), .st_pause(ms[36]), .st_loop(ms[35]),
                .st_per_s(period_sec(per_lvl)), .st_menu(menu_open), .st_msel(msel),
                .st_mlv({lv_t, lv_s, lv_k, lv_b}), .st_rst(rst_tgl),
                .st_strip(strip_vis), .st_sf(st_sf_eff), .st_sst(sp_sstate), .st_hide(strip_hide),
                .st_slv(slv), .st_srem(sl_rem), .st_son(sl_son), .st_warn(sl_warn), .st_sleep(sl_sleep),
                .mode_next(key_press), .mode(osd_mode),
                .sub_wclk(sub_wclk), .sub_we(sub_we), .sub_waddr(sub_waddr), .sub_wdata(sub_wdata),
                .sub_ok_async(sub_ok_async), .sub_w(sub_w), .sub_top(sub_top),
                .bg_rgb(color_rgb), .out_rgb(osd_rgb)
            );
            // 音频频谱 + VU 表小面板（OSD 模式为"全部"或"只状态"时显示）；
            //   全屏模式时小面板淡出，全屏层（fullscreen_viz）用同一张显示表画 64 根柱 + 波形线
            wire        viz_raw;
            sync_2ff u_viz_sync (.clk(pix_clk), .rst_n(pix_rst_n), .async_in(viz_full_async), .sync_out(viz_raw));
            // 提示框 / 关机时退出全屏层；菜单、胶片条（开着或还看得见）时也暂时让开，关掉后淡入回来
            wire        viz_s = viz_raw && !sl_warn && !sl_sleep && !menu_open && !strip_hide;
            wire [4:0]  fs_ra;
            wire [17:0] fs_rq;
            wire        sp_hs, sp_vs, sp_de;
            wire [23:0] sp_rgb;
            audio_spectrum #(
                .H_BLANK(H_FP + H_SYNC + H_BP), .V_BLANK(V_FP + V_SYNC + V_BP)
            ) u_wave (
                .clk(pix_clk), .rst_n(pix_rst_n), .frame_start(frame_start),
                .show((osd_mode == 2'd0 || osd_mode == 2'd2) && !strip_hide && !viz_s && !sl_sleep),   // 胶片条 / 全屏层 / 关机时藏起来
                .pcm_valid(wave_pcm_valid), .pcm_left(wave_pcm_left), .pcm_right(wave_pcm_right),
                .raw_x(c_x), .raw_y(c_y),
                .hs_in(c_hs), .vs_in(c_vs), .de_in(c_de), .rgb_in(osd_rgb),
                .fs_ra(fs_ra), .fs_rq(fs_rq),
                .hs_out(sp_hs), .vs_out(sp_vs), .de_out(sp_de), .rgb_out(sp_rgb)
            );
            fullscreen_viz #(
                .H_BLANK(H_FP + H_SYNC + H_BP), .V_BLANK(V_FP + V_SYNC + V_BP), .FADE_STEP(FULL_FADE_STEP)
            ) u_full (
                .clk(pix_clk), .rst_n(pix_rst_n), .frame_start(frame_start), .viz(viz_s),
                .pcm_valid(wave_pcm_valid), .pcm_left(wave_pcm_left), .pcm_right(wave_pcm_right),
                .raw_x(c_x), .raw_y(c_y), .fs_ra(fs_ra), .fs_rq(fs_rq),
                .hs_in(sp_hs), .vs_in(sp_vs), .de_in(sp_de), .rgb_in(sp_rgb),
                .hs_out(out_hs), .vs_out(out_vs), .de_out(out_de), .rgb_out(out_rgb)
            );
        end else begin : g_no_osd
            assign osd_rgb = color_rgb;
            assign osd_mode = 2'd3;
            assign sl_rem = 16'd0; assign sl_son = 1'b0; assign sl_warn = 1'b0; assign sl_clr = 1'b0; assign sl_f = 9'd256;
            assign sl_mask = 1'b0; assign sl_tmds_off = 1'b0; assign sl_apause = 1'b0; assign sl_sleep = 1'b0;
            assign sl_off = 1'b0; assign sl_blink = 1'b0;
            assign out_hs = c_hs; assign out_vs = c_vs; assign out_de = c_de;
            assign out_rgb = osd_rgb;
        end
    endgenerate

    rgb_video_adapter u_video_adapter (
        .pclk(pix_clk), .rst_n(pix_rst_n),
        .hs_in(out_hs), .vs_in(out_vs), .de_in(out_de), .rgb_in(out_rgb),
        .video_clk(),
        .video_hs(video_hs), .video_vs(video_vs),
        .video_de(video_de), .video_rgb(video_rgb)
    );

    // ---------------- 错误码（粘滞，只记第一个） ----------------
    reg [7:0] reader_error_pix1, reader_error_pix2;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            reader_error_pix1 <= 8'd0;
            reader_error_pix2 <= 8'd0;
            video_error       <= 4'd0;
        end else begin
            reader_error_pix1 <= reader_error;
            reader_error_pix2 <= reader_error_pix1;
            if (video_error == 4'd0) begin
                if (reader_underflow)
                    video_error <= 4'd3;
                else if (reader_error_pix2 == 8'h61)
                    video_error <= 4'd1;
                else if (reader_error_pix2 == 8'h62)
                    video_error <= 4'd2;
                else if (reader_error_pix2 == 8'h64)
                    video_error <= 4'd4;
            end
        end
    end
endmodule
