// =============================================================================
// top：插卡即播数字广告牌系统（安路 EG4S20）
//   TF 卡（SPI，FAT32）读取根目录 PIC1~PIC99.BMP / *.WAV / SUB.BMP
//   -> 缩放到 640x480 -> SDRAM 6 块整帧缓存（按需换入、后台预载）+ 112x84 缩略图
//   -> 转场 / 调色 / OSD / 字幕 / 频谱 -> 自写 HDMI 编码（含 48kHz 音频数据岛）-> 640x480@60
// 时钟（1 个 PLL）：clk 50MHz（SD 卡与控制逻辑）、ext_mem_clk 40MHz（SDRAM）、
//   video_clk 25MHz（像素与音频）、hdmi_5x_clk 125MHz（TMDS 串行）。
// 按键分层由 SW4（sw[3]）决定：0 = 图片层，1 = 音乐层（各键功能见端口注释）。
// PIC_MAX 一个数控制图片张数上限（1~16）。
// =============================================================================
module top(
    input                       clk,
    input                       rst_n,          // 板上 KEY1：短按 = 胶片条进 / 出（音乐层 = 全屏音乐可视化开 / 关），按住 3 秒 = 系统复位（见 key_reset_long.v）
    input                       key1,           // 板上 KEY2：图片层 下一张；音乐层 短按 下一首 / 长按 上一首；菜单里 +1
    input                       key2,           // 板上 KEY4：图片层 短按 自动轮播开/关、长按 上一张；音乐层 短按 暂停 / 长按 单曲循环；菜单里 -1
    input                       key3,           // 板上 KEY3：图片层 短按 OSD 显示模式 / 长按 打开画面调节菜单；音乐层 短按 音量+ / 长按 音量-
    output [3:0]                led,            // LED1 下一张已缓存 / LED2 上一张已缓存 / LED3 后台加载完成 / LED4 正在加载
    input  [3:0]                sw,             // 拨码 {SW3,SW2,SW1} 选转场（0=直接切换）；SW4(sw[3]) = 按键层（0 图片 / 1 音乐）

    output [7:0]                seg_sel,        // 8 位数码管位选，[0] = 左起第 1 个
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
// SDRAM 地址分配（32 位字，共 2M 字）：
//   整帧缓存 6 块：bank k 基地址 = k * 307200（bank 0~5，共 1843200 字），哪张图放哪块由 sd_card_bmp 按需分配；
//   缩略图区：1843200 起，第 k 张图的缩略图 = 1843200 + k * 10752
// 地址在 frame_write_path(写) 和 media_frame_reader(读) 两处使用，两边必须一致

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data;
wire        display_valid;

wire [6:0]  state_code;      // 当前第几张图 1~99（0 = 没有）
wire        menu_open;           // 画面调节菜单开着（video_clk 域）
wire [3:0]  per_lvl;             // 轮播周期档位 0~8（8 = 随音乐；video_clk 域）
wire [3:0]  fs_err_code;     // FAT32细分错误码（0=熄灭, 加载1~5/扫描6~8）
wire [6:0]  dead_count;     // 坏图张数 0~99（重扫清零）

// 显示通路（读）与写通路之间的读写互斥（SDRAM 控制器只有一个口）
wire        mem_rd_busy;
wire        mem_wr_busy;
wire [3:0]  video_err_code;      // 显示通路错误码，0=正常（见 media_display_path.v）

// 板上合成琶音本身的音量（总音量由音乐层 KEY3 调，见 media_display_path）
localparam [7:0] AUDIO_VOLUME = 8'd255;

// WAV 音频播放
wire        ring_we, ring_flush, ring_flush_busy, ring_eos, ring_space_ok, ring_empty, ring_overflow;
wire [31:0] ring_wdata, ring_rdata;
wire [12:0] ring_level;
wire        ring_pop, ring_flush_pend, ring_flush_do, ring_eos_r;
wire        wav_active;
wire [6:0]  mus_idx, mus_cnt;
wire        mus_tot_known;
wire [15:0] mus_tot_bcd;
wire [2:0]  mus_err;
// 卡上 SUB.BMP 字幕：sd_card_bmp（sd_card_clk 域）-> osd_engine 的字幕条 RAM
wire        sub_we, sub_ok;
wire [13:0] sub_waddr, sub_top;
wire [7:0]  sub_wdata;
wire [9:0]  sub_w;
wire        aud_pause;           // 暂停（sd_card_clk 域电平）
wire        mus_loop;            // 单曲循环
wire        pcm_tap_valid;       // 送进 HDMI 的音频样本（video_clk 域，给频谱显示）
wire signed [15:0] pcm_tap_left, pcm_tap_right;
wire [15:0] mus_elapsed;
wire [8:0]  vol_gain;

wire [6:0]  osd_live_cnt;          // 好图张数 0~99
wire        osd_auto;
wire        osd_loading;
wire [3:0]  loaded_banks;          // sd_card_clk 域，直接驱动 LED（LED 是慢信号，不需要同步）
wire        strip_open;            // 胶片条开着（sd_card_clk 域）
wire        viz_full;              // 全屏音乐可视化（sd_card_clk 域）
localparam integer PIC_MAX = 12;   // 图片最多几张（1~16，改这里一个数即可；取 12 以节省资源）
wire [3*PIC_MAX+8:0] strip_info;   //   {缩略图翻转, 有缩略图, 坏图, 张数, 焦点}（sd_card_clk 域）
wire        strip_cfm_tgl;         //   胶片条里 KEY3 短按（video_clk 域翻转电平）
// 定时关机（video_clk 域，见 display/sleep_ctrl.v）
wire        sl_mask, sl_tmds_off, sl_apause, sl_sleep, sl_off, sl_blink;
wire        sl_mask_sd;            //   同步到 sd_card_clk 域：屏蔽 sd_card_bmp 的 KEY1 短按、KEY2、KEY4

// LED 高电平点亮。LED1 下一张已缓存 / LED2 上一张已缓存 / LED3 后台加载完成 / LED4 正在读卡加载
// 定时关机关着时 LED1~3 灭、LED4 慢闪（每秒亮灭一次）
assign led = sl_off ? {sl_blink, 3'b000} : loaded_banks;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire [8:0]  sd_card_write_used;     // 写 FIFO 已用字数（sd_card_clk 域）
wire        sd_card_write_vflip;    // 本张图是否上下翻转
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [2:0]  write_buf_idx;     // 正在加载的图写入的 bank（0~5，加载时由 sd_card_bmp 挑选）
wire [3:0]  write_thumb_idx;   // 正在加载的图片序号 = 缩略图格号
wire [5:0]  bank_busy;         // 显示通路本帧 / 上一帧读过的 bank（像素时钟域），换入新图时不覆盖
wire [2:0]  disp_buf_idx;      // 正在显示的 bank

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

// 音频全部在 video_clk(25MHz) 域生成，无需音频 PLL / I2S / 厂商 HDMI IP
wire [2:0]  media_id_sd;         // {换图翻转位, 旋律号}
wire [2:0]  media_id_s;          // media_id 逐位两级同步到 video_clk 域
reg  [2:0]  media_id_s_q;
reg  [2:0]  media_id_pix;        // 连续两拍一致才采用，避免多位跨域采到中间值
wire [9:0]  hdmi_video_x;        // "有效区在前"的行坐标，给音频桥用
wire        audio_seq_error;     // 数据岛时序错误（粘滞）
wire        audio_tone_overflow; // 音源FIFO溢出（粘滞）
wire        audio_pcm_error;     // PCM握手违约（粘滞）
// 音频错误码，0=正常；4=数据岛时序错 2=音源溢出 1=PCM握手错（可叠加）
wire [3:0]  audio_err_code = {1'b0, audio_seq_error, audio_tone_overflow, audio_pcm_error};

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// ==== 板上 KEY1 一键两用 ====
//   短按（松手生效）= key_strip 单拍（板载 50MHz = sd_card_clk 域）；按住满 3 秒 = 复位，一直保持到松手
wire key_strip;
wire key1_long_rst;
wire key1_held;            // KEY1 消抖后的"按住"，给定时关机当任意键
key_reset_long #(.CLK_HZ(50_000_000), .LONG_MS(3000)) u_key1 (
    .clk         (clk),
    .key_n       (rst_n),
    .short_press (key_strip),
    .held        (key1_held),
    .sys_rst     (key1_long_rst)
);
wire sys_rst_n = ~key1_long_rst;

// ==== 复位源 + 每个时钟域一个复位同步器 ====
// 复位源：KEY1 没有长按复位 且 PLL锁定（未锁定时保持复位）
wire sys_pll_lock;
wire ext_rst_n = sys_rst_n & sys_pll_lock;   // 全系统只有 sys_pll_promax_40 一个 PLL

wire sdcard_rst_n;   // sd_card_clk 域（板载 50MHz）
wire mem_rst_n;      // ext_mem_clk 域（SDRAM）
wire video_rst_n;    // video_clk 域（25MHz，音频也在这个域）

// 异步复位、同步释放：复位按下立刻生效，释放经过两级触发器同步，消除亚稳态
reset_sync u_sdcard_reset (
    .clk        (sd_card_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (sdcard_rst_n)
);
reset_sync u_mem_reset (
    .clk        (ext_mem_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (mem_rst_n)
);
reset_sync u_video_reset (
    .clk        (video_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (video_rst_n)
);

// 定时关机的按键屏蔽同步到 sd_card_clk 域
sync_2ff u_sl_mask_sync (
    .clk        (sd_card_clk),
    .rst_n      (sdcard_rst_n),
    .async_in   (sl_mask),
    .sync_out   (sl_mask_sd)
);

// Sdr_init_done 在 ext_mem_clk 域，同步到 sd_card_clk 域给 sd_card_bmp 用
wire Sdr_init_done_sd;
sync_2ff u_sdram_init_sync (
    .clk        (sd_card_clk),
    .rst_n      (sdcard_rst_n),
    .async_in   (Sdr_init_done),
    .sync_out   (Sdr_init_done_sd)
);
// ==== 复位与同步结束 ====

// TF/SD 逻辑直接用板载 50MHz（promax PLL 没有 100MHz 输出）
assign sd_card_clk = clk;

// 一个 PLL 出 SDRAM 40MHz(+144°)、像素 25MHz、串行 125MHz
sys_pll_promax_40 sys_pll_m0(
    .refclk     (clk),
    .reset      (~sys_rst_n),
    .clk0_out   (),                 // 125MHz 0°，PLL 内部反馈用
    .clk1_out   (ext_mem_clk),      // 40MHz 0°
    .clk2_out   (ext_mem_clk_sft),  // 40MHz 144°
    .clk3_out   (video_clk),        // 25MHz 0°
    .clk4_out   (hdmi_5x_clk),      // 125MHz 90°
    .locked     (sys_pll_lock)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or negedge mem_rst_n) begin
    if (!mem_rst_n)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 卡扫描、图片 / WAV / 字幕读取与缓存调度 =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (50_000_000),
    .SCAN_START_SECTOR (32'd0),           // SCAN_* 参数已不使用，保留为兼容例化
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4),
    .SECTOR_TIMEOUT_CYCLES (32'd25_000_000),  // 500ms@50MHz
    .BMP_MAX           (PIC_MAX)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (~sdcard_rst_n),   // 本时钟域同步复位（模块高有效，取反）
    .Sdr_init_done     (Sdr_init_done_sd), // SDRAM初始化完成才启动扫描
    .key_next          (key1 | sl_mask_sd),  // 定时关机屏蔽按键时当作没按
    .key_auto          (key2 | sl_mask_sd),
    .layer_async       (sw[3]),             // SW4 按键分层
    .menu_async        (menu_open),         // 菜单开着时 KEY2/KEY4 归菜单
    .per_lvl_async     (per_lvl),           // 轮播周期档位
    .key_strip         (key_strip & ~sl_mask_sd), // KEY1 短按（屏蔽时不算）
    .sleep_async       (sl_sleep),          // 关机时不自动轮播
    .strip_cfm_async   (strip_cfm_tgl),     // 胶片条里 KEY3 确认
    .strip_open        (strip_open),
    .viz_full          (viz_full),
    .strip_info        (strip_info),
    .aud_pause         (aud_pause),
    .mus_loop          (mus_loop),
    .state_code        (state_code),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),
    .media_id          (media_id_sd),
    .fs_err_code       (fs_err_code),       // FAT32细分错误码 -> 数码管
    .dead_count        (dead_count),        // 坏图张数 -> OSD 状态条
    .osd_live_cnt      (osd_live_cnt),      // OSD 状态条 / 数码管图片总数
    .osd_auto          (osd_auto),
    .osd_loading       (osd_loading),
    .loaded_banks      (loaded_banks),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .write_thumb_idx   (write_thumb_idx),
    .bank_busy_async   (bank_busy),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .write_used        (sd_card_write_used),  // 写 FIFO 已用字数 -> 缩放模块控制输出速度
    .write_v_flip      (sd_card_write_vflip), // 本张图是否上下翻转（倒存的 BMP 不翻）
    // WAV
    .ring_we           (ring_we),
    .ring_wdata        (ring_wdata),
    .ring_space_ok     (ring_space_ok),
    .ring_empty        (ring_empty),
    .ring_flush        (ring_flush),
    .ring_flush_busy   (ring_flush_busy),
    .ring_eos          (ring_eos),
    .wav_active        (wav_active),
    .mus_idx           (mus_idx),
    .mus_cnt           (mus_cnt),
    .mus_tot_known     (mus_tot_known),
    .mus_tot_bcd       (mus_tot_bcd),
    .mus_err           (mus_err),
    .sub_we            (sub_we),
    .sub_waddr         (sub_waddr),
    .sub_wdata         (sub_wdata),
    .sub_ok            (sub_ok),
    .sub_w             (sub_w),
    .sub_top           (sub_top),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

// 8 位数码管（SW6 拨上才显示）：1-2 图片号 / 3-4 图片总数 / 5-6 歌曲号 / 7 错误码 / 8 状态 L/A/P，见 seg_scan.v
seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (sys_rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .pic_idx           (state_code),
    .pic_total         (osd_live_cnt),
    .song_on           (wav_active),
    .song_idx          (mus_idx),
    .fs_err            (fs_err_code),
    .video_err         (video_err_code),
    .audio_err         (audio_err_code),
    .mus_err           (mus_err),
    .loading           (osd_loading),
    .auto_play         (osd_auto),
    .paused            (aud_pause)
);

// ===================== 显示通路 + 写通路 =====================
// 显示：media_display_path（视频时序 + SDRAM 帧读取 + 转场/调色/OSD/频谱 + 输出寄存），显示 disp_buf_idx 号 bank。
// 写入：frame_write_path（写 FIFO 跨域 + 256 字写 burst + BMP 垂直翻转 + 缩略图），与读侧互斥。
media_display_path #(
    .IMAGE_WORDS       (307200),
    .RGB_IN_HIGH       (1),              // 像素字格式 {RGB, 8'h00}
    .PIC_MAX           (PIC_MAX)
) u_display_path (
    .mem_clk             (ext_mem_clk),
    .mem_rst_n           (mem_rst_n),
    .controller_init_done(Sdr_init_done),
    .app_rd_en           (App_rd_en),
    .app_rd_addr         (App_rd_addr),
    .sdram_rd_valid      (Sdr_rd_en),
    .sdram_rd_data       (Sdr_rd_dout),
    .wr_busy             (mem_wr_busy),
    .rd_busy             (mem_rd_busy),

    .pix_clk             (video_clk),
    .pix_rst_n           (video_rst_n),
    .display_index_async (disp_buf_idx),
    .display_valid_async (display_valid),  // 首图提交前黑屏
    .video_hs            (hs),
    .video_vs            (vs),
    .video_de            (de),
    .video_rgb           (vout_data),
    .frame_start         (),
    .video_error         (video_err_code),
    // OSD
    .osd_status_async    ({state_code, osd_live_cnt, osd_auto, osd_loading, fs_err_code, dead_count}),
    .osd_key_n           (key3),
    .osd_mode            (),
    // WAV：音乐状态 + 音量
    .mus_status_async    ({aud_pause, mus_loop, wav_active, mus_idx, mus_cnt, mus_tot_known, mus_tot_bcd, mus_err}),
    .mus_elapsed         (mus_elapsed),
    .vol_gain            (vol_gain),
    // 转场
    .trans_sw_async      (sw[2:0]),
    // SW4 分层（KEY3：图片层 = OSD 模式 / 菜单，音乐层 = 音量 +/-）
    .layer_async         (sw[3]),
    // 画面调节菜单：KEY2(+1) / KEY4(-1) 也接到显示通路；菜单状态、轮播周期送给 sd_card_bmp
    .menu_inc_key_n      (key1),
    .menu_dec_key_n      (key2),
    .menu_open           (menu_open),
    .per_lvl             (per_lvl),
    // 音频可视化（HDMI 实际送出的样本）
    .wave_pcm_valid      (pcm_tap_valid),
    .wave_pcm_left       (pcm_tap_left),
    .wave_pcm_right      (pcm_tap_right),
    .trans_active        (),
    // 胶片条 / 全屏可视化 / 缓存占用
    .strip_open_async    (strip_open),
    .viz_full_async      (viz_full),
    .strip_info_async    (strip_info),
    .bank_busy           (bank_busy),
    .strip_cfm_tgl       (strip_cfm_tgl),
    // 定时关机
    .key1_held_async     (key1_held),        // 板上 KEY1 也算"任意键"（长按 3 秒复位不受影响，在 key_reset_long 里）
    .sl_mask             (sl_mask),
    .sl_tmds_off         (sl_tmds_off),
    .sl_apause           (sl_apause),
    .sl_sleep            (sl_sleep),
    .sl_off              (sl_off),
    .sl_blink            (sl_blink),
    .sub_wclk            (sd_card_clk),
    .sub_we              (sub_we),
    .sub_waddr           (sub_waddr),
    .sub_wdata           (sub_wdata),
    .sub_ok_async        (sub_ok),
    .sub_w               (sub_w),
    .sub_top             (sub_top)
);

// 写整帧的同时缩出 112x84 缩略图（行距 128 字），第 k 张图（序号 0~15）的缩略图放在 1843200 + k x 10752，
//   最多到 2015232（SDRAM 2097152 字以内）
frame_write_path #(
    .WRITE_V_FLIP      (1),
    .FRAME_WIDTH       (640),
    .FRAME_HEIGHT      (480),
    .THUMB_W           (112),
    .THUMB_H           (84),
    .THUMB_STRIDE      (128),
    .THUMB_BASE        (1843200)
) u_frame_write_path (
    .rst               (~mem_rst_n),     // ext_mem_clk域同步复位
    .mem_clk           (ext_mem_clk),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .rd_busy           (mem_rd_busy),
    .wr_busy           (mem_wr_busy),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_index  (write_buf_idx),  // bank k 基地址 = k x 307200（frame_fifo_write 里算）
    .write_thumb_index (write_thumb_idx),// 缩略图按图片序号存放
    .write_v_flip      (sd_card_write_vflip),
    .write_used        (sd_card_write_used),
    .write_len         (21'd307200),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (~mem_rst_n),     // ext_mem_clk域同步复位
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频（全部在 video_clk 域） =====================
// pcm_media_tone 对 25MHz 做分数累加得到精确 48kHz 采样，ACR 固定 N=6144 / CTS=25000，
// 二者都以像素时钟为基准，不需要音频PLL，也没有音频跨时钟域。

// media_id（sd_card_clk 域）-> video_clk 域
sync_2ff u_media_id_sync0 (
    .clk        (video_clk),
    .rst_n      (video_rst_n),
    .async_in   (media_id_sd[0]),
    .sync_out   (media_id_s[0])
);
sync_2ff u_media_id_sync1 (
    .clk        (video_clk),
    .rst_n      (video_rst_n),
    .async_in   (media_id_sd[1]),
    .sync_out   (media_id_s[1])
);
sync_2ff u_media_id_sync2 (
    .clk        (video_clk),
    .rst_n      (video_rst_n),
    .async_in   (media_id_sd[2]),
    .sync_out   (media_id_s[2])
);
always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        media_id_s_q <= 3'd0;
        media_id_pix <= 3'd0;
    end else begin
        media_id_s_q <= media_id_s;
        if (media_id_s == media_id_s_q)
            media_id_pix <= media_id_s;
    end
end

// 本工程时序"有效区在后"，symbol_core 要"有效区在前"的坐标，这里转换（与 hs/vs/de 同拍）
hdmi_video_x_align u_hdmi_video_x_align (
    .clk        (video_clk),
    .rst_n      (video_rst_n),
    .vs         (vs),
    .de         (de),
    .video_x    (hdmi_video_x)
);

// ===================== WAV 环形缓冲（写 sd_card_clk，读 video_clk） =====================
wav_ring #(.AW(12)) u_wav_ring (
    .wclk          (sd_card_clk),
    .wrst_n        (sdcard_rst_n),
    .we            (ring_we),
    .wdata         (ring_wdata),
    .space_ok      (ring_space_ok),
    .empty_w       (ring_empty),
    .flush         (ring_flush),
    .flush_busy    (ring_flush_busy),
    .eos_w         (ring_eos),
    .overflow      (ring_overflow),
    .rclk          (video_clk),
    .rrst_n        (video_rst_n),
    .level_r       (ring_level),
    .pop           (ring_pop),
    .rdata         (ring_rdata),
    .flush_pend_r  (ring_flush_pend),
    .flush_do      (ring_flush_do),
    .eos_r         (ring_eos_r)
);

// ===================== 带音频的 HDMI 符号（TMDS 编码 + 数据岛：音频采样包 / ACR / InfoFrame） =====================
media_hdmi_audio_bridge u_hdmi_audio_bridge (
    .pixel_clk          (video_clk),
    .pixel_rst_n        (video_rst_n),
    .video_hsync        (hs),
    .video_vsync        (vs),
    .video_de           (de),
    .video_x            (hdmi_video_x),
    .video_rgb          (vout_data),
    .media_id           (media_id_pix),      // 无 WAV 时的琶音：按图片选旋律，切图有提示音
    .volume             (AUDIO_VOLUME),
    .channel0_symbol    (tmds_ch0_data),
    .channel1_symbol    (tmds_ch1_data),
    .channel2_symbol    (tmds_ch2_data),
    .sequence_error     (audio_seq_error),
    .tone_overflow      (audio_tone_overflow),
    .pcm_contract_error (audio_pcm_error),
    .blip_active        (),
    .pcm_tap_valid      (pcm_tap_valid),     // 送给音频可视化
    .pcm_tap_left       (pcm_tap_left),
    .pcm_tap_right      (pcm_tap_right),
    // WAV
    .wav_active_async   (wav_active),
    .vol_gain           (vol_gain),
    .pause_async        (aud_pause | sl_apause), // 暂停（定时关机时也暂停，此时 OSD 不显示暂停符号）
    .ring_level         (ring_level),
    .ring_pop           (ring_pop),
    .ring_data          (ring_rdata),
    .ring_flush_pend    (ring_flush_pend),
    .ring_flush_do      (ring_flush_do),
    .ring_eos           (ring_eos_r),
    .elapsed_bcd        (mus_elapsed),
    .wav_underruns      ()
);

// TMDS 时钟通道：每个像素周期 5 个 1 + 5 个 0
assign tmds_clk_data = 10'b1111100000;

// 不读 EDID：DDC 两根线释放为高阻（管脚约束里已有上拉）
assign HDMI_DDC_SCL = 1'bz;
assign HDMI_DDC_SDA = 1'bz;

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (~video_rst_n | sl_tmds_off),   // HDMI 相关模块统一用 video_clk 域同步复位；定时关机时保持复位 = 没有 TMDS 信号
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule