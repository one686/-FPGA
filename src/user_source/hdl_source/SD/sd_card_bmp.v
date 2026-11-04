
// ===========================================================================
// sd_card_bmp.v —— TF 卡媒体管理：FAT32 扫描、图片加载与缓存调度、按键导航、音乐读取
//
// 功能概述：
//   1) SDRAM 初始化完成后扫描 FAT32 卷（fat32_volume_scanner）：根目录 PIC1~PIC99.BMP
//      （号码可补零、可不连续）按号码排序，扫描时先检查文件头，取号码最小的 BMP_MAX 张合格图
//      组成图片表；同时登记根目录 .WAV 歌单（最多 12 首）和 SUB.BMP。
//   2) 卡上有 SUB.BMP 时先把它读进字幕条 RAM（sub_loader），然后开始加载图片。
//   3) 图片加载：fat32_file_streamer 按簇链读文件 -> bmp_scaler（任意尺寸、24/32 位、正存/倒存，
//      等比缩放成 IMG_W x IMG_H 并留黑边）-> 写 FIFO -> SDRAM。
//   4) SDRAM 中 6 块整帧缓存按需换入（见"缓存"一节），首图显示后在后台预载；缩略图按图片序号存放。
//   5) 第二个 fat32_file_streamer 读 WAV，wav_sd_ctrl 解析 WAV 头并写环形缓冲；
//      读图、读歌两路按扇区仲裁共用一个 sd_sector_adapter（读歌优先）。
//
// 按键（每次按键按"按下那一刻"的 SW4 层 / 菜单状态归属，按住时拨 SW4 不会串层）：
//   key_next = 板上 KEY2，key_auto = 板上 KEY4，key_strip = 板上 KEY1 短按（顶层 key_reset_long 给出）
//   SW4=0 图片层：KEY2 按下 = 下一张；KEY4 短按 = 自动轮播开/关，长按 = 上一张；KEY1 短按 = 胶片条
//   SW4=1 音乐层：KEY2 短按 = 下一首，长按 = 上一首（按住每 0.5 秒再上一首，松手开始播）；
//                 KEY4 短按 = 暂停/继续（换歌、重扫时自动继续），长按 = 单曲循环/列表循环；
//                 KEY1 短按 = 全屏音乐可视化开/关（viz_full）
//   menu_async = 1（画面调节菜单开着，菜单在 media_display_path）时 KEY2/KEY4 归菜单，这里不响应。
//   卡出错（need_rescan）或一张好图都没有时，图片层 KEY2 改为触发"复位 SD + 重扫"。
//
// 自动轮播：per_lvl_async = 周期档位 0~8（2 3 5 8 10 15 30 60 秒 / 随音乐），用"1 毫秒节拍 + 毫秒计数"计时，
//   周期一变计时从头开始。AUTO_PLAY_MS = 档位 1（默认 3 秒）的毫秒数，其他档位按比例（仿真可压缩）。
//   胶片条开着、定时关机（sleep_async）时不计时。
//   档位 8 = 随音乐：在播 WAV 时一首歌正常播完（u_wav_ctrl 的 song_end）换下一张，不按时间；
//     没有 WAV（琶音）时按 3 秒（同档位 1）轮播。song_end 先锁存（song_end_pend），正在换图时不会丢。
//
// 胶片条（strip_open；画面由 media_display_path / filmstrip 绘制）：
//   图片层短按 KEY1 打开，记下当前这张（strip_orig）；胶片条里：
//     KEY2 按下 = 下一张，KEY4 按下 = 上一张；大图跟着焦点切（已缓存的立刻切，没缓存的读完再切），
//       推进中也收新按键，以最新一次为准（正在读的上一个目标按"按键插队"中止）；
//     KEY3 短按（strip_cfm_async 翻转）= 确认，收起，停在焦点这张；
//     短按 KEY1 或拨到音乐层 = 取消，收起，回到 strip_orig（它一直受保护留在缓存里，和收起同一拍换回去，
//       显示通路只播一次转场）。
//   黑屏（重扫等）时自动收起。
//   strip_info = {缩略图翻转[NP], 有缩略图[NP], 坏图[NP], 张数[4:0], 焦点[3:0]}（3 x NP + 9 位，NP = BMP_MAX）；
//   缩略图翻转 = 这张 BMP 是正存的（写侧整帧翻转，缩略图按文件行序存，显示时要倒着读），整帧写完时记下。
//
// 状态输出：
//   state_code   第几张好图（推进中 = 目标），0 = 没有好图（数码管 / OSD）
//   osd_live_cnt 好图总张数
//   fs_err_code  细分错误码（0 = 熄灭）：
//      加载阶段: 0x10 扇区读错/超时→1  0x23 FAT 环→2  0x24 簇越界→3  0x25 提前 EOF→4  0x26 遍历超限→5
//                文件截断 / BMP 头不合格→4；加载错误 ERR_SHOW_MS 后自动熄灭
//      扫描阶段: 0x20 无 FAT32 分区→6  0x21 BPB 不支持→7  0x22 找不到图片→8；保持到恢复成功
//   dead_count   坏图张数 = 扫描时文件头不合格的 + 加载时发现的；重扫清零；卡级故障（0x10）不计入
//   loaded_banks LED：[0] 下一张已缓存 [1] 上一张已缓存 [2] 后台该读的都读完了 [3] 正在加载
//   media_id     {换图翻转位, (第 n 张 - 1) mod 4}，音频据此选旋律、每次换图响提示音
//
// 坏图与出错处理：
//   文件级错误（0x23~0x26 / 截断 / BMP 头不合格）只把该图标记为坏图，导航自动跳过它：
//     已有图在显示 -> 停在当前好图（一次按键只试一张，最多报一次错）；
//     还没显示过图（上电启动）-> 保持黑屏，自动去下一张好图，直到出图或全部是坏图；
//     后台预载遇到坏图 -> 只标记，画面和按键都不受影响。
//   0x10 属卡级故障（拔卡 / 读超时），不标记坏图，置 need_rescan（图片列表不可信）：
//     黑屏时每 AUTO_RESCAN_MS 自动"复位 SD + 重扫"，插回卡即自动出图；已在显示时等按 KEY2 触发。
//   双看门狗：加载 LOAD_WDT_MS 没有进展 / 请求挂起 PEND_WDT_MS 无法启动 -> 中止，并发 load_recover
//     让顶层复位图像链路。
//
// 未使用的端口 / 参数：SCAN_START_SECTOR / SCAN_MAX_SECTOR / SCAN_TARGET_COUNT 与 bmp_width / bmp_height
//   不再使用（图片按文件名查找，尺寸由 IMG_W / IMG_H 决定），保留端口只为兼容顶层例化。
// 时钟域：clk = SD/控制时钟（板上 50MHz）；layer / menu / per_lvl / strip_cfm / sleep / bank_busy
//   为其他时钟域或异步输入，在本模块内同步。
// ===========================================================================
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,      // 不再使用(图片按文件名查找), 保留仅为兼容顶层例化
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071, // 同上, 保留兼容
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,       // 同上, 保留兼容
    parameter [31:0]  SECTOR_TIMEOUT_CYCLES = 32'd50_000_000, // 扇区读超时拍数(顶层按 50MHz 给 500ms), 防拔卡挂死
     // === 长按时间、轮播周期和各计时器（毫秒）===
    parameter integer LONG_PRESS_MS      = 1000,   // KEY2/KEY4 长按触发时间，默认1秒
    parameter integer AUTO_PLAY_MS       = 3000,   // 自动轮播间隔，默认3秒（= 档位 1 的毫秒数，其他档位按比例）
    parameter integer LOAD_WDT_MS        = 3000,   // 加载看门狗：一帧图加载超时时间
    parameter integer PEND_WDT_MS        = 3000,   // 请求看门狗：按键请求挂起超时时间
    parameter integer GAP_MS             = 10,     // 帧间隔：提交后至少隔这么久才允许启动下一次加载
    parameter integer ERR_SHOW_MS        = 5000,   // 细分码显示时长（毫秒），到时间自动熄灭
    parameter integer SD_RST_MS          = 10,     // 拔卡恢复时复位SD控制器的脉宽（毫秒）
    parameter integer AUTO_RESCAN_MS     = 1000,   // 还没显示过图时卡出错，每隔这么久自动"复位SD+重扫"
    parameter integer IMG_W              = 640,    // 缩放后的图片尺寸（= SDRAM 里一帧）
    parameter integer IMG_H              = 480,
    parameter integer BMP_MAX            = 12       // 图片最多几张（给扫描器；1~16，顶层 PIC_MAX 给 12）
)(
    input                       clk,
    input                       rst,
    input                       Sdr_init_done,  // SDRAM初始化完成才允许启动扫描（顶层经sync_2ff同步到本时钟域）
    input                       key_next,
    input                       key_auto,
    input                       layer_async,    // SW4（0 = 图片层，1 = 音乐层），异步
    input                       menu_async,     // 画面调节菜单开着（video_clk 域电平），开着时不响应 KEY2/KEY4
    input      [3:0]            per_lvl_async,  // 轮播周期档位 0~8（8 = 随音乐；video_clk 域）
    input                       key_strip,      // KEY1 短按（本时钟域单拍，顶层 key_reset_long 给）
    input                       strip_cfm_async,// 胶片条里 KEY3 短按（像素时钟域，每按一次翻转一次）
    input                       sleep_async,    // 定时关机淡出 / 关着（像素时钟域电平），这时不自动轮播
    output reg                  strip_open,     // 胶片条开着
    output reg                  viz_full,       // 全屏音乐可视化（音乐层短按 KEY1 翻转；拨 SW4 不清 0）
    output     [3*BMP_MAX+8:0]  strip_info,     // {缩略图翻转, 有缩略图, 坏图, 图片表张数, 焦点}（3 x BMP_MAX + 9 位）
    output reg                  aud_pause,      // 1 = 暂停（电平，给音源）
    output reg                  mus_loop,       // 1 = 单曲循环
    output reg [6:0]                state_code,     // 第几张好图(推进中 = 目标), 0 = 没有
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output reg [2:0]            media_id,   // {换图翻转位, (第n张-1) mod 4} -> 音频选旋律、每次换图响提示音
    // 错误显示统一由 fs_err_code 细分码给出
    output reg [3:0]            fs_err_code, // FAT32细分错误码(0=熄灭, 加载1~5 / 扫描6~8, 见文件头注释)
    output     [6:0]            dead_count,  // 坏图张数(0~99)
    output reg                  load_recover, // 看门狗自愈：输出1个时钟周期脉冲，请求顶层复位图像链路
    // OSD 状态条用（只读输出，不影响本模块逻辑）
    output     [6:0]            osd_live_cnt,   // 好图总张数
    output                      osd_auto,       // 自动轮播开
    output                      osd_loading,    // 正在读卡加载（含后台预加载）
    output     [3:0]            loaded_banks,   // LED: [0] 下一张已缓存 [1] 上一张已缓存 [2] 后台该读的都读完了 [3] 正在加载
                                                //   （好图 <= 6 张时 [2] = 全部好图已缓存）

    input                       write_finish_toggle,
    output reg [2:0]            write_buf_idx,  // 写哪块 bank(0~5) = 加载这张图时挑中的 bank
    output reg [2:0]            disp_buf_idx,   // 显示哪块 bank = 正在显示的图(0~5)
    output     [3:0]            write_thumb_idx,// 缩略图写到第几格 = 正在加载的图(序号 0~15)，和 write_buf_idx 一起给写侧
    input      [5:0]            bank_busy_async,// 显示通路本帧 / 上一帧读过的 bank（像素时钟域，每位单独同步）

    output reg                  write_req,  // 写 SDRAM 请求(加载开始置位, 收到 ack 清零)
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    input      [8:0]            write_used,     // 写 FIFO 已用字数（本时钟域）
    output                      write_v_flip,   // 本张图写入时是否上下翻转（倒存的 BMP = 0）
    // WAV：环形缓冲写侧 + 状态
    output                      ring_we,
    output     [31:0]           ring_wdata,
    input                       ring_space_ok,
    input                       ring_empty,
    output                      ring_flush,
    input                       ring_flush_busy,
    output                      ring_eos,
    output                      wav_active,
    output     [6:0]            mus_idx,
    output     [6:0]            mus_cnt,
    output                      mus_tot_known,
    output     [15:0]           mus_tot_bcd,
    output     [2:0]            mus_err,
    // 字幕从卡读取：字幕条 RAM 写口 + 结果（本时钟域；ok 为 0 时 w/top 可能在变）
    output                      sub_we,
    output     [13:0]           sub_waddr,
    output     [7:0]            sub_wdata,
    output                      sub_ok,
    output     [9:0]            sub_w,
    output     [13:0]           sub_top,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);




wire key_next_press;

    reg         prev_req_pending;      // 上一张请求挂起标志（和next_req_pending对称）

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
// 下面这些定时器都按毫秒计数（共用 1 毫秒节拍 auto_ms，比按时钟拍数计省寄存器；
//   这些时间都是毫秒到秒级，1 毫秒精度足够）
// 计时器位宽按参数算，只够放下 0~上限
localparam integer W_ERR  = $clog2(ERR_SHOW_MS + 1);
localparam integer W_ARS  = $clog2(AUTO_RESCAN_MS + 1);
localparam integer W_SRST = $clog2(SD_RST_MS + 1);
localparam integer W_LWDT = $clog2(LOAD_WDT_MS + 1);
localparam integer W_PWDT = $clog2(PEND_WDT_MS + 1);
localparam integer W_GAP  = $clog2(GAP_MS + 1);
localparam integer W_AC   = $clog2(AUTO_PLAY_MS * 20 + 1);   // 轮播最长档 AP_P7 = AUTO_PLAY_MS x 20
reg  [W_ERR-1:0] fs_err_show_cnt; // 细分码显示倒计时(加载错误5秒后自动熄灭)，毫秒
wire             data_done_pulse; // 一帧像素已送完(单周期脉冲, 来自 bmp_scaler)
reg              data_done_seen; // 本次加载的像素数据已送完FIFO（data_done脉冲置位）
// === 拔卡恢复 ===
reg              need_rescan;    // 读超时后置1=图片列表不可信，下次按键触发"复位SD+重扫"
// 还没显示过任何图（上电就出错）时不用按键，自动每 AUTO_RESCAN_MS 复位SD+重扫一次，插回卡就自动出图。
//   触发条件：黑屏(!display_valid) + 卡级故障(need_rescan) 或 扫描失败(scan_failed)；已经显示过图之后出错则等按 KEY2 触发恢复。
reg              scan_failed;    // 上一次扫描报了错（细分码 6/7/8/1），重扫成功后清零
reg  [W_ARS-1:0] arescan_cnt;    // 自动重扫计时，毫秒
reg              sd_rst;         // SD控制器软件复位脉冲（高电平，脉宽SD_RST_MS）
reg [W_SRST-1:0] sd_rst_cnt;     // 复位脉宽倒计时，毫秒
wire             scan_done;      // 来自 fat32_volume_scanner(单拍脉冲)

reg              scan_start_pulse;
reg              load_start_pulse;
// 字幕：扫描成功且有 SUB.BMP -> 先用读图的流读取器读它（sub_phase = 1，约 20 ms），读完 / 出错再开始加载图片
//   这期间缩放模块收不到字节，加载选择暂停；读不出来或格式不对就显示默认标语，不报错、不算坏图
//   SUB.BMP 的簇号 / 字节数：sub_phase = 1 时扫描器图片表读口读它（bmp_rd_sub），和加载图片一样隔 2 拍再启动读文件
reg        sub_phase;
reg        sub_start;
reg  [1:0] sub_dly;
wire       scan_sub_found;
// 起始簇号 / 字节数 = 扫描器图片表读口（地址 load_img，加载期间不变）
wire [31:0]      load_sector;    // 实为起始簇号(沿用旧名)
wire [31:0]      load_file_size; // 本次加载文件的字节数
reg              scan_kicked;
reg              first_found_latched;   // 扫描结束且至少有 1 张图（首图门控）
reg              first_image_committed;
reg              goto_active;       // 推进中: 正在去第 goto_img 张(含上电时坏图自动往后找)
reg  [3:0]       goto_img;          // 目标 = 图片序号(0 ~ img_cnt-1)
reg              goto_reseek;       // 上电时目标是坏图, 等 nav 给出下一张好图
reg  [3:0]       cur_img;           // 正在显示的图片序号
reg              media_pend;        // 换过图, 等 nav 算好编号再更新 media_id
reg              auto_play_en;
reg [W_AC-1:0]   auto_cnt;       // 轮播计时，毫秒
reg              song_end_pend;  // 随音乐档：一首歌播完了，等着换下一张
reg              per_mus_wav;    // 随音乐档且在播 WAV（寄存，不加长到推进决策的组合路径）
wire             wav_song_end;   // u_wav_ctrl：一首正常播完（单拍）
reg              next_req_pending;
reg              load_busy;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;

// === 写完检测：比较 toggle 电平，不依赖单周期脉冲，不会漏检 ===
reg              wrfin_level_consumed; // 本次加载启动时已消费的 toggle 电平
wire             wrfin_new_done;       // 电平发生过变化 = 有新帧写完（先后顺序无关，永不丢）

// === 双看门狗 ===
reg [W_LWDT-1:0] load_wdt;             // 加载看门狗：load_busy 下多久没有进展，毫秒
reg [W_PWDT-1:0] pend_wdt;             // 请求看门狗：有挂起请求但一直无法启动加载，毫秒

// === 帧间隔计数（提交后隔 GAP_MS 才允许下一次加载） ===
reg [W_GAP-1:0]  gap_cnt;              // 帧间隔，毫秒

wire auto_tick;

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};

// 轮播周期可调（毫秒计数，周期 = AUTO_PLAY_MS x {2 3 5 8 10 15 30 60} / 3）
localparam integer AP_P0 = AUTO_PLAY_MS * 2 / 3,  AP_P1 = AUTO_PLAY_MS,      AP_P2 = AUTO_PLAY_MS * 5 / 3,
                   AP_P3 = AUTO_PLAY_MS * 8 / 3,  AP_P4 = AUTO_PLAY_MS * 10 / 3, AP_P5 = AUTO_PLAY_MS * 5,
                   AP_P6 = AUTO_PLAY_MS * 10,     AP_P7 = AUTO_PLAY_MS * 20;
localparam integer AP_MS_CYC = CLK_FREQ_HZ / 1000;
localparam integer W_AP = $clog2(AP_MS_CYC + 1);   // 1 毫秒分频计数器位宽
wire [3:0]  per_lvl_s;
osd_status_sync #(.N(4)) u_per_sync (.clk(clk), .rst_n(~rst), .async_in(per_lvl_async), .sync_out(per_lvl_s));
reg  [3:0]  per_lvl_q;
reg  [W_AP-1:0] auto_pre;                 // 1 毫秒节拍分频
wire        auto_ms = (auto_pre == AP_MS_CYC - 1);
reg  [W_AC-1:0] auto_period_m1;           // 直接给出 周期 - 1，省一个减法器
always @* begin
    case (per_lvl_s)
        4'd0: auto_period_m1 = AP_P0 - 1;  4'd1: auto_period_m1 = AP_P1 - 1;  4'd2: auto_period_m1 = AP_P2 - 1;
        4'd3: auto_period_m1 = AP_P3 - 1;  4'd4: auto_period_m1 = AP_P4 - 1;  4'd5: auto_period_m1 = AP_P5 - 1;
        4'd6: auto_period_m1 = AP_P6 - 1;  4'd7: auto_period_m1 = AP_P7 - 1;
        default: auto_period_m1 = AP_P1 - 1;      // 8 随音乐：没有 WAV 时按 3 秒
    endcase
end
always @(posedge clk or posedge rst) begin
    if (rst) begin auto_pre <= 0; per_lvl_q <= 4'd1; per_mus_wav <= 1'b0; end
    else begin
        auto_pre <= auto_ms ? 1'b0 : auto_pre + 1'b1;
        per_lvl_q <= per_lvl_s;
        per_mus_wav <= (per_lvl_s == 4'd8) && wav_active;
    end
end
wire per_changed = (per_lvl_s != per_lvl_q);
wire auto_timer = auto_ms && (auto_cnt == auto_period_m1);   // auto_cnt 为毫秒数
assign auto_tick = per_mus_wav ? song_end_pend : auto_timer;  // 随音乐档在播 WAV：歌播完才换

assign wrfin_new_done = (wrfin_tgl_sync[2] != wrfin_level_consumed);

// ===========================================================================
// === FAT32 模块（扫描器 / 流读器 / 适配器）互连信号 ===
// ===========================================================================
// 复位: SD 软件复位(sd_rst)时必须把 FAT32 模块与适配器一起清零,
// 否则残留半次事务会让 sector_busy 永不清零, 恢复流程反而被卡死。
wire             fat_rst_n = ~(rst | sd_rst);

// 扇区读仲裁: 扫描阶段用扫描器, 加载阶段用流读器(两者从不同时工作)
reg              use_streamer;         // 0=扫描器占用SD, 1=流读器占用SD
reg              scan_finished;        // 一次扫描已结束(成功或失败), 供恢复分支判断

// 扫描器侧
wire             scanner_sector_req;
wire [31:0]      scanner_sector_lba;
wire             scanner_sector_busy;
wire             scanner_sector_valid;
wire [7:0]       scanner_sector_byte;
wire [8:0]       scanner_sector_byte_index;
wire             scanner_sector_done;
wire [7:0]       scanner_sector_error;
wire             scanner_busy;
wire [7:0]       scanner_fs_error;
wire [7:0]       scanner_spc;
wire [31:0]      scanner_fat_start;
wire [31:0]      scanner_data_start;
wire [31:0]      scanner_max_cluster;

// 流读器侧
wire             streamer_sector_req;
wire [31:0]      streamer_sector_lba;
wire             streamer_sector_busy;
wire             streamer_sector_valid;
wire [7:0]       streamer_sector_byte;
wire [8:0]       streamer_sector_byte_index;
wire             streamer_sector_done;
wire [7:0]       streamer_sector_error;
wire             streamer_file_busy;
wire             streamer_file_done;   // 单拍脉冲
wire             streamer_byte_valid;
wire [7:0]       streamer_byte;
wire [31:0]      streamer_byte_offset;
wire [7:0]       streamer_fs_error;

// 图片侧(扫描器/读图)仲裁结果
reg              img_sector_req;
reg  [31:0]      img_sector_lba;
// 读歌侧
wire             aud_sector_req;
wire [31:0]      aud_sector_lba;
wire             aud_want;
reg              aud_own;              // 1 = 适配器当前/下一次事务属于读歌
// 仲裁输出(接适配器)
wire             arb_sector_req = aud_own ? aud_sector_req : img_sector_req;
wire [31:0]      arb_sector_lba = aud_own ? aud_sector_lba : img_sector_lba;
wire             arb_sector_busy;
wire             arb_sector_valid;
wire [7:0]       arb_sector_byte;
wire [8:0]       arb_sector_byte_index;
wire             arb_sector_done;
wire [7:0]       arb_sector_error;

// WAV 歌单(扫描器 -> 播放控制)
wire [3:0]       wav_count;         // 歌单最多 12 首
wire [3:0]       wav_rd_addr;
wire [31:0]      wav_rd_cluster, wav_rd_size;

// ---- 仲裁逻辑 ----
always @(*) begin
    if (use_streamer) begin
        img_sector_req = streamer_sector_req;
        img_sector_lba = streamer_sector_lba;
    end else begin
        img_sector_req = scanner_sector_req;
        img_sector_lba = scanner_sector_lba;
    end
end
// 图片侧 / 读歌侧 按扇区轮流用适配器（读歌优先）。
//   适配器空闲、这一拍没有读请求、且"歌想读 != 当前主人是歌"时换主人(switch_now)。
//   换主人的这一拍两边都看到"忙"(组合)，所以旧主人这一拍不会发请求，不会有请求被丢掉；
//   不是主人的一方一直看到"忙"，停在发请求前等着(fat32 两个模块都是"不忙才发请求")。
//   歌的缓冲有空位才"想读"，读歌只占 SD 带宽的约 16%（立体声），其余时间都给图片。
wire switch_now = !arb_sector_busy && !arb_sector_req && (aud_want != aud_own);
wire img_busy_v = arb_sector_busy | aud_own  | switch_now;
wire aud_busy_v = arb_sector_busy | ~aud_own | switch_now;
always @(posedge clk or negedge fat_rst_n) begin
    if (!fat_rst_n)
        aud_own <= 1'b0;
    else if (switch_now)
        aud_own <= ~aud_own;
end
// 返回信号按 use_streamer / aud_own 分路(非主人一侧强制无效, 防止误吞 done/valid)
assign scanner_sector_busy       = use_streamer ? 1'b0            : img_busy_v;
assign scanner_sector_valid      = use_streamer ? 1'b0            : (arb_sector_valid & ~aud_own);
assign scanner_sector_byte       = arb_sector_byte;
assign scanner_sector_byte_index = arb_sector_byte_index;
assign scanner_sector_done       = use_streamer ? 1'b0            : (arb_sector_done & ~aud_own);
assign scanner_sector_error      = (use_streamer | aud_own) ? 8'd0 : arb_sector_error;
assign streamer_sector_busy       = use_streamer ? img_busy_v : 1'b0;
assign streamer_sector_valid      = use_streamer ? (arb_sector_valid & ~aud_own) : 1'b0;
assign streamer_sector_byte       = arb_sector_byte;
assign streamer_sector_byte_index = arb_sector_byte_index;
assign streamer_sector_done       = use_streamer ? (arb_sector_done & ~aud_own) : 1'b0;
assign streamer_sector_error      = (use_streamer & ~aud_own) ? arb_sector_error : 8'd0;

// bmp_ready = 源文件已读完(全部字节已送缩放模块), 用流读器空闲电平表示
assign bmp_ready = ~streamer_file_busy;

// ---- 加载出错单拍脉冲 ----
//  三类来源:
//   A) 流读器上报文件系统错误(适配器扇区超时0x10 / 簇链错误0x23~0x26)
//      -> 细分码1~5;  0x10 置 need_rescan 走拔卡恢复,
//         0x23~0x26 是单文件损坏 -> 标记坏图
//   B) 文件读完但像素数不够一帧(文件截断/尺寸不足)
//      -> 与 0x25(提前EOF) 同类: 细分码4 + 标记坏图
//   C) BMP头校验失败(签名/偏移/宽高/位深, 错规格或坏签名文件)
//      -> 与截断同类: 细分码4 + 标记坏图; 校验标志在文件读完时才生效,
//         保证释放 load_busy 时流读器必已读完, 无并发风险
// (load_error_* 定义在 bmp_scaler 实例之后, 见下方;
//  保持先声明后使用, 部分仿真器不接受先用后声明)

// ===========================================================================
// === bmp_scaler: BMP 头解析 + 等比缩放 + 按写 SDRAM 顺序输出 RGB ===
// ===========================================================================
wire        scl_valid;
wire [23:0] scl_rgb;
wire        scl_done, scl_err_hdr, scl_err_short, scl_hold, scl_flip;
bmp_scaler #(
    .OUT_W (IMG_W),
    .OUT_H (IMG_H)
) u_scaler (
    .clk       (clk),
    .rst       (rst),
    .start     (load_start_pulse),
    .active    (load_busy),
    .in_valid  (streamer_byte_valid & ~sub_phase),   // 读 SUB.BMP 时字节不给缩放模块
    .in_byte   (streamer_byte),
    .in_eof    (streamer_file_done & ~sub_phase),
    .hold      (scl_hold),
    .out_used  (write_used),
    .out_valid (scl_valid),
    .out_rgb   (scl_rgb),
    .done      (scl_done),
    .err_hdr   (scl_err_hdr),
    .err_short (scl_err_short),
    .flip      (scl_flip)
);
assign bmp_data_wr_en = scl_valid;
assign bmp_data       = scl_rgb;
assign write_v_flip   = scl_flip;

// 加载出错单拍脉冲(说明见上方"加载出错单拍脉冲")
//   B) 截断 / C) 头不合格 由 bmp_scaler 判断, 都在文件读完(且缓冲读空)之后才报,
//      保证释放 load_busy 时流读器已经结束
wire load_error_fs    = load_busy && streamer_file_done && (streamer_fs_error != 8'd0);
wire load_error_trunc = load_busy && scl_err_short;
wire load_error_hdr   = load_busy && scl_err_hdr;
wire load_error = load_error_fs | load_error_trunc | load_error_hdr;
// data_done: 一帧像素发完(截断时也给)
assign data_done_pulse = scl_done | scl_err_short;
// === bmp_scaler 结束 ===

// SW4 分层（见文件头）。KEY2 = key_next，KEY4 = key_auto
wire layer_s;
sync_2ff u_layer_sync (.clk(clk), .rst_n(~rst), .async_in(layer_async), .sync_out(layer_s));
wire menu_s;                                            // 菜单开着
sync_2ff u_menu_sync (.clk(clk), .rst_n(~rst), .async_in(menu_async), .sync_out(menu_s));

wire k2_press, k2_short, k2_long, k2_held;
key_short_long #(
    .CLK_HZ      (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20),
    .LONG_MS     (LONG_PRESS_MS),
    .REPEAT_MS   (LONG_PRESS_MS / 2),
    .EXT_MS      (1)                 // 用外部 1 毫秒节拍 auto_ms（同时钟、同复位），省掉模块内分频器
) u_key_next (
    .clk         (clk),
    .rst_n       (~rst),
    .ms_in       (auto_ms),
    .key_n       (key_next),
    .press       (k2_press),
    .short_press (k2_short),
    .long_press  (k2_long),
    .held        (k2_held),
    .long_active ()
);
wire k4_press, k4_short, k4_long;
// KEY3 确认（像素域翻转电平 -> 本时钟域边沿）
wire cfm_s;
sync_2ff u_cfm_sync (.clk(clk), .rst_n(~rst), .async_in(strip_cfm_async), .sync_out(cfm_s));
wire sleep_s;                                           // 定时关机
sync_2ff u_sleep_sync (.clk(clk), .rst_n(~rst), .async_in(sleep_async), .sync_out(sleep_s));
reg  cfm_q;
wire strip_cfm = (cfm_s != cfm_q);
key_short_long #(
    .CLK_HZ      (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20),
    .LONG_MS     (LONG_PRESS_MS),
    .REPEAT_MS   (LONG_PRESS_MS),
    .REPEAT      (0),
    .EXT_MS      (1)
) u_key_auto (
    .clk         (clk),
    .rst_n       (~rst),
    .ms_in       (auto_ms),
    .key_n       (key_auto),
    .press       (k4_press),
    .short_press (k4_short),
    .long_press  (k4_long),
    .held        (),
    .long_active ()
);
// 按下那一拍记住层号、菜单开没开，之后这次按键（松手短按 / 长按 / 按住）都按它算
reg  k2_lay_q, k4_lay_q, k2_men_q, k4_men_q, k4_str_q;
wire k2_lay = k2_press ? layer_s : k2_lay_q;
wire k4_lay = k4_press ? layer_s : k4_lay_q;
wire k2_men = k2_press ? menu_s : k2_men_q;             // 这次按下归菜单
wire k4_men = k4_press ? menu_s : k4_men_q;
wire k4_str = k4_press ? strip_open : k4_str_q;          // 这次按下时胶片条开着（KEY4 归胶片条）
assign key_next_press = k2_press && !layer_s && !menu_s; // 图片层：按下即下一张（菜单开着时不算）
wire song_next = k2_short && k2_lay && !k2_men;
wire song_prev = k2_long  && k2_lay && !k2_men;
wire song_hold = k2_held  && k2_lay && !k2_men;
wire key4_short_pic = k4_short && !k4_lay && !k4_men && !k4_str;   // 图片层：自动轮播开/关
wire key4_long_pic  = k4_long  && !k4_lay && !k4_men && !k4_str;   // 图片层：上一张
wire strip_prev     = k4_press && !layer_s && !menu_s && strip_open; // 胶片条里 KEY4 按下 = 上一张
always @(posedge clk or posedge rst) begin
    if (rst) begin
        k2_lay_q <= 1'b0; k4_lay_q <= 1'b0; k2_men_q <= 1'b0; k4_men_q <= 1'b0; k4_str_q <= 1'b0;
        aud_pause <= 1'b0; mus_loop <= 1'b0; cfm_q <= 1'b0; viz_full <= 1'b0;
    end else begin
        // 音乐层里 KEY1 短按 = 进 / 出全屏音乐可视化（胶片条只在图片层打开）
        // SW4 只管按键归哪一层，不管显示；拨回图片层全屏照常（菜单 / 胶片条打开时由显示通路暂时让开）
        if (layer_s && key_strip) viz_full <= ~viz_full;
        k4_str_q <= k4_str;
        cfm_q <= cfm_s;
        k2_lay_q <= k2_lay;
        k4_lay_q <= k4_lay;
        k2_men_q <= k2_men;
        k4_men_q <= k4_men;
        if (k4_short && k4_lay && !k4_men) aud_pause <= ~aud_pause;
        if (song_next || song_prev || scan_done) aud_pause <= 1'b0;   // 换歌 / 新歌单：自动继续
        if (k4_long && k4_lay && !k4_men) mus_loop <= ~mus_loop;
    end
end

// 起始簇号/文件大小来自扫描器的图片表读口(地址 = load_img, 加载期间不变)
// ===========================================================================
// === 多图导航 + 缓存调度: 最多 BMP_MAX 张图, 6 块整帧缓存按需换入 ===
// === (SDRAM 只放得下 6 帧 + 缩略图区, 图片多于 6 张时轮换使用) ===
// ===========================================================================
// 图片表在扫描器里(号码最小的 BMP_MAX 张好图, 按号码排好序, 序号 0 ~ img_cnt-1), 这里只按序号工作:
//   bad_bits[i]  第 i 张是坏图(加载时发现, 重扫清零); bad_cnt = 坏图张数
//   scan_bad     扫描时文件头检查就没通过的张数(不在图片表里); 坏图数显示 = scan_bad + bad_cnt
//   good         好图位图 = 前 img_cnt 位里不是坏图的;  live_cnt = 好图张数
//   cur_img      正在显示的图(序号);  goto_img = 正在去的图(goto_active=1 时)
//   焦点 focus   = goto_active ? goto_img : cur_img
//
// 导航(nav): NP 位, 直接从好图位图和焦点用组合逻辑算, 打一拍寄存:
//   nav_rank  焦点是第几张好图(从 1 数, OSD "第 n/N 张"、旋律号)
//   nav_first 第一张好图;  nav_nxt / nav_prv = 焦点后面 / 前面最近的好图(到头回绕)
//   nav_valid = 寄存的结果对应的焦点、好图位图和现在一样(焦点或坏图一变, 下一拍就是新结果)
//
// 缓存: 第 i 张放哪块 bank 不固定, 用一张对照表(两个方向一起改):
//   img_ok[i] = 第 i 张在缓存里;  bank_ok[b] / bank_img_v[b] = bank b 里有完整的图 / 是第几张
//   第 x 张在哪块 bank: 拿 x 和 6 块 bank 的记录比(bank_hit, 6 位独热码, 全 0 = 没缓存), 不另外存"图 -> bank"表(省 LUT)。
//     两者一致: 只有挑 bank(清掉旧图的 img_ok 和这块 bank_ok)、写完(置这张的 img_ok 和这块 bank_ok)、整体清零三处改它们。
//   缩略图按图片序号放(THUMB_BASE + 序号 x 一张缩略图字数, 16 格), 整帧被换出去以后缩略图还在:
//     th_ok[i] = 第 i 张有缩略图(整帧写完时置 1, 重扫清 0)。同一张图重新加载时缩略图区写进去的是同样的内容, 胶片条照常显示。
//   要加载时(选中那一拍)挑一块 bank 写, 同一拍把它标成无效、改对照表, 原先在里面的那张算没缓存:
//     硬保护(不写): 显示通路回报的本帧 / 上一帧读过的 bank(bank_busy, 转场中再按键时屏幕上的两张可能不是当前图和上一张)、
//                   当前图、上一张显示的图、焦点、胶片条打开时的原图(取消时同一拍换回去)
//     软保护(尽量不写): 焦点的下一张、上一张
//     先挑空 bank(没硬保护的); 没有空 bank 时: 要去的图(按键 / 轮播)换掉任何没硬保护的, 后台预载只换掉没硬保护也没软保护的;
//     有空 bank 但都在硬保护里(只在转场的那一会儿): 等。硬保护最多 4 张 + 转场时屏幕上的 2 张, 转场放完就有能写的 bank。
//   显示中的图(含转场的旧图、新图)一定在硬保护里, 所以加载永远不会写屏幕上看得到的图。
//   要加载的图: 推进目标没缓存 -> 先加载它; 否则显示后在后台预载,
//     顺序 = 下一张 > 上一张 > 从焦点往后最近的一张没缓存的(还有空 bank 时) / 没缩略图的(6 块都满了时), 到头回绕。
//     <= 6 张时总有空 bank, 所有好图都会载进来; > 6 张时 6 块填满以后只补缩略图, 补完就停
//     (后台不换下一张、上一张, 没缩略图的只会越来越少, 所以不会来回换)。
//   切图: 目标已缓存 -> 只改 disp_buf_idx, 下一帧就是新图(转场照常);
//         目标没缓存 -> 画面停在当前图, 加载目标, 读完再切; 正在后台预载别的图时先中止它(按键插队)。
//   坏图: 加载失败 -> 标记坏图(nav 自动跳过它), 规则:
//         已有图在显示 -> 停在当前图; 还没显示过图 -> 自动去下一张好图; 后台预载的坏图直接跳过。
//
// 旋律: 每次换图(显示切换后 nav 算好)更新 media_id = {换图翻转位, (n-1) mod 4},
//   第 n 张好图用第 ((n-1) mod 4)+1 段旋律; 换图翻转位保证每次换图都响提示音、旋律从头开始。
wire [4:0]  img_cnt;                      // 扫描器图片表张数(0~BMP_MAX, 文件头检查通过的, 扫描结束后不变)
wire [6:0]  scan_bad;                     // 扫描器文件头检查没通过的张数
wire [31:0] img_rd_cluster, img_rd_size;  // 图片表读口(地址 load_img, 第 2 拍出数据)
assign load_sector    = img_rd_cluster;
assign load_file_size = img_rd_size;
localparam integer NP = BMP_MAX;           // 位图位数 = 图片最多几张
localparam [NP-1:0] ALL1 = {NP{1'b1}};
reg  [NP-1:0] bad_bits;
reg  [4:0]  bad_cnt;
wire [NP-1:0] good   = ~(ALL1 << img_cnt) & ~bad_bits;   // 前 img_cnt 张里的好图(img_cnt = NP 时移出去 = 全 1)
wire [3:0]  focus    = goto_active ? goto_img : cur_img;

// NP 位位图: 最低 / 最高的 1 在第几位(全 0 时结果不用), 1 的个数
function [3:0] lo_n;
    input [NP-1:0] v;
    integer j;
    begin
        lo_n = NP - 1;
        for (j = NP - 2; j >= 0; j = j - 1)
            if (v[j]) lo_n = j;
    end
endfunction
function [3:0] hi_n;
    input [NP-1:0] v;
    integer j;
    begin
        hi_n = 4'd0;
        for (j = 1; j < NP; j = j + 1)
            if (v[j]) hi_n = j;
    end
endfunction
function [4:0] cnt_n;
    input [NP-1:0] v;
    integer j;
    begin
        cnt_n = 5'd0;
        for (j = 0; j < NP; j = j + 1)
            cnt_n = cnt_n + {4'd0, v[j]};
    end
endfunction
// 6 位(bank): 最低的 1 在第几位, 独热码
function [2:0] lo6;
    input [5:0] v;
    lo6 = v[0] ? 3'd0 : v[1] ? 3'd1 : v[2] ? 3'd2 : v[3] ? 3'd3 : v[4] ? 3'd4 : 3'd5;
endfunction
function [5:0] oh6;
    input [2:0] b;
    oh6 = 6'd1 << b;
endfunction
wire [4:0]  live_cnt = cnt_n(good);                      // 好图张数(直接数位图, 重扫时计数先后清零也不会算错)

// ---------------- 导航 ----------------
// （用组合逻辑一次算出再打一拍；逐位扫描的串行实现综合后 LUT 并不更少）
wire [NP-1:0] aft_foc = (ALL1 << 1) << focus;   // 焦点后面的位
wire [NP-1:0] bef_foc = ~(ALL1 << focus);       // 焦点前面的位
wire [NP-1:0] g_aft   = good & aft_foc;
wire [NP-1:0] g_bef   = good & bef_foc;
reg  [3:0]  nav_f;                        // 这次结果对应的焦点
reg  [NP-1:0] nav_g;                      // 这次结果对应的好图位图
reg  [4:0]  nav_rank;
reg  [3:0]  nav_first, nav_nxt, nav_prv;
wire        nav_valid = (nav_f == focus) && (nav_g == good);
always @(posedge clk or posedge rst) begin
    if (rst) begin
        nav_f <= 4'd0; nav_g <= {NP{1'b0}};
        nav_rank <= 5'd0; nav_first <= 4'd0; nav_nxt <= 4'd0; nav_prv <= 4'd0;
    end else begin
        nav_f     <= focus;
        nav_g     <= good;
        nav_rank  <= cnt_n(good & ~aft_foc);                       // 焦点和它前面的好图张数
        nav_first <= lo_n(good);
        nav_nxt   <= (g_aft != 0) ? lo_n(g_aft) : lo_n(good);      // 后面没有好图就回绕到第一张
        nav_prv   <= (g_bef != 0) ? hi_n(g_bef) : hi_n(good);      // 前面没有就回绕到最后一张
    end
end

// ---------------- 缓存 ----------------
reg  [NP-1:0] img_ok;                       // 第 i 张在缓存里(整帧在 bank_hit 找到的那块 bank 里)
reg  [5:0]  bank_ok;                      // bank b 里有完整的图
reg  [23:0] bank_img_v;                   // bank b 里是第几张 = [b*4 +: 4]
reg  [NP-1:0] th_ok;                      // 第 i 张有缩略图
reg  [NP-1:0] th_flip;                      // 第 i 张是正存的 BMP（缩略图在 SDRAM 里倒着放）
reg  [3:0]  strip_orig;                   // 打开胶片条时正在显示的那张（取消时回到它）
reg  [2:0]  orig_bank;                    //   它在哪块 bank
reg         prev_v;                       // 上一张显示的图(换图前的那张)还要保护
reg  [2:0]  prev_bank;                    //   它在哪块 bank
reg  [3:0]  load_img;                     // 正在加载的图(序号), 也是图片表读地址和缩略图格号
reg  [1:0]  load_prep;                    // 读图片表: 2 = 给出地址, 1 = 这拍启动加载(流读器下一拍取簇号/字节数)
(* async_reg = "true" *) reg [5:0] bb_m1;
reg  [5:0]  bank_busy;                    // 显示通路正在读的 bank(已同步; 像素域按帧保持, 每位单独同步没问题)
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bb_m1 <= 6'd0; bank_busy <= 6'd0;
    end else begin
        bb_m1 <= bank_busy_async; bank_busy <= bb_m1;
    end
end
// 第 x 张在哪块 bank(独热码, 全 0 = 没缓存)
function [5:0] bank_hit;
    input [3:0]  x;
    input [5:0]  ok;
    input [23:0] tab;
    integer j;
    for (j = 0; j < 6; j = j + 1)
        bank_hit[j] = ok[j] && (tab[j*4 +: 4] == x);
endfunction
wire [5:0]  foc_hit      = bank_hit(focus,   bank_ok, bank_img_v);
wire [5:0]  nxt_hit      = bank_hit(nav_nxt, bank_ok, bank_img_v);
wire [5:0]  prv_hit      = bank_hit(nav_prv, bank_ok, bank_img_v);
wire        focus_cached = (foc_hit != 6'd0);    // = img_ok[focus]
wire        nxt_cached   = (nxt_hit != 6'd0);
wire        prv_cached   = (prv_hit != 6'd0);
wire [2:0]  foc_bank     = {foc_hit[4] | foc_hit[5], foc_hit[2] | foc_hit[3], foc_hit[1] | foc_hit[3] | foc_hit[5]};
wire [5:0]  hard_pr = !display_valid ? 6'd0 :
                      bank_busy | oh6(disp_buf_idx) | (prev_v ? oh6(prev_bank) : 6'd0) |
                      (strip_open ? oh6(orig_bank) : 6'd0) | foc_hit;
wire [5:0]  soft_pr = nxt_hit | prv_hit;
wire        has_free = (bank_ok != 6'h3F);
wire [5:0]  v_free  = ~bank_ok & ~hard_pr;
wire [5:0]  v_fg    = (v_free != 6'd0) ? v_free : ~hard_pr;                          // 要去的图能写的 bank
wire [5:0]  v_bg    = (v_free != 6'd0) ? v_free : has_free ? 6'd0 : (~hard_pr & ~soft_pr);   // 后台预载能写的 bank
wire [NP-1:0] need_bits = good & ~img_ok;                 // 还没缓存的好图
wire [NP-1:0] bg_set    = need_bits & (has_free ? ALL1 : ~th_ok);   // 后台还要读的(下一张、上一张另算)
wire [NP-1:0] n_aft     = bg_set & aft_foc;
wire [3:0]  near_need = (n_aft != 0) ? lo_n(n_aft) : lo_n(bg_set);   // 从焦点往后最近的一张(回绕)
wire        bg_done   = (bg_set == 0) && ((live_cnt == 5'd0) || (nxt_cached && prv_cached));  // 后台该读的都读完了(没有好图时下一张/上一张不算)
// 下一次要加载哪张: 推进目标(坏图等 nav 改目标) > 下一张 > 上一张 > 往后最近的(显示后、列表可信时预载)
wire        bg_ok    = display_valid && !goto_active && !need_rescan;
wire        need_foc = goto_active && !goto_reseek && !focus_cached;
wire        pick_any = nav_valid && ((need_foc && (v_fg != 6'd0)) || (!need_foc && bg_ok && !bg_done && (v_bg != 6'd0)));
wire [3:0]  pick_img = need_foc ? focus : !nxt_cached ? nav_nxt : !prv_cached ? nav_prv : near_need;
wire [2:0]  pick_bank = lo6(need_foc ? v_fg : v_bg);
// 按键插队: 要去的图没缓存, 正在加载的是别的图(只可能是后台预载: 推进中不预载, 推进中也不收新按键),
//   而且它的像素还没发完(发完了这张很快就写完, 让它载完, 也保证写侧不会在中止之后才报"写完")。
//   只有要去的图能打断预载, 预载之间不会互相打断。
wire       preempt  = load_busy && need_foc && (load_img != goto_img) && !data_done_seen && !data_done_pulse
                      && !wrfin_new_done && !load_error;
reg        img_st_abort;                  // 1 拍: 复位读图的文件流读取器(停止读这个文件)

// 坏图张数(数码管/OSD)、OSD 状态、LED
assign dead_count     = scan_bad + {2'd0, bad_cnt};
assign osd_live_cnt   = {2'd0, live_cnt};
assign osd_auto       = auto_play_en;
assign osd_loading    = load_busy;
assign loaded_banks   = {load_busy, display_valid && bg_done, display_valid && prv_cached, display_valid && nxt_cached};
assign strip_info     = {th_flip, th_ok, bad_bits, img_cnt, focus};
assign write_thumb_idx = load_img;

// 启动一次加载(load_prep 最后一拍: 流读器下一拍取起始簇号/字节数时, 图片表正好读出来)
task start_load;
    begin
        load_start_pulse     <= 1'b1;
        load_busy            <= 1'b1;
        source_done_seen     <= 1'b0;
        load_wdt             <= 1'b0;
        pend_wdt             <= 1'b0;
        wrfin_level_consumed <= wrfin_tgl_sync[2];
        write_req            <= 1'b1;      // 发写 SDRAM 请求
    end
endtask

// 换显示: 第 x 张(在 bank b 里); 换下来的那张记成"上一张显示的图"(转场时还在屏幕上)
task show_img;
    input [3:0] x;
    input [2:0] b;
    begin
        disp_buf_idx          <= b;
        cur_img               <= x;
        prev_bank             <= disp_buf_idx;
        prev_v                <= display_valid;
        display_valid         <= 1'b1;
        first_image_committed <= 1'b1;
        goto_active           <= 1'b0;
        media_pend            <= 1'b1;
    end
endtask

// FAT32 内部错误码 -> 细分码数码管(1~8) 映射
//   加载阶段(流读器): 0x10 扇区读错误(含适配器超时=拔卡) -> 1
//                     0x23 FAT环(下一簇==当前簇/链尾非EOF)  -> 2
//                     0x24 簇号越界                          -> 3
//                     0x25 提前EOF(没读完file_size链就断)    -> 4
//                     0x26 遍历簇数超上限                    -> 5
//   扫描阶段(扫描器): 0x20 无FAT32分区(签名不对)            -> 6
//                     0x21 BPB参数不支持                     -> 7
//                     0x22 根目录找不到 .BMP                 -> 8
//   其余(含0) -> 0(数码管熄灭);  文件截断(像素不足)在调用处按4上报
function [3:0] fs_err_map;
    input [7:0] code;
    begin
        case (code)
            8'h10:   fs_err_map = 4'd1;
            8'h23:   fs_err_map = 4'd2;
            8'h24:   fs_err_map = 4'd3;
            8'h25:   fs_err_map = 4'd4;
            8'h26:   fs_err_map = 4'd5;
            8'h20:   fs_err_map = 4'd6;
            8'h21:   fs_err_map = 4'd7;
            8'h22:   fs_err_map = 4'd8;
            default: fs_err_map = 4'd0;
        endcase
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        wrfin_level_consumed  <= 1'b0;
        data_done_seen        <= 1'b0;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        scan_kicked           <= 1'b0;
        first_found_latched   <= 1'b0;
        first_image_committed <= 1'b0;
        bad_bits              <= {NP{1'b0}};
        bad_cnt               <= 5'd0;
        goto_active           <= 1'b0;
        goto_img              <= 4'd0;
        goto_reseek           <= 1'b0;
        cur_img               <= 4'd0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 1'b0;
        song_end_pend         <= 1'b0;
        img_ok                <= {NP{1'b0}};    // 缓存全部作废
        bank_ok               <= 6'd0;
        th_ok                 <= {NP{1'b0}};
        th_flip               <= {NP{1'b0}};
        prev_v                <= 1'b0;
        prev_bank             <= 3'd0;
        orig_bank             <= 3'd0;
        bank_img_v            <= 24'd0;
        strip_open            <= 1'b0;
        strip_orig            <= 4'd0;
        load_img              <= 4'd0;
        load_prep             <= 2'd0;
        write_buf_idx         <= 3'd0;
        disp_buf_idx          <= 3'd0;
        next_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        display_valid         <= 1'b0;
        state_code            <= 7'd0;
        media_id              <= 3'd0;
        media_pend            <= 1'b0;
        prev_req_pending  <= 1'b0;
        load_wdt          <= 1'b0;
        pend_wdt          <= 1'b0;
        load_recover      <= 1'b0;
        gap_cnt           <= 1'b0;
        fs_err_show_cnt   <= 1'b0;
        need_rescan       <= 1'b0;      // 拔卡恢复标志复位
        scan_failed       <= 1'b0;
        arescan_cnt       <= 1'b0;
        sd_rst            <= 1'b0;
        sd_rst_cnt        <= 1'b0;
        write_req         <= 1'b0;
        use_streamer      <= 1'b0;
        scan_finished     <= 1'b0;
        fs_err_code       <= 4'd0;      // 细分码复位
        img_st_abort      <= 1'b0;
        sub_phase         <= 1'b0;
        sub_start         <= 1'b0;
        sub_dly           <= 2'd0;
        // scanner_* 为线网由扫描器驱动, 复位无需处理
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_recover     <= 1'b0;   // 默认无自愈请求，只在看门狗触发时打1拍
        img_st_abort     <= 1'b0;
        sub_start        <= 1'b0;
        if (sub_phase && streamer_file_done) sub_phase <= 1'b0;
        if (sub_dly != 2'd0) begin
            sub_dly <= sub_dly - 2'd1;
            if (sub_dly == 2'd1) sub_start <= 1'b1;
        end   // 读完或出错（出错时流读取器也给 file_done）

        // 帧间隔倒计时（提交/自愈时会被后面的语句重新赋值，故放最前面）
        if (auto_ms && (gap_cnt != 1'b0))
            gap_cnt <= gap_cnt - 1'b1;


        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_found_latched   <= 1'b0;
            first_image_committed <= 1'b0;
            bad_bits              <= {NP{1'b0}};
            bad_cnt               <= 5'd0;
            goto_active           <= 1'b0;
            goto_reseek           <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 1'b0;
            img_ok                <= {NP{1'b0}};    // 缓存全部作废（缩略图也作废）
            bank_ok               <= 6'd0;
            th_ok                 <= {NP{1'b0}};
            strip_open            <= 1'b0;
            load_prep             <= 2'd0;
            next_req_pending      <= 1'b0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            display_valid         <= 1'b0;
            prev_req_pending<=1'b0;
            load_wdt          <= 1'b0;
            pend_wdt          <= 1'b0;
            fs_err_show_cnt   <= 1'b0;
            need_rescan       <= 1'b0;      // SD重新初始化=恢复流程已接管，列表作废标记清除
            sd_rst            <= 1'b0;      // 保险：初始化进行中确保复位脉冲已收起
            sd_rst_cnt        <= 1'b0;
            wrfin_level_consumed <= wrfin_tgl_sync[2];  // 电平对齐，避免把旧电平当新事件
            write_req         <= 1'b0;
            use_streamer      <= 1'b0;
            sub_phase         <= 1'b0;
            scan_finished     <= 1'b0;
            fs_err_code       <= 4'd0;      // 细分码清零

        end else begin
            // ==== 扫描完成这一拍, 一次性锁存全部结果 ====
            // (FAT32扫描是整个卷的一次性操作, first_found_latched = "扫描结束且至少找到1张")
            // 图片表就在扫描器里(img_cnt 张, 按号码排好), 这里只清坏图和缓存
            if (scan_done) begin
                scan_finished   <= 1'b1;
                first_found_latched <= (img_cnt != 3'd0);
                use_streamer        <= (img_cnt != 3'd0);     // 有图才切到流读器
                if (img_cnt != 3'd0 && scanner_fs_error == 8'd0 && scan_sub_found) begin
                    sub_phase <= 1'b1;                          // 先读 SUB.BMP
                    sub_dly   <= 2'd2;
                end
                bad_bits            <= {NP{1'b0}};               // 新一轮重扫: 坏图标记清零
                bad_cnt             <= 5'd0;
                img_ok              <= {NP{1'b0}};                 // 新卡内容, 缓存全部作废（缩略图也作废）
                bank_ok             <= 6'd0;
                th_ok               <= {NP{1'b0}};
                // 扫描失败(0x20/0x21/0x22/0x10): 细分码6/7/8/1;
                // 扫描成功: 细分码熄灭, 错误显示全部清零(新一轮结果取代旧显示)
                fs_err_code    <= fs_err_map(scanner_fs_error);
                fs_err_show_cnt <= 1'b0;
                scan_failed    <= (scanner_fs_error != 8'd0);
            end

            // ===================== 细分码锁存 + 坏图处理 =====================
            // 加载失败统一: 细分码数码管=1~5, 5秒后自动熄灭; 释放加载锁(不提交坏帧);
            // 0x10 卡级故障 -> need_rescan, 下次按键"复位SD+重扫"(不标记坏图, 卡的问题不是图的问题);
            // 0x23~0x26/截断/BMP头校验失败 文件级损坏 -> 标记坏图(按序号), 然后:
            //   加载的是推进目标 且 已有图在显示(按键场景) -> 停在当前好图, 一次按键只试一张;
            //   加载的是推进目标 且 还没显示过图(上电启动) -> 保持黑屏, 等 nav 算出下一张好图再去,
            //     全部是坏图 -> 待命, 按键走"复位SD+重扫";
            //   后台预载的坏图 -> 只标记, 跳过, 画面和按键都不受影响
            // 加载失败的 bank 在挑中它时已经标成无效, 不用再处理
            if (load_error) begin
                fs_err_show_cnt <= ERR_SHOW_MS;
                fs_err_code  <= load_error_fs ? fs_err_map(streamer_fs_error) : 4'd4;
                // 释放加载锁, 不提交坏帧
                load_busy            <= 1'b0;
                source_done_seen     <= 1'b0;
                load_wdt             <= 1'b0;
                wrfin_level_consumed <= wrfin_tgl_sync[2];
                write_req            <= 1'b0;
                if (goto_active && goto_img == load_img) begin
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    pend_wdt         <= 1'b0;
                end
                if (load_error_fs && (streamer_fs_error == 8'h10)) begin
                    // 卡级故障: 走重扫恢复(后台预加载也随之停止, 见 need_rescan)
                    need_rescan  <= 1'b1;
                    gap_cnt      <= GAP_MS;
                    goto_active  <= 1'b0;
                end else begin
                    // 文件级损坏: 标记坏图(加载的图一定还不是坏图, 计数不会重复)
                    bad_bits[load_img] <= 1'b1;
                    bad_cnt            <= bad_cnt + 5'd1;
                    if (!(goto_active && goto_img == load_img)) begin
                        // 后台预加载的坏图: 跳过
                    end else if (display_valid) begin
                        goto_active <= 1'b0;       // 按键场景: 停在当前好图
                    end else if (live_cnt <= 5'd1) begin
                        goto_active <= 1'b0;       // 上电启动, 最后一张好图也坏了: 待命
                    end else begin
                        goto_reseek <= 1'b1;       // 上电启动: 等 nav 算出下一张好图再去
                    end
                end
            end else if (auto_ms && (fs_err_show_cnt != 1'b0)) begin
                fs_err_show_cnt <= fs_err_show_cnt - 1'b1;
                if (fs_err_show_cnt <= 1'b1)
                    fs_err_code <= 4'd0;
            end

            // === SD控制器复位脉宽发生器 ===
            if (auto_ms && (sd_rst_cnt != 1'b0)) begin
                sd_rst_cnt <= sd_rst_cnt - 1'b1;
                if (sd_rst_cnt <= 1'b1)
                    sd_rst <= 1'b0;
            end

            // 记住源图已送完 FIFO，但还不能切显示，得等整帧写完
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // 记住本次加载的数据已送完FIFO（data_done单周期脉冲）
            // data_done_seen自管理：加载结束（提交/看门狗中止）load_busy掉零时自动清零
            if (load_busy && data_done_pulse)
                data_done_seen <= 1'b1;
            else if (!load_busy)
                data_done_seen <= 1'b0;

            // === 写完提交：检测 toggle 电平变化（不依赖单周期脉冲恰好同拍到达）===
            // 整帧写完 -> 这块 bank 有图了; 加载的正是推进目标 -> 切显示
            if (load_busy && source_done_seen && wrfin_new_done) begin
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                img_ok[load_img]      <= 1'b1;              // 第 load_img 张在 bank write_buf_idx 里（对照表挑 bank 时已写好）
                bank_ok[write_buf_idx] <= 1'b1;
                th_ok[load_img]       <= 1'b1;              // 缩略图也写完了（整帧和缩略图都写完才报写完）
                th_flip[load_img]     <= scl_flip;          // 缩略图要不要倒着读（缩放模块到下一次加载才改）
                wrfin_level_consumed  <= wrfin_tgl_sync[2]; // 消费掉这次电平变化
                load_wdt              <= 1'b0;
                gap_cnt               <= GAP_MS;        // 帧间隔：给写通道留收尾时间
                need_rescan           <= 1'b0;   // 加载成功=列表可信，解除恢复待命
                if (goto_active && goto_img == load_img) begin
                    show_img(load_img, write_buf_idx);
                    fs_err_show_cnt       <= 1'b0;
                    fs_err_code           <= 4'd0;   // 加载成功，细分码熄灭(后台加载成功不清, 坏图错误码照常显示5秒)
                end
            end

            // ===================== 数码管 / OSD 编号 =====================
            // 推进中显示"正在去第几张"(焦点 = 目标), 空闲时显示当前是第几张; 0 = 没有好图
            if (live_cnt == 5'd0)
                state_code <= 7'd0;
            else if (nav_valid)
                state_code <= {2'd0, nav_rank};
            // 旋律: 换图后等 nav 算好当前这张的编号, 再更新(翻转位每次换图都变 -> 提示音 + 旋律从头开始)
            if (media_pend && nav_valid && !goto_active) begin
                media_id   <= {~media_id[2], nav_rank[1:0] - 2'd1};
                media_pend <= 1'b0;
            end

            // === 按键插队（条件见 preempt）===
            //   释放加载锁, 同看门狗中止, 但不重扫、不报错: 这块 bank 开始写时已标成无效;
            //   读文件模块复位 1 拍(正在读的扇区由仲裁器让它读完, 同停歌); 缩放模块看到 load_busy=0 自己停;
            //   写侧收到下一次写请求时清 FIFO、换 bank。隔 GAP_MS 再开始加载要去的图(写侧把剩下的 burst 写完)
            if (preempt) begin
                load_busy            <= 1'b0;
                source_done_seen     <= 1'b0;
                load_wdt             <= 1'b0;
                wrfin_level_consumed <= wrfin_tgl_sync[2];
                write_req            <= 1'b0;
                gap_cnt              <= GAP_MS;
                img_st_abort         <= 1'b1;
            end

            // === 加载看门狗 ===
            if (load_busy) begin
                if (load_wdt >= LOAD_WDT_MS) begin
                    load_busy        <= 1'b0;
                    source_done_seen <= 1'b0;
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    load_wdt         <= 1'b0;
                    pend_wdt         <= 1'b0;
                    wrfin_level_consumed <= wrfin_tgl_sync[2];
                    gap_cnt          <= GAP_MS;
                    load_recover     <= 1'b1;   // 顶层收到后复位图像链路 ~2ms
                    write_req        <= 1'b0;
                    need_rescan      <= 1'b1;   // 看门狗超时同样视为列表不可信
                end else if (streamer_byte_valid || scl_valid) begin
                    load_wdt <= 1'b0;          // 有进展(读进字节/写出像素)就重新计时
                end else if (auto_ms) begin
                    load_wdt <= load_wdt + 1'b1;
                end
            end else begin
                load_wdt <= 1'b0;
            end

            // === 请求看门狗 ===
            if ((next_req_pending || prev_req_pending) && !load_busy) begin
                if (pend_wdt >= PEND_WDT_MS) begin
                    next_req_pending <= 1'b0;
                    prev_req_pending <= 1'b0;
                    pend_wdt         <= 1'b0;
                    load_recover     <= 1'b1;
                end else if (auto_ms) begin
                    pend_wdt <= pend_wdt + 1'b1;
                end
            end else begin
                pend_wdt <= 1'b0;
            end

            // 上电后自动发起一次"FAT32 卷扫描"
            // 等SDRAM初始化完成(Sdr_init_done)才启动扫描，避免SDRAM未就绪时丢写数据
            if (!scan_kicked && Sdr_init_done) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_found_latched   <= 1'b0;
                first_image_committed <= 1'b0;
                bad_bits              <= {NP{1'b0}};
                bad_cnt               <= 5'd0;
                goto_active           <= 1'b0;
                goto_reseek           <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 1'b0;
                img_ok                <= {NP{1'b0}};    // 缓存全部作废（缩略图也作废）
                bank_ok               <= 6'd0;
                th_ok                 <= {NP{1'b0}};
                strip_open            <= 1'b0;
                load_prep             <= 2'd0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                prev_req_pending<=1'b0;
                load_wdt              <= 1'b0;
                pend_wdt              <= 1'b0;
                wrfin_level_consumed  <= wrfin_tgl_sync[2];
                write_req             <= 1'b0;
                use_streamer          <= 1'b0;   // 重扫从扫描器开始
                sub_phase             <= 1'b0;
                scan_finished         <= 1'b0;
                fs_err_code           <= 4'd0;   // 重新扫描，旧细分码清除

            end else begin
                // write_req 清除: 等到 SDRAM 控制器应答即清;
                // 放在置位语句之前, 新加载触发时置位优先(后赋值生效)
                if (write_req_ack)
                    write_req <= 1'b0;

                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
                // === 按键分流 ===
                // 列表可信 → 正常记"下一张"请求；
                // 列表不可信(发生过读超时) 或 一张好图都没有 → 按键改为触发恢复：
                //   软件复位SD控制器(重新初始化) → sd_init_done重新拉高后
                //   上电扫描分支自动重扫并显示第一张图，全程无需再按键。
                // scan_done 是单拍脉冲, 这里用锁存的 scan_finished 电平判断
                // 后台预加载期间也接受按键(已缓存的图零延迟切过去)
                // 胶片条里推进中也收（直接改目标，以最新一次为准）
                if (key_next_press && display_valid && (live_cnt != 3'd0) && !need_rescan
                    && (!goto_active || strip_open))
                    next_req_pending <= 1'b1;
                else if (key_next_press && sd_init_done && scan_finished && !load_busy &&
                         (need_rescan || (live_cnt == 3'd0))) begin
                    sd_rst     <= 1'b1;
                    sd_rst_cnt <= SD_RST_MS;
                end

                // 黑屏时卡出错，自动重扫（见 scan_failed 声明处的说明）
                if (!display_valid && sd_init_done && scan_finished && !load_busy && !sd_rst
                    && (need_rescan || scan_failed)) begin
                    if (auto_ms) begin
                        if (arescan_cnt >= AUTO_RESCAN_MS - 1) begin
                            arescan_cnt <= 1'b0;
                            sd_rst      <= 1'b1;
                            sd_rst_cnt  <= SD_RST_MS;
                        end else begin
                            arescan_cnt <= arescan_cnt + 1'b1;
                        end
                    end
                end else begin
                    arescan_cnt <= 1'b0;
                end

                // KEY4（图片层）：长按 1 秒 = 上一张；短按（松手）= 自动轮播开/关
                if (key4_long_pic && display_valid && (live_cnt != 3'd0) && !goto_active)
                    prev_req_pending <= 1'b1;
                if (strip_prev && display_valid && (live_cnt != 3'd0) && !need_rescan)   // 胶片条里 KEY4 = 上一张
                    prev_req_pending <= 1'b1;
                if (key4_short_pic && (live_cnt > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 1'b0;
                end

                // 自动播放计数（毫秒数，周期可调，周期一变从头计）
                if (auto_play_en && display_valid && !goto_active && first_image_committed && (live_cnt > 3'd1)
                    && !per_changed && !strip_open && !sleep_s) begin   // 胶片条开着、定时关机时不计时
                    if (auto_timer)
                        auto_cnt <= 1'b0;
                    else if (auto_ms)
                        auto_cnt <= auto_cnt + 1'b1;
                end else begin
                    auto_cnt <= 1'b0;
                end
                // 随音乐档：歌播完先记下，等能换图时换（胶片条开着、定时关机、轮播关了不记）
                if (!auto_play_en || !per_mus_wav || strip_open || sleep_s)
                    song_end_pend <= 1'b0;
                else if (wav_song_end)
                    song_end_pend <= 1'b1;

                // ================================================================
                // 推进决策: 首图 / 下一张 / 上一张 / 自动播放
                // 这里只决定"去第几张"(goto_img, 由 nav 算好的结果给), 不直接启动加载;
                // 缓存里有就立刻切, 没有由后面的"加载选择"按优先级先加载它。
                // ================================================================
                // 卡出过错(need_rescan)就不再自动重试首图，由自动重扫或按 KEY2 恢复
                if (first_found_latched && !first_image_committed && !goto_active
                    && (live_cnt != 3'd0) && !need_rescan && nav_valid) begin
                    // 首图: 去第一张好图(若它是坏图, 错误块会让 nav 再往后找)
                    goto_active      <= 1'b1;
                    goto_img         <= nav_first;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 1'b0;
                end
                else if (display_valid && (!goto_active || strip_open) && (gap_cnt == 1'b0) && nav_valid
                         && next_req_pending && (live_cnt != 3'd0)) begin
                    // 下一张: 后面最近的好图, 到末尾回绕到开头
                    goto_active      <= 1'b1;
                    goto_img         <= nav_nxt;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 1'b0;
                end
                else if (display_valid && (!goto_active || strip_open) && (gap_cnt == 1'b0) && nav_valid
                         && prev_req_pending && (live_cnt != 3'd0)) begin
                    // 上一张: 前面最近的好图, 开头再按回绕到末尾
                    goto_active      <= 1'b1;
                    goto_img         <= nav_prv;
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 1'b0;
                end
                else if (display_valid && !goto_active && (gap_cnt == 1'b0) && nav_valid
                         && auto_play_en && auto_tick && (live_cnt > 3'd1) && !need_rescan && !strip_open) begin
                    // 自动播放下一张: 与"下一张"同规则
                    goto_active   <= 1'b1;
                    goto_img      <= nav_nxt;
                    auto_cnt      <= 1'b0;
                    song_end_pend <= 1'b0;
                end
                // 上电启动时目标是坏图: nav 已把它算作坏图, 改去它后面最近的好图
                else if (goto_active && goto_reseek && nav_valid) begin
                    goto_img    <= nav_nxt;
                    goto_reseek <= 1'b0;
                end

                // === 推进落地: 目标已经在缓存里 -> 只换显示的 bank, 不读卡 ===
                if (goto_active && !goto_reseek && focus_cached)
                    show_img(focus, foc_bank);

                // === 加载选择: 缺哪张就加载哪张, 见"缓存"一节 ===
                //   这一拍挑好写哪块 bank(pick_bank), 同时把它标成无效、改对照表(原先在里面的那张图算没缓存),
                //   所以从这一拍起它不会再被显示; 图片表读地址给 load_img;
                //   扫描器图片表第 2 拍出数据(序号 -> 排序表 -> 簇号/字节数), 所以隔 2 拍再启动读文件和缩放
                if (load_prep != 2'd0) begin
                    load_prep <= load_prep - 2'd1;
                    if (load_prep == 2'd1)
                        start_load;
                end else if (!load_busy && !sub_phase && bmp_ready && (gap_cnt == 1'b0) && use_streamer && pick_any) begin
                    load_img      <= pick_img;
                    write_buf_idx <= pick_bank;
                    load_prep     <= 2'd2;
                    bank_ok[pick_bank] <= 1'b0;
                    bank_img_v[pick_bank*4 +: 4] <= pick_img;
                    if (bank_ok[pick_bank])
                        img_ok[bank_img_v[pick_bank*4 +: 4]] <= 1'b0;    // 被换掉的那张(整帧没了, 缩略图还在)
                end

                // === 胶片条 打开 / 确认 / 取消（见文件头）===
                if (strip_open) begin
                    if (!display_valid) begin
                        strip_open <= 1'b0;                     // 黑屏（出错重扫）：收起
                    end else if (strip_cfm) begin
                        strip_open <= 1'b0;                     // 确认：停在焦点这张（还没缓存的照常读完再切）
                    end else if (key_strip || layer_s) begin
                        strip_open       <= 1'b0;               // 取消：回到打开时那张（一定已缓存）
                        next_req_pending <= 1'b0;
                        prev_req_pending <= 1'b0;
                        if (strip_orig != cur_img)
                            show_img(strip_orig, orig_bank);    // 和收起同一拍换回去（原图一直在硬保护里，还在 orig_bank）
                        else
                            goto_active <= 1'b0;                // 大图本来就是它：只停掉正在去的目标
                    end
                end else if (key_strip && !layer_s && display_valid && (live_cnt != 3'd0)) begin
                    strip_open <= 1'b1;
                    strip_orig <= cur_img;
                    orig_bank  <= disp_buf_idx;
                end
            end
        end
    end
end
// ===========================================================================
// === FAT32 模块实例 ===
// ===========================================================================

// ---------- FAT32 卷扫描器 ----------
fat32_volume_scanner #(.BMP_MAX(BMP_MAX[4:0])) u_scanner (
    .clk                 (clk),
    .rst_n               (fat_rst_n),     // 低有效; SD 软复位时一起清零
    .scan_start          (scan_start_pulse),
    .scan_busy           (scanner_busy),
    .scan_done           (scan_done),
    .fs_error            (scanner_fs_error),
    .partition_lba       (),
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .bmp_count           (img_cnt),       // 图片表(PIC 号码顺序, 文件头检查通过的)
    .bmp_bad_count       (scan_bad),
    .bmp_rd_addr         (load_img),
    .bmp_rd_cluster      (img_rd_cluster),
    .bmp_rd_size         (img_rd_size),
    .sector_req          (scanner_sector_req),
    .sector_lba          (scanner_sector_lba),
    .sector_busy         (scanner_sector_busy),
    .sector_valid        (scanner_sector_valid),
    .sector_byte         (scanner_sector_byte),
    .sector_byte_index   (scanner_sector_byte_index),
    .sector_done         (scanner_sector_done),
    .sector_error        (scanner_sector_error),
    .wav_count           (wav_count),
    .wav_rd_addr         (wav_rd_addr),
    .wav_rd_cluster      (wav_rd_cluster),
    .wav_rd_size         (wav_rd_size),
    .sub_found           (scan_sub_found),
    .bmp_rd_sub          (sub_phase)           // 读 SUB.BMP 期间图片表读口给它的簇号 / 字节数（整个文件读完前不变）
);

// ---------- FAT32 文件流读取器 ----------
fat32_file_streamer u_streamer (
    .clk                 (clk),
    .rst_n               (fat_rst_n & ~img_st_abort),   // 按键插队时复位 1 拍 = 停止读这个文件
    .file_start          (load_start_pulse | sub_start),
    .hold                (scl_hold & ~sub_phase),  // 缩放模块的字节缓冲快满了, 先别读下一个扇区
    .first_cluster       (load_sector),        // 起始簇号（sub_phase 时是 SUB.BMP 的）
    .file_size           (load_file_size),     // 文件字节数(簇链可提前结束)
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .file_busy           (streamer_file_busy),
    .file_done           (streamer_file_done), // 单拍脉冲
    .file_valid          (streamer_byte_valid),
    .file_byte           (streamer_byte),
    .file_byte_offset    (streamer_byte_offset),
    .file_remaining      (),
    .fs_error            (streamer_fs_error),
    .sector_req          (streamer_sector_req),
    .sector_lba          (streamer_sector_lba),
    .sector_busy         (streamer_sector_busy),
    .sector_valid        (streamer_sector_valid),
    .sector_byte         (streamer_sector_byte),
    .sector_byte_index   (streamer_sector_byte_index),
    .sector_done         (streamer_sector_done),
    .sector_error        (streamer_sector_error)
);

// ---------- SUB.BMP -> 字幕条 RAM ----------
sub_loader u_sub_loader (
    .clk      (clk),
    .rst_n    (fat_rst_n),
    .start    (sub_start | scan_start_pulse),   // 每次重扫先清掉（新卡没有 SUB.BMP 就回到默认标语）
    .in_valid (streamer_byte_valid & sub_phase),
    .in_byte  (streamer_byte),
    .in_off   (streamer_byte_offset[14:0]),
    .we       (sub_we),
    .waddr    (sub_waddr),
    .wdata    (sub_wdata),
    .ok       (sub_ok),
    .w        (sub_w),
    .top      (sub_top)
);

// ---------- 读歌(第二个 FAT32 文件流读取器) + WAV 播放控制 ----------
wire        aud_st_start, aud_st_abort;
wire [31:0] aud_st_cluster, aud_st_size;
wire        aud_st_busy, aud_st_done, aud_st_valid;
wire [7:0]  aud_st_err, aud_st_byte;
wire [31:0] aud_st_remaining;
wire        aud_sector_valid = arb_sector_valid & aud_own;
wire        aud_sector_done  = arb_sector_done  & aud_own;
wire [7:0]  aud_sector_error = aud_own ? arb_sector_error : 8'd0;

fat32_file_streamer #(
    .MAX_VISITED_CLUSTERS (1 << 20)        // 歌可能很长(4KB 簇时 4GB), 簇链保险丝放大
) u_aud_streamer (
    .clk                 (clk),
    .rst_n               (fat_rst_n & ~aud_st_abort),   // 停歌 = 复位它(正在读的扇区由仲裁器让它读完)
    .file_start          (aud_st_start),
    .hold                (1'b0),
    .first_cluster       (aud_st_cluster),
    .file_size           (aud_st_size),
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .file_busy           (aud_st_busy),
    .file_done           (aud_st_done),
    .file_valid          (aud_st_valid),
    .file_byte           (aud_st_byte),
    .file_byte_offset    (),
    .file_remaining      (aud_st_remaining),
    .fs_error            (aud_st_err),
    .sector_req          (aud_sector_req),
    .sector_lba          (aud_sector_lba),
    .sector_busy         (aud_busy_v),
    .sector_valid        (aud_sector_valid),
    .sector_byte         (arb_sector_byte),
    .sector_byte_index   (arb_sector_byte_index),
    .sector_done         (aud_sector_done),
    .sector_error        (aud_sector_error)
);

wire [3:0]  mus_idx4, mus_cnt4;
assign mus_idx = {3'd0, mus_idx4};
assign mus_cnt = {3'd0, mus_cnt4};
wav_sd_ctrl #(
    .CLK_HZ          (CLK_FREQ_HZ),
    .ERR_SHOW_MS     (ERR_SHOW_MS),
    .NW              (4)                  // 歌单最多 12 首
) u_wav_ctrl (
    .clk             (clk),
    .ms_tick         (auto_ms),           // 共用 1 毫秒节拍（错误码显示计时）
    .rst_n           (fat_rst_n),
    .list_load       (scan_done),
    .list_count      (wav_count),
    .pl_addr         (wav_rd_addr),
    .pl_cluster      (wav_rd_cluster),
    .pl_size         (wav_rd_size),
    .song_next       (song_next),
    .song_prev       (song_prev),
    .song_hold       (song_hold),
    .loop_one        (mus_loop),
    .st_start        (aud_st_start),
    .st_cluster      (aud_st_cluster),
    .st_size         (aud_st_size),
    .st_abort        (aud_st_abort),
    .st_busy         (aud_st_busy),
    .st_done         (aud_st_done),
    .st_err          (aud_st_err),
    .st_valid        (aud_st_valid),
    .st_byte         (aud_st_byte),
    .st_remaining    (aud_st_remaining),
    .ring_we         (ring_we),
    .ring_wdata      (ring_wdata),
    .ring_space_ok   (ring_space_ok),
    .ring_empty      (ring_empty),
    .ring_flush      (ring_flush),
    .ring_flush_busy (ring_flush_busy),
    .ring_eos        (ring_eos),
    .sd_want         (aud_want),
    .wav_active      (wav_active),
    .song_end        (wav_song_end),
    .show_idx        (mus_idx4),
    .show_cnt        (mus_cnt4),
    .tot_known       (mus_tot_known),
    .tot_bcd         (mus_tot_bcd),
    .err_code        (mus_err),
    .dbg_state       ()
);


// ---------- SD 卡接口适配层(脉冲式握手 <-> 电平式 sd_card_top) ----------
sd_sector_adapter #(
    .SECTOR_TIMEOUT_CYCLES (SECTOR_TIMEOUT_CYCLES)
) u_adapter (
    .clk                    (clk),
    .rst_n                  (fat_rst_n),
    // 主机侧(仲裁器输出)
    .sector_req             (arb_sector_req),
    .sector_addr            (arb_sector_lba),
    .sector_busy            (arb_sector_busy),
    .sector_valid           (arb_sector_valid),
    .sector_byte            (arb_sector_byte),
    .sector_byte_index      (arb_sector_byte_index),
    .sector_done            (arb_sector_done),
    .sector_error           (arb_sector_error),
    // sd_card_top 侧
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_data       (sd_sec_read_data)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst | sd_rst),   // 拔卡恢复时软件复位SD控制器（重新初始化）
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule
