`timescale 1ns/1ps

// media_frame_reader：整帧缓存读取器。mem_clk 域发 SDRAM 读 burst，经异步读 FIFO + 2 级预取送到像素域（pix_clk）。
//   每帧在 frame_start 后锁存要显示的块号（6 块整帧缓存之一）、转场几何参数和胶片条参数，按行读出一帧。
//
// 读写互斥：SDRAM 控制器（加密 IP sdr_as_ram）一次只能服务读或写中的一种，读 burst 和写 burst 不能重叠：
//   wr_busy : 写侧正在（或这一拍就要开始）写 burst，reader 不发起新的读 burst
//   rd_busy : reader 正在发读 burst 或处于 burst 后的间隔期（BURST_GAP_CYCLES=12 拍收尾），写侧不开始写 burst
// rd_busy 只用寄存器，wr_busy 可以是组合信号，两边不会形成组合环。
//
// 读 FIFO 复位握手（跨时钟域安全）：
//   mem 域发起复位：fifo_reset_count_mem 置 16（400ns，保证像素域一定能采到）
//   跨域信号全部走两级同步：pix_rst_n -> mem，mem 复位请求 -> pix，pix 释放 -> mem
//   读 burst 只在两侧都确认已释放复位后才发起（fifo_ready_mem），写 FIFO 同样只看 mem 域信号
//   FIFO IP 的 rst 端口是异步输入、内部自带两侧复位同步器，所以 rst 仍可由各域寄存器相或
//
// 转场：行内分段读新/旧图（每帧读的总量仍是一张图，带宽不变）
//   像素域在帧起点给出 active_index(旧图) / trans_new_index(新图) 和几何参数（含义见 video_transition.v）：
//     gmode 0 矩形 [r0,r1)x[c0,c1) 读新图   1 横百叶窗 行%SLAT<v（2、3 保留未用，按矩形处理）
//     new/old_dy、new/old_dx：源坐标 = 屏幕坐标 + 偏移（推移）
//   规划器（planner）逐段规划。每行开头算出本行的新图区间，然后从当前列起取一段：
//       段长 = min(到本段新/旧边界, 到下一个 BURST_SIZE(128) 字对齐边界)
//   【整块对齐读】板上实测：burst 起点不是 128 对齐（推移、中心展开）时，加密 SDRAM 控制器读回的
//     数据会一小段一小段错位（横向条纹/撕裂，字数却不少，错误码为 0），推断是突发在组内回绕。
//     所以每个读 burst 一律是"128 字对齐、读满 128 字"，只把本段需要的 [k0, k1) 写进 FIFO，其余丢弃。
//     每个已发出的 burst 在返回队列里记一项 {k0, k1}，数据按顺序返回时按序号筛选。
//     代价：转场时每行最多读 7 块（896 字，不转场为 640），仍在 SDRAM 带宽以内；不转场 / 淡入淡出 /
//     上下擦除 / 横百叶窗每行是 5 块且全部保留。
//   规划在上一个 burst 的发送/间隔期间完成；下一段的长度先寄存一拍再参与 FIFO 余量判断。
//   请求跨域多加一级同步（sync3）：数据在 sync2 已稳定后才采用，消除多位数据与 toggle 的竞争。
//
// 胶片条（STRIP_EN = 1）：像素域在帧起点给出 strip_on、5 格的 {读缩略图, 缩略图格号[3:0], 翻转}（每格 6 位，
//   slots 共 30 位；16 格缩略图按图片序号存放）和本帧字数 frame_words。
//   胶片条那些行（row >= STRIP_Y0）不读大图，只读要显示的格的缩略图：每格每行正好一个"128 字对齐、读满 128 字"的整块
//   （缩略图行距 128 字，写入时就这样排好），焦点格保留前 112 个字，两边的格保留其中 84 个（每 4 个丢第 4 个 = 横向 4 取 3）；
//   两边的格纵向也 4 取 3：第 r 行读第 r + r/3 行。正存的 BMP 缩略图在 SDRAM 里是倒着放的（翻转 = 1），读第 83 - 行。
//   一行最多 5 块（和平时一样）。像素域只在缩略图像素上取 FIFO，所以本帧字数 = 上面的大图行 + 缩略图字数
//   （像素域用 filmstrip 给的 frame_words 核对下溢；mem 域重装时按 {strip_on, slots} 自己算 fw_mem，算法相同）。
//   {strip_on, slots} 跨域：请求翻转前就写好、一帧之内不变，mem 域在 request_sync3 翻转时直接取（不过同步寄存器）。
//   strip_on = 0 时本帧字数 = IMAGE_WORDS。
//
// 参数 RGB_IN_HIGH：SDRAM 里一个像素字的格式。
//   0 = {8'h00, RGB}
//   1 = {RGB, 8'h00}（本工程 sd_card_bmp / frame_write_path 写入的格式）
module media_frame_reader #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer FIFO_DEPTH = 512,
    parameter integer FIFO_MARGIN = 8,
    parameter integer BURST_SIZE = 128,
    parameter integer BURST_GAP_CYCLES = 12,
    parameter integer MAX_INFLIGHT_BURSTS = 2,
    parameter integer DIAG_RETURN_PATTERN = 0,
    parameter integer DIAG_SYNTH_STREAM = 0,
    parameter integer RGB_IN_HIGH = 0,
    parameter integer LINE_WORDS = 640,      // 每行像素数
    parameter integer SLAT = 40,             // 横百叶窗每片行数
    parameter integer PAGE_WORDS = 256,      // SDRAM 一行（页）的字数（128 字整块读不会跨页，保留作说明）
    // 胶片条（几何见 display/strip_layout.vh，由 media_display_path 传进来）
    parameter integer STRIP_EN = 0,
    parameter integer STRIP_Y0 = 354,        // 这一行起不读大图
    parameter integer STRIP_FY0 = 390,       // 焦点格的行范围 [FY0, FY0 + THUMB_H)
    parameter integer STRIP_SY0 = 401,       // 两边格的行范围 [SY0, SY0 + STRIP_SH)
    parameter integer STRIP_SH = 63,
    parameter integer THUMB_W = 112,         // 焦点格每行保留的字数
    parameter integer THUMB_SW = 84,         // 两边格每行保留的字数
    parameter integer THUMB_H = 84,
    parameter integer THUMB_BASE = 1843200,  // 第 k 格缩略图 = THUMB_BASE + k x THUMB_WORDS，行距 BURST_SIZE（k = 图片序号 0~15）
    parameter integer THUMB_WORDS = 10752
) (
    input  wire                  mem_clk,
    input  wire                  mem_rst_n,
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    output reg                   app_rd_en,
    output reg  [ADDR_WIDTH-1:0] app_rd_addr,
    input  wire                  sdram_rd_valid,
    input  wire [31:0]           sdram_rd_data,
    input  wire                  wr_busy,          // 写侧忙（读写互斥）
    output wire                  rd_busy,          // 读侧忙（寄存器输出）

    input  wire                  pix_clk,
    input  wire                  pix_rst_n,
    input  wire                  frame_start,
    input  wire [2:0]            active_index,      // 当前（转场时为旧图）整帧缓存块号 0~5
    input  wire [2:0]            trans_new_index,   // 转场新图的块号
    input  wire [1:0]            trans_gmode,       // 转场几何参数（见文件头）
    input  wire [9:0]            trans_r0, trans_r1, trans_c0, trans_c1, trans_v,
    input  wire [10:0]           trans_new_dy, trans_old_dy, trans_new_dx, trans_old_dx,
    input  wire                  strip_on,          // 本帧显示胶片条（像素域，帧起点后一拍取）
    input  wire [29:0]           strip_slots,       //   5 x {读缩略图, 缩略图格号[3:0], 翻转}，[5:0] = 第 0 格（最左）
    input  wire [20:0]           frame_words,       //   本帧要送出的字数
    input  wire                  pixel_consume,
    output wire [23:0]           pixel_rgb,
    output wire                  pixel_valid,
    output reg                   underflow,

    output reg                   reader_busy,
    output reg                   frame_read_done,
    output reg  [2:0]            reader_index,
    output reg  [7:0]            reader_error
);
    localparam [7:0] ERR_NONE          = 8'h00;
    localparam [7:0] ERR_FRAME_OVERRUN = 8'h61;
    localparam [7:0] ERR_FIFO_OVERFLOW = 8'h62;
    localparam [7:0] ERR_UNDERFLOW     = 8'h63;
    localparam [7:0] ERR_SHORT_FRAME   = 8'h64;
    localparam integer FIFO_COUNT_W = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam integer BURST_COUNT_W =
        (BURST_SIZE < 2) ? 1 : $clog2(BURST_SIZE + 1);
    localparam integer INFLIGHT_COUNT_W =
        (MAX_INFLIGHT_BURSTS * BURST_SIZE < 2) ? 1 :
        $clog2(MAX_INFLIGHT_BURSTS * BURST_SIZE + 1);

    reg request_pending_pix;
    reg request_toggle_pix;
    reg [2:0] request_index_pix;
    reg [2:0] request_new_pix;
    localparam integer GEO_W = 2 + 5 * 10 + 4 * 11;
    reg [GEO_W-1:0] request_geo_pix;
    localparam integer SGEO_W = 1 + 30;        // {strip_on, slots}（翻转握手传数据，mem 域直接取，见文件头）
    reg [SGEO_W-1:0] request_sgeo_pix;
    reg [20:0] exp_words_pix;                  // 这一帧像素域要取的字数（下一个帧起点核对）
    reg frame_armed_pix;
    reg [ADDR_WIDTH:0] consumed_count_pix;

    reg request_sync1_mem;
    reg request_sync2_mem;
    reg request_sync3_mem;
    reg [2:0] new_sync1_mem, new_sync2_mem, reload_new_mem, rd_new_mem;
    reg [GEO_W-1:0] geo_sync1_mem, geo_sync2_mem, reload_geo_mem;
    reg [SGEO_W-1:0] reload_sgeo_mem;

    // ---- 规划器 ----
    localparam integer V_LINES = IMAGE_WORDS / LINE_WORDS;
    localparam [1:0] G_RECT = 2'd0, G_HBLIND = 2'd1;
    localparam [2:0] P_IDLE = 3'd0, P_LINE = 3'd2, P_SCAN = 3'd3, P_CALC = 3'd4, P_READY = 3'd5;
    reg [1:0]  gm_mem;
    reg [9:0]  gr0_mem, gr1_mem, gc0_mem, gc1_mem, gv_mem;
    reg [2:0]  pl_state;
    reg [10:0] col_mem;                  // 下一段起点（屏幕列）
    reg [9:0]  row_mem, slat_mem;        // 当前行、行%SLAT
    reg [ADDR_WIDTH-1:0] lbn_mem, lbo_mem; // 本行新/旧图的"列 0"地址（已含块基址、行偏移、列偏移）
    reg [10:0] lo_mem, hi_mem;           // 本行新图区间 [lo,hi)（矩形）
    reg        line_new_mem;             // 本行整行读新图（横百叶窗）
    reg [ADDR_WIDTH-1:0] plan_addr_mem, burst_addr_mem;   // 本块起点（128 对齐）
    reg [BURST_COUNT_W-1:0] plan_len_mem;                  // 本块保留的字数（k1 - k0）
    reg [BURST_COUNT_W-1:0] plan_k0_mem;                   // 本块从第 k0 个字开始保留
    reg                     plan_dec_mem;                  // 本块每 4 个字丢第 4 个（两边的缩略图格）
    // 胶片条（帧起点重装时取）
    reg        st_mem;                       // 本帧显示胶片条
    reg [29:0] sk_mem;                       // 5 格 {读缩略图, 缩略图格号, 翻转}
    reg [20:0] fw_mem;                       // 本帧字数
    reg [2:0]  sj_mem;                       // 本行正在看第几格
    reg        srf_mem, srs_mem;             // 本行有焦点格 / 两边格
    reg [6:0]  ssr_mem;                      // 两边格本行的源行（r + r/3）
    reg [1:0]  ss3_mem;                      // 两边格行号 r mod 3
    // 返回队列：已发出、数据还没全部返回的 burst 的保留区间（最多 MAX_INFLIGHT_BURSTS 个在途）
    // 发起新 burst 时在途字数 <= (MAX_INFLIGHT_BURSTS-1) x BURST_SIZE，而上一块已经整块发完，
    //   所以队列里最多 MAX_INFLIGHT_BURSTS 项，深度按它取（默认 2 项，指针 1 位）以节省 LUT
    localparam integer RQ_AW = (MAX_INFLIGHT_BURSTS <= 2) ? 1 : 2;
    reg [BURST_COUNT_W-1:0] rq_k0 [0:(1<<RQ_AW)-1];
    reg [BURST_COUNT_W-1:0] rq_k1 [0:(1<<RQ_AW)-1];
    reg                     rq_dec [0:(1<<RQ_AW)-1];
    reg [RQ_AW-1:0] rq_wr_mem, rq_rd_mem;
    reg [BURST_COUNT_W-1:0] ret_idx_mem;                   // 当前返回字在本块里的序号
    reg        plan_valid_mem, plan_ok_mem;
    reg request_seen_mem;
    reg [2:0] index_sync1_mem;
    reg [2:0] index_sync2_mem;
    reg reload_pending_mem;
    reg [2:0] reload_index_mem;
    reg [4:0] fifo_reset_count_mem;
    reg [ADDR_WIDTH:0] issued_count_mem;
    reg [ADDR_WIDTH:0] returned_count_mem;   // 本帧还差几个保留字（倒计数）
    reg [INFLIGHT_COUNT_W-1:0] inflight_count_mem;
    reg inflight_empty_mem /* fehdl keep="true" */;
    reg burst_active_mem;
    // burst 一律读满 BURST_SIZE 个字，起点按 BURST_SIZE 对齐，所以"地址低位 == BURST_SIZE-1"就是最后一个，
    //   不需要单独的 burst 字数计数器。burst 内只给地址低位加 1（最后一拍的进位不会被用到：
    //   burst_addr_mem 只在发请求时输出，下一个 burst 开始时整个重装）
    localparam integer BLW = (BURST_SIZE < 2) ? 1 : $clog2(BURST_SIZE);
    reg [7:0] burst_gap_count_mem;
    reg [ADDR_WIDTH:0] diag_written_count_mem;
    reg [9:0] diag_column_mem;
    reg [23:0] diag_rgb_mem;
    reg [1:0] fifo_release_mem;
    reg [1:0] fifo_release_pix;

    reg [1:0] pix_rst_sync_mem;       // pix_rst_n -> mem 域
    reg       fifo_reset_req_mem_q;   // mem 域复位请求（寄存器，给像素域同步用）
    reg [1:0] fifo_reset_req_sync_pix;
    reg [1:0] fifo_release_pix_sync_mem;
    wire fifo_reset_req_mem = !pix_rst_sync_mem[1] || (fifo_reset_count_mem != 0);
    wire fifo_reset_req_pix = fifo_reset_req_sync_pix[1];
    wire fifo_ready_mem = !fifo_reset_req_mem && fifo_release_mem[1] &&
                          fifo_release_pix_sync_mem[1];
    wire fifo_reset = fifo_reset_req_mem_q || !fifo_release_mem[1] ||
                      fifo_reset_req_pix || !fifo_release_pix[1];
    wire fifo_full;
    wire fifo_empty;
    wire fifo_almost_full;
    wire [FIFO_COUNT_W-1:0] fifo_wrusedw;
    wire [31:0] fifo_raw_data;
    wire fifo_raw_valid;
    wire fifo_raw_read_en;
    wire prefetch_valid;
    wire [31:0] prefetch_data;
    wire pixel_pop = pixel_consume && prefetch_valid;
    wire [ADDR_WIDTH:0] inflight_words =
        {{(ADDR_WIDTH+1-INFLIGHT_COUNT_W){1'b0}}, inflight_count_mem};
    wire [20:0] fw_cur = STRIP_EN ? fw_mem : IMAGE_WORDS;       // 本帧字数（胶片条帧比整帧少）
    wire [20:0] fw_cur_n;                                       // 重装后的 fw_cur（倒计数的起点，在 rl_words 后面赋值）
    // returned_count_mem 是"本帧还差几个字"倒计数（重装时置本帧字数），剩 1 个时这一字就是最后一个，
    //   不需要 fw - 1 的减法和宽位比较
    wire final_return = returned_count_mem == 1;
    wire [ADDR_WIDTH+1:0] reserved_words =
        {{(ADDR_WIDTH+2-FIFO_COUNT_W){1'b0}}, fifo_wrusedw} +
        {{1{1'b0}}, inflight_words};
    wire [ADDR_WIDTH:0] next_burst_words =
        {{(ADDR_WIDTH+1-BURST_COUNT_W){1'b0}}, plan_len_mem};
    wire [ADDR_WIDTH+2:0] reserved_after_burst =
        {{1{1'b0}}, reserved_words} + {{2{1'b0}}, next_burst_words};
    wire inflight_window_ok = (MAX_INFLIGHT_BURSTS == 1) ?
        inflight_empty_mem :
        ((inflight_words + BURST_SIZE) <=              // 在途按整块计
         (MAX_INFLIGHT_BURSTS * BURST_SIZE));
    wire fifo_reservation_ok =
        reserved_after_burst <= (FIFO_DEPTH - FIFO_MARGIN);
    reg inflight_window_ok_q;
    reg fifo_reservation_ok_q;
    assign rd_busy = burst_active_mem || burst_gap_count_mem != 0;
    wire start_burst = !DIAG_SYNTH_STREAM && reader_busy && controller_init_done &&
                       !wr_busy &&
                       fifo_ready_mem && !burst_active_mem &&
                       burst_gap_count_mem == 0 &&
                       issued_count_mem != fw_cur && plan_ok_mem &&
                       inflight_window_ok_q && fifo_reservation_ok_q;
    // 整块对齐读：帧的最后一个保留字返回后（reader_busy 已清），当前块仍发完，保证每个 burst 都是整块
    wire issue_allowed = !DIAG_SYNTH_STREAM && burst_active_mem;
    wire diag_stream_write = DIAG_SYNTH_STREAM && reader_busy &&
                             fifo_ready_mem && !fifo_full;
    wire ret_keep = (ret_idx_mem >= rq_k0[rq_rd_mem]) && (ret_idx_mem < rq_k1[rq_rd_mem]) &&
                    !(rq_dec[rq_rd_mem] && ret_idx_mem[1:0] == 2'b11);   // 两边的缩略图格：横向 4 取 3
    wire fifo_write_enable = DIAG_SYNTH_STREAM ? diag_stream_write :
                             (sdram_rd_valid && ret_keep && fifo_ready_mem && !fifo_full);
    wire [31:0] fifo_write_data = (DIAG_RETURN_PATTERN || DIAG_SYNTH_STREAM) ?
                                  {8'h00, diag_rgb_mem} : sdram_rd_data;

    always @* begin
        if (diag_column_mem < 10'd80)
            diag_rgb_mem = 24'hffffff;
        else if (diag_column_mem < 10'd160)
            diag_rgb_mem = 24'hffff00;
        else if (diag_column_mem < 10'd240)
            diag_rgb_mem = 24'h00ffff;
        else if (diag_column_mem < 10'd320)
            diag_rgb_mem = 24'h00ff00;
        else if (diag_column_mem < 10'd400)
            diag_rgb_mem = 24'hff00ff;
        else if (diag_column_mem < 10'd480)
            diag_rgb_mem = 24'hff0000;
        else if (diag_column_mem < 10'd560)
            diag_rgb_mem = 24'h0000ff;
        else
            diag_rgb_mem = 24'h000000;
    end

    // ---------------- 规划器（组合部分） ----------------
    // 几何参数打包顺序：{gmode, r0, r1, c0, c1, v, new_dy, old_dy, new_dx, old_dx}
    wire [1:0]  rl_gm  = reload_geo_mem[95:94];
    wire [9:0]  rl_r0  = reload_geo_mem[93:84];
    wire [9:0]  rl_r1  = reload_geo_mem[83:74];
    wire [9:0]  rl_c0  = reload_geo_mem[73:64];
    wire [9:0]  rl_c1  = reload_geo_mem[63:54];
    wire [9:0]  rl_v   = reload_geo_mem[53:44];
    wire [10:0] rl_ndy = reload_geo_mem[43:33];
    wire [10:0] rl_ody = reload_geo_mem[32:22];
    wire [10:0] rl_ndx = reload_geo_mem[21:11];
    wire [10:0] rl_odx = reload_geo_mem[10:0];

    function [ADDR_WIDTH-1:0] sext11;     // 11 位补码 -> 地址宽度（地址按模 2^ADDR_WIDTH 运算）
        input [10:0] v;
        begin
            sext11 = {{(ADDR_WIDTH-11){v[10]}}, v};
        end
    endfunction

    function [ADDR_WIDTH-1:0] line_off;   // 行偏移 × LINE_WORDS：移位相加，不占 DSP
        input [10:0] dy;
        integer i;
        begin
            line_off = {ADDR_WIDTH{1'b0}};
            for (i = 0; i < 11; i = i + 1)
                if ((LINE_WORDS >> i) & 1) line_off = line_off + (sext11(dy) << i);
        end
    endfunction

    // 当前列所在段：读新图还是旧图、段尾、地址、段长
    wire        in_iv    = (col_mem >= lo_mem) && (col_mem < hi_mem);
    wire        seg_new  = (gm_mem == G_HBLIND) ? line_new_mem : in_iv;
    wire [10:0] seg_end  = (gm_mem == G_HBLIND) ? LINE_WORDS :
                           (col_mem < lo_mem) ? lo_mem : (col_mem < hi_mem) ? hi_mem : LINE_WORDS;
    wire [ADDR_WIDTH-1:0] seg_addr = (seg_new ? lbn_mem : lbo_mem) + col_mem;
    wire [10:0] seg_len  = seg_end - col_mem;
    wire [10:0] seg_k0   = seg_addr % BURST_SIZE;                  // 段起点在 128 字块里的位置
    wire [10:0] blk_room = BURST_SIZE - seg_k0;                    // 到块尾还有几个字
    wire [10:0] plan_len = (seg_len < blk_room) ? seg_len : blk_room;
    wire [10:0] col_after = col_mem + plan_len_mem;

    function [ADDR_WIDTH-1:0] bank_base;       // 整帧缓存块 k 基地址 = k x 一帧字数（6 块）
        input [2:0] index;
        begin
            bank_base = index * IMAGE_WORDS;
        end
    endfunction

    // ---------------- 胶片条：本帧字数（重装时算）----------------
    wire [2:0]  rl_side = {2'd0, reload_sgeo_mem[5]} + {2'd0, reload_sgeo_mem[11]} +
                          {2'd0, reload_sgeo_mem[23]} + {2'd0, reload_sgeo_mem[29]};
    reg  [14:0] rl_side_w;
    always @* begin
        case (rl_side)
            3'd0: rl_side_w = 15'd0;
            3'd1: rl_side_w = THUMB_SW * STRIP_SH;
            3'd2: rl_side_w = 2 * THUMB_SW * STRIP_SH;
            3'd3: rl_side_w = 3 * THUMB_SW * STRIP_SH;
            default: rl_side_w = 4 * THUMB_SW * STRIP_SH;
        endcase
    end
    wire [20:0] rl_words = !reload_sgeo_mem[30] ? IMAGE_WORDS :
                           STRIP_Y0 * LINE_WORDS + (reload_sgeo_mem[17] ? THUMB_W * THUMB_H : 0) + rl_side_w;
    assign fw_cur_n = STRIP_EN ? rl_words : IMAGE_WORDS;

    // ---------------- 胶片条：本行第 sj 格的读块 ----------------
    wire        strip_row = STRIP_EN && st_mem && (row_mem >= STRIP_Y0);
    reg  [5:0]  sk_cur;                                              // {读缩略图, 缩略图格号, 翻转}
    always @* begin
        case (sj_mem)
            3'd0: sk_cur = sk_mem[5:0];
            3'd1: sk_cur = sk_mem[11:6];
            3'd2: sk_cur = sk_mem[17:12];
            3'd3: sk_cur = sk_mem[23:18];
            default: sk_cur = sk_mem[29:24];
        endcase
    end
    wire        sj_foc    = (sj_mem == 3'd2);
    wire        sj_hit    = sk_cur[5] && (sj_foc ? srf_mem : srs_mem); // 这一格本行要读
    wire [9:0]  frow      = row_mem - STRIP_FY0;
    wire [6:0]  srow      = sj_foc ? frow[6:0] : ssr_mem;            // 缩略图第几行（显示顺序）
    wire [6:0]  rsel      = sk_cur[0] ? (THUMB_H - 1) - srow : srow; // 正存的 BMP 缩略图倒着放
    wire [ADDR_WIDTH-1:0] tbase = THUMB_BASE + sk_cur[4:1] * THUMB_WORDS;
    wire [ADDR_WIDTH-1:0] taddr = tbase + {rsel, 7'd0};             // 行距 = BURST_SIZE = 128 字

    // Delay the request by one pixel clock so media_player_control can commit
    // active_index on the frame_start edge before this module captures it.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            request_pending_pix <= 1'b0;
            request_toggle_pix <= 1'b0;
            request_index_pix <= 3'd0;
            request_new_pix <= 3'd0;
            request_geo_pix <= {GEO_W{1'b0}};
            request_sgeo_pix <= {SGEO_W{1'b0}};
            exp_words_pix <= IMAGE_WORDS;
            frame_armed_pix <= 1'b0;
            consumed_count_pix <= 0;
            underflow <= 1'b0;
            fifo_release_pix <= 2'b00;
            fifo_reset_req_sync_pix <= 2'b11;
        end else begin
            fifo_reset_req_sync_pix <= {fifo_reset_req_sync_pix[0], fifo_reset_req_mem_q};
            if (fifo_reset_req_pix)
                fifo_release_pix <= 2'b00;
            else
                fifo_release_pix <= {fifo_release_pix[0], 1'b1};

            if (frame_start) begin
                if (frame_armed_pix && consumed_count_pix != exp_words_pix) begin
                    underflow <= 1'b1;
                end
                consumed_count_pix <= 0;
                request_pending_pix <= 1'b1;
            end else if (request_pending_pix) begin
                request_index_pix <= active_index;
                request_new_pix <= trans_new_index;
                request_geo_pix <= {trans_gmode, trans_r0, trans_r1, trans_c0, trans_c1, trans_v,
                                    trans_new_dy, trans_old_dy, trans_new_dx, trans_old_dx};
                request_sgeo_pix <= {strip_on, strip_slots};
                exp_words_pix <= STRIP_EN ? frame_words : IMAGE_WORDS;
                request_toggle_pix <= ~request_toggle_pix;
                request_pending_pix <= 1'b0;
                frame_armed_pix <= 1'b1;
            end

            if (pixel_consume) begin
                if (prefetch_valid)
                    consumed_count_pix <= consumed_count_pix + 1'b1;
                else if (frame_armed_pix)
                    underflow <= 1'b1;
            end
        end
    end

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            request_sync1_mem <= 1'b0;
            request_sync2_mem <= 1'b0;
            request_sync3_mem <= 1'b0;
            request_seen_mem <= 1'b0;
            new_sync1_mem <= 3'd0; new_sync2_mem <= 3'd0; reload_new_mem <= 3'd0; rd_new_mem <= 3'd0;
            geo_sync1_mem <= {GEO_W{1'b0}}; geo_sync2_mem <= {GEO_W{1'b0}}; reload_geo_mem <= {GEO_W{1'b0}};
            reload_sgeo_mem <= {SGEO_W{1'b0}};
            st_mem <= 1'b0; sk_mem <= 30'd0; fw_mem <= IMAGE_WORDS;
            sj_mem <= 3'd0; srf_mem <= 1'b0; srs_mem <= 1'b0; ssr_mem <= 7'd0; ss3_mem <= 2'd0; plan_dec_mem <= 1'b0;
            gm_mem <= G_RECT; gr0_mem <= 10'd0; gr1_mem <= 10'd0; gc0_mem <= 10'd0; gc1_mem <= 10'd0; gv_mem <= 10'd0;
            pl_state <= P_IDLE;
            col_mem <= 11'd0; row_mem <= 10'd0; slat_mem <= 10'd0;
            lbn_mem <= 0; lbo_mem <= 0; lo_mem <= 11'd0; hi_mem <= 11'd0; line_new_mem <= 1'b0;
            plan_addr_mem <= 0; burst_addr_mem <= 0; plan_len_mem <= 0; plan_k0_mem <= 0;
            rq_wr_mem <= 0; rq_rd_mem <= 0; ret_idx_mem <= 0;
            plan_valid_mem <= 1'b0; plan_ok_mem <= 1'b0;
            index_sync1_mem <= 3'd0;
            index_sync2_mem <= 3'd0;
            reload_pending_mem <= 1'b0;
            reload_index_mem <= 3'd0;
            fifo_reset_count_mem <= 5'd0;
            pix_rst_sync_mem <= 2'b00;
            fifo_reset_req_mem_q <= 1'b1;
            fifo_release_pix_sync_mem <= 2'b00;
            issued_count_mem <= 0;
            returned_count_mem <= IMAGE_WORDS;
            inflight_count_mem <= 0;
            inflight_empty_mem <= 1'b1;
            burst_active_mem <= 1'b0;
            burst_gap_count_mem <= 0;
            diag_written_count_mem <= 0;
            diag_column_mem <= 0;
            fifo_release_mem <= 2'b00;
            inflight_window_ok_q <= 1'b1;
            fifo_reservation_ok_q <= 1'b1;
            app_rd_en <= 1'b0;
            app_rd_addr <= 0;
            reader_busy <= 1'b0;
            frame_read_done <= 1'b0;
            reader_index <= 3'd0;
            reader_error <= ERR_NONE;
        end else begin
            pix_rst_sync_mem <= {pix_rst_sync_mem[0], pix_rst_n};
            fifo_reset_req_mem_q <= fifo_reset_req_mem;
            fifo_release_pix_sync_mem <= {fifo_release_pix_sync_mem[0], fifo_release_pix[1]};
            if (fifo_reset_req_mem)
                fifo_release_mem <= 2'b00;
            else
                fifo_release_mem <= {fifo_release_mem[0], 1'b1};

            request_sync1_mem <= request_toggle_pix;
            request_sync2_mem <= request_sync1_mem;
            request_sync3_mem <= request_sync2_mem;
            index_sync1_mem <= request_index_pix;
            index_sync2_mem <= index_sync1_mem;
            new_sync1_mem <= request_new_pix;     new_sync2_mem <= new_sync1_mem;
            geo_sync1_mem <= request_geo_pix;     geo_sync2_mem <= geo_sync1_mem;
            plan_ok_mem <= plan_valid_mem && !start_burst;   // 段长已寄存一拍，FIFO 余量判断已用上它
            app_rd_en <= 1'b0;
            frame_read_done <= 1'b0;
            inflight_window_ok_q <= inflight_window_ok;
            fifo_reservation_ok_q <= fifo_reservation_ok;

            if (fifo_reset_count_mem != 0)
                fifo_reset_count_mem <= fifo_reset_count_mem - 1'b1;
            if (burst_gap_count_mem != 0)
                burst_gap_count_mem <= burst_gap_count_mem - 1'b1;

            case ({issue_allowed, sdram_rd_valid})
                2'b10: begin
                    inflight_count_mem <= inflight_count_mem + 1'b1;
                    inflight_empty_mem <= 1'b0;
                end
                2'b01: begin
                    if (!inflight_empty_mem) begin
                        inflight_count_mem <= inflight_count_mem - 1'b1;
                        if (inflight_count_mem == 1)
                            inflight_empty_mem <= 1'b1;
                    end
                end
                default: begin
                    inflight_count_mem <= inflight_count_mem;
                    inflight_empty_mem <= inflight_empty_mem;
                end
            endcase

            if (request_sync3_mem != request_seen_mem) begin
                request_seen_mem <= request_sync3_mem;
                reload_pending_mem <= 1'b1;
                reload_index_mem <= index_sync2_mem;
                reload_new_mem <= new_sync2_mem;
                reload_geo_mem <= geo_sync2_mem;
                reload_sgeo_mem <= request_sgeo_pix;     // toggle 翻转前就稳定了，直接取
                if (reader_busy || !inflight_empty_mem) begin
                    reader_busy <= 1'b0;                     // 当前块仍读完（整块对齐读），再等在途清空后重装
                    reader_error <= ERR_FRAME_OVERRUN;
                end
            end

            // ---------------- 规划器 ----------------
            case (pl_state)
                P_LINE: begin                               // 本行常数
                    lo_mem <= LINE_WORDS; hi_mem <= LINE_WORDS;           // 默认：本行没有新图区间
                    line_new_mem <= (slat_mem < gv_mem);
                    pl_state <= strip_row ? P_SCAN : P_CALC;
                    if (gm_mem != G_HBLIND && row_mem >= gr0_mem && row_mem < gr1_mem && gc0_mem < gc1_mem) begin
                        lo_mem <= gc0_mem; hi_mem <= gc1_mem;
                    end
                    // 胶片条行，从第 0 格开始看
                    sj_mem  <= 3'd0;
                    srf_mem <= (row_mem >= STRIP_FY0) && (row_mem < STRIP_FY0 + THUMB_H);
                    srs_mem <= (row_mem >= STRIP_SY0) && (row_mem < STRIP_SY0 + STRIP_SH);
                end
                P_SCAN: begin                               // 胶片条行，一拍看一格
                    if (sj_mem == 3'd5) begin               // 5 格看完：下一行
                        col_mem <= 11'd0;
                        row_mem <= row_mem + 10'd1;
                        slat_mem <= (slat_mem == SLAT - 1) ? 10'd0 : slat_mem + 10'd1;
                        lbn_mem <= lbn_mem + LINE_WORDS;
                        lbo_mem <= lbo_mem + LINE_WORDS;
                        pl_state <= (row_mem == V_LINES - 1) ? P_IDLE : P_LINE;
                        if (srs_mem) begin                  // 两边格：下一行读第 r + r/3 行
                            ssr_mem <= ssr_mem + ((ss3_mem == 2'd2) ? 7'd2 : 7'd1);
                            ss3_mem <= (ss3_mem == 2'd2) ? 2'd0 : ss3_mem + 2'd1;
                        end
                    end else if (sj_hit) begin
                        plan_addr_mem <= taddr;
                        plan_len_mem <= sj_foc ? THUMB_W : THUMB_SW;
                        plan_k0_mem <= 0;
                        plan_dec_mem <= !sj_foc;
                        plan_valid_mem <= 1'b1;
                        pl_state <= P_READY;
                    end else
                        sj_mem <= sj_mem + 3'd1;
                end
                P_CALC: begin                               // 规划下一段
                    plan_addr_mem <= seg_addr - seg_k0;     // 块起点（128 对齐）
                    plan_len_mem <= plan_len[BURST_COUNT_W-1:0];
                    plan_k0_mem <= seg_k0[BURST_COUNT_W-1:0];
                    plan_dec_mem <= 1'b0;
                    plan_valid_mem <= 1'b1;
                    pl_state <= P_READY;
                end
                P_READY: if (start_burst) begin             // 这一段已发出，前进到下一段
                    plan_valid_mem <= 1'b0;
                    if (strip_row) begin                    // 胶片条行，看下一格
                        sj_mem <= sj_mem + 3'd1;
                        pl_state <= P_SCAN;
                    end else if (col_after == LINE_WORDS) begin
                        col_mem <= 11'd0;
                        row_mem <= row_mem + 10'd1;
                        slat_mem <= (slat_mem == SLAT - 1) ? 10'd0 : slat_mem + 10'd1;
                        lbn_mem <= lbn_mem + LINE_WORDS;
                        lbo_mem <= lbo_mem + LINE_WORDS;
                        pl_state <= (row_mem == V_LINES - 1) ? P_IDLE : P_LINE;
                    end else begin
                        col_mem <= col_after;
                        pl_state <= P_CALC;
                    end
                end
                default: ;
            endcase

            if (reload_pending_mem && inflight_empty_mem && !burst_active_mem) begin   // 当前块发完、数据全部返回后才重装
                reload_pending_mem <= 1'b0;
                reader_index <= reload_index_mem;
                rd_new_mem <= reload_new_mem;
                gm_mem <= rl_gm; gr0_mem <= rl_r0; gr1_mem <= rl_r1;
                gc0_mem <= rl_c0; gc1_mem <= rl_c1; gv_mem <= rl_v;
                st_mem <= reload_sgeo_mem[30]; sk_mem <= reload_sgeo_mem[29:0]; fw_mem <= rl_words;
                ssr_mem <= 7'd0; ss3_mem <= 2'd0;
                col_mem <= 11'd0; row_mem <= 10'd0; slat_mem <= 10'd0;
                lbn_mem <= bank_base(reload_new_mem)   + line_off(rl_ndy) + sext11(rl_ndx);
                lbo_mem <= bank_base(reload_index_mem) + line_off(rl_ody) + sext11(rl_odx);
                plan_valid_mem <= 1'b0;
                plan_ok_mem <= 1'b0;
                pl_state <= P_LINE;
                rq_wr_mem <= 0; rq_rd_mem <= 0; ret_idx_mem <= 0;   // 此时在途已清空
                issued_count_mem <= 0;
                returned_count_mem <= fw_cur_n;
                inflight_count_mem <= 0;
                inflight_empty_mem <= 1'b1;
                burst_active_mem <= 1'b0;
                burst_gap_count_mem <= 0;
                diag_written_count_mem <= 0;
                diag_column_mem <= 0;
                fifo_reset_count_mem <= 5'd16;
                reader_busy <= 1'b1;
                reader_error <= ERR_NONE;
            end

            if (start_burst) begin                      // 一律读满一整块，记下保留区间
                burst_active_mem <= 1'b1;
                burst_addr_mem <= plan_addr_mem;
                issued_count_mem <= issued_count_mem + plan_len_mem;
                rq_k0[rq_wr_mem] <= plan_k0_mem;
                rq_k1[rq_wr_mem] <= plan_dec_mem ? THUMB_W : plan_k0_mem + plan_len_mem;   // 4 取 3：保留区间是整个 112 字
                rq_dec[rq_wr_mem] <= plan_dec_mem;
                rq_wr_mem <= rq_wr_mem + 1'b1;
            end

            if (issue_allowed) begin
                app_rd_en <= 1'b1;
                app_rd_addr <= burst_addr_mem;
                burst_addr_mem[BLW-1:0] <= burst_addr_mem[BLW-1:0] + 1'b1;
                if (burst_addr_mem[BLW-1:0] == BURST_SIZE - 1) begin
                    burst_active_mem <= 1'b0;
                    burst_gap_count_mem <= BURST_GAP_CYCLES;
                end
            end

            if (!DIAG_SYNTH_STREAM && sdram_rd_valid) begin   // 返回字在本块里的序号，一块读完出队
                if (ret_idx_mem == BURST_SIZE - 1) begin
                    ret_idx_mem <= 0;
                    rq_rd_mem <= rq_rd_mem + 1'b1;
                end else
                    ret_idx_mem <= ret_idx_mem + 1'b1;
            end
            if (!DIAG_SYNTH_STREAM && sdram_rd_valid && ret_keep) begin
                returned_count_mem <= returned_count_mem - 1'b1;
                if (fifo_full) begin
                    reader_busy <= 1'b0;
                    reader_error <= ERR_FIFO_OVERFLOW;
                end else if (final_return) begin
                    reader_busy <= 1'b0;
                    frame_read_done <= 1'b1;
                end
            end

            if (DIAG_SYNTH_STREAM && diag_stream_write) begin
                diag_written_count_mem <= diag_written_count_mem + 1'b1;
                diag_column_mem <= (diag_column_mem == 10'd639) ?
                                   10'd0 : diag_column_mem + 1'b1;
                if (diag_written_count_mem == IMAGE_WORDS - 1) begin
                    reader_busy <= 1'b0;
                    frame_read_done <= 1'b1;
                end
            end else if (!DIAG_SYNTH_STREAM && DIAG_RETURN_PATTERN &&
                         sdram_rd_valid) begin
                diag_column_mem <= (diag_column_mem == 10'd639) ?
                                   10'd0 : diag_column_mem + 1'b1;
            end
        end
    end

    rfifo_32_32_512 #(
        .DATA_WIDTH_W(32), .DATA_WIDTH_R(32),
        .ADDR_WIDTH_W(FIFO_COUNT_W), .ADDR_WIDTH_R(FIFO_COUNT_W),
        .AL_FULL_NUM(FIFO_DEPTH-FIFO_MARGIN), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_read_fifo (
        .rst(fifo_reset), .clkw(mem_clk), .clkr(pix_clk),
        .we(fifo_write_enable), .di(fifo_write_data),
        .re(fifo_raw_read_en), .dout(fifo_raw_data), .valid(fifo_raw_valid),
        .full_flag(fifo_full), .empty_flag(fifo_empty),
        .afull(fifo_almost_full), .aempty(),
        .wrusedw(fifo_wrusedw), .rdusedw()
    );

    fifo_prefetch_2 #(.DATA_WIDTH(32)) u_prefetch (
        .clk(pix_clk), .rst(fifo_reset), .raw_empty(fifo_empty),
        .raw_valid(fifo_raw_valid), .raw_data(fifo_raw_data),
        .raw_read_en(fifo_raw_read_en), .consume(pixel_pop),
        .out_valid(prefetch_valid), .out_empty(), .out_data(prefetch_data)
    );

    assign pixel_rgb = RGB_IN_HIGH ? prefetch_data[31:8] : prefetch_data[23:0];
    assign pixel_valid = prefetch_valid;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            // The memory-domain error remains authoritative for controller faults.
        end else if (underflow) begin
            // underflow is exported separately; the Top may merge it as 8'h63.
        end
    end

`ifdef OSD_SIM
    // 整块对齐读的前提检查（仿真）：BURST_SIZE 是 2 的幂，每个 burst 起点都按 BURST_SIZE 对齐（plan_addr = 段地址 - 块内偏移）
    initial begin
        if ((1 << BLW) != BURST_SIZE) begin
            $display("[ASSERT] %m: BURST_SIZE 要是 2 的幂");
            $finish;
        end
    end
    always @(posedge mem_clk)
        if (start_burst && plan_addr_mem[BLW-1:0] != 0)
            $display("[ASSERT] %m: burst 起点 %0d 没有按 BURST_SIZE 对齐, t=%0t", plan_addr_mem, $time);
    // 返回队列深度的前提检查：返回队列里的项数不超过 MAX_INFLIGHT_BURSTS（入队 +1、整块返回 -1）
    integer rq_n = 0;
    always @(posedge mem_clk) begin
        if (!mem_rst_n) rq_n = 0;
        else begin
            if (start_burst) rq_n = rq_n + 1;
            if (!DIAG_SYNTH_STREAM && sdram_rd_valid && ret_idx_mem == BURST_SIZE - 1) rq_n = rq_n - 1;
            if (rq_n > MAX_INFLIGHT_BURSTS || rq_n < 0)
                $display("[ASSERT] %m: 返回队列 %0d 项，超出 %0d, t=%0t", rq_n, MAX_INFLIGHT_BURSTS, $time);
        end
    end
`endif
endmodule
