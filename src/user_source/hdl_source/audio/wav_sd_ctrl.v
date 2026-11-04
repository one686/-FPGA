`timescale 1ns/1ps

// WAV 播放控制（sd_card_clk 域，50MHz）
//
// 歌单来自 fat32_volume_scanner（根目录里的 .WAV，按目录顺序，最多 12 首，由上层决定）。
// 曲号 / 首数位宽为参数 NW（歌单最多 2^NW - 1 首；上层例化用 NW=4）。
// 一首歌的流程：
//   清空缓冲(FLUSH) -> 取歌单(OPEN) -> 启动第二个 fat32_file_streamer 读文件 ->
//   解析文件头(HDR)：RIFF/WAVE 标识、fmt 块（格式/声道/采样率/位深）、跳过其他块、找到 data 块 ->
//   送样本(DATA)：小端 16 位，单声道复制成左右相同，立体声 L,R；每帧 32 位写进环形缓冲 ->
//   data 读完(TAIL)：停掉读文件（文件尾部的其他块不读），标记"本首已写完"(eos) ->
//   等缓冲播空(DRAIN) -> 下一首（最后一首之后回到第一首，循环）
// 读卡流量控制：缓冲里空位够一个扇区才向仲裁器要 SD（sd_want），所以缓冲永远不会写溢出。
//
// 文件头不对：报错误码并跳到下一首（这首仍算在歌单里）；连续 N 首（= 歌单总数）都不能播 -> 停止，
// 回到内置琶音（wav_active=0），再按一次选歌键（下一首/上一首）可重试。
//   1 不是 RIFF/WAVE    2 不是 PCM 编码    3 采样率不是 48kHz    4 位深不是 16 位
//   5 声道数不是 1/2     6 没有 data 块/文件损坏（簇链错）       7 读卡出错（超时）
// 播放中途出错（6/7）：报错误码，已经读进缓冲的部分照常播完，再去下一首。
// 错误码保持 ERR_SHOW_MS 后自动清零（OSD 上的"音乐错误 n"随之淡出）。
//
// 选歌（音乐层 KEY2：短按 song_next = 下一首，长按 song_prev = 上一首）：
//   立刻停止当前歌、清空缓冲（声音约 20ms 淡出），每来一次选中下一首/上一首（OSD 上曲号跟着变），
//   松手（song_hold=0）后从选中的那首开头播放。
// 单曲循环（loop_one=1）：一首正常播完后重播同一首；出错跳过、手动选歌不受影响。
// song_end：一首正常播完（缓冲播空）的单拍脉冲，单曲循环每遍也发；出错跳过、手动选歌不发（轮播"随音乐"档用）。
module wav_sd_ctrl #(
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer ERR_SHOW_MS = 5000,     // 错误码显示时长（毫秒，按外部 ms_tick 计）
    parameter integer NW          = 7         // 曲号 / 首数位宽（歌单最多 2^NW - 1 首）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ms_tick,         // 1 毫秒节拍（与上层共用）
    // 歌单
    input  wire        list_load,       // 扫描结束单拍脉冲
    input  wire [NW-1:0] list_count,
    output reg  [NW-1:0] pl_addr,
    input  wire [31:0] pl_cluster,
    input  wire [31:0] pl_size,
    // 按键
    input  wire        song_next,
    input  wire        song_prev,
    input  wire        song_hold,
    input  wire        loop_one,
    // 读文件（第二个 fat32_file_streamer）
    output reg         st_start,
    output reg  [31:0] st_cluster,
    output reg  [31:0] st_size,
    output reg         st_abort,        // 1 = 让读文件模块保持复位（停止）
    input  wire        st_busy,
    input  wire        st_done,
    input  wire [7:0]  st_err,
    input  wire        st_valid,
    input  wire [7:0]  st_byte,
    input  wire [31:0] st_remaining,    // 读文件模块里"本字节之后还剩几个字节"（与 st_valid 同拍有效）
    // 环形缓冲（写侧）
    output reg         ring_we,
    output reg  [31:0] ring_wdata,      // {左, 右}
    input  wire        ring_space_ok,
    input  wire        ring_empty,
    output reg         ring_flush,
    input  wire        ring_flush_busy,
    output reg         ring_eos,
    // SD 仲裁
    output wire        sd_want,
    // 状态（OSD / 混音）
    output wire        wav_active,      // 1 = 播 WAV，0 = 琶音
    output reg         song_end,        // 一首正常播完（单拍）
    output reg  [NW-1:0] show_idx,       // 第几首（1 起）；寄存器输出（晚 1 拍），切断到数码管/OSD 的长组合路径
    output wire [NW-1:0] show_cnt,
    output reg         tot_known,
    output reg  [15:0] tot_bcd,         // 总时长 m1 m0 s1 s0
    output reg  [2:0]  err_code,
    output wire [3:0]  dbg_state       // 上板排查用：当前状态
);
    localparam [3:0] S_IDLE = 4'd0, S_FLUSH = 4'd1, S_OPEN0 = 4'd2, S_OPEN1 = 4'd3, S_HDR = 4'd4,
                     S_DATA = 4'd5, S_TAIL = 4'd6, S_DRAIN = 4'd7, S_SELECT = 4'd8, S_DEAD = 4'd9,
                     S_START = 4'd10;


    reg [3:0]  state;
    reg [NW-1:0] count, idx, sel, fails;
    reg [3:0]  cnt;                    // 小延时计数
    reg        flushed;                // SELECT 里清空缓冲已完成
    reg [$clog2(ERR_SHOW_MS + 1)-1:0] err_timer;   // 错误码剩余显示时间（毫秒，位宽按参数）

    // 文件头解析：用几个小计数器 + 一个跳过计数器，避免"文件位置 + 块位置"的 32 位比较（省 LUT）
    //   hpos  : RIFF 头的第几个字节（0~11，12 = RIFF 头已过）
    //   in_ck : 1 = 正在收 8 字节块头（chk = 第几个字节），0 = 在块体里（left = 块体还剩几个字节）
    //   bidx  : 块体里第几个字节（只用到 0~15，到 16 停住），fmt 块的字段按它取
    //   left  : 块体还剩的字节数；播放时 = data 块还剩的字节数（两个阶段共用一个计数器）
    //   文件里还剩多少字节直接用读文件模块的计数（st_remaining），data 块比文件长时按它截
    //   fmt 字段收的时候就比对好，只留结果标志
    reg [3:0]  hpos;
    reg [2:0]  chk;
    reg        in_ck;
    reg [31:0] left;
    reg [4:0]  bidx;
    reg [31:0] ck_id;
    reg [23:0] ck_size;                // 块长度的低 3 字节（最高字节最后到，直接用 st_byte）
    reg        is_fmt, fmt_seen, riff_bad;
    reg [7:0]  lo_b;                   // 16 位字段的低字节
    reg        tag_ok, ch_mono, ch_st, rate_ok, bits_ok;
    reg        stereo;
    reg [1:0]  bsel;                   // 帧内第几个字节（字节直接写进 ring_wdata 对应位置，不另设暂存）

    assign wav_active = (state != S_IDLE) && (state != S_DEAD);
    assign dbg_state  = state;
    assign show_cnt   = count;
    assign sd_want    = ((state == S_HDR) || (state == S_DATA)) && st_busy && ring_space_ok;

    // 下一首/上一首/显示曲号共用一套加减（选歌时以 sel 为准，其余时候以 idx 为准）
    wire [NW-1:0] cur      = (state == S_SELECT) ? sel : idx;
    wire [NW-1:0] cur_inc  = cur + 1'b1;
    wire [NW-1:0] cur_next = (cur_inc >= count) ? {NW{1'b0}} : cur_inc;
    wire [NW-1:0] cur_prev = (cur == 0) ? count - 1'b1 : cur - 1'b1;
    wire       song_step = song_next || song_prev;

    // RIFF 头前 12 字节里需要核对的 8 个字节
    function [8:0] riff_expect;        // {要核对, 字节}
        input [3:0] p;
        begin
            case (p)
                4'd0: riff_expect = {1'b1, 8'h52};  // R
                4'd1: riff_expect = {1'b1, 8'h49};  // I
                4'd2: riff_expect = {1'b1, 8'h46};  // F
                4'd3: riff_expect = {1'b1, 8'h46};  // F
                4'd8: riff_expect = {1'b1, 8'h57};  // W
                4'd9: riff_expect = {1'b1, 8'h41};  // A
                4'd10: riff_expect = {1'b1, 8'h56}; // V
                4'd11: riff_expect = {1'b1, 8'h45}; // E
                default: riff_expect = 9'd0;
            endcase
        end
    endfunction
    wire [8:0]  rexp = riff_expect(hpos);
    wire [31:0] ck_size_full = {st_byte, ck_size};                 // 块长度最后一个字节到达时
    wire [31:0] ck_body = ck_size_full + {31'd0, ck_size_full[0]}; // 奇数长度补 1 字节
    wire [31:0] file_left = st_remaining;                          // 本字节之后文件里还剩的字节
    wire [31:0] data_len  = (ck_size_full < file_left) ? ck_size_full : file_left;   // 实际能读到的样本字节
    // 48000 = 0x0000BB80，按小端逐字节比对
    wire [7:0]  rate_byte = (bidx[1:0] == 2'd0) ? 8'h80 : (bidx[1:0] == 2'd1) ? 8'hBB : 8'h00;
    // data 块开始时的格式检查（0 = 可以播）
    wire [2:0]  fmt_err = !fmt_seen ? 3'd6 :
                          !tag_ok ? 3'd2 :
                          !rate_ok ? 3'd3 :
                          !bits_ok ? 3'd4 :
                          !(ch_mono || ch_st) ? 3'd5 : 3'd0;
    // 读文件结束（出错）时的错误码
    wire [2:0]  st_err_code = (st_err == 8'h10) ? 3'd7 : 3'd6;

    // ---------------- 总时长：data 字节数里有几个"1 秒"（单声道 96000 字节、立体声 192000 字节） ----------------
    // 每拍减一次、BCD 秒数加 1（99:59 封顶），一首歌最多几千拍（几十微秒）就算完；只用一个 32 位减法器
    // 96000 = 375 x 256、192000 = 750 x 256，低 8 位减了不变也不借位，只存、只减高 24 位
    //   dv_go 那一拍 left 刚装好 data 块长度（同拍即使在减，读到的也是减之前的值），直接拿来用；
    //   声道数用 stereo（一首歌里不变，换歌前 dv_run 已清掉）
    reg        dv_go, dv_run;
    reg [23:0] dv_bytes;               // data 字节数 / 256
    wire [23:0] dv_step = stereo ? 24'd750 : 24'd375;
    wire [24:0] dv_try  = {1'b0, dv_bytes} - {1'b0, dv_step};
    function [15:0] bcd_inc;           // mm:ss 加 1 秒，99:59 封顶
        input [15:0] t;
        begin
            bcd_inc = t;
            if (t != 16'h9959) begin
                if (t[3:0] != 4'd9) bcd_inc[3:0] = t[3:0] + 4'd1;
                else begin
                    bcd_inc[3:0] = 4'd0;
                    if (t[7:4] != 4'd5) bcd_inc[7:4] = t[7:4] + 4'd1;
                    else begin
                        bcd_inc[7:4] = 4'd0;
                        if (t[11:8] != 4'd9) bcd_inc[11:8] = t[11:8] + 4'd1;
                        else begin
                            bcd_inc[11:8] = 4'd0;
                            bcd_inc[15:12] = t[15:12] + 4'd1;
                        end
                    end
                end
            end
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dv_run <= 1'b0; tot_known <= 1'b0; tot_bcd <= 16'd0; dv_bytes <= 24'd0;
        end else begin
            if (dv_go) begin
                dv_run <= 1'b1; tot_known <= 1'b0; tot_bcd <= 16'd0; dv_bytes <= left[31:8];
            end else if (dv_run) begin
                if (!dv_try[24]) begin
                    dv_bytes <= dv_try[23:0];
                    tot_bcd  <= bcd_inc(tot_bcd);
                end else begin
                    dv_run <= 1'b0;
                    tot_known <= 1'b1;
                end
            end
            if (state == S_SELECT || state == S_FLUSH || state == S_IDLE || state == S_DEAD) begin
                tot_known <= 1'b0;
                dv_run <= 1'b0;
            end
        end
    end

    task begin_flush;                  // 停读文件，去清空缓冲，然后打开第 idx 首
        begin
            st_abort <= 1'b1;
            ring_eos <= 1'b0;
            cnt <= 4'd0;
            state <= S_FLUSH;
        end
    endtask
    task show_err;
        input [2:0] code;
        begin
            err_code <= code;
            err_timer <= ERR_SHOW_MS;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            count <= 0; idx <= 0; sel <= 0; fails <= 0; cnt <= 0; flushed <= 0;
            show_idx <= 1;
            pl_addr <= 0;
            st_start <= 1'b0; st_cluster <= 0; st_size <= 0; st_abort <= 1'b1;
            ring_we <= 1'b0; ring_wdata <= 0; ring_flush <= 1'b0; ring_eos <= 1'b0;
            err_code <= 0; err_timer <= 0;
            hpos <= 0; chk <= 0; in_ck <= 1'b1; left <= 0; bidx <= 0;
            ck_id <= 0; ck_size <= 0; is_fmt <= 0; fmt_seen <= 0; riff_bad <= 0; lo_b <= 0;
            tag_ok <= 0; ch_mono <= 0; ch_st <= 0; rate_ok <= 0; bits_ok <= 0;
            stereo <= 0;
            bsel <= 0;
            dv_go <= 1'b0;
            song_end <= 1'b0;
        end else begin
            show_idx   <= cur_inc;
            song_end   <= 1'b0;
            st_start   <= 1'b0;
            ring_we    <= 1'b0;
            ring_flush <= 1'b0;
            dv_go      <= 1'b0;

            if (ms_tick && (err_timer != 0)) begin
                err_timer <= err_timer - 1'b1;
                if (err_timer == 1'b1) err_code <= 3'd0;
            end

            case (state)
                S_IDLE, S_DEAD: st_abort <= 1'b1;

                // 清空缓冲：停读文件 8 拍后发清空请求，等读侧(淡出后)确认
                S_FLUSH: begin
                    if (cnt != 4'd15) cnt <= cnt + 4'd1;
                    if (cnt == 4'd8) ring_flush <= 1'b1;
                    if (cnt == 4'd15 && !ring_flush_busy) begin
                        pl_addr <= idx;
                        cnt <= 4'd0;
                        state <= S_OPEN0;
                    end
                end
                S_OPEN0: begin                              // 等歌单表读出（2 拍）
                    cnt <= cnt + 4'd1;
                    if (cnt == 4'd2) begin
                        st_cluster <= pl_cluster;
                        st_size    <= pl_size;
                        st_abort   <= 1'b0;
                        cnt <= 4'd0;
                        state <= S_OPEN1;
                    end
                end
                S_OPEN1: begin                              // 读文件模块出复位后再启动
                    cnt <= cnt + 4'd1;
                    if (cnt == 4'd2) begin
                        st_start <= 1'b1;
                        hpos <= 0; chk <= 0; in_ck <= 1'b1; left <= 0; bidx <= 0;
                        is_fmt <= 0; fmt_seen <= 0; riff_bad <= 0;
                        tag_ok <= 0; ch_mono <= 0; ch_st <= 0; rate_ok <= 0; bits_ok <= 0;
                        state <= S_START;
                    end
                end
                S_START: state <= S_HDR;                    // st_busy 在启动后一拍才拉高

                // 解析文件头
                S_HDR: begin
                    if (st_valid) begin
                        if (hpos != 4'd12) begin
                            // RIFF 头 12 字节："RIFF" + 长度 + "WAVE"
                            hpos <= hpos + 4'd1;
                            if (rexp[8] && st_byte != rexp[7:0]) riff_bad <= 1'b1;
                            if (hpos == 4'd11 && (riff_bad || (rexp[8] && st_byte != rexp[7:0]))) begin
                                show_err(3'd1);
                                state <= S_TAIL;           // 不是 WAV：停读，按"头错误"处理
                                fails <= fails + 1'b1;
                            end
                        end else if (in_ck) begin
                            // 块头 8 字节：4 字节标识（大端拼）+ 4 字节长度（小端）
                            chk <= chk + 3'd1;
                            if (!chk[2]) ck_id <= {ck_id[23:0], st_byte};
                            if (chk == 3'd4) ck_size[7:0]   <= st_byte;
                            if (chk == 3'd5) ck_size[15:8]  <= st_byte;
                            if (chk == 3'd6) ck_size[23:16] <= st_byte;
                            if (chk == 3'd7) begin
                                if (ck_id == 32'h64617461) begin                    // "data"
                                    if (fmt_err != 3'd0 || data_len < (ch_st ? 32'd4 : 32'd2)) begin
                                        show_err((fmt_err != 3'd0) ? fmt_err : 3'd6);   // 格式不对 / 一帧样本都没有
                                        fails <= fails + 1'b1;
                                        state <= S_TAIL;
                                    end else begin
                                        fails <= 0;
                                        stereo <= ch_st;
                                        left <= data_len;
                                        dv_go <= 1'b1;
                                        bsel <= 2'd0;
                                        state <= S_DATA;
                                    end
                                end else begin
                                    is_fmt <= (ck_id == 32'h666D7420);              // "fmt "
                                    bidx <= 5'd0;
                                    left <= ck_body;
                                    in_ck <= (ck_body == 32'd0);                     // 空块：直接收下一个块头
                                end
                            end
                        end else begin
                            // 块体：fmt 块取前 16 字节的字段，其余跳过
                            if (!bidx[4]) bidx <= bidx + 5'd1;
                            if (is_fmt && !bidx[4]) begin
                                case (bidx[3:0])
                                    4'd0, 4'd2, 4'd14: lo_b <= st_byte;
                                    4'd1: tag_ok <= (lo_b == 8'h01 && st_byte == 8'h00) ||
                                                    (lo_b == 8'hFE && st_byte == 8'hFF);   // PCM / EXTENSIBLE
                                    4'd3: begin
                                        ch_mono <= (lo_b == 8'd1) && (st_byte == 8'd0);
                                        ch_st   <= (lo_b == 8'd2) && (st_byte == 8'd0);
                                    end
                                    4'd4: rate_ok <= (st_byte == rate_byte);
                                    4'd5, 4'd6, 4'd7: rate_ok <= rate_ok && (st_byte == rate_byte);
                                    4'd15: begin
                                        bits_ok <= (lo_b == 8'd16) && (st_byte == 8'd0);
                                        fmt_seen <= 1'b1;
                                    end
                                    default: ;
                                endcase
                            end
                            left <= left - 32'd1;
                            if (left == 32'd1) begin
                                in_ck <= 1'b1;
                                chk <= 3'd0;
                            end
                        end
                    end
                    if (st_done && state == S_HDR) begin    // 文件读完/出错时还没找到 data 块
                        show_err((st_err != 8'd0) ? st_err_code : (hpos != 4'd12) ? 3'd1 : 3'd6);
                        fails <= fails + 1'b1;
                        state <= S_TAIL;
                    end
                end

                // 送样本：小端 16 位；单声道 2 字节一帧（左右相同），立体声 4 字节一帧
                //   ring_wdata = {左高, 左低, 右高, 右低}，每个字节到达时直接写到它的位置
                //   （环形缓冲只在 ring_we 那一拍取数据，中间状态不影响）
                S_DATA: begin
                    if (st_valid) begin
                        left <= left - 32'd1;
                        case (bsel)
                            2'd0: begin
                                ring_wdata[23:16] <= st_byte;
                                if (!stereo) ring_wdata[7:0] <= st_byte;
                                bsel <= 2'd1;
                            end
                            2'd1: begin
                                ring_wdata[31:24] <= st_byte;
                                if (stereo) bsel <= 2'd2;
                                else begin
                                    ring_wdata[15:8] <= st_byte;
                                    ring_we <= 1'b1;
                                    bsel <= 2'd0;
                                end
                            end
                            2'd2: begin ring_wdata[7:0] <= st_byte; bsel <= 2'd3; end
                            default: begin
                                ring_wdata[15:8] <= st_byte;
                                ring_we <= 1'b1;
                                bsel <= 2'd0;
                            end
                        endcase
                        if (left == 32'd1) state <= S_TAIL;   // data 读完（后面的块不读）
                    end
                    if (st_done && state == S_DATA) begin   // 文件比 data 块短 / 中途出错
                        if (st_err != 8'd0) show_err(st_err_code);
                        state <= S_TAIL;
                    end
                end

                // 本首写完：停读文件；头错误 -> 直接下一首；正常 -> 标记 eos，等播完
                S_TAIL: begin
                    st_abort <= 1'b1;
                    cnt <= cnt + 4'd1;
                    if (cnt == 4'd8) begin
                        cnt <= 4'd0;
                        if (fails != 0) begin
                            if (fails >= count) state <= S_DEAD;      // 全都不能播：回到琶音
                            else begin idx <= cur_next; begin_flush; end
                        end else begin
                            ring_eos <= 1'b1;
                            state <= S_DRAIN;
                        end
                    end
                end
                S_DRAIN: if (ring_empty) begin              // 缓冲播空 = 本首播完
                    if (!loop_one) idx <= cur_next;         // 单曲循环：还是这一首
                    song_end <= 1'b1;
                    begin_flush;
                end

                // 选歌：清空缓冲，松手后播选中的那首
                S_SELECT: begin
                    st_abort <= 1'b1;
                    ring_eos <= 1'b0;
                    if (!flushed) begin
                        if (cnt != 4'd15) cnt <= cnt + 4'd1;
                        if (cnt == 4'd8) ring_flush <= 1'b1;
                        if (cnt == 4'd15 && !ring_flush_busy) flushed <= 1'b1;
                    end else if (!song_hold) begin
                        idx <= sel;
                        fails <= 0;
                        pl_addr <= sel;
                        cnt <= 4'd0;
                        state <= S_OPEN0;
                    end
                end
                default: state <= S_IDLE;
            endcase

            // 选歌：下一首 / 上一首（覆盖上面的状态转移）
            if (song_step && count != 0) begin
                sel <= song_prev ? cur_prev : cur_next;   // 选歌中以 sel 为准，否则以 idx 为准
                if (state != S_SELECT) begin
                    flushed <= 1'b0;
                    cnt <= 4'd0;
                    st_abort <= 1'b1;
                    ring_eos <= 1'b0;
                    state <= S_SELECT;
                end
            end

            // 新歌单（上电扫描 / 重扫完成）：从第一首开始
            if (list_load) begin
                count <= list_count;
                idx <= 0; fails <= 0;
                err_code <= 3'd0; err_timer <= 1'b0;
                if (list_count == 0) begin
                    st_abort <= 1'b1; ring_eos <= 1'b0;
                    state <= S_IDLE;
                end else
                    begin_flush;
            end
        end
    end
endmodule
