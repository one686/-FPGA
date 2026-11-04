`timescale 1ns/1ps

// 音源：内置琶音 / WAV 二选一 + 淡入淡出 + 音量 + 暂停（video_clk 域，25MHz）
//
// 48kHz 节拍：由 pcm_media_tone 产生（对 25MHz 分数累加，精确 48kHz），它每出一个样本就是一拍；
//   琶音模式输出它的样本，WAV 模式在同一拍从环形缓冲取一帧（琶音样本丢弃）。
// 每拍处理：
//   1) 换音源 / 正在播的时候清空缓冲（手动换歌）：增益 fade 先降到 0（1024 拍 ≈ 21ms 淡出），
//      切换后再升回去（淡入），所以切歌、琶音<->WAV 都不会"咔"一声；
//      一首歌自然播完时缓冲已空、没有声音，清空请求立刻执行（不必等 21ms 淡出），下一首紧接着开始
//   2) WAV：先攒够 PREFILL 帧（≈21ms）才开始播，播的过程中缓冲空了 = 下溢（输出 0，重新攒；仿真里计数）；
//      本首已全部写入(eos)后缓冲播空 = 正常结束，不算下溢
//   3) 输出 = 样本 x 增益 / 256，增益 = (fade/4) x 音量 / 256；增益 256 时输出与输入逐位相同
//      （琶音、满音量时与 pcm_media_tone 的原始输出完全一致）
//   4) 音量从当前值每拍 ±1 走到目标值（256 拍 ≈ 5ms），调音量没有爆音
// 输出接 8 级 FIFO，valid/ready 接口与 pcm_media_tone 相同，可直接接后面的 HDMI 音频链。
// 已播放时间：每播出 48000 帧加 1 秒（mm:ss，BCD，最大 99:59），换歌时清零。
// 暂停（pause_async=1）：增益淡出到 0（≈21ms）后停止从缓冲取样（WAV 停在原处、已播放时间停走，
//   缓冲满了 SD 侧自然停读）；继续时从停下的地方接着取、增益淡入。琶音也一样静音。
//   暂停中换歌：照常清空缓冲、预缓冲，新歌也停在开头，等继续（SD 侧换歌时会自动清掉暂停）。
module media_audio_source #(
    parameter integer CLOCK_HZ = 25000000,
    parameter integer AW       = 12,
    parameter integer PREFILL  = 1024
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [2:0]         media_id,     // {换图翻转位, 旋律号}，选择琶音旋律
    input  wire [7:0]         tone_volume,
    input  wire               wav_active_async,   // sd_card_clk 域电平
    input  wire [8:0]         vol_gain,           // 0~256
    input  wire               pause_async,        // sd_card_clk 域电平，1 = 暂停
    // 环形缓冲读侧
    input  wire [AW:0]        ring_level,
    output reg                ring_pop,
    input  wire [31:0]        ring_data,
    input  wire               ring_flush_pend,
    output reg                ring_flush_do,
    input  wire               ring_eos,
    // 输出
    output wire               sample_valid,
    input  wire               sample_ready,
    output wire signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output reg                overflow,
    output wire               blip_active,
    output reg  [15:0]        elapsed_bcd,        // m1 m0 s1 s0
    output reg  [31:0]        underruns           // 仿真检查用
);
    localparam [10:0] FADE_FULL = 11'd1024;

    // ---------------- 琶音（也是 48kHz 节拍源） ----------------
    wire               tone_valid;
    wire signed [15:0] tone_left, tone_right;
    wire               tone_overflow;
    pcm_media_tone #(.CLOCK_HZ(CLOCK_HZ)) u_tone (
        .clk(clk), .rst_n(rst_n),
        .media_id(media_id), .volume(tone_volume),
        .sample_valid(tone_valid), .sample_ready(1'b1),
        .sample_left(tone_left), .sample_right(tone_right),
        .overflow(tone_overflow), .blip_active(blip_active)
    );
    wire tick = tone_valid;            // 一拍一个样本（ready 恒 1，FIFO 立刻取走）

    (* async_reg = "true" *) reg wa1, wa2;
    (* async_reg = "true" *) reg pa1, pa2;
    reg        cur_wav;                // 当前音源 0=琶音 1=WAV
    reg [10:0] fade;
    reg [8:0]  vol_s;
    reg        playing;
    reg [15:0] fcnt;                   // 一秒内已播帧数
    reg [3:0]  ph;
    reg        popped, src_wav;
    reg signed [15:0] arp_q;
    reg signed [15:0] l_src, r_src;
    reg [8:0]  g;
    reg signed [15:0] l_out;
    // 8 级输出 FIFO
    reg [31:0] ofifo [0:7];
    reg [2:0]  owp, orp;
    reg [3:0]  ocnt;
    reg        opush;
    reg [31:0] odata;

    wire want_switch = (wa2 != cur_wav) || ring_flush_pend;
    wire flush_now   = ring_flush_pend && (wa2 == cur_wav) && (!cur_wav || !playing);   // 没在出声：立刻清空
    wire hold        = pa2 && (fade == 11'd0);                 // 暂停且已淡出：不取样
    wire [17:0] g_mul = fade[10:2] * vol_s;     // (fade/4) x 音量
    // 省 LUT：音量平滑的 < 和 > 两个 9 位比较合并成一次 10 位减法：借位 = 小于，非 0 且无借位 = 大于
    wire [9:0]  vol_d = {1'b0, vol_s} - {1'b0, vol_gain};

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

    // 乘法器只用一个：第 3 拍算左声道、第 4 拍算右声道
    wire signed [15:0] mul_a = (ph == 4'd3) ? l_src : r_src;
    wire signed [25:0] mul_p = mul_a * $signed({1'b0, g});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wa1 <= 1'b0; wa2 <= 1'b0; pa1 <= 1'b0; pa2 <= 1'b0;
            cur_wav <= 1'b0; fade <= FADE_FULL; vol_s <= 9'd256;
            playing <= 1'b0; fcnt <= 16'd0; elapsed_bcd <= 16'd0; underruns <= 32'd0;
            ph <= 4'd0; popped <= 1'b0; src_wav <= 1'b0; arp_q <= 16'sd0;
            l_src <= 16'sd0; r_src <= 16'sd0; g <= 9'd256; l_out <= 16'sd0;
            ring_pop <= 1'b0; ring_flush_do <= 1'b0;
            opush <= 1'b0; odata <= 32'd0;
        end else begin
            wa1 <= wav_active_async; wa2 <= wa1;
            pa1 <= pause_async; pa2 <= pa1;
            ring_pop <= 1'b0;
            ring_flush_do <= 1'b0;
            opush <= 1'b0;

            if (tick) begin
                ph <= 4'd1;
                arp_q <= tone_left;
                src_wav <= cur_wav;
                popped <= 1'b0;
                g <= g_mul[16:8];
                // 音量平滑
                if (vol_d[9])              vol_s <= vol_s + 9'd1;
                else if (vol_d != 10'd0)   vol_s <= vol_s - 9'd1;
                // 淡出 -> 切换 / 清空 -> 淡入；暂停 = 淡出后停在 0
                if (flush_now) begin
                    ring_flush_do <= 1'b1;          // 没在出声：立刻清空，增益不动
                    playing <= 1'b0;
                    fcnt <= 16'd0;
                    elapsed_bcd <= 16'd0;
                end else if (want_switch || pa2) begin
                    if (fade == 11'd0) begin
                        if (want_switch) begin      // 暂停时只停在 0，不切换
                            cur_wav <= wa2;
                            src_wav <= wa2;
                            if (ring_flush_pend) begin
                                ring_flush_do <= 1'b1;
                                playing <= 1'b0;
                                fcnt <= 16'd0;
                                elapsed_bcd <= 16'd0;
                            end
                        end
                    end else
                        fade <= fade - 11'd1;
                end else if (fade != FADE_FULL)
                    fade <= fade + 11'd1;
                // WAV 取样：淡出期间照常取（声音是渐弱的，不是突然没了），清空这一拍不取；
                //   清空请求挂着时不开始新的播放（缓冲里是要丢掉的旧歌）
                if (cur_wav && !flush_now && !(want_switch && fade == 11'd0) && !hold) begin
                    if (playing) begin
                        if (ring_level != 0) begin
                            ring_pop <= 1'b1; popped <= 1'b1;
                        end else begin
                            playing <= 1'b0;
                            if (!ring_eos && !ring_flush_pend) underruns <= underruns + 32'd1;
                        end
                    end else if (!ring_flush_pend && (ring_level >= PREFILL || (ring_eos && ring_level != 0))) begin
                        playing <= 1'b1;
                        ring_pop <= 1'b1; popped <= 1'b1;
                    end
                end
            end

            case (ph)
                4'd1: ph <= 4'd2;          // 缓冲读出要 2 拍（pop 寄存一拍 + BRAM 一拍）
                4'd2: begin
                    if (src_wav) begin
                        l_src <= popped ? ring_data[31:16] : 16'sd0;
                        r_src <= popped ? ring_data[15:0]  : 16'sd0;
                    end else begin
                        l_src <= arp_q; r_src <= arp_q;
                    end
                    if (popped) begin      // 已播放时间
                        if (fcnt == 16'd47999) begin
                            fcnt <= 16'd0;
                            elapsed_bcd <= bcd_inc(elapsed_bcd);
                        end else
                            fcnt <= fcnt + 16'd1;
                    end
                    ph <= 4'd3;
                end
                4'd3: begin l_out <= mul_p[23:8]; ph <= 4'd4; end
                4'd4: begin
                    odata <= {l_out, mul_p[23:8]};
                    opush <= 1'b1;
                    ph <= 4'd0;
                end
                default: ;
            endcase
        end
    end

    // ---------------- 输出 FIFO ----------------
    wire opop = (ocnt != 0) && sample_ready;
    assign sample_valid = (ocnt != 0);
    assign sample_left  = ofifo[orp][31:16];
    assign sample_right = ofifo[orp][15:0];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owp <= 0; orp <= 0; ocnt <= 0; overflow <= 1'b0;
        end else begin
            if (tone_overflow) overflow <= 1'b1;
            if (opush && (ocnt != 8 || opop)) begin
                ofifo[owp] <= odata;
                owp <= owp + 3'd1;
            end else if (opush)
                overflow <= 1'b1;
            if (opop) orp <= orp + 3'd1;
            case ({opush && (ocnt != 8 || opop), opop})
                2'b10: ocnt <= ocnt + 4'd1;
                2'b01: ocnt <= ocnt - 4'd1;
                default: ;
            endcase
        end
    end
endmodule
