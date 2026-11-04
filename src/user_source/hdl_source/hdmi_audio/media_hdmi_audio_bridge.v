`timescale 1ns/1ps

// HDMI 音视频桥（pixel_clk 域，25MHz）：视频信号寄存一拍后与音频一起送进 hdmi_audio_symbol_core，
//   输出三路 10 位 TMDS 符号（含数据岛：音频采样包、ACR、InfoFrame）。串行化 / DDR 输出原语放在顶层，
//   本模块不含厂商原语，可直接做 RTL 仿真。
// 音源 media_audio_source：内置琶音 / TF 卡 WAV 二选一 + 淡入淡出 + 音量 + 暂停；
//   WAV 样本来自 wav_ring 的读侧；没有 WAV 时（wav_active=0）播琶音，media_id 选旋律、换图时响提示音。
// pcm_tap_*：实际送进 HDMI 的音频样本，供频谱 / 全屏可视化使用。
module media_hdmi_audio_bridge #(
    parameter integer RING_AW = 12
) (
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        video_hsync,
    input  wire        video_vsync,
    input  wire        video_de,
    input  wire [9:0]  video_x,
    input  wire [23:0] video_rgb,
    input  wire [2:0]  media_id,    // {换图翻转位, 旋律号}
    input  wire [7:0]  volume,
    output wire [9:0]  channel0_symbol,
    output wire [9:0]  channel1_symbol,
    output wire [9:0]  channel2_symbol,
    output wire        sequence_error,
    output wire        tone_overflow,
    output wire        pcm_contract_error,
    output wire        blip_active,
    output wire        pcm_tap_valid,
    output wire signed [15:0] pcm_tap_left,
    output wire signed [15:0] pcm_tap_right,      // 送进 HDMI 的样本（频谱显示用）
    // WAV 音源接口
    input  wire        wav_active_async,
    input  wire [8:0]  vol_gain,
    input  wire        pause_async,        // 暂停（sd_card_clk 域电平）
    input  wire [RING_AW:0] ring_level,
    output wire        ring_pop,
    input  wire [31:0] ring_data,
    input  wire        ring_flush_pend,
    output wire        ring_flush_do,
    input  wire        ring_eos,
    output wire [15:0] elapsed_bcd,
    output wire [31:0] wav_underruns
);
    wire source_valid;
    wire source_ready;
    wire signed [15:0] source_left;
    wire signed [15:0] source_right;
    wire sample_valid;
    wire sample_ready;
    wire signed [15:0] sample_left;
    wire signed [15:0] sample_right;

    reg video_hsync_q;
    reg video_vsync_q;
    reg video_de_q;
    reg [9:0] video_x_q;
    reg [23:0] video_rgb_q;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            video_hsync_q <= 1'b1;
            video_vsync_q <= 1'b1;
            video_de_q <= 1'b0;
            video_x_q <= 10'd0;
            video_rgb_q <= 24'h000000;
        end else begin
            video_hsync_q <= video_hsync;
            video_vsync_q <= video_vsync;
            video_de_q <= video_de;
            video_x_q <= video_x;
            video_rgb_q <= video_rgb;   // 不按 de 清零（省 LUT）：只有视频期（de = 1）才用它，上游 rgb_video_adapter 消隐期也已是 0
        end
    end

    media_audio_source #(.AW(RING_AW)) u_source (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .media_id(media_id), .tone_volume(volume),
        .wav_active_async(wav_active_async), .vol_gain(vol_gain), .pause_async(pause_async),
        .ring_level(ring_level), .ring_pop(ring_pop), .ring_data(ring_data),
        .ring_flush_pend(ring_flush_pend), .ring_flush_do(ring_flush_do), .ring_eos(ring_eos),
        .sample_valid(source_valid), .sample_ready(source_ready),
        .sample_left(source_left), .sample_right(source_right),
        .overflow(tone_overflow), .blip_active(blip_active),
        .elapsed_bcd(elapsed_bcd), .underruns(wav_underruns)
    );

    m06_pcm_contract_adapter u_m06_pcm_contract (
        .clk(pixel_clk), .rst_n(pixel_rst_n),
        .source_valid(source_valid), .source_ready(source_ready),
        .source_left(source_left), .source_right(source_right),
        .pcm_valid(sample_valid), .pcm_ready(sample_ready),
        .pcm_left(sample_left), .pcm_right(sample_right),
        .contract_error(pcm_contract_error)
    );

    hdmi_audio_symbol_core u_symbol_core (
        .pixel_clk(pixel_clk), .rst_n(pixel_rst_n),
        .video_hsync(video_hsync_q), .video_vsync(video_vsync_q),
        .video_de(video_de_q), .video_x(video_x_q), .video_rgb(video_rgb_q),
        .sample_valid(sample_valid), .sample_ready(sample_ready),
        .sample_left(sample_left), .sample_right(sample_right),
        .channel0_symbol(channel0_symbol),
        .channel1_symbol(channel1_symbol),
        .channel2_symbol(channel2_symbol),
        .sequence_error(sequence_error)
    );

    assign pcm_tap_valid = sample_valid && sample_ready;
    assign pcm_tap_left = sample_left;
    assign pcm_tap_right = sample_right;
endmodule
