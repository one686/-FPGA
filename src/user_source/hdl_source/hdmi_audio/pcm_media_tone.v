`timescale 1ns/1ps

// 内置琶音音源（音画联动，没有 WAV 时使用；同时作为 48kHz 采样节拍源）
//   4 段六音琶音循环（C 大调上行、A 小调、五声音阶、高低摆动），按图片序号选旋律；
//   每次换图先响 100ms 的 1.5kHz 提示音，再从第一个音开始播新旋律；volume 缩放输出。
//   48kHz 由 25MHz 分数累加精确产生；输出经 8 级 FIFO，valid/ready 握手，左右声道相同。
//
// 每个音（包括提示音）都套"拨弦"包络：短线性起音 + 指数衰减。新音从当前电平重新起音，增益不会突跳；
//   配合相位连续的振荡器，换音、换图、上电都没有"咔"声。
//
// media_id = {换图翻转位, 旋律号}：第 n 张图用第 ((n-1) mod 4)+1 段旋律；两张图旋律号相同时
//   靠翻转位变化识别换图，照样响提示音、旋律从头开始。
module pcm_media_tone #(
    parameter integer CLOCK_HZ = 25000000,
    parameter integer SAMPLE_HZ = 48000,
    parameter [19:0] BLIP_INC = 20'd32768,   // ~1500 Hz switch cue
    parameter integer BLIP_SAMPLES = 4800,   // 100 ms at 48 kHz
    parameter integer NOTE_SAMPLES = 12000,  // 250 ms per melody note
    // 起音时间：从 0 升到满幅用的样本数。240 = 5 ms（48 kHz）。
    // 调小起音更"脆"（太小会有轻微咔哒声），调大更柔和（像弓弦乐器）。
    parameter integer ATTACK_SAMPLES = 240,
    // 音尾衰减速度：每个样本增益减少 1/2**DECAY_SHIFT，时间常数 = 2**DECAY_SHIFT 个样本。
    // 觉得音尾衰减太快或太慢，改这个参数就行：改成 13 余音更长，改成 11 更短促。
    //   取值  时间常数   一个音(250 ms)结尾的音量
    //    11    43 ms      -50 dB（很短促，像木琴）
    //    12    85 ms      -25 dB（当前值，像拨弦）
    //    13   171 ms      -12 dB（余音更长，像钢琴）
    //    14   341 ms       -6 dB（几乎不衰减）
    // 注意：改完后 sim/tone_model.py 里的 DECAY_SHIFT 也要改成同样的值，否则仿真比对会报不一致。
    parameter integer DECAY_SHIFT = 12
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire [2:0]         media_id,     // {换图翻转位, 旋律号 0~3}
    input  wire [7:0]         volume,
    output wire               sample_valid,
    input  wire               sample_ready,
    output wire signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output reg                overflow,
    output wire               blip_active
);
    reg [24:0] rate_accumulator;
    reg [19:0] phase;
    reg [2:0] media_id_q;
    reg [12:0] blip_countdown;
    reg [2:0] note_index;
    reg [14:0] note_sample_count;
    reg signed [15:0] fifo [0:7];
    reg [2:0] write_pointer;
    reg [2:0] read_pointer;
    reg [3:0] fifo_count;
    reg [23:0] env_gain;        // envelope gain, 24'hFFFFFF = 1.0
    reg env_attack;             // 1: rising, 0: decaying
    wire sample_pop;
    wire sample_tick;
    wire sample_push;

    assign sample_valid = fifo_count != 0;
    assign sample_left = fifo[read_pointer];
    assign sample_right = fifo[read_pointer];
    assign sample_pop = sample_valid && sample_ready;
    assign sample_tick = rate_accumulator >= CLOCK_HZ - SAMPLE_HZ;
    assign sample_push = sample_tick && (fifo_count != 8 || sample_pop);
    assign blip_active = blip_countdown != 0;

    // phase_inc = freq * 2**20 / SAMPLE_HZ (freq * 21.8453)
    function [19:0] melody_inc;
        input [1:0] melody;
        input [2:0] note;
        begin
            case ({melody, note})
                // 0: C-major rise C5 E5 G5 C6 G5 E5
                5'b00_000: melody_inc = 20'd11426;  // C5 523 Hz
                5'b00_001: melody_inc = 20'd14396;  // E5 659 Hz
                5'b00_010: melody_inc = 20'd17127;  // G5 784 Hz
                5'b00_011: melody_inc = 20'd22860;  // C6 1046 Hz
                5'b00_100: melody_inc = 20'd17127;  // G5
                5'b00_101: melody_inc = 20'd14396;  // E5
                // 1: A-minor A4 C5 E5 A5 E5 C5
                5'b01_000: melody_inc = 20'd9634;   // A4 440 Hz
                5'b01_001: melody_inc = 20'd11426;  // C5
                5'b01_010: melody_inc = 20'd14396;  // E5
                5'b01_011: melody_inc = 20'd19269;  // A5 880 Hz
                5'b01_100: melody_inc = 20'd14396;  // E5
                5'b01_101: melody_inc = 20'd11426;  // C5
                // 2: pentatonic rise C5 D5 E5 G5 A5 C6
                5'b10_000: melody_inc = 20'd11426;  // C5
                5'b10_001: melody_inc = 20'd12833;  // D5 587 Hz
                5'b10_010: melody_inc = 20'd14396;  // E5
                5'b10_011: melody_inc = 20'd17127;  // G5
                5'b10_100: melody_inc = 20'd19269;  // A5
                5'b10_101: melody_inc = 20'd22860;  // C6
                // 3: low-high swing C5 G4 E5 C5 G4 E5
                5'b11_000: melody_inc = 20'd11426;  // C5
                5'b11_001: melody_inc = 20'd8566;   // G4 392 Hz
                5'b11_010: melody_inc = 20'd14396;  // E5
                5'b11_011: melody_inc = 20'd11426;  // C5
                5'b11_100: melody_inc = 20'd8566;   // G4
                5'b11_101: melody_inc = 20'd14396;  // E5
                default:   melody_inc = 20'd11426;
            endcase
        end
    endfunction

    wire [19:0] melody_note_inc = melody_inc(media_id_q[1:0], note_index);
    wire [19:0] phase_inc = (blip_countdown != 0) ? BLIP_INC : melody_note_inc;

    // 256-point sine LUT stored as one quarter wave (64 entries, no BRAM).
    // phase[19:12] is the 8-bit address: [7] selects the negative half,
    // [6] mirrors the index inside the half, [5:0] is the quarter index.
    // Entries sit on half-step angles, sin(2*pi*(k+0.5)/256), so the mirror
    // is an exact bitwise NOT and the table needs no separate 90-degree entry.
    // 256 points keep LUT spurs about 48 dB below the tone (16 points would
    // give only ~24 dB). Peak is 12000 (about 1/3 full scale).
    function [15:0] quarter_sine;
        input [5:0] index;
        begin
            case (index)
                // ---- generated by sim/gen_sine_lut.py, do not edit ----
                6'd0:     quarter_sine = 16'd147;
                6'd1:     quarter_sine = 16'd442;
                6'd2:     quarter_sine = 16'd736;
                6'd3:     quarter_sine = 16'd1030;
                6'd4:     quarter_sine = 16'd1323;
                6'd5:     quarter_sine = 16'd1615;
                6'd6:     quarter_sine = 16'd1906;
                6'd7:     quarter_sine = 16'd2196;
                6'd8:     quarter_sine = 16'd2485;
                6'd9:     quarter_sine = 16'd2773;
                6'd10:    quarter_sine = 16'd3058;
                6'd11:    quarter_sine = 16'd3342;
                6'd12:    quarter_sine = 16'd3624;
                6'd13:    quarter_sine = 16'd3904;
                6'd14:    quarter_sine = 16'd4181;
                6'd15:    quarter_sine = 16'd4456;
                6'd16:    quarter_sine = 16'd4728;
                6'd17:    quarter_sine = 16'd4997;
                6'd18:    quarter_sine = 16'd5263;
                6'd19:    quarter_sine = 16'd5526;
                6'd20:    quarter_sine = 16'd5786;
                6'd21:    quarter_sine = 16'd6042;
                6'd22:    quarter_sine = 16'd6295;
                6'd23:    quarter_sine = 16'd6544;
                6'd24:    quarter_sine = 16'd6789;
                6'd25:    quarter_sine = 16'd7030;
                6'd26:    quarter_sine = 16'd7266;
                6'd27:    quarter_sine = 16'd7498;
                6'd28:    quarter_sine = 16'd7726;
                6'd29:    quarter_sine = 16'd7949;
                6'd30:    quarter_sine = 16'd8167;
                6'd31:    quarter_sine = 16'd8381;
                6'd32:    quarter_sine = 16'd8589;
                6'd33:    quarter_sine = 16'd8792;
                6'd34:    quarter_sine = 16'd8990;
                6'd35:    quarter_sine = 16'd9182;
                6'd36:    quarter_sine = 16'd9369;
                6'd37:    quarter_sine = 16'd9550;
                6'd38:    quarter_sine = 16'd9725;
                6'd39:    quarter_sine = 16'd9895;
                6'd40:    quarter_sine = 16'd10059;
                6'd41:    quarter_sine = 16'd10216;
                6'd42:    quarter_sine = 16'd10368;
                6'd43:    quarter_sine = 16'd10513;
                6'd44:    quarter_sine = 16'd10652;
                6'd45:    quarter_sine = 16'd10784;
                6'd46:    quarter_sine = 16'd10910;
                6'd47:    quarter_sine = 16'd11029;
                6'd48:    quarter_sine = 16'd11142;
                6'd49:    quarter_sine = 16'd11248;
                6'd50:    quarter_sine = 16'd11347;
                6'd51:    quarter_sine = 16'd11440;
                6'd52:    quarter_sine = 16'd11525;
                6'd53:    quarter_sine = 16'd11604;
                6'd54:    quarter_sine = 16'd11675;
                6'd55:    quarter_sine = 16'd11740;
                6'd56:    quarter_sine = 16'd11797;
                6'd57:    quarter_sine = 16'd11848;
                6'd58:    quarter_sine = 16'd11891;
                6'd59:    quarter_sine = 16'd11927;
                6'd60:    quarter_sine = 16'd11956;
                6'd61:    quarter_sine = 16'd11977;
                6'd62:    quarter_sine = 16'd11992;
                default:  quarter_sine = 16'd11999;
                // ---- end of generated table ----
            endcase
        end
    endfunction

    wire [7:0] lut_address = phase[19:12];
    wire [5:0] quarter_index = lut_address[6] ? ~lut_address[5:0] :
                                                lut_address[5:0];
    wire signed [15:0] quarter_value = quarter_sine(quarter_index);
    wire signed [15:0] raw_sample = lut_address[7] ? -quarter_value :
                                                     quarter_value;

    // Envelope: attack adds a fixed step up to full scale; decay removes
    // gain/2**DECAY_SHIFT per sample, an exponential fall of ~24 dB per note.
    localparam [23:0] ENV_MAX = 24'hFFFFFF;
    localparam [23:0] ATTACK_STEP = 24'hFFFFFF / ATTACK_SAMPLES;
    wire signed [32:0] enveloped = raw_sample * $signed({1'b0, env_gain[23:8]});
    wire signed [15:0] env_sample = enveloped[31:16];   // |value| < 12000

    wire signed [23:0] scaled = env_sample * $signed({1'b0, volume});
    wire signed [15:0] scaled_clamped =
        (scaled > 24'sd8388352)  ? 16'sd32767 :
        (scaled < -24'sd8388608) ? -16'sd32768 : scaled >>> 8;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rate_accumulator <= 25'd0;
            phase <= 20'd0;
            media_id_q <= 3'd0;
            blip_countdown <= 13'd0;
            note_index <= 3'd0;
            note_sample_count <= 15'd0;
            write_pointer <= 3'd0;
            read_pointer <= 3'd0;
            fifo_count <= 4'd0;
            overflow <= 1'b0;
            env_gain <= 24'd0;
            env_attack <= 1'b1;         // fade in after reset, no power-up pop
        end else begin
            media_id_q <= media_id;

            if (sample_tick)
                rate_accumulator <= rate_accumulator + SAMPLE_HZ - CLOCK_HZ;
            else
                rate_accumulator <= rate_accumulator + SAMPLE_HZ;

            if (sample_push) begin
                fifo[write_pointer] <= scaled_clamped;
                write_pointer <= write_pointer + 1'b1;
                phase <= phase + phase_inc;

                if (env_attack) begin
                    if (env_gain > ENV_MAX - ATTACK_STEP) begin
                        env_gain <= ENV_MAX;
                        env_attack <= 1'b0;
                    end else begin
                        env_gain <= env_gain + ATTACK_STEP;
                    end
                end else begin
                    env_gain <= env_gain - (env_gain >> DECAY_SHIFT);
                end

                // Each new note re-attacks; assigned after the envelope step
                // so the retrigger wins on the same sample.
                if (blip_countdown != 0) begin
                    blip_countdown <= blip_countdown - 1'b1;
                    if (blip_countdown == 13'd1)
                        env_attack <= 1'b1;     // blip over, first melody note
                end else if (note_sample_count >= NOTE_SAMPLES - 1) begin
                    note_sample_count <= 15'd0;
                    note_index <= (note_index == 3'd5) ? 3'd0 : note_index + 1'b1;
                    env_attack <= 1'b1;
                end else begin
                    note_sample_count <= note_sample_count + 1'b1;
                end
            end else if (sample_tick) begin
                overflow <= 1'b1;
            end

            if (sample_pop)
                read_pointer <= read_pointer + 1'b1;

            case ({sample_push, sample_pop})
                2'b10: fifo_count <= fifo_count + 1'b1;
                2'b01: fifo_count <= fifo_count - 1'b1;
                default: fifo_count <= fifo_count;
            endcase

            // A switch reloads the cue blip and restarts the melody from its
            // first note. Load last so a reload wins over the same-edge
            // decrement or note advance.
            if (media_id != media_id_q) begin
                blip_countdown <= BLIP_SAMPLES[12:0];
                note_index <= 3'd0;
                note_sample_count <= 15'd0;
                env_attack <= 1'b1;             // cue blip starts its own attack
            end
        end
    end
endmodule
