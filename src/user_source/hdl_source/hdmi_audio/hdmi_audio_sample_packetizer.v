`timescale 1ns/1ps

// HDMI 音频采样包（Audio Sample Packet）打包：每 4 个立体声采样组成一个包（4 个子包），送数据岛调度。
//   采样一到就写进 packet_body 里对应的子包，ECC 也在这一拍算好寄存：只用 1 个 BCH（算当前这个子包），
//   4 个子包的 ECC 分别寄存，不另存采样（省资源）。
//   packet_valid = 1 期间 packet_body / ECC 保持不变；收集期间 packet_body 是半新半旧的，下游只在 packet_valid 时取用。
//   IEC 60958 帧号（每包 4 帧，0~191 循环）只存 帧号 / 4（0~47）；包头 HB2.B0 在第 0 帧所在的包置 1（块起点）。
//   通道状态位 C 只在帧号 25、33 为 1（48 kHz / 16 位），也就是第 1 个子包（cc = 1）且 fb = 6 或 8。
//   奇偶位：采样字低 8 位是 0，不影响奇偶，所以只对 16 位采样和 C 求奇偶。
module hdmi_audio_sample_packetizer (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_valid,
    output wire               sample_ready,
    input  wire signed [15:0] sample_left,
    input  wire signed [15:0] sample_right,
    output reg                packet_valid,
    input  wire               packet_ready,
    output reg  [23:0]        packet_header,
    output wire [7:0]         packet_header_ecc,
    output reg  [223:0]       packet_body,
    output wire [31:0]        packet_body_ecc,
    output wire [7:0]         packet_frame_base
);
    reg [1:0]  collect_count;           // 这个包已经收到几个采样
    reg [5:0]  frame_quad;              // 下一个包第 0 个子包的帧号 / 4（0~47）
    reg [5:0]  packet_quad;             // 当前包的帧号 / 4
    reg [31:0] body_ecc;                // 4 个子包的 ECC（和子包同一拍写入）

    assign sample_ready      = !packet_valid;
    assign packet_frame_base = {packet_quad, 2'b00};
    assign packet_body_ecc   = body_ecc;

    // 当前采样的子包。16-bit LPCM 左对齐放在 24 位 IEC 60958 采样字里；
    //   控制半字节顺序是右声道、左声道各自的 P/C/U/V
    wire        channel_status = (collect_count == 2'd1) &&
                                 (frame_quad == 6'd6 || frame_quad == 6'd8);   // 帧号 25 / 33
    wire        left_parity  = ^{sample_left, channel_status};
    wire        right_parity = ^{sample_right, channel_status};
    wire [55:0] subpacket = {right_parity, channel_status, 2'b00,
                             left_parity, channel_status, 2'b00,
                             sample_right, 8'h00, sample_left, 8'h00};
    wire [7:0]  subpacket_ecc;

    hdmi_bch8 #(.DATA_BYTES(3)) u_header_bch (
        .data(packet_header), .ecc(packet_header_ecc)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body_bch (
        .data(subpacket), .ecc(subpacket_ecc)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            collect_count <= 2'd0;
            packet_valid <= 1'b0;
            packet_header <= 24'd0;
            packet_body <= 224'd0;
            body_ecc <= 32'd0;
            frame_quad <= 6'd0;
            packet_quad <= 6'd0;
        end else if (packet_valid) begin
            if (packet_ready)
                packet_valid <= 1'b0;
        end else if (sample_valid) begin
            collect_count <= collect_count + 2'd1;
            case (collect_count)
                2'd0: begin packet_body[55:0]    <= subpacket; body_ecc[7:0]   <= subpacket_ecc; end
                2'd1: begin packet_body[111:56]  <= subpacket; body_ecc[15:8]  <= subpacket_ecc; end
                2'd2: begin packet_body[167:112] <= subpacket; body_ecc[23:16] <= subpacket_ecc; end
                default: begin
                    packet_body[223:168] <= subpacket; body_ecc[31:24] <= subpacket_ecc;
                    // Low byte is HB0. Layout=0 and all four subpackets are
                    // present. HB2.B0 marks the first IEC status block frame.
                    packet_header <= {(frame_quad == 6'd0) ? 8'h10 : 8'h00, 8'h0f, 8'h02};
                    packet_quad <= frame_quad;
                    packet_valid <= 1'b1;
                    frame_quad <= (frame_quad == 6'd47) ? 6'd0 : frame_quad + 6'd1;
                end
            endcase
        end
    end
endmodule
