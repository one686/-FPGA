`timescale 1ns/1ps

// HDMI InfoFrame 打包：每次 emit（场同步开始）连续发两个 InfoFrame，先 Audio InfoFrame，再 AVI InfoFrame。
//   有些显示器在收到 AVI InfoFrame 之前一直静音，所以两个都要发。
module hdmi_audio_infoframe_packetizer (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         emit,
    output reg          packet_valid,
    input  wire         packet_ready,
    output wire [23:0]  packet_header,
    output wire [7:0]   packet_header_ecc,
    output wire [223:0] packet_body,
    output wire [31:0]  packet_body_ecc
);
    wire [7:0] ecc0, ecc1, ecc2, ecc3;
    reg send_avi;     // 0: Audio InfoFrame on the bus, 1: AVI InfoFrame

    // Low byte is HB0. Audio InfoFrame: type 0x84, version 1, length 10.
    // PB0 checksum=0x70, PB1.CC=1 means two channels. Coding type,
    // sample frequency/size and channel allocation refer to the stream.
    localparam [23:0]  AUDIO_HEADER = 24'h0a0184;
    localparam [223:0] AUDIO_BODY   = {208'd0, 8'h01, 8'h70};
    // AVI InfoFrame: type 0x82, version 2, length 13.
    // PB1=0x10 RGB, active format present; PB2=0x18 4:3, same as picture;
    // PB4=0x01 VIC 1 (640x480p60). PB0 checksum makes all bytes sum to 0.
    localparam [23:0]  AVI_HEADER = 24'h0d0282;
    localparam [223:0] AVI_BODY   = {184'd0, 8'h01, 8'h00, 8'h18,
                                     8'h10, 8'h46};

    assign packet_header = send_avi ? AVI_HEADER : AUDIO_HEADER;
    assign packet_body   = send_avi ? AVI_BODY   : AUDIO_BODY;

    hdmi_bch8 #(.DATA_BYTES(3)) u_header_bch (
        .data(packet_header), .ecc(packet_header_ecc)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body0_bch (
        .data(packet_body[55:0]), .ecc(ecc0)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body1_bch (
        .data(packet_body[111:56]), .ecc(ecc1)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body2_bch (
        .data(packet_body[167:112]), .ecc(ecc2)
    );
    hdmi_bch8 #(.DATA_BYTES(7)) u_body3_bch (
        .data(packet_body[223:168]), .ecc(ecc3)
    );
    assign packet_body_ecc = {ecc3, ecc2, ecc1, ecc0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            packet_valid <= 1'b0;
            send_avi <= 1'b0;
        end else if (packet_valid) begin
            // The packet only changes after it has been fully sent (ready is
            // pulsed on the last payload character), so the island scheduler
            // never sees it change mid-island.
            if (packet_ready) begin
                if (send_avi)
                    packet_valid <= 1'b0;
                send_avi <= ~send_avi;
            end
        end else if (emit) begin
            packet_valid <= 1'b1;
            send_avi <= 1'b0;
        end
    end
endmodule
