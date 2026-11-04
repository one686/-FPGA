`timescale 1ns/1ps

// 给 hdmi_audio_symbol_core 生成"有效区在前"的行坐标 video_x。
//
// 本工程视频时序是"有效区在后"：前肩 -> 行同步 -> 后肩 -> 有效区；
// symbol_core 要求：0~639 有效区，652 起数据岛，790~797 视频前导，798~799 视频保护带。
// 做法：用 de 上升沿把 0~799 的行计数对齐到有效区起点，输出与输入 hs/vs/de 同拍。
//
// 视频前导/保护带只能出现在"下一行有有效像素"的行尾，否则接收端会把后面的
// 控制期误当成视频。场消隐行里把 790~799 钳到 789，symbol_core 就当普通控制期处理。
// 哪些行有效由上一帧统计得到。只有从真实场同步沿开始的完整帧才可信（synced），
// 所以上电后第一帧不发视频前导，从第二帧起正常。
module hdmi_video_x_align #(
    parameter [9:0] H_TOTAL = 10'd800,
    parameter [9:0] H_ACTIVE = 10'd640
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       vs,           // 低有效场同步
    input  wire       de,
    output wire [9:0] video_x
);
    reg       de_q;
    reg       vs_q;
    reg [9:0] x_cnt;
    reg [9:0] y_cnt;                // 自场同步开始起的行号
    reg       seen_active;
    reg [9:0] first_y;              // 本帧第一个有效行
    reg [9:0] last_y;               // 本帧最后一个有效行
    reg [9:0] first_y_prev;         // 上一帧结果，用于预判下一行是否有效
    reg [9:0] last_y_prev;
    reg       prev_valid;
    reg       synced;               // 已经见过一次真实的场同步沿

    wire       de_rise = de & ~de_q;
    wire       vs_fall = vs_q & ~vs;
    wire [9:0] x_now   = de_rise ? 10'd0 : x_cnt;
    wire       line_end = (x_now == H_TOTAL - 10'd1);
    wire [9:0] y_next  = y_cnt + 10'd1;
    wire       next_line_active = prev_valid &&
                                  (y_next >= first_y_prev) &&
                                  (y_next <= last_y_prev);

    assign video_x = ((x_now >= H_TOTAL - 10'd10) && !next_line_active) ?
                     (H_TOTAL - 10'd11) : x_now;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_q         <= 1'b0;
            vs_q         <= 1'b0;       // 复位后 vs 可能就是 0，不能当成一次场同步沿
            x_cnt        <= H_ACTIVE;   // 复位后先当消隐期，等第一个 de 上升沿对齐
            y_cnt        <= 10'd0;
            seen_active  <= 1'b0;
            first_y      <= 10'd0;
            last_y       <= 10'd0;
            first_y_prev <= 10'd0;
            last_y_prev  <= 10'd0;
            prev_valid   <= 1'b0;
            synced       <= 1'b0;
        end else begin
            de_q  <= de;
            vs_q  <= vs;
            x_cnt <= line_end ? 10'd0 : x_now + 10'd1;

            if (vs_fall) begin
                y_cnt        <= 10'd0;
                first_y_prev <= first_y;
                last_y_prev  <= last_y;
                prev_valid   <= seen_active & synced;
                synced       <= 1'b1;
                seen_active  <= 1'b0;
            end else if (line_end) begin
                y_cnt <= y_next;
            end

            if (de_rise) begin
                if (!seen_active)
                    first_y <= y_cnt;
                last_y      <= y_cnt;
                seen_active <= 1'b1;
            end
        end
    end
endmodule
