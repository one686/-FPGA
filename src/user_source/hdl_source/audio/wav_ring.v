`timescale 1ns/1ps

// WAV 环形缓冲（异步 FIFO）：写侧 sd_card_clk（50MHz），读侧 video_clk（25MHz）
//   2^AW 帧 x 32 位（{左16, 右16}），AW=12 -> 4096 帧 = 85ms @48kHz，用 4 块 BRAM32K。
//   读写指针多 1 位（区分满/空），格雷码 + 两级同步跨时钟域。
//   写侧：space_ok = 空位 >= SPACE_MIN（够放一个扇区的样本，才允许再读一个扇区）；empty_w = 已播空。
//   清空：写侧 flush 单拍 -> 请求翻转 -> 读侧看到 flush_pend_r，等声音淡出后给 flush_do：
//         读指针直接跳到写指针（丢掉剩下的样本），回一个确认翻转；写侧 flush_busy 在确认回来前为 1。
//         写侧保证清空期间不再写，所以读侧看到的写指针是稳定的。
//   eos_w（本首已全部写入）两级同步到读侧。
module wav_ring #(
    parameter integer AW = 12,
    parameter integer SPACE_MIN = 320
) (
    // 写侧
    input  wire          wclk,
    input  wire          wrst_n,
    input  wire          we,
    input  wire [31:0]   wdata,
    output wire          space_ok,
    output wire          empty_w,
    input  wire          flush,
    output wire          flush_busy,
    input  wire          eos_w,
    output reg           overflow,          // 仿真检查用：满了还写（不应发生）
    // 读侧
    input  wire          rclk,
    input  wire          rrst_n,
    output wire [AW:0]   level_r,
    input  wire          pop,               // 取一帧，下一拍 rdata 有效
    output reg  [31:0]   rdata,
    output wire          flush_pend_r,
    input  wire          flush_do,
    output wire          eos_r
);
    localparam integer DEPTH = 1 << AW;

    reg [31:0] mem [0:DEPTH-1];

    function [AW:0] to_gray;
        input [AW:0] b;
        begin
            to_gray = b ^ (b >> 1);
        end
    endfunction
    function [AW:0] from_gray;
        input [AW:0] g;
        integer i;
        reg [AW:0] b;
        begin
            b[AW] = g[AW];
            for (i = AW - 1; i >= 0; i = i - 1)
                b[i] = b[i + 1] ^ g[i];
            from_gray = b;
        end
    endfunction

    reg [AW:0] rp, rp_gray;            // 读侧指针（先声明，写侧要同步它）
    reg        ack_t;

    // ---------------- 写侧 ----------------
    reg [AW:0] wp, wp_gray;
    (* async_reg = "true" *) reg [AW:0] rpg_w1, rpg_w2;
    reg        req_t;
    (* async_reg = "true" *) reg ack_w1, ack_w2;
    wire [AW:0] rp_w    = from_gray(rpg_w2);
    wire [AW:0] level_w = wp - rp_w;
    assign space_ok   = (level_w <= DEPTH - SPACE_MIN);
    assign empty_w    = (level_w == 0);
    assign flush_busy = (req_t != ack_w2);

    always @(posedge wclk) begin
        if (we)
            mem[wp[AW-1:0]] <= wdata;
    end
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wp <= 0; wp_gray <= 0; rpg_w1 <= 0; rpg_w2 <= 0;
            req_t <= 1'b0; ack_w1 <= 1'b0; ack_w2 <= 1'b0; overflow <= 1'b0;
        end else begin
            if (we) begin
                wp <= wp + 1'b1;
                if (level_w == DEPTH) overflow <= 1'b1;
            end
            wp_gray <= to_gray(we ? wp + 1'b1 : wp);
            rpg_w1 <= rp_gray; rpg_w2 <= rpg_w1;
            ack_w1 <= ack_t;   ack_w2 <= ack_w1;
            if (flush) req_t <= ~req_t;
        end
    end

    // ---------------- 读侧 ----------------
    (* async_reg = "true" *) reg [AW:0] wpg_r1, wpg_r2;
    (* async_reg = "true" *) reg req_r1, req_r2, eos_r1, eos_r2;
    wire [AW:0] wp_r = from_gray(wpg_r2);
    assign level_r      = wp_r - rp;
    assign flush_pend_r = (req_r2 != ack_t);
    assign eos_r        = eos_r2;

    always @(posedge rclk) begin
        if (pop)
            rdata <= mem[rp[AW-1:0]];
    end
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rp <= 0; rp_gray <= 0; wpg_r1 <= 0; wpg_r2 <= 0;
            req_r1 <= 1'b0; req_r2 <= 1'b0; eos_r1 <= 1'b0; eos_r2 <= 1'b0; ack_t <= 1'b0;
        end else begin
            wpg_r1 <= wp_gray; wpg_r2 <= wpg_r1;
            req_r1 <= req_t;   req_r2 <= req_r1;
            eos_r1 <= eos_w;   eos_r2 <= eos_r1;
            if (flush_do) begin
                rp <= wp_r;
                rp_gray <= to_gray(wp_r);
                ack_t <= req_r2;
            end else begin
                if (pop) rp <= rp + 1'b1;
                rp_gray <= to_gray(pop ? rp + 1'b1 : rp);
            end
        end
    end
endmodule
