`timescale 1ns/1ps

// frame_write_path：整帧缓存的 SDRAM 写通道。
//
// 结构：写 FIFO wfifo_32_32_512（write_clk -> mem_clk 跨时钟域）+ frame_fifo_write（写 burst 控制，
//   默认 256 字 burst，可按图做 BMP 上下翻转）+ thumb_gen（缩略图）。
// 读写互斥（SDRAM 控制器只有一个口）：
//   rd_busy : 来自 media_frame_reader，读 burst 进行中时不开始写 burst
//   wr_busy : 给 media_frame_reader，写 burst 进行中（或这一拍就要开始）时不开始读 burst
// 每张图：write_addr_index 选 6 块整帧缓存中的哪一块（基地址 = 块号 x 一帧字数，在 frame_fifo_write 里算）；
//   write_v_flip 决定是否上下翻转（BMP 正常是从下往上存，高度为负的倒存 BMP 不翻）；
//   write_used 为写 FIFO 已用字数，缩放模块据此控制输出速度；两次写 burst 之间至少隔 BURST_GAP_CYCLES。
// 缩略图：整帧 burst 写进 SDRAM 的像素（mem_clk 域，文件里的行顺序）同时送进 thumb_gen，
//   缩出 THUMB_W x THUMB_H 的缩略图（每行补到 THUMB_STRIDE 字），经同步 FIFO 写到
//   THUMB_BASE + write_thumb_index（图片序号 0~15）x 一张缩略图字数。缩略图按图片序号存放，整帧块被换出后仍保留。
//   缩略图不翻转（按文件里的行顺序存），显示时按这张图的翻转标志选行。整帧和缩略图共用写口（见 frame_fifo_write）。
//   THUMB_H = 0：不生成缩略图。
module frame_write_path #(
    parameter MEM_DATA_BITS   = 32,
    parameter WRITE_DATA_BITS = 32,
    parameter ADDR_BITS       = 21,
    parameter BURST_BITS      = 9,
    parameter BURST_SIZE      = 256,
    parameter WRITE_V_FLIP    = 1,
    parameter FRAME_WIDTH     = 640,
    parameter FRAME_HEIGHT    = 480,
    parameter BURST_GAP_CYCLES = 8000,    // 两次写 burst 之间至少隔这么多 mem_clk（原因见 frame_fifo_write.v）
    parameter THUMB_W         = 112,      // 缩略图宽、高、行距、在 SDRAM 里的起始地址
    parameter THUMB_H         = 84,       //   THUMB_STRIDE x THUMB_H 必须是 BURST_SIZE 的整数倍
    parameter THUMB_STRIDE    = 128,
    parameter THUMB_BASE      = 1843200
) (
    input                          rst,
    input                          mem_clk,
    input                          Sdr_init_done,
    input                          Sdr_init_ref_vld,
    input                          Sdr_busy,
    input                          rd_busy,
    output                         wr_busy,

    output                         App_wr_en,
    output [ADDR_BITS-1:0]         App_wr_addr,
    output [MEM_DATA_BITS-1:0]     App_wr_din,
    output [3:0]                   App_wr_dm,

    input                          write_clk,
    input                          write_req,
    output                         write_req_ack,
    output                         write_finish,
    input  [2:0]                   write_addr_index,  // 写哪块整帧缓存（0~5）
    input  [3:0]                   write_thumb_index, // 缩略图写第几格（= 图片序号 0~15）
    input                          write_v_flip,      // 本张图是否上下翻转（写时钟域，读完 BMP 头后给出）
    output [8:0]                   write_used,        // 写 FIFO 已用字数（写时钟域），缩放模块按它控制输出速度
    input  [ADDR_BITS-1:0]         write_len,
    input                          write_en,
    input  [WRITE_DATA_BITS-1:0]   write_data
);
    localparam integer THUMB_WORDS = THUMB_STRIDE * THUMB_H;
    wire [BURST_BITS-1:0] rdusedw;
    wire                  write_fifo_aclr;
    wire [31:0]           frame_dout;
    wire                  thumb_rd_en, thumb_sel;
    reg  [31:0]           tq_q;                 // 缩略图 FIFO 读数据（先出现：= 下一个要读的字）
    reg  [9:0]            tq_lvl;

    assign App_wr_dm = 4'b0000;
    // 写数据的低 8 位不用选：整帧字（sd_card_bmp 给的 {RGB, 8'h00}）和缩略图字（thumb_gen 给的 {RGB, 8'h00}）
    //   低 8 位都恒为 0，直接取 frame_dout[7:0]。注意不能直接接常数 0：那样 TD 会把写 FIFO 的 RAM 推成寄存器
    assign App_wr_din = {thumb_sel ? tq_q[31:8] : frame_dout[31:8], frame_dout[7:0]};     // burst 期间 thumb_sel 不变

    wfifo_32_32_512 write_buf (
        .clkr       (mem_clk),
        .clkw       (write_clk),
        .rst        (write_fifo_aclr),
        .we         (write_en),
        .re         (App_wr_en && !thumb_sel),
        .di         (write_data),
        .empty_flag (),
        .full_flag  (),
        .wrusedw    (write_used),
        .rdusedw    (rdusedw),
        .dout       (frame_dout)
    );

    // ---- 缩略图 ----
    //   同步 FIFO 512 字（块 RAM）。读数据每拍按"下一个要读的地址"重新读一次，所以 tq_q 总是 FIFO 头上那个字；
    //   缩略图 burst 只在 FIFO 里有 >= 一个 burst 的字时才开始，读的字都是早就写好的。
    //   最多同时存 2 个缩略图行多一点（写口每 2 行写一个 burst，缩略图优先），512 字不会满。
    generate if (THUMB_H != 0) begin : g_thumb
        wire        t_we;
        wire [31:0] t_data;
        reg  [8:0]  tq_wp, tq_rp;
        wire [8:0]  tq_rp_n = tq_rp + {8'd0, thumb_rd_en};
        (* ram_style = "bram" *) reg [31:0] tq_mem [0:511];
        thumb_gen #(
            .FW (FRAME_WIDTH), .FH (FRAME_HEIGHT), .TW (THUMB_W), .TH (THUMB_H), .TS (THUMB_STRIDE)
        ) u_thumb_gen (
            .clk    (mem_clk),
            .rst    (rst),
            .clr    (write_fifo_aclr),      // 一张图开始写
            .pix_en (App_wr_en && !thumb_sel),
            .pix    (frame_dout[31:8]),
            .t_we   (t_we),
            .t_data (t_data)
        );
        always @(posedge mem_clk) begin
            if (t_we) tq_mem[tq_wp] <= t_data;
            tq_q <= tq_mem[tq_rp_n];
        end
        always @(posedge mem_clk or posedge rst) begin
            if (rst) begin
                tq_wp <= 9'd0; tq_rp <= 9'd0; tq_lvl <= 10'd0;
            end else if (write_fifo_aclr) begin
                tq_wp <= 9'd0; tq_rp <= 9'd0; tq_lvl <= 10'd0;
            end else begin
                tq_wp  <= tq_wp + {8'd0, t_we};
                tq_rp  <= tq_rp_n;
                tq_lvl <= tq_lvl + {9'd0, t_we} - {9'd0, thumb_rd_en};
            end
        end
    end else begin : g_no_thumb
        always @(posedge mem_clk) begin tq_q <= 32'd0; tq_lvl <= 10'd0; end
    end endgenerate

    frame_fifo_write #(
        .MEM_DATA_BITS (MEM_DATA_BITS),
        .ADDR_BITS     (ADDR_BITS),
        .BURST_BITS    (BURST_BITS),
        .BURST_SIZE    (BURST_SIZE),
        .WRITE_V_FLIP  (WRITE_V_FLIP),
        .FRAME_WIDTH   (FRAME_WIDTH),
        .FRAME_HEIGHT  (FRAME_HEIGHT),
        .BURST_GAP_CYCLES (BURST_GAP_CYCLES),
        .THUMB_BASE    (THUMB_BASE),
        .THUMB_WORDS   (THUMB_WORDS)
    ) frame_fifo_write_m0 (
        .rst              (rst),
        .mem_clk          (mem_clk),
        .Sdr_init_done    (Sdr_init_done),
        .Sdr_init_ref_vld (Sdr_init_ref_vld),
        .Sdr_busy         (Sdr_busy),
        .App_rd_busy      (rd_busy),
        .O_wr_busy        (wr_busy),
        .App_wr_en        (App_wr_en),
        .App_wr_addr      (App_wr_addr),
        .write_req        (write_req),
        .write_req_ack    (write_req_ack),
        .write_finish     (write_finish),
        .write_addr_index (write_addr_index),
        .write_thumb_index(write_thumb_index),
        .write_v_flip     (write_v_flip),
        .write_len        (write_len),
        .fifo_aclr        (write_fifo_aclr),
        .rdusedw          ({1'b0, rdusedw}),
        .thumb_level      (tq_lvl),
        .thumb_rd_en      (thumb_rd_en),
        .thumb_sel        (thumb_sel)
    );
endmodule
