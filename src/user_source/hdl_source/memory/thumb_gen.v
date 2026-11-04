`timescale 1ns/1ps
// ===========================================================================
// thumb_gen.v —— 加载整帧时顺便缩出缩略图（默认 640x480 -> 112x84），供胶片条显示
//
// 位置：frame_write_path 里，SDRAM 时钟域，接在整帧写 FIFO 的输出后面（整帧 burst 写进 SDRAM 的那些字）。
//   像素按 BMP 文件里的行顺序进来（还没翻转），恰好 FW x FH 个；
//   缩略图也按文件里的行顺序输出，每行 TW 个像素 + (TS - TW) 个补 0 的字，共 TS x TH 个字，
//   SDRAM 里按这个顺序连续存（不翻转）；显示时按这张图是否翻转决定从哪一行读起。
//
// 算法：方块平均。源像素 x 属于第 floor(x*TW/FW) 列，源行 y 属于第 floor(y*TH/FH) 行，
//   640/112 = 480/84 = 5.714，每个方块 5 或 6 个像素宽 / 高（横、纵都按 6,6,6,5,6,6,5 循环）。
//   out = round(方块和 / 方块像素数)（四舍五入，.5 进位）；纯色区域结果和原色完全相同。
//   实现：sum * R + 2^(S-1) >> S，R = ceil(2^S / n)，S = 19（n = 25/30/36 时和整数四舍五入处处相同，
//   已用脚本对全部可能的方块和逐个验证）。
//
// 结构：
//   位置：TW/FW、TH/FH 先约分（640/112 -> 40/7），rx = x*TW' mod FW'，ry 同理；rx + TW' >= FW' 就是方块最后一列。
//   横向：每个颜色一个方块累加器，方块最后一个像素到达时得到这一方块的横向和（事件 ev）。
//   纵向：每列一个累加器（3 个颜色 x 16 位，放在 128 x 48 的块 RAM 里），方块第一行从 0 开始，
//         最后一行算出这一列的方块和，送去归一化；不是最后一行就写回。
//         读地址 = 当前列号 bx（方块至少 5 个像素，读数据早就准备好了）。
//   归一化：一个乘法器（1 个 DSP），三个颜色轮流用，每个缩略图像素 4 拍。
//   补 0：每个缩略图行的最后一个像素之后，再输出 TS - TW 个 0（下一个缩略图行至少 5 个源行之后才来）。
//
// 清零：clr（= 写侧开始一张图时的 FIFO 清零）为 1 时所有计数器回到起点；块 RAM 不用清零。
// 要求：FW/TW、FH/TH 都在 5 ~ 15 之间（方块至少 5 个像素：归一化 4 拍做完之前不会来下一个事件）；
//       像素最快每拍一个（整帧 burst 里就是每拍一个）。
// ===========================================================================
module thumb_gen #(
    parameter integer FW = 640,      // 整帧宽、高
    parameter integer FH = 480,
    parameter integer TW = 112,      // 缩略图宽、高
    parameter integer TH = 84,
    parameter integer TS = 128       // 缩略图行距（字），>= TW
) (
    input             clk,
    input             rst,
    input             clr,           // 1 = 新的一张图：计数器回到起点
    input             pix_en,
    input      [23:0] pix,           // {R, G, B}
    output reg        t_we,          // 缩略图字（{R, G, B, 8'h00}，补位字为 0）
    output reg [31:0] t_data
);
    function integer gcd;
        input integer a, b;
        integer t;
        begin
            while (b != 0) begin t = a % b; a = b; b = t; end
            gcd = a;
        end
    endfunction
    localparam integer GX = gcd(FW, TW), GY = gcd(FH, TH);
    localparam integer FX2 = FW / GX, TX2 = TW / GX;       // 640/112 -> 40/7
    localparam integer FY2 = FH / GY, TY2 = TH / GY;
    localparam integer XW = $clog2(FX2 + TX2), YW = $clog2(FY2 + TY2);
    localparam integer FX = FW / TW;                      // 方块宽：FX 或 FX+1
    localparam integer FY = FH / TH;
    localparam integer S  = 19;
    localparam [15:0] R00 = ((1 << S) + FX * FY - 1) / (FX * FY);              // R = ceil(2^S / n)
    localparam [15:0] R01 = ((1 << S) + FX * (FY + 1) - 1) / (FX * (FY + 1));
    localparam [15:0] R10 = ((1 << S) + (FX + 1) * FY - 1) / ((FX + 1) * FY);
    localparam [15:0] R11 = ((1 << S) + (FX + 1) * (FY + 1) - 1) / ((FX + 1) * (FY + 1));
    localparam integer PAD = TS - TW;
    // 累加器、计数器按实际最大值取位宽以节省资源（方块最大 (FX+1) x (FY+1)；默认 6x6 -> 横向和 <= 1530 用 11 位，
    //   纵向和 <= 9180 用 14 位，cx/cy <= 5 用 3 位，补 0 计数 <= 16 用 5 位）
    localparam integer BXM = FX + 1, BYM = FY + 1;                       // 方块最大宽、高
    localparam integer HSW = $clog2(255 * BXM + 1);                      // 横向和位宽
    localparam integer VSW = $clog2(255 * BXM * BYM + 1);                // 纵向和位宽
    localparam integer CW  = $clog2(((BXM > BYM) ? BXM : BYM));          // cx/cy（0 ~ 方块边长-1）
    localparam integer PW  = (PAD > 0) ? $clog2(PAD + 1) : 1;            // 补 0 计数

    // ---------------- 位置 ----------------
    reg [XW-1:0] rx;           // x*TX2 mod FX2
    reg [YW-1:0] ry;           // y*TY2 mod FY2（行内不变）
    reg [6:0]    bx;           // 当前方块列号
    reg [CW-1:0] cx, cy;       // 当前方块里已经过去的像素数 / 行数（不含当前）
    wire       hend = (rx + TX2 >= FX2);            // 这个像素是方块的最后一列
    wire       vend = (ry + TY2 >= FY2);            // 这一行是方块的最后一行
    wire       vfirst = (cy == 0);                  // 这一行是方块的第一行
    wire       row_end = hend && (bx == TW - 1);    // 这个像素是一行的最后一个

    // ---------------- 横向累加 ----------------
    reg  [HSW-1:0] hs_r, hs_g, hs_b;
    wire [HSW-1:0] h_r = (cx == 0 ? {HSW{1'b0}} : hs_r) + pix[23:16];
    wire [HSW-1:0] h_g = (cx == 0 ? {HSW{1'b0}} : hs_g) + pix[15:8];
    wire [HSW-1:0] h_b = (cx == 0 ? {HSW{1'b0}} : hs_b) + pix[7:0];

    // ---------------- 纵向累加器（块 RAM） ----------------
    (* ram_style = "bram" *) reg [3*VSW-1:0] col_mem [0:127];
    reg  [3*VSW-1:0] col_q;
    reg         ev;                                 // 事件：上一拍一个方块的横向和算好了
    reg  [HSW-1:0] e_r, e_g, e_b;
    reg  [6:0]  e_bx;
    reg         e_first, e_last_row, e_last_col, e_nx, e_ny;
    wire [VSW-1:0] v_r = (e_first ? {VSW{1'b0}} : col_q[3*VSW-1:2*VSW]) + e_r;
    wire [VSW-1:0] v_g = (e_first ? {VSW{1'b0}} : col_q[2*VSW-1:VSW])   + e_g;
    wire [VSW-1:0] v_b = (e_first ? {VSW{1'b0}} : col_q[VSW-1:0])       + e_b;
    always @(posedge clk) begin
        col_q <= col_mem[bx];
        if (ev && !e_last_row)
            col_mem[e_bx] <= {v_r, v_g, v_b};
    end

    // ---------------- 归一化（一个乘法器，三个颜色轮流） ----------------
    reg  [VSW-1:0] n_r, n_g, n_b;
    reg  [15:0] n_k;
    reg  [2:0]  n_ph;                               // 0 = 空闲；1~3 = 乘 R/G/B；4 = 最后一个结果
    reg         n_last;                             // 这个像素是缩略图行的最后一个
    reg  [31:0] prod;
    reg  [7:0]  o_r, o_g;
    wire [VSW-1:0] m_a = (n_ph == 3'd1) ? n_r : (n_ph == 3'd2) ? n_g : n_b;
    wire [31:0] prod_rnd = prod + (32'd1 << (S - 1));
    wire [7:0]  rnd = prod_rnd[S+7:S];              // <= 255（n * R <= 2^S + n）
    reg  [PW-1:0] pad_cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx <= {XW{1'b0}}; ry <= {YW{1'b0}}; bx <= 7'd0; cx <= {CW{1'b0}}; cy <= {CW{1'b0}};
            hs_r <= {HSW{1'b0}}; hs_g <= {HSW{1'b0}}; hs_b <= {HSW{1'b0}};
            ev <= 1'b0; e_r <= {HSW{1'b0}}; e_g <= {HSW{1'b0}}; e_b <= {HSW{1'b0}}; e_bx <= 7'd0;
            e_first <= 1'b0; e_last_row <= 1'b0; e_last_col <= 1'b0; e_nx <= 1'b0; e_ny <= 1'b0;
            n_r <= {VSW{1'b0}}; n_g <= {VSW{1'b0}}; n_b <= {VSW{1'b0}}; n_k <= 16'd0; n_ph <= 3'd0; n_last <= 1'b0;
            prod <= 32'd0; o_r <= 8'd0; o_g <= 8'd0; pad_cnt <= {PW{1'b0}};
            t_we <= 1'b0; t_data <= 32'd0;
        end else if (clr) begin
            rx <= {XW{1'b0}}; ry <= {YW{1'b0}}; bx <= 7'd0; cx <= {CW{1'b0}}; cy <= {CW{1'b0}};
            ev <= 1'b0; n_ph <= 3'd0; pad_cnt <= {PW{1'b0}}; t_we <= 1'b0;
        end else begin
            // ---- 位置 / 横向 ----
            ev <= 1'b0;
            if (pix_en) begin
                hs_r <= h_r; hs_g <= h_g; hs_b <= h_b;
                if (hend) begin
                    ev <= 1'b1;
                    e_r <= h_r; e_g <= h_g; e_b <= h_b;
                    e_bx <= bx;
                    e_first <= vfirst; e_last_row <= vend; e_last_col <= row_end;
                    e_nx <= (cx == FX);             // 方块宽 = cx + 1 = FX + 1
                    e_ny <= (cy == FY);             // 只在方块最后一行用：方块高 = cy + 1 = FY + 1
                    cx <= {CW{1'b0}};
                    bx <= row_end ? 7'd0 : bx + 7'd1;
                    rx <= rx + TX2 - FX2;           // 一行最后一个像素后正好回到 0
                end else begin
                    cx <= cx + 1'b1;
                    rx <= rx + TX2;
                end
                if (row_end) begin
                    if (vend) begin
                        ry <= ry + TY2 - FY2;
                        cy <= {CW{1'b0}};
                    end else begin
                        ry <= ry + TY2;
                        cy <= cy + 1'b1;
                    end
                end
            end

            // ---- 方块最后一行：送去归一化 ----
            if (ev && e_last_row) begin
                n_r <= v_r; n_g <= v_g; n_b <= v_b;
                n_k <= e_nx ? (e_ny ? R11 : R10) : (e_ny ? R01 : R00);
                n_last <= e_last_col;
                n_ph <= 3'd1;
            end else if (n_ph != 3'd0)
                n_ph <= (n_ph == 3'd4) ? 3'd0 : n_ph + 3'd1;
            if (n_ph != 3'd0 && n_ph != 3'd4)
                prod <= m_a * n_k;
            if (n_ph == 3'd2) o_r <= rnd;
            if (n_ph == 3'd3) o_g <= rnd;

            // ---- 输出：像素，行尾补 0 ----
            t_we <= 1'b0;
            if (n_ph == 3'd4) begin
                t_we    <= 1'b1;
                t_data  <= {o_r, o_g, rnd, 8'h00};
                if (n_last) pad_cnt <= PAD;
            end else if (pad_cnt != 0) begin
                t_we    <= 1'b1;
                t_data  <= 32'd0;
                pad_cnt <= pad_cnt - 1'b1;
            end
        end
    end
endmodule
