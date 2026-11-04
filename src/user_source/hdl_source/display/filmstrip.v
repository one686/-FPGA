`timescale 1ns/1ps

// 胶片条（像素时钟域）：决定每一格放哪张图，并逐像素画底板、金框、占位块、箭头
//
// 5 格从左到右 j = 0~4，焦点在中间（j = 2），第 j 格放第 (焦点 + j - 2) mod M 张（M = 图片表张数，含坏图），
//   到头回绕；不到 5 张时按 焦点 > 右 1 > 左 1 > 右 2 > 左 2 的顺序只放 M 格（同一张不会出现两次）。
// 每格的状态：0 没有这一格  1 有缩略图  2 加载中（缩略图还没生成）  3 坏图
//   状态 1 的格才从 SDRAM 读缩略图（slots 给 reader），2、3 画灰色占位块（文字"加载中 / 坏图片"由 OSD 写）。
//
// 格表的算法是组合逻辑一次只算一格，5 拍轮一遍（next_*），帧起点整张拷到 slots / sstate，一帧之内不变；
//   reader 在帧起点后一拍取 slots（和转场的几何参数同一拍），所以读侧和这里用的一定是同一张表。
//   info 变了以后最多 5 拍 next_* 就全部更新（按键、加载完成都比这慢得多）。
//
// 像素（与 raw_x / raw_y 同拍，组合）：
//   take  这个像素从读 FIFO 取一个字：不显示胶片条，或在胶片条上面（大图），或落在状态 1 的格里（缩略图）
//   paint 这个像素换成常数颜色 paint_rgb：胶片条里除了缩略图以外的地方
//   dim   大图压暗（显示胶片条时胶片条上面的行）
//   和读侧一样只看行列：状态 1 的格每行读 112（焦点）/ 84（两边）个字。
// 图片上限 NP 最大 16（序号 4 位）；缩略图按图片序号存放，整帧被换出缓存后仍然可用，slots 里放缩略图格号（4 位）。
module filmstrip #(
    parameter integer H_BLANK = 160,
    parameter integer V_BLANK = 45,
    parameter integer H_ACT   = 640,
    parameter integer V_ACT   = 480,
    parameter integer NP      = 12      // 图片最多几张（= 顶层 PIC_MAX，1~16）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire [11:0] raw_x,
    input  wire [11:0] raw_y,
    input  wire [3*NP+8:0] info,      // 已同步：{缩略图翻转[NP], 有缩略图[NP], 坏图[NP], 张数 M[4:0], 焦点[3:0]}
    input  wire        strip_vis,     // 本帧显示胶片条（转场控制器，帧起点更新）
    output reg  [29:0] slots,         // 5 x {读缩略图, 缩略图格号[3:0], 翻转}，[5:0] = 第 0 格（不读的格 = 0）
    output reg  [9:0]  sstate,        // 5 x 状态（[1:0] = 第 0 格）
    output wire [20:0] frame_words,   // 本帧读 FIFO 要取的字数
    output wire        take,
    output wire        paint,
    output reg  [23:0] paint_rgb,
    output wire        dim
);
    `include "strip_layout.vh"

    // ---------------- 格表：5 拍轮一遍 ----------------
    wire [3:0]  i_focus = info[3:0];
    wire [4:0]  i_cnt   = info[8:4];
    wire [NP-1:0] i_bad  = info[NP+8:9];
    wire [NP-1:0] i_ok   = info[2*NP+8:NP+9];
    wire [NP-1:0] i_flip = info[3*NP+8:2*NP+9];

    reg  [2:0] rj;                                  // 正在算第几格
    reg  [5:0] next_slot [0:4];
    reg  [1:0] next_st   [0:4];
    // 第 rj 格：偏移 d = rj - 2，图 = (焦点 + d) mod M（M >= 1 时只需修正一次）
    wire signed [6:0] s_raw = $signed({3'b000, i_focus}) + $signed({4'b0000, rj}) - 7'sd2;
    wire [6:0] s_wrap = (s_raw < 0) ? s_raw + {2'b00, i_cnt} : (s_raw >= $signed({2'b00, i_cnt})) ? s_raw - {2'b00, i_cnt} : s_raw;
    wire [3:0] s_img  = s_wrap[3:0];
    reg        s_exist;
    always @* begin
        case (i_cnt)
            5'd0:    s_exist = 1'b0;
            5'd1:    s_exist = (rj == 3'd2);
            5'd2:    s_exist = (rj == 3'd2) || (rj == 3'd3);
            5'd3:    s_exist = (rj >= 3'd1) && (rj <= 3'd3);
            5'd4:    s_exist = (rj >= 3'd1);
            default: s_exist = 1'b1;
        endcase
    end
    wire [1:0] s_st = !s_exist ? 2'd0 : i_bad[s_img] ? 2'd3 : i_ok[s_img] ? 2'd1 : 2'd2;

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rj <= 3'd0;
            for (k = 0; k < 5; k = k + 1) begin next_slot[k] <= 6'd0; next_st[k] <= 2'd0; end
            slots <= 30'd0; sstate <= 10'd0;
        end else begin
            next_slot[rj] <= (s_st == 2'd1) ? {1'b1, s_img, i_flip[s_img]} : 6'd0;   // 不读的格清 0
            next_st[rj]   <= s_st;
            rj <= (rj == 3'd4) ? 3'd0 : rj + 3'd1;
            if (frame_start) begin
                slots  <= {next_slot[4], next_slot[3], next_slot[2], next_slot[1], next_slot[0]};
                sstate <= {next_st[4], next_st[3], next_st[2], next_st[1], next_st[0]};
            end
        end
    end

    // 本帧要取的字数：胶片条上面 SP_Y0 行大图 + 状态 1 的格（焦点 112x84，两边 84x63）
    wire       ok0 = slots[5], ok1 = slots[11], ok2 = slots[17], ok3 = slots[23], ok4 = slots[29];
    wire [2:0] n_side = {2'd0, ok0} + {2'd0, ok1} + {2'd0, ok3} + {2'd0, ok4};
    reg  [14:0] side_words;
    always @* begin
        case (n_side)
            3'd0: side_words = 15'd0;
            3'd1: side_words = SP_SW * SP_SH;
            3'd2: side_words = 2 * SP_SW * SP_SH;
            3'd3: side_words = 3 * SP_SW * SP_SH;
            default: side_words = 4 * SP_SW * SP_SH;
        endcase
    end
    assign frame_words = !strip_vis ? H_ACT * V_ACT :
                         SP_Y0 * H_ACT + (ok2 ? SP_TW * SP_TH : 0) + side_words;

    // ---------------- 像素 ----------------
    wire [11:0] ax12 = raw_x - H_BLANK;
    wire [11:0] ay12 = raw_y - V_BLANK;
    wire [9:0]  ax = ax12[9:0], ay = ay12[9:0];
    wire        in_sp  = strip_vis && (ay >= SP_Y0);           // 有效区以外的像素 take / paint 都用不到
    wire        fy     = (ay >= SP_FY0) && (ay < SP_FY0 + SP_TH);
    wire        sy     = (ay >= SP_SY0) && (ay < SP_SY0 + SP_SH);
    wire        fx     = (ax >= SP_FX0) && (ax < SP_FX0 + SP_TW);
    wire        sx0    = (ax >= SP_SX0) && (ax < SP_SX0 + SP_SW);
    wire        sx1    = (ax >= SP_SX1) && (ax < SP_SX1 + SP_SW);
    wire        sx3    = (ax >= SP_SX3) && (ax < SP_SX3 + SP_SW);
    wire        sx4    = (ax >= SP_SX4) && (ax < SP_SX4 + SP_SW);
    // 落在哪一格、那一格的状态
    wire        in_f   = fy && fx;
    wire        in_s   = sy && (sx0 || sx1 || sx3 || sx4);
    wire [1:0]  st_s   = sx0 ? sstate[1:0] : sx1 ? sstate[3:2] : sx3 ? sstate[7:6] : sstate[9:8];
    wire [1:0]  cell_st = in_f ? sstate[5:4] : in_s ? st_s : 2'd0;
    wire        thumb  = in_sp && (cell_st == 2'd1);
    assign take  = !in_sp || thumb;
    assign paint = in_sp && !thumb;
    assign dim   = strip_vis && (ay < SP_Y0);
    // 金框：焦点格外面一圈（焦点格总是有）
    wire        ring   = (ay >= SP_FY0 - SP_FRAME) && (ay < SP_FY0 + SP_TH + SP_FRAME) &&
                         (ax >= SP_FX0 - SP_FRAME) && (ax < SP_FX0 + SP_TW + SP_FRAME) && !in_f;
    // 箭头：实心三角，尖朝外；|y - 中心| <= 离尖的距离（图片表 >= 2 张才画 = 右 1 格存在）
    // |ay - AYC| 只用到 0~15（再大也钳到 15），先算 u = ay - (AYC - 15)，窗口 [0,31) 内用 5 位求绝对值
    wire [9:0]  dyu    = ay - (SP_AYC - 15);
    wire [4:0]  dya    = (dyu[4:0] >= 5'd15) ? dyu[4:0] - 5'd15 : 5'd15 - dyu[4:0];
    wire [3:0]  dy4    = (dyu < 10'd31) ? dya[3:0] : 4'd15;
    wire [9:0]  dl     = ax - SP_ALX;
    wire [9:0]  dr     = SP_ARX - ax;
    wire        arrow  = (sstate[7:6] != 2'd0) &&
                         (((ax >= SP_ALX) && (ax < SP_ALX + SP_AW) && (dy4 <= dl[3:0])) ||
                          ((ax <= SP_ARX) && (ax > SP_ARX - SP_AW) && (dy4 <= dr[3:0])));
    always @* begin
        if (cell_st != 2'd0)               paint_rgb = SP_C_PH;      // 占位块（状态 2、3；状态 1 不画）
        else if (ring)                     paint_rgb = SP_C_GOLD;
        else if (arrow)                    paint_rgb = SP_C_ARROW;
        else if (ay < SP_Y0 + SP_EDGE_H)   paint_rgb = SP_C_EDGE;
        else                               paint_rgb = SP_C_PANEL;
    end
endmodule
