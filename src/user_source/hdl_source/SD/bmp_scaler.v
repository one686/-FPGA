`timescale 1ns/1ps
// ===========================================================================
// bmp_scaler.v —— 任意尺寸 BMP -> 固定 OUT_W x OUT_H（保持比例 + 黑边）
//   缩小 = 精确区域平均，放大 = 双三次（Catmull-Rom）。时钟：SD/控制时钟域（50MHz）。
//
// 位置：sd_card_bmp 里 fat32_file_streamer 的字节流后面，负责 BMP 头解析、BGR -> RGB 和缩放。
// 输出按 SDRAM 写入顺序（= 文件里的行顺序）给出恰好 OUT_W x OUT_H 个像素，交给写侧 frame_write_path；
// 是否上下翻转由 flip 告诉写侧（正存 = 从下往上 = 1，倒存（高度为负）= 0）。
//
// 支持：24 位 / 32 位（BGRA，A 不用），压缩方式 0；32 位另接受 BITFIELDS(3) 的标准掩码
//       宽 1~MAX_W、高 1~MAX_H，行尾按 4 字节对齐填充
// 不支持（按坏图处理，err_hdr）：签名不是 BM、调色板(1/4/8 位)、16 位、RLE 压缩、尺寸超限等
//
// 保持比例：宽图（W*OUT_H >= H*OUT_W）输出宽 = OUT_W，输出高 oh = round(H*OUT_W/W)；
//           否则输出高 = OUT_H，输出宽 ow = round(W*OUT_H/H)。黑边：左 (OUT_W-ow)/2，先输出的行 (OUT_H-oh)/2。
//   保持比例时横、纵两个方向要么都缩小，要么都放大（或一样大），所以每张图只走下面一种模式。
//
// ---- 缩小模式（W > ow 或 H > oh）：精确区域平均 ----
//   横向：源像素 x 占 [x*ow, (x+1)*ow)，输出列 i 占 [i*W, (i+1)*W)，权重 = 两段的重叠长度（整数，<= ow），
//         一个源像素最多跨 2 列。小宽度时权重左移 ls = max(0, 9-bw) 位；
//         一列累加完 A = sum(p * 权重) 后右移 rs = max(0, bw-9) 位得 A'（< 2^17），bw = W 的二进制位数。
//   纵向：源行 y 占 [y*oh, (y+1)*oh)，输出行 j 占 [j*H, (j+1)*H)，重叠长度为权重（<= oh），
//         每列一个累加器 V += A' * 权重（V < 2^28，三个颜色共 84 位，放在 4 条 1024x24 的行存储里）。
//         一源行最多跨 2 个输出行：输出行在这一源行结束时，算出这一列的像素，累加器换成下一行的那一份。
//   归一：out = min(255, ((V >> bh) * R + 2^22) >> 23)，R = round(2^(14+bw+bh) / (W*H))（<= 65536），bh = H 的位数。
//         纯色区域（含纯黑 0、纯白 255）结果与原色完全相同；与浮点区域平均最多差 1。
// ---- 放大 / 一样大（其余情况）：双三次 Catmull-Rom（a = -0.5），像素中心对齐 ----
//   坐标（16.16 定点）：step = round(W*65536/ow)，第 i 列 pos = step/2 - 0.5 + i*step；x0 = pos>>16，t = pos[15:8]；
//   4 个抽头 x0-1 .. x0+2，超出边界取边上的像素。纵向同理（step_y、y0、ty）。
//   权重表（256 项，T = 0~255）：a = (-T^3 + 512T^2 - 65536T + 65536) >> 17，c = (-3T^3 + 1024T^2 + 65536T + 65536) >> 17，
//     d = (T^3 - 256T^2 + 65536) >> 17，b = 256 - a - c - d（权重和恰好 256）。
//     这张表和图片无关，做成常数 ROM（bmp_cubic.mif，design/bmp_scale.py --rom 生成），
//     每项直接存 {a, b, c, d}（30 位），不占用乘法器现算。
//   先纵向：v = sum(wy * p)（4 条源行，行存储环形存放最近 4 行），再横向：out = clamp((32768 + sum(wx * v)) >> 16)。
//   一样大时 t = 0，权重 (0,256,0,0)：与原图逐位相同（640x480 的图原样输出）。
//
// 乘法器：一个 18x18（1 个 DSP），三个颜色、各个抽头轮流用：
//   MAC 任务 mac_md：acc[c] = 初值(md, c) + sum_t A(md, t, c) * B(md, t, c)，A/B 直接从各自的寄存器选，不另存操作数；
//   三个颜色的累加器轮转（一个加法器）。放大时每个输出像素约 30 拍（640x480 约 0.2 秒）；
//   缩小时每个源像素约 15~35 拍，比读卡（约 100 拍/像素）快。
// 每张图的参数：宽高乘积用同一个乘法器，输出区、R、step 用移位相减的除法器。
// 缩小时纵向累加器不用先清零：第一个源行从 0 开始累加（frow），之后每列的累加器都已写过。
//
// 速度匹配：
//   - 前面有一个 2^FIFO_AW 字节的缓冲。缓冲里超过 HOLD_LEVEL 字节时 hold=1，读文件模块暂不请求下一个扇区
//     （已请求的扇区最多再来 512 字节，HOLD_LEVEL + 512 < 缓冲深度，不会溢出）。
//   - 输出看写 FIFO 已用字数 out_used，少于 OUT_LIMIT 才发下一个像素。所以本模块随时可以停下来。
//
// 结束：
//   done      = OUT_W x OUT_H 个像素全部发完（文件后面多余的字节照样读掉、丢弃）
//   err_hdr   = 头不合格；读完整个文件后才报（保证读文件模块已经结束）
//   err_short = 文件读完了，像素还不够（截断）
//   active=0（加载结束/中止）时只读掉缓冲里的字节，不再输出。
//
// TD 注意：存储器深度都是 2 的幂，下标正好 log2(深度) 位，否则 TD 不会推成 BRAM。
// Python 逐位一致模型：design/bmp_scale.py 的 scale()。
// ===========================================================================
module bmp_scaler #(
    parameter integer OUT_W      = 640,                    // <= 1024
    parameter integer OUT_H      = 480,
    parameter integer MAX_W      = 1920,                    // rs = max(0, bw-9) <= 2，要求 MAX_W < 2048
    parameter integer MAX_H      = 1920,
    parameter integer FIFO_AW    = 10,                     // 字节缓冲 1024
    parameter integer HOLD_LEVEL = (1 << FIFO_AW) - 576,   // 448：再来一个扇区 512 字节也放得下
    parameter integer OUT_LIMIT  = 384                     // 写 FIFO 512 深
)(
    input             clk,
    input             rst,
    input             start,        // 开始加载（单拍，同时清空缓冲）
    input             active,       // 加载进行中（= load_busy）
    input             in_valid,
    input      [7:0]  in_byte,
    input             in_eof,       // 读文件结束（单拍）
    output            hold,
    input      [8:0]  out_used,
    output reg        out_valid,
    output reg [23:0] out_rgb,
    output reg        done,
    output reg        err_hdr,
    output reg        err_short,
    output reg        flip
);
    // ---------------- 字节缓冲（先进先出，读出端带一拍寄存） ----------------
    localparam integer FD = 1 << FIFO_AW;
    reg  [7:0]       fmem [0:FD-1];
    reg  [FIFO_AW:0] wp, wp_d, rp;
    reg  [7:0]       fq;
    reg              eof_q;
    wire             f_av   = (wp_d != rp);                  // 写进去至少一拍的字节才读（避开同址读写）
    wire             f_dry  = !f_av && (wp_d == wp) && eof_q; // 文件读完且缓冲空
    wire [FIFO_AW:0] f_cnt  = wp - rp;
    reg              pop;
    wire [FIFO_AW:0] rp_n   = rp + {{FIFO_AW{1'b0}}, pop};
    assign hold = (f_cnt >= HOLD_LEVEL);

    always @(posedge clk) begin
        if (in_valid) fmem[wp[FIFO_AW-1:0]] <= in_byte;
        fq <= fmem[rp_n[FIFO_AW-1:0]];
    end

    // ---------------- 4 条行存储（1024 x 24） ----------------
    //   放大：环形存最近 4 个源行（源行 r 放在第 r mod 4 条），4 条同一地址一起读
    //   缩小：4 条拼成 96 位，每列一个纵向累加器 {B[83:56], G[55:28], R[27:0]}
    reg  [3:0]  l_we;
    reg  [9:0]  l_wa, l_ra;
    wire [95:0] l_wd;               // 写数据（见下面"行存储写数据"）
    reg  [23:0] lq0, lq1, lq2, lq3;
    (* ram_style = "bram" *) reg [23:0] lm0 [0:1023];
    (* ram_style = "bram" *) reg [23:0] lm1 [0:1023];
    (* ram_style = "bram" *) reg [23:0] lm2 [0:1023];
    (* ram_style = "bram" *) reg [23:0] lm3 [0:1023];
    always @(posedge clk) begin
        if (l_we[0]) lm0[l_wa] <= l_wd[23:0];
        if (l_we[1]) lm1[l_wa] <= l_wd[47:24];
        if (l_we[2]) lm2[l_wa] <= l_wd[71:48];
        if (l_we[3]) lm3[l_wa] <= l_wd[95:72];
        lq0 <= lm0[l_ra];
        lq1 <= lm1[l_ra];
        lq2 <= lm2[l_ra];
        lq3 <= lm3[l_ra];
    end
    wire [95:0] lq = {lq3, lq2, lq1, lq0};

    // ---------------- 双三次权重表：常数 ROM 256 x {a[5:0], b[8:0], c[8:0], d[5:0]}，读延迟 1 拍 ----------------
    //   a、d 有符号（-19~0），b、c 无符号（0~256）；TD 不加载推导出来的 RAM 的初值（eram_init off），所以直接例化 BRAM + mif
    reg  [7:0]  w_ra;
    wire [29:0] wq;
`ifdef OSD_SIM
    reg  [29:0] wrom [0:255];
    reg  [29:0] wq_r;
    initial $readmemh("../src/user_source/hdl_source/SD/bmp_cubic.hex", wrom);
    always @(posedge clk) wq_r <= wrom[w_ra];
    assign wq = wq_r;
`else
    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(30), .ADDR_WIDTH_A(8), .DATA_DEPTH_A(256),
        .DATA_WIDTH_B(30), .ADDR_WIDTH_B(8), .DATA_DEPTH_B(256),
        .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("9K"),
        .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("bmp_cubic.mif"), .FILL_ALL("NONE")
    ) u_cubic (
        .dia(30'd0), .dib(30'd0), .addra(w_ra), .addrb(8'd0),
        .cea(1'b1), .ceb(1'b0), .ocea(1'b0), .oceb(1'b0),
        .clka(clk), .clkb(1'b0), .wea(1'b0), .web(1'b0),
        .bea(1'b0), .beb(1'b0), .rsta(1'b0), .rstb(1'b0),
        .doa(wq), .dob()
    );
`endif
    wire signed [9:0] wt_a = {{4{wq[29]}}, wq[29:24]};
    wire signed [9:0] wt_b = {1'b0, wq[23:15]};
    wire signed [9:0] wt_c = {1'b0, wq[14:6]};
    wire signed [9:0] wt_d = {{4{wq[5]}}, wq[5:0]};

    // ---------------- 状态 ----------------
    localparam [5:0] S_IDLE = 6'd0,  S_HDR = 6'd1,  S_CHK = 6'd2,  S_MUL = 6'd3,  S_DIV = 6'd4,
                     S_GEO  = 6'd5,  S_PRM = 6'd6,  S_PRM2 = 6'd7, S_PRM3 = 6'd8, S_INI = 6'd9,
                     S_MTOP = 6'd10, S_MBOT = 6'd11, S_BLK = 6'd12, S_PX1 = 6'd13, S_PIX = 6'd14,
                     S_PAD  = 6'd15, S_BAD = 6'd16, S_SHORT = 6'd17, S_MUL2 = 6'd18,
                     // 缩小
                     D_ROW = 6'd20, D_PIXS = 6'd21, D_H1 = 6'd22, S_PSH = 6'd23, D_H3 = 6'd24,
                     D_V0  = 6'd25, D_V1 = 6'd26,   D_V2 = 6'd27, D_V3 = 6'd28, D_SH = 6'd29,
                     D_V4  = 6'd30, D_V5 = 6'd31,   D_V6 = 6'd32, D_CN = 6'd33, D_NX = 6'd34,
                     D_RE  = 6'd35, D_RE2 = 6'd36,
                     // 放大
                     U_ROW = 6'd40, U_RX = 6'd41,   U_RX2 = 6'd42, U_RG0 = 6'd43, U_RG1 = 6'd44,
                     U_RG2 = 6'd45, U_PX0 = 6'd46,  U_VC0 = 6'd47, U_VC1 = 6'd48, U_VC2 = 6'd49,
                     U_VC3 = 6'd50, U_PXW = 6'd51,  U_PW1 = 6'd52, U_PW2 = 6'd53, U_PW3 = 6'd54,
                     U_PN  = 6'd55, U_RN = 6'd56;
    reg  [5:0]  st, ret_st, pix_ret, pad_ret, div_ret;

    // MAC 任务
    localparam [3:0] M_GEO = 4'd0,  // W*OUT_H, H*OUT_W, W*H
                     // 4'd1 保留未用（权重表是常数 ROM）
                     M_DH1 = 4'd2,  // 缩小 横向：acc + p * ov1（横向累加留在 acc 里，x == 0 从 0 开始）
                     M_DH2 = 4'd3,  //             p * ov2
                     M_DV1 = 4'd4,  // 缩小 纵向：V + A' * ovy1
                     M_DV2 = 4'd5,  //             A' * ovy2
                     M_DN  = 4'd6,  // 缩小 归一：2^22 + (V>>bh) * R
                     M_UV  = 4'd7,  // 放大 纵向：sum wy * p
                     M_UH  = 4'd8;  // 放大 横向：32768 + sum wx * v

    // 头字段
    reg  [15:0] pos;                // 当前字节在文件里的偏移（头部）
    reg  [15:0] off;                // 像素数据偏移
    reg  [15:0] w16, h16;
    reg         hok;                // 逐字节检查的结果
    reg         h_hi0, h_hi1;       // 高度高 16 位全 0 / 全 1（负数 = 倒存）
    reg         is32, comp3, mask_ok;
    wire [15:0] habs = h_hi1 ? (~h16 + 16'd1) : h16;

    // 每张图的参数
    reg  [10:0] sw, sh;             // 源宽高
    reg  [10:0] ow, oh;             // 输出区宽高
    reg  [10:0] ml;                 // 左黑边
    reg  [9:0]  mrow, mbot;         // 黑边行计数 / 后面的黑边行数
    reg         wide, down;
    reg  [3:0]  bw, bh;             // W、H 的二进制位数
    reg  [3:0]  ls;                 // 缩小：横向权重左移（0~8）
    reg  [1:0]  rs;                 // 缩小：列累加右移
    reg  [16:0] rr_r;               // 缩小：归一化倒数 R（<= 65536）
    reg  [16:0] sx, sy;             // 放大：16.16 步长（<= 65536）
    reg  [21:0] mpc;                // W*H
    // 除法（移位相减，每拍 1 位）：dn / dd，40 位被除数
    reg  [39:0] dn;
    reg  [23:0] drm;
    reg  [22:0] dd;
    reg  [5:0]  dcnt;
    wire [23:0] dt   = {drm[22:0], dn[39]};
    wire [24:0] dsub = {1'b0, dt} - {2'b0, dd};          // 比较和减法共用一个减法器（借位 = 不够减）
    wire        dge  = !dsub[24];

    // 像素
    reg  [1:0]  bc;                 // 像素内第几个字节 B G R (A)
    reg  [7:0]  pr, pg, pb;
    reg  [1:0]  padc;
    reg  [10:0] x, y, i;
    reg  [9:0]  j;
    reg  [10:0] blk_n;              // 还要发几个黑像素
    // 要发的像素直接放在 out_rgb 里（S_PX1 只等写 FIFO 有空再拉 out_valid），不另存一份
    reg         frow;               // 缩小：第一个源行（纵向累加从 0 开始；之后的行存储内容都是写过的）
    // 缩小
    reg  [10:0] cap, rcap, rcap_n;  // 当前列 / 当前输出行还差多少单位
    reg  [10:0] ov2, ovy1, ovy2;
    reg         cdone, rdone;
    // synthesis translate_off
    initial if (MAX_W >= 2048) begin $display("bmp_scaler: A7 requires MAX_W < 2048 (rs <= 2)"); $finish; end
    // synthesis translate_on
    reg  [19:0] ap0, ap1, ap2;        // 完成的一列 A'（先存 20 位的 A，在 D_V0 / D_V1 两个等待拍里各右移 1 位）
    reg  [27:0] vs0, vs1, vs2;        // 纵向累加结果，归一前右移 bh 位
    reg  [5:0]  shc;                // 逐位移位的计数（V >> bh、2^(14+bw+bh)）
    // 放大
    reg  [10:0] rr;                 // 已收到的源行数
    reg  signed [27:0] posx, posy;
    wire signed [11:0] x0 = posx[27:16];   // 当前源列 = posx 的整数部分
    reg  [1:0]  vk;                 // 窗口第几个 v / 第几个纵向抽头
    reg  [1:0]  sl0, sl1, sl2, sl3; // 4 个纵向抽头所在的行存储
    reg  signed [9:0] wy0, wy1, wy2, wy3;
    // 横向窗口：4 列，每列 {B, G, R} 各 18 位有符号的 v，放在一块 4 x 54 的存储器里（比 4 个寄存器 +
    //   移位 + 54 位 4 选 1 省资源）。新的一列写到最老那一列的位置，wbase 指向最老的一列；窗口第 t 列在 (wbase + t) mod 4。
    reg  [1:0]  wbase;
    reg         wn_we;
    reg  [53:0] wn_wd;
    reg  [53:0] wn_q;
    (* ram_style = "bram" *) reg [53:0] wnmem [0:3];

    // ---------------- MAC：一个 18x18 乘法器 ----------------
    reg         mac_go;
    reg  [3:0]  mac_md;
    reg  [2:0]  mac_n;              // 抽头数 1~4
    reg         m_issue, s1_v, s2_v;
    reg  [1:0]  m_t, m_c;
    reg  signed [17:0] ma, mb;
    reg  signed [35:0] prod;
    reg  signed [35:0] acc0, acc1, acc2;   // 轮转：任何时候 acc0 是下一个乘积要加到的颜色
    wire        mac_busy = mac_go | m_issue | s1_v | s2_v;

    // 行存储写数据：写使能晚一拍，那一拍 acc / 像素寄存器都还没变
    //   缩小：本列的纵向累加器 {B, G, R}（各 28 位）；放大：当前源像素（4 条行存储同一个值，由 l_we 选哪条）
    assign l_wd = down ? {12'd0, acc2[27:0], acc1[27:0], acc0[27:0]} : {4{pr, pg, pb}};

    // 放大：纵向抽头 t 所在行存储里颜色 c 的字节
    reg  [1:0]  u_sl;
    reg  [23:0] u_lane;
    always @* begin
        case (m_t)
            2'd0: u_sl = sl0;
            2'd1: u_sl = sl1;
            2'd2: u_sl = sl2;
            default: u_sl = sl3;
        endcase
        case (u_sl)
            2'd0: u_lane = lq0;
            2'd1: u_lane = lq1;
            2'd2: u_lane = lq2;
            default: u_lane = lq3;
        endcase
    end
    function [17:0] pick3;          // 按颜色取（c = 0 R / 1 G / 2 B）
        input [1:0]  c;
        input [17:0] v0, v1, v2;
        begin
            case (c)
                2'd0:    pick3 = v0;
                2'd1:    pick3 = v1;
                default: pick3 = v2;
            endcase
        end
    endfunction
    function [17:0] pick4;          // 按抽头取
        input [1:0]  t;
        input [17:0] v0, v1, v2, v3;
        begin
            case (t)
                2'd0:    pick4 = v0;
                2'd1:    pick4 = v1;
                2'd2:    pick4 = v2;
                default: pick4 = v3;
            endcase
        end
    endfunction
    function [17:0] sx10;           // 10 位有符号 -> 18 位
        input [9:0] v;
        sx10 = {{8{v[9]}}, v};
    endfunction

    reg  [17:0] bop;                // 缩小模式的 B 操作数（横向权重 << ls / 纵向权重 / R），发起 MAC 时一起装入
    always @(posedge clk) begin
        if (wn_we) wnmem[wbase] <= wn_wd;
        wn_q <= wnmem[wbase + m_t];         // 发起这一拍给地址，下一拍（乘法器那一拍）用
    end
    reg  [17:0] a_sel, b_sel;
    always @* begin
        a_sel = 18'd0;
        b_sel = 18'd0;
        case (mac_md)
            M_GEO: begin
                a_sel = pick3(m_c, {7'd0, sw}, {7'd0, sh}, {7'd0, sw});
                b_sel = pick3(m_c, OUT_H, OUT_W, {7'd0, sh});
            end
            M_DH1, M_DH2: begin a_sel = pick3(m_c, {10'd0, pr}, {10'd0, pg}, {10'd0, pb}); b_sel = bop; end
            M_DV1, M_DV2: begin a_sel = pick3(m_c, {1'b0, ap0[16:0]}, {1'b0, ap1[16:0]}, {1'b0, ap2[16:0]}); b_sel = bop; end
            M_DN:  begin a_sel = pick3(m_c, {1'b0, vs0[16:0]}, {1'b0, vs1[16:0]}, {1'b0, vs2[16:0]}); b_sel = bop; end
            M_UV:  begin
                a_sel = pick3(m_c, {10'd0, u_lane[23:16]}, {10'd0, u_lane[15:8]}, {10'd0, u_lane[7:0]});
                b_sel = sx10(pick4(m_t, wy0, wy1, wy2, wy3));
            end
            M_UH:  begin                         // A 操作数从窗口存储器读，在乘法那一拍选（见下）
                b_sel = sx10(pick4(m_t, wt_a, wt_b, wt_c, wt_d));
            end
            default: ;
        endcase
    end
    reg  signed [35:0] i0, i1, i2;  // 初值
    always @* begin
        case (mac_md)
            M_DV1:   begin                       // 第一个源行从 0 开始累加（行存储不用先清零）
                         i0 = frow ? 36'sd0 : {8'd0, lq[27:0]};
                         i1 = frow ? 36'sd0 : {8'd0, lq[55:28]};
                         i2 = frow ? 36'sd0 : {8'd0, lq[83:56]};
                     end
            M_DN:    begin i0 = 36'sd4194304; i1 = 36'sd4194304; i2 = 36'sd4194304; end
            M_UH:    begin i0 = 36'sd32768;   i1 = 36'sd32768;   i2 = 36'sd32768;   end
            default: begin i0 = 36'sd0;       i1 = 36'sd0;       i2 = 36'sd0;       end
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            m_issue <= 1'b0; s1_v <= 1'b0; s2_v <= 1'b0; m_t <= 2'd0; m_c <= 2'd0;
        end else begin
            s1_v <= 1'b0;
            if (mac_go) begin
                m_issue <= 1'b1; m_t <= 2'd0; m_c <= 2'd0;
            end else if (m_issue) begin
                s1_v <= 1'b1;
                if (m_c == 2'd2) begin
                    m_c <= 2'd0;
                    if ({1'b0, m_t} == mac_n - 3'd1) m_issue <= 1'b0;
                    else                             m_t <= m_t + 2'd1;
                end else begin
                    m_c <= m_c + 2'd1;
                end
            end
            s2_v <= s1_v;
        end
    end
    reg  [1:0]  m_c_d;                  // 与 ma 同一拍的颜色号
    wire signed [17:0] ma_x = (mac_md == M_UH) ? pick3(m_c_d, wn_q[17:0], wn_q[35:18], wn_q[53:36]) : ma;
    always @(posedge clk) begin
        ma    <= a_sel;
        mb    <= b_sel;
        m_c_d <= m_c;
        prod  <= ma_x * mb;
        if (mac_go && !(mac_md == M_DH1 && x != 11'd0)) begin   // 缩小横向接着上一个像素累加
            acc0 <= i0; acc1 <= i1; acc2 <= i2;
        end else if (s2_v) begin
            acc0 <= acc1; acc1 <= acc2; acc2 <= acc0 + prod;   // 乘积按 R G B R G B ... 的顺序到
        end
    end

    // ---------------- 组合逻辑 ----------------
    wire        last_b  = is32 ? (bc == 2'd3) : (bc == 2'd2);
    wire        e_go    = (out_used < OUT_LIMIT);
    wire        geo_wide = (acc0[21:0] >= acc1[21:0]);           // S_MUL2：W*OUT_H >= H*OUT_W
    wire [21:0] geo_num  = geo_wide ? acc1[21:0] : acc0[21:0];
    wire [10:0] geo_den  = geo_wide ? sw : sh;
    wire [10:0] x_inc   = x + 11'd1;                       // 源列计数加 1（终点比较 x+1 == W 也用它）
    wire [10:0] mright  = ml + {10'd0, ow[0] ^ OUT_W[0]};   // = (OUT_W - ow) - ml（用一个加 1 器代替两个减法）

    // 头读完 = 到了像素数据偏移（偏移不合理时读满 54 字节就停，交给 S_CHK 判坏）
    wire        hdr_end = (pos >= 16'd54) && ((pos == off) || (off < 16'd54));

    // 二进制位数（1~11）
    function [3:0] blen;
        input [10:0] v;
        integer k;
        begin
            blen = 4'd0;
            for (k = 0; k < 11; k = k + 1)
                if (v[k]) blen = k + 1;
        end
    endfunction

    function [7:0] sat_up;                   // 放大：(acc >>> 16) 限幅 0~255
        input signed [35:0] a;
        begin
            if (a[35])                      sat_up = 8'd0;
            else if (a[34:16] > 19'd255)    sat_up = 8'd255;
            else                            sat_up = a[23:16];
        end
    endfunction

    function [7:0] sat_dn;                   // 缩小：(acc >> 23) 限幅 255
        input signed [35:0] a;
        begin
            if (a[35:23] > 13'd255)          sat_dn = 8'd255;
            else                             sat_dn = a[30:23];
        end
    endfunction

    // 缩小：横向本像素的权重分配（cap = 当前列还差多少单位，源像素占 ow 单位）
    //   只用一个减法 h_d = cap - ow（12 位有符号），几个比较都从它得出
    wire signed [11:0] h_d = $signed({1'b0, cap}) - $signed({1'b0, ow});
    wire        h_fit   = !h_d[11];                       // ow <= cap
    wire [10:0] h_ov1   = h_fit ? ow : cap;
    wire        h_cdone = h_d[11] || (h_d == 12'sd0);     // ow >= cap
    wire [10:0] h_ov2   = -h_d[10:0];                     // ow - cap，仅 ow > cap 时有效
    wire [10:0] h_capn  = h_d[10:0] + (h_cdone ? sw : 11'd0);   // ow < cap: cap-ow；ow == cap: sw；否则 sw-(ow-cap)
    wire [10:0] ovh     = (st == D_H1) ? h_ov1 : ov2;     // 本次横向 MAC 的权重
    // 横向权重 << ls（唯一的移位器）。缩小时 ovh <= ow <= W：ls > 0（W < 256）时 ovh < 2^bw，移完 < 512；
    //   ls = 0 时 ovh <= OUT_W <= 1024。所以结果只要 11 位
    wire [10:0] ovh_sh  = ovh << ls;
    wire [17:0] ovh_s   = {7'd0, ovh_sh};

    // 缩小 纵向：v_d = rcap - oh（同横向，一个减法）
    wire signed [11:0] v_d = $signed({1'b0, rcap}) - $signed({1'b0, oh});
    wire        v_rdone = v_d[11] || (v_d == 12'sd0);     // oh >= rcap

    // 放大：纵向
    wire signed [11:0] y0    = posy[27:16];
    wire        u_need = ($signed({2'b0, rr}) <= y0 + 13'sd2) && (rr < sh);   // 第 rr 行还要收（rr <= min(y0+2, H-1)）
    wire [27:0] posx_n  = posx + {11'd0, sx};
    wire signed [27:0] posx0 = $signed({12'd0, sx[16:1]}) - 28'sd32768;   // 第 0 列：step/2 - 0.5
    // 共用的限幅：放大 横向窗口取 clamp(x0-1+vk, W-1)，纵向抽头取 clamp(y0-1+vk, H-1)
    wire        cl_y   = (st == U_RG0);
    wire signed [12:0] cl_v  = (cl_y ? y0 : x0) - 13'sd1 + $signed({11'd0, vk});
    wire [10:0] cl_mx  = cl_y ? (sh - 11'd1) : (sw - 11'd1);
    wire [10:0] cl_o   = (cl_v < 0) ? 11'd0 : ((cl_v > $signed({2'b0, cl_mx})) ? cl_mx : cl_v[10:0]);

    // 字节缓冲读：哪些状态吃字节
    always @* begin
        case (st)
            S_IDLE, S_BAD, S_PIX:        pop = f_av;
            S_PAD:                       pop = f_av && (padc != 2'd0);   // 不用补的行（padc=0）一个字节也不能吃
            S_HDR:                       pop = f_av && !hdr_end;
            default:                     pop = 1'b0;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wp <= 0; wp_d <= 0; rp <= 0; eof_q <= 1'b0;
            st <= S_IDLE; ret_st <= S_IDLE; pix_ret <= S_IDLE; pad_ret <= S_IDLE; div_ret <= S_IDLE;
            pos <= 16'd0; off <= 16'd0; w16 <= 16'd0; h16 <= 16'd0;
            hok <= 1'b0; h_hi0 <= 1'b0; h_hi1 <= 1'b0; is32 <= 1'b0; comp3 <= 1'b0; mask_ok <= 1'b0;
            sw <= 11'd0; sh <= 11'd0; ow <= 11'd0; oh <= 11'd0; ml <= 11'd0; mrow <= 10'd0; mbot <= 10'd0;
            wide <= 1'b0; down <= 1'b0; bw <= 4'd0; bh <= 4'd0; ls <= 4'd0; rs <= 2'd0;
            rr_r <= 17'd0; sx <= 17'd0; sy <= 17'd0; mpc <= 22'd0;
            dn <= 40'd0; drm <= 24'd0; dd <= 23'd1; dcnt <= 6'd0;
            bc <= 2'd0; pr <= 8'd0; pg <= 8'd0; pb <= 8'd0; padc <= 2'd0;
            x <= 11'd0; y <= 11'd0; i <= 11'd0; j <= 10'd0; blk_n <= 11'd0;
            cap <= 11'd0; rcap <= 11'd0; rcap_n <= 11'd0; bop <= 18'd0; ov2 <= 11'd0; ovy1 <= 11'd0; ovy2 <= 11'd0;
            cdone <= 1'b0; rdone <= 1'b0; shc <= 6'd0;
            ap0 <= 20'd0; ap1 <= 20'd0; ap2 <= 20'd0;
            vs0 <= 28'd0; vs1 <= 28'd0; vs2 <= 28'd0;
            rr <= 11'd0; posx <= 28'sd0; posy <= 28'sd0; vk <= 2'd0;
            sl0 <= 2'd0; sl1 <= 2'd0; sl2 <= 2'd0; sl3 <= 2'd0;
            wy0 <= 10'sd0; wy1 <= 10'sd0; wy2 <= 10'sd0; wy3 <= 10'sd0;
            wbase <= 2'd0; wn_we <= 1'b0; wn_wd <= 54'd0;
            mac_go <= 1'b0; mac_md <= M_GEO; mac_n <= 3'd1;
            l_we <= 4'd0; l_wa <= 10'd0; l_ra <= 10'd0; frow <= 1'b1;
            w_ra <= 8'd0;
            out_valid <= 1'b0; out_rgb <= 24'd0;
            done <= 1'b0; err_hdr <= 1'b0; err_short <= 1'b0; flip <= 1'b1;
        end else begin
            done      <= 1'b0;
            err_hdr   <= 1'b0;
            err_short <= 1'b0;
            out_valid <= 1'b0;
            mac_go    <= 1'b0;
            l_we      <= 4'd0;
            wn_we     <= 1'b0;
            if (wn_we) wbase <= wbase + 2'd1;   // 写完新的一列，最老的一列往后挪

            // 缓冲指针
            if (in_valid) wp <= wp + 1'b1;
            wp_d <= wp;
            rp   <= rp_n;
            if (in_eof) eof_q <= 1'b1;

            if (start) begin
                wp <= 0; wp_d <= 0; rp <= 0; eof_q <= 1'b0;
                st <= S_HDR;
                pos <= 16'd0; off <= 16'd0; w16 <= 16'd0; h16 <= 16'd0;
                hok <= 1'b1; h_hi0 <= 1'b1; h_hi1 <= 1'b1; is32 <= 1'b0; comp3 <= 1'b0; mask_ok <= 1'b1;
            end else if (!active && st != S_IDLE) begin
                st <= S_IDLE;                         // 加载结束/中止：只读掉剩下的字节
            end else begin
                case (st)
                // ================= 文件头 =================
                S_HDR: begin
                    if (hdr_end) begin
                        st <= S_CHK;                  // 到像素数据了（这一字节先不吃）
                    end else if (f_av) begin
                        pos <= pos + 16'd1;
                        case (pos)
                            16'd0:  hok <= hok & (fq == 8'h42);           // 'B'
                            16'd1:  hok <= hok & (fq == 8'h4D);           // 'M'
                            16'd10: off[7:0]  <= fq;
                            16'd11: off[15:8] <= fq;
                            16'd12, 16'd13, 16'd20, 16'd21, 16'd27, 16'd29, 16'd31, 16'd32, 16'd33:
                                    hok <= hok & (fq == 8'h00);           // 偏移/宽的高位、平面数/位深/压缩的高字节
                            16'd18: w16[7:0]  <= fq;
                            16'd19: w16[15:8] <= fq;
                            16'd22: h16[7:0]  <= fq;
                            16'd23: h16[15:8] <= fq;
                            16'd24, 16'd25: begin
                                    h_hi0 <= h_hi0 & (fq == 8'h00);
                                    h_hi1 <= h_hi1 & (fq == 8'hFF);
                            end
                            16'd26: hok <= hok & (fq == 8'd1);            // 平面数 1
                            16'd28: begin                                  // 位深 24 / 32
                                    hok  <= hok & ((fq == 8'd24) || (fq == 8'd32));
                                    is32 <= (fq == 8'd32);
                            end
                            16'd30: begin                                  // 压缩 0 / 3(BITFIELDS)
                                    hok   <= hok & ((fq == 8'd0) || (fq == 8'd3));
                                    comp3 <= (fq == 8'd3);
                            end
                            default: ;
                        endcase
                        // BITFIELDS 掩码（偏移 54~65）：R=00FF0000 G=0000FF00 B=000000FF
                        if (pos >= 16'd54 && pos <= 16'd65)
                            mask_ok <= mask_ok & (fq == (((pos == 16'd56) || (pos == 16'd59) || (pos == 16'd62)) ? 8'hFF : 8'h00));
                    end else if (f_dry) begin
                        st <= S_SHORT;                // 头都没读完文件就结束了
                    end
                end
                S_CHK: begin
                    if (hok && (h_hi0 || h_hi1) && (off >= 16'd54)
                        && (w16 != 16'd0) && (w16 <= MAX_W) && (habs != 16'd0) && (habs <= MAX_H)
                        && (!comp3 || (is32 && off >= 16'd66 && mask_ok))) begin
                        sw   <= w16[10:0];
                        sh   <= habs[10:0];
                        flip <= !h_hi1;               // 正存（从下往上）才翻转
                        st   <= S_MUL;
                    end else begin
                        st <= S_BAD;
                    end
                end
                S_MUL: begin                          // acc = W*OUT_H, H*OUT_W, W*H
                    mac_md <= M_GEO; mac_n <= 3'd1; mac_go <= 1'b1;
                    st <= S_MUL2;
                end
                S_MUL2: if (!mac_busy) begin
                    // 宽图：W*OUT_H >= H*OUT_W -> 输出高 = round(H*OUT_W / W)；否则输出宽 = round(W*OUT_H / H)
                    // 三个乘积都 < 2^22（1920 x 1024），只比低 22 位；先按宽/高图选操作数，再用一个加法器
                    wide <= geo_wide;
                    mpc  <= acc2[21:0];
                    dn   <= {18'd0, geo_num + {11'd0, geo_den[10:1]}};
                    dd   <= {12'd0, geo_den};
                    drm  <= 24'd0; dcnt <= 6'd40; div_ret <= S_GEO;
                    st   <= S_DIV;
                end
                // 通用除法：dn / dd -> 商在 dn，40 拍
                S_DIV: begin
                    if (dcnt != 6'd0) begin
                        dcnt <= dcnt - 6'd1;
                        if (dge) begin drm <= dsub[23:0];      dn <= {dn[38:0], 1'b1}; end
                        else     begin drm <= dt;              dn <= {dn[38:0], 1'b0}; end
                    end else begin
                        st <= div_ret;
                    end
                end
                S_GEO: begin
                    if (wide) begin ow <= OUT_W; oh <= (dn[10:0] == 11'd0) ? 11'd1 : dn[10:0]; end
                    else      begin oh <= OUT_H; ow <= (dn[10:0] == 11'd0) ? 11'd1 : dn[10:0]; end
                    bw <= blen(sw);
                    bh <= blen(sh);
                    st <= S_PRM;
                end
                S_PRM: begin
                    ml   <= (OUT_W - ow) >> 1;
                    mrow <= (OUT_H - oh) >> 1;
                    mbot <= ((OUT_H - oh) >> 1) + (oh[0] ^ OUT_H[0]);   // = 总黑边 - 一半
                    down <= (sw > ow) || (sh > oh);
                    ls   <= (bw < 4'd9) ? (4'd9 - bw) : 4'd0;
                    rs   <= (bw > 4'd9) ? (bw - 4'd9) : 2'd0;
                    drm  <= 24'd0; dcnt <= 6'd40;
                    if ((sw > ow) || (sh > oh)) begin
                        // R = round(2^(14+bw+bh) / (W*H))：先在 S_PSH 里把 1 左移 14+bw+bh 位
                        dn  <= 40'd1;
                        shc <= 6'd14 + {2'b0, bw} + {2'b0, bh};
                        dd  <= {1'b0, mpc};
                        div_ret <= S_PRM3;
                        st  <= S_PSH;
                    end else begin
                        // sx = round(W*65536 / ow)
                        dn <= {13'd0, sw, 6'd0, ow[10:1]};        // = W*65536 + ow/2（低 16 位本来是 0，拼上即可）
                        dd <= {12'd0, ow};
                        div_ret <= S_PRM2;
                        st <= S_DIV;
                    end
                end
                S_PSH: begin
                    if (shc != 6'd0) begin
                        dn  <= {dn[38:0], 1'b0};
                        shc <= shc - 6'd1;
                    end else begin
                        dn  <= dn | {19'd0, mpc[21:1]};      // = 2^(14+bw+bh) + W*H/2（两者的位不重叠，不用加法器）
                        st  <= S_DIV;
                    end
                end
                S_PRM2: begin
                    sx   <= dn[16:0];
                    // sy = round(H*65536 / oh)
                    dn   <= {13'd0, sh, 6'd0, oh[10:1]};
                    dd   <= {12'd0, oh};
                    drm  <= 24'd0; dcnt <= 6'd40; div_ret <= S_PRM3;
                    st   <= S_DIV;
                end
                S_PRM3: begin
                    if (down) rr_r <= dn[16:0];
                    else      sy   <= dn[16:0];
                    st <= S_INI;
                end
                // 开始出像素前的初始化（权重表是常数 ROM，不需要预先计算）
                S_INI: begin
                    x <= 11'd0; y <= 11'd0; i <= 11'd0; j <= 10'd0; bc <= 2'd0;
                    rcap <= sh; rr <= 11'd0; frow <= 1'b1;
                    posy <= $signed({12'd0, sy[16:1]}) - 28'sd32768;
                    st <= S_MTOP;
                end
                // ================= 黑边行、发像素 =================
                S_MTOP: begin
                    if (mrow != 10'd0) begin
                        mrow <= mrow - 10'd1; blk_n <= OUT_W; ret_st <= S_MTOP; st <= S_BLK;
                    end else begin
                        st <= down ? D_ROW : U_ROW;
                    end
                end
                S_MBOT: begin
                    if (mrow != 10'd0) begin
                        mrow <= mrow - 10'd1; blk_n <= OUT_W; ret_st <= S_MBOT; st <= S_BLK;
                    end else begin
                        done <= 1'b1;
                        st   <= S_IDLE;                        // 剩下的字节读掉
                    end
                end
                S_BLK: begin
                    if (blk_n == 11'd0) st <= ret_st;
                    else if (e_go) begin out_valid <= 1'b1; out_rgb <= 24'd0; blk_n <= blk_n - 11'd1; end
                end
                S_PX1: begin
                    if (e_go) begin out_valid <= 1'b1; st <= ret_st; end
                end
                // 读一个源像素（B G R (A)）
                S_PIX: begin
                    if (f_av) begin
                        case (bc)
                            2'd0: pb <= fq;
                            2'd1: pg <= fq;
                            2'd2: pr <= fq;
                            default: ;
                        endcase
                        if (last_b) begin bc <= 2'd0; st <= pix_ret; end
                        else        bc <= bc + 2'd1;
                    end else if (f_dry) begin
                        st <= S_SHORT;
                    end
                end
                // 行尾填充字节
                S_PAD: begin
                    if (padc == 2'd0) st <= pad_ret;
                    else if (f_av) begin
                        padc <= padc - 2'd1;
                        if (padc == 2'd1) st <= pad_ret;
                    end else if (f_dry) begin
                        st <= S_SHORT;
                    end
                end
                S_BAD: begin
                    if (f_dry) begin err_hdr <= 1'b1; st <= S_IDLE; end
                end
                S_SHORT: begin
                    err_short <= 1'b1; st <= S_IDLE;
                end

                // ================= 缩小：区域平均 =================
                D_ROW: begin
                    // 这一源行的纵向权重：ovy1 给当前输出行；输出行在这里结束时 ovy2 给下一行
                    ovy1   <= v_d[11] ? rcap : oh;
                    rdone  <= v_rdone;
                    ovy2   <= v_d[11] ? -v_d[10:0] : 11'd0;       // oh > rcap 时 = oh - rcap
                    rcap_n <= v_d[10:0] + (v_rdone ? sh : 11'd0);  // oh < rcap: rcap-oh；oh == rcap: sh；否则 sh-(oh-rcap)
                    x <= 11'd0; i <= 11'd0; cap <= sw;
                    padc <= is32 ? 2'd0 : sw[1:0];    // 24 位：每行 3W 字节，补到 4 的倍数正好补 W mod 4 个
                    blk_n <= ml; ret_st <= D_PIXS;
                    st <= v_rdone ? S_BLK : D_PIXS;        // 这一源行结束一个输出行：先发左黑边
                end
                D_PIXS: begin
                    pix_ret <= D_H1; st <= S_PIX;
                end
                D_H1: begin
                    // 横向：acc + p * ov1（x == 0 时从 0 开始）
                    bop <= ovh_s; cdone <= h_cdone; ov2 <= h_ov2; cap <= h_capn;
                    mac_md <= M_DH1; mac_n <= 3'd1; mac_go <= 1'b1;
                    st <= D_H3;
                end
                D_H3: if (!mac_busy) begin
                    if (!cdone) begin
                        st <= D_NX;                         // 这一列还没完：横向和留在 acc 里
                    end else begin
                        // 一列完成：A' = A >> rs；这个像素剩下的部分（ov2）给下一列，在纵向做完以后（D_CN）再算
                        ap0 <= acc0[19:0]; ap1 <= acc1[19:0]; ap2 <= acc2[19:0];   // A >> rs 拆到下面两拍
                        st <= D_V0;
                    end
                end
                D_V0: begin
                    l_ra <= i[9:0];
                    if (rs != 2'd0) begin ap0 <= ap0 >> 1; ap1 <= ap1 >> 1; ap2 <= ap2 >> 1; end   // rs >= 1
                    st <= D_V1;
                end
                D_V1: begin
                    if (rs[1]) begin ap0 <= ap0 >> 1; ap1 <= ap1 >> 1; ap2 <= ap2 >> 1; end        // rs = 2
                    st <= D_V2;
                end
                D_V2: begin
                    // 纵向：V + A' * ovy1（初值直接取行存储读出的 lq）
                    bop <= {7'd0, ovy1}; mac_md <= M_DV1; mac_n <= 3'd1; mac_go <= 1'b1;
                    st <= D_V3;
                end
                D_V3: if (!mac_busy) begin
                    if (!rdone) begin
                        l_we <= 4'b1111; l_wa <= i[9:0];
                        st <= D_CN;
                    end else begin
                        vs0 <= acc0[27:0]; vs1 <= acc1[27:0]; vs2 <= acc2[27:0];
                        shc <= {2'b0, bh};
                        st <= D_SH;
                    end
                end
                D_SH: begin
                    // 归一前 V >> bh：每拍右移 1 位
                    if (shc != 6'd0) begin
                        vs0 <= vs0 >> 1; vs1 <= vs1 >> 1; vs2 <= vs2 >> 1;
                        shc <= shc - 6'd1;
                    end else begin
                        bop <= {1'b0, rr_r}; mac_md <= M_DN; mac_go <= 1'b1;
                        st <= D_V4;
                    end
                end
                D_V4: if (!mac_busy) begin
                    out_rgb <= {sat_dn(acc0), sat_dn(acc1), sat_dn(acc2)};
                    // 这一源行剩下的部分是下一输出行的开头：V = A' * ovy2（ovy2 = 0 时为 0）
                    bop <= {7'd0, ovy2}; mac_md <= M_DV2; mac_go <= 1'b1;
                    ret_st <= D_V6; st <= S_PX1;
                end
                D_V6: if (!mac_busy) begin
                    l_we <= 4'b1111; l_wa <= i[9:0];
                    st <= D_CN;
                end
                D_CN: begin
                    i <= i + 11'd1;
                    // 下一列的开头 = p * ov2（ov2 = 0 时为 0），结果留在 acc 里
                    bop <= ovh_s; mac_md <= M_DH2; mac_n <= 3'd1; mac_go <= 1'b1;
                    st <= D_NX;
                end
                D_NX: if (!mac_busy) begin
                    if (x_inc == sw) begin
                        pad_ret <= D_RE; st <= S_PAD;
                    end else begin
                        x <= x_inc; st <= D_PIXS;
                    end
                end
                D_RE: begin
                    blk_n <= mright; ret_st <= D_RE2;
                    st <= rdone ? S_BLK : D_RE2;           // 结束了一个输出行：补右黑边
                end
                D_RE2: begin
                    rcap <= rcap_n;
                    frow <= 1'b0;                       // 只有第一个源行从 0 开始，之后都从行存储累加
                    if (y + 11'd1 == sh) begin
                        mrow <= mbot; st <= S_MBOT;
                    end else begin
                        y <= y + 11'd1; st <= D_ROW;
                    end
                end

                // ================= 放大 / 一样大：双三次 =================
                U_ROW: begin
                    // 先把这一输出行要用的源行（到 y0+2 为止）收进行存储
                    if (u_need) begin
                        x <= 11'd0; padc <= is32 ? 2'd0 : sw[1:0];
                        pix_ret <= U_RX; st <= S_PIX;
                    end else begin
                        vk <= 2'd0; w_ra <= posy[15:8];
                        st <= U_RG0;
                    end
                end
                U_RX: begin
                    l_we <= 4'b0001 << rr[1:0]; l_wa <= x[9:0];
                    if (x_inc == sw) begin
                        pad_ret <= U_RX2; st <= S_PAD;
                    end else begin
                        x <= x_inc; pix_ret <= U_RX; st <= S_PIX;
                    end
                end
                U_RX2: begin rr <= rr + 11'd1; st <= U_ROW; end
                U_RG0: begin                                // 4 个纵向抽头所在的行存储（行号 mod 4），4 拍
                    case (vk)
                        2'd0: sl0 <= cl_o[1:0];
                        2'd1: sl1 <= cl_o[1:0];
                        2'd2: sl2 <= cl_o[1:0];
                        default: sl3 <= cl_o[1:0];
                    endcase
                    vk <= vk + 2'd1;
                    if (vk == 2'd3) st <= U_RG2;
                end
                U_RG2: begin
                    wy0 <= wt_a; wy1 <= wt_b; wy2 <= wt_c; wy3 <= wt_d;
                    blk_n <= ml; ret_st <= U_PX0; st <= S_BLK;
                end
                U_PX0: begin
                    i  <= 11'd0;
                    posx <= posx0;
                    vk <= 2'd0;
                    st <= U_VC0;
                end
                // 算源列 clamp(x0-1+vk) 的 v，从右边移进窗口
                U_VC0: begin l_ra <= cl_o[9:0]; st <= U_VC1; end
                U_VC1: st <= U_VC2;
                U_VC2: begin
                    mac_md <= M_UV; mac_n <= 3'd4; mac_go <= 1'b1;
                    st <= U_VC3;
                end
                U_VC3: if (!mac_busy) begin
                    wn_we <= 1'b1;
                    wn_wd <= {acc2[17:0], acc1[17:0], acc0[17:0]};
                    if (vk == 2'd3) st <= U_PXW;
                    else begin vk <= vk + 2'd1; st <= U_VC0; end
                end
                U_PXW: begin w_ra <= posx[15:8]; st <= U_PW1; end
                U_PW1: st <= U_PW2;
                U_PW2: begin
                    mac_md <= M_UH; mac_n <= 3'd4; mac_go <= 1'b1;
                    st <= U_PW3;
                end
                U_PW3: if (!mac_busy) begin
                    out_rgb <= {sat_up(acc0), sat_up(acc1), sat_up(acc2)};
                    ret_st <= U_PN; st <= S_PX1;
                end
                U_PN: begin
                    if (i + 11'd1 == ow) begin
                        blk_n <= mright; ret_st <= U_RN; st <= S_BLK;
                    end else begin
                        i <= i + 11'd1;
                        posx <= posx_n;
                        if ($signed(posx_n[27:16]) != x0) begin
                            // 进到下一源列：新算 clamp(x0+2) 的 v 移进窗口（x0 随 posx 一起变）
                            vk <= 2'd3;
                            st <= U_VC0;
                        end else begin
                            st <= U_PXW;
                        end
                    end
                end
                U_RN: begin
                    posy <= posy + {11'd0, sy};
                    if ({1'b0, j} + 11'd1 == oh) begin
                        mrow <= mbot; st <= S_MBOT;
                    end else begin
                        j <= j + 10'd1; st <= U_ROW;
                    end
                end
                default: ;                                     // S_IDLE：读掉缓冲里的字节
                endcase
            end
        end
    end
endmodule
