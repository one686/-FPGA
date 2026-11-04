`timescale 1ns/1ps

// 画面调色（像素时钟域）：亮度 / 对比度 / 色温用查表实现，饱和度逐像素计算
//
// 原理：亮度、对比度、色温对每个像素都是"每个通道 0~255 -> 0~255"的同一个映射，
//   所以每帧开始时在场消隐里把 R/G/B 三张 256 项的对照表算好，像素直接查表：像素通路 0 个乘法器。
// 每张表的公式（全部整数，design/color_gen.py 是逐位一致的参考模型）：
//   v0 = (x  * gb ) >> 8                      亮度（gb = 256 + 档位*26，档位 -5..+5）
//   v1 = (v0 * gc ) >> 8                      色温（R: gc = 256 + 档位*10，G: 256，B: 256 - 档位*10）
//   v2 = (((v1 - 128) * gk) >>> 8) + 128      对比度（gk = 256 + 档位*26），以 128 为中心拉伸/压缩
//   out = 限幅到 0~255
//   三个参数都在 0 档时 gb = gc = gk = 256，表就是 out = x，输出与输入逐位相同。
// 平滑：每个帧起点，当前增益向目标增益走 STEP（到了就停），表按当前增益重算 -> 调节时画面平滑过渡，
//   一帧之内表不变（只在场消隐里写），不会撕裂。
// 算表：一个乘法器分时用，每项每通道 4 拍（3 次乘 + 写），768 x 4 = 3072 拍，场同步开始到有效区约 28000 拍。
// 饱和度（查表之后，逐像素）：灰度 Y = (77R + 150G + 29B) >> 8（移位相加）
//   out = 限幅(Y + ((c - Y) * m) >>> 5)，m = gs / 8（gs = 饱和度增益，0~416，都是 8 的倍数）
//   (c - Y) * m 用 3 个乘法器（DSP），比移位相加约省 400 LUT
//   gs 档位表：0 48 104 152 208 256 288 320 352 384 416（-5 = 黑白，0 档 = 256 = 不变，+5 约 1.6 倍）
//   m = 32 时 ((c-Y)*32)>>>5 = c-Y，输出与输入逐位相同；灰度 Y 的常数系数用移位相加，不占 DSP。
// 延迟：本模块比输入晚 3 拍（BRAM 读出、算 Y、饱和度），media_display_path 把坐标/同步信号一起延迟（COLOR_LAT）。
module color_adjust #(
    parameter integer STEP = 8          // 每帧增益最多变化多少（256 = 1 倍；仿真可改大加快）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire [3:0]  lv_b,             // 亮度档位 0~10（5 = 不调）
    input  wire [3:0]  lv_k,             // 对比度档位 0~10（5 = 不调）
    input  wire [3:0]  lv_t,             // 色温档位 0~10（5 = 不调，>5 偏暖，<5 偏冷）
    input  wire [3:0]  lv_s,             // 饱和度档位 0~10（5 = 不调，0 = 黑白）
    input  wire [23:0] rgb_in,
    output reg  [23:0] rgb_out,
    // 当前增益（仿真比对用）
    output reg  [8:0]  cur_gb,
    output reg  [8:0]  cur_gk,
    output reg  [8:0]  cur_gr,
    output wire [8:0]  cur_gbl,
    output reg  [8:0]  cur_gs
);
    // ---------------- 目标增益 ----------------
    // 档位 l（0~10），偏移 d = l - 5；d*26 = d*16 + d*8 + d*2，d*10 = d*8 + d*2（移位相加）
    function [8:0] gain26;
        input [3:0] l;
        reg signed [9:0] d;
        begin
            d = $signed({6'd0, l}) - 10'sd5;
            gain26 = 10'sd256 + (d <<< 4) + (d <<< 3) + (d <<< 1);
        end
    endfunction
    function [8:0] gain10;
        input [3:0] l;
        input       neg;
        reg signed [9:0] d;
        begin
            d = $signed({6'd0, l}) - 10'sd5;
            if (neg) d = -d;
            gain10 = 10'sd256 + (d <<< 3) + (d <<< 1);
        end
    endfunction

    function [8:0] gain_s;               // 饱和度增益（8 的倍数，平滑步长也是 8 的倍数，所以 gs 永远是 8 的倍数）
        input [3:0] l;
        begin
            case (l)
                4'd0: gain_s = 9'd0;    4'd1: gain_s = 9'd48;   4'd2: gain_s = 9'd104;
                4'd3: gain_s = 9'd152;  4'd4: gain_s = 9'd208;  4'd6: gain_s = 9'd288;
                4'd7: gain_s = 9'd320;  4'd8: gain_s = 9'd352;  4'd9: gain_s = 9'd384;
                4'd10: gain_s = 9'd416; default: gain_s = 9'd256;
            endcase
        end
    endfunction

    function [8:0] approach;
        input [8:0] cur;
        input [8:0] tgt;
        begin
            if (cur + STEP <= tgt)      approach = cur + STEP;
            else if (cur >= tgt + STEP) approach = cur - STEP;
            else                        approach = tgt;
        end
    endfunction

    // 4 个增益共用一个 approach（省 LUT）：帧起点那拍更新亮度 gb（用当拍的档位），
    //   之后 3 拍依次更新色温 gr、对比度 gk、饱和度 gs（用帧起点锁存的档位）。算表在帧起点后第 1/2/3 拍才分别用到
    //   gb / gr / gk，正好都已更新；gs 第 3 拍更新，像素通路要到有效区才用。效果与 4 个增益各自独立平滑逐位相同。
    reg  [3:0] l_k, l_t, l_s;            // 帧起点锁存的档位
    reg  [1:0] ap_k;                     // 0 空闲  1 gr  2 gk  3 gs
    wire [3:0] g26_l  = frame_start ? lv_b : l_k;
    wire [8:0] g26    = gain26(g26_l);
    wire [8:0] ap_cur = frame_start ? cur_gb : (ap_k == 2'd1) ? cur_gr : (ap_k == 2'd2) ? cur_gk : cur_gs;
    wire [8:0] ap_tgt = (frame_start || ap_k == 2'd2) ? g26 : (ap_k == 2'd1) ? gain10(l_t, 1'b0) : gain_s(l_s);
    wire [8:0] ap_new = approach(ap_cur, ap_tgt);

    // ---------------- 算表状态机 ----------------
    reg        busy;
    reg [7:0]  idx;
    reg [1:0]  ch;                       // 0 R  1 G  2 B
    reg [1:0]  ph;                       // 0 亮度乘  1 色温乘  2 对比度乘  3 写
    reg [8:0]  v0, v1;
    reg [7:0]  vout;
    // 色温的蓝增益和红增益目标对称（256 ± 10d），复位都是 256、步长相同，
    //   approach 的两个判断互为镜像，所以每一帧都有 cur_gbl = 512 - cur_gr（Python 穷举验证），因此不单独存、单独平滑
    assign cur_gbl = 10'd512 - {1'b0, cur_gr};
    wire [8:0] gch = (ch == 2'd0) ? cur_gr : (ch == 2'd1) ? 9'd256 : cur_gbl;

    // 一个乘法器：有符号 11 位 x 10 位
    reg  signed [10:0] ma;
    reg  signed [9:0]  mb;
    always @* begin
        case (ph)
            2'd0:    begin ma = {3'd0, idx};                       mb = {1'b0, cur_gb}; end
            2'd1:    begin ma = {2'd0, v0};                        mb = {1'b0, gch};    end
            default: begin ma = $signed({2'd0, v1}) - 11'sd128;    mb = {1'b0, cur_gk}; end
        endcase
    end
    wire signed [20:0] mp = ma * mb;
    wire signed [12:0] v2 = (mp >>> 8) + 13'sd128;

    reg        we;
    reg [1:0]  wch;
    reg [7:0]  wa;
    reg [7:0]  wd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_gb <= 9'd256; cur_gk <= 9'd256; cur_gr <= 9'd256; cur_gs <= 9'd256;
            l_k <= 4'd5; l_t <= 4'd5; l_s <= 4'd5; ap_k <= 2'd0;
            busy <= 1'b0; idx <= 0; ch <= 0; ph <= 0;
            v0 <= 0; v1 <= 0; vout <= 0;
            we <= 1'b0; wch <= 0; wa <= 0; wd <= 0;
        end else begin
            we <= 1'b0;
            if (frame_start) begin
                cur_gb <= ap_new;
                l_k <= lv_k; l_t <= lv_t; l_s <= lv_s; ap_k <= 2'd1;
            end else if (ap_k != 2'd0) begin
                case (ap_k)
                    2'd1:    cur_gr <= ap_new;
                    2'd2:    cur_gk <= ap_new;
                    default: cur_gs <= ap_new;
                endcase
                ap_k <= ap_k + 2'd1;
            end
            if (frame_start) begin
                busy <= 1'b1; idx <= 0; ch <= 0; ph <= 0;
            end else if (busy) begin
                case (ph)
                    2'd0: begin v0 <= mp[16:8]; ph <= 2'd1; end
                    2'd1: begin v1 <= mp[16:8]; ph <= 2'd2; end
                    2'd2: begin
                        vout <= v2[12] ? 8'd0 : (v2 > 13'sd255) ? 8'd255 : v2[7:0];
                        ph <= 2'd3;
                    end
                    default: begin
                        we <= 1'b1; wch <= ch; wa <= idx; wd <= vout;
                        ph <= 2'd0;
                        if (ch == 2'd2) begin
                            ch <= 2'd0;
                            if (idx == 8'd255) busy <= 1'b0;
                            idx <= idx + 8'd1;
                        end else
                            ch <= ch + 2'd1;
                    end
                endcase
            end
        end
    end

    // ---------------- 三张表（每张 256 x 8，写口给算表，读口给像素） ----------------
    reg [7:0] tab_r [0:255];
    reg [7:0] tab_g [0:255];
    reg [7:0] tab_b [0:255];
    always @(posedge clk) begin
        if (we && wch == 2'd0) tab_r[wa] <= wd;
        if (we && wch == 2'd1) tab_g[wa] <= wd;
        if (we && wch == 2'd2) tab_b[wa] <= wd;
    end
    reg [7:0] qr, qg, qb;
    always @(posedge clk) begin
        qr <= tab_r[rgb_in[23:16]];
        qg <= tab_g[rgb_in[15:8]];
        qb <= tab_b[rgb_in[7:0]];
    end

    // ---------------- 饱和度 ----------------
    // 第 2 拍：灰度 Y = (77R + 150G + 29B) >> 8；77 = 64+8+4+1，150 = 128+16+4+2，29 = 16+8+4+1
    wire [15:0] y_sum = ({8'd0, qr} << 6) + ({8'd0, qr} << 3) + ({8'd0, qr} << 2) + {8'd0, qr} +
                        ({8'd0, qg} << 7) + ({8'd0, qg} << 4) + ({8'd0, qg} << 2) + ({8'd0, qg} << 1) +
                        ({8'd0, qb} << 4) + ({8'd0, qb} << 3) + ({8'd0, qb} << 2) + {8'd0, qb};
    reg [7:0] r2, g2, b2, y2;
    reg [5:0] m2;                        // gs / 8（0~52），一帧之内不变
    // 饱和度这两级是纯流水寄存器，不带异步复位（复位期间没有画面，复位后 3 拍就是正确值）。
    //   若带复位，TD 会把它们和读 FIFO 的复位同步器并到同一张复位网络，出现复位释放（removal）时序违例。
    always @(posedge clk) begin
        r2 <= qr; g2 <= qg; b2 <= qb; y2 <= y_sum[15:8]; m2 <= cur_gs[8:3];
    end
    // 第 3 拍：out = 限幅(Y + ((c - Y) * m) >>> 5)
    //   (c - Y) * m 用乘法器（3 个 DSP）
    wire signed [9:0]  d_r = $signed({2'b00, r2}) - $signed({2'b00, y2});
    wire signed [9:0]  d_g = $signed({2'b00, g2}) - $signed({2'b00, y2});
    wire signed [9:0]  d_b = $signed({2'b00, b2}) - $signed({2'b00, y2});
    wire signed [6:0]  m_s = $signed({1'b0, m2});
    wire signed [16:0] p_r = d_r * m_s;
    wire signed [16:0] p_g = d_g * m_s;
    wire signed [16:0] p_b = d_b * m_s;
    function [7:0] sat;
        input [7:0]         y;
        input signed [16:0] p;
        reg signed [11:0] v;
        begin
            v = $signed({4'd0, y}) + (p >>> 5);
            sat = v[11] ? 8'd0 : (v > 12'sd255) ? 8'd255 : v[7:0];
        end
    endfunction
    always @(posedge clk) begin
        rgb_out <= {sat(y2, p_r), sat(y2, p_g), sat(y2, p_b)};
    end
endmodule
