`timescale 1ns/1ps

// 音频频谱分析 + "音浪"叠加层（video_clk 域，25MHz）
//   实时算 32 个频带的电平（柱高 0~30 + 峰值帽），在画面下部（音浪区）画成一条中心对称、平滑的彩色山脊线，
//   叠加在 OSD 之后的视频上；show 控制 16 帧淡入/淡出。显示表另有一个读口（fs_ra/fs_rq）给全屏可视化 fullscreen_viz 用。
//   视频延迟 2 拍（hs/vs/de 同步延迟）。
//
// 算法（design/spec_gen.py 是逐位一致的参考模型，版面/系数由它生成 spec_layout.vh、spec_coef.mif/.hex）：
//   单声道 m = (L + R) >>> 1；32 根柱 50 Hz~16 kHz 对数分布，每根 = 两个相同的二阶带通节串联（四阶恒 Q）
//   高组（第 17~31 根，1 kHz 以上）48 kHz 算；低组（第 0~16 根）先 8 个样本求和 >>> 3 抽到 6 kHz 再算（低频系数精度才够）
//   每节 y = sat18(trunc0((B*d + C1*s1 + C2*s2) / 2^16))，向零舍入（静音时不会卡在极限环）
//   每帧累加 |z|，帧标记时换成对数电平（0.75 dB/级）-> 柱高 0~30：上升立即、下降每帧 1，峰值帽停 20 帧后每帧落 1
//
// 结构（按省资源设计）：
//   采样：单声道样本和帧标记一起进 8 深队列
//   引擎：1 个乘法器分时复用，每根柱 12 拍；高组 15 根，每 8 个样本再加低组 17 根；样本间隔约 520 拍
//     - 滤波历史轮换存放：y1/y2（z1/z2）占两格，奇偶位指明哪格是新的，新值只覆盖旧格（不需整体搬移）
//     - 系数表第 0 项存 B0 << G，累加器只有"清零 / 累加"两种，一个加法器
//     - 帧标记后各组第一次计算不读旧的累加值（当 0），不用回头清零
//   帧标记：0~31 根柱换算电平，按"上升立即、下降每帧 1、峰值保持"更新，结果写进显示表 {保持计数 6, 峰值 6, 柱高 6}
//   存储：系数 128x18（BRAM，mif 初值）；状态 256x18（{柱, k}：k0/k1 y 两格，k2/k3 z 两格，k4 acc 低 18 位，k5 acc 高 7 位）；
//         显示表 32x18
//
// 显示流水线：输入延迟 2 拍输出；第 0 拍判区域、读显示表，第 1 拍分类配色，第 2 拍混合输出。
// 显示效果"音浪"：32 个柱高连成平滑的山脊线（中心对称，低频在中间）+ 节拍脉冲（低频跳高时浪变亮、变粗）
//   + 流动色带（彩虹色每帧滚动），见下面"显示"一节。
module audio_spectrum #(
    parameter integer H_BLANK = 160,
    parameter integer V_BLANK = 45
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        show,
    input  wire        pcm_valid,
    input  wire signed [15:0] pcm_left,
    input  wire signed [15:0] pcm_right,
    input  wire [11:0] raw_x,
    input  wire [11:0] raw_y,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    // 全屏可视化用：显示表第二个读口（第 fs_ra 根柱的 {保持计数, 峰值, 柱高}，读延迟 1 拍）
    input  wire [4:0]  fs_ra,
    output reg  [17:0] fs_rq,
    output reg         hs_out,
    output reg         vs_out,
    output reg         de_out,
    output wire [23:0] rgb_out
);
    `include "spec_layout.vh"

    // ======================== 淡入淡出 ========================
    // f16 = 叠加层不透明度 0~16：show=1 时每帧 +1，show=0 时每帧 -1
    reg  [4:0]  f16;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f16 <= 0;
        end else if (frame_start) begin
            if (show) f16 <= (f16 == 5'd16) ? f16 : f16 + 5'd1;
            else      f16 <= (f16 == 5'd0) ? f16 : f16 - 5'd1;
        end
    end

    // ======================== 输入队列：{帧标记, 有样本, 单声道 16 位} ========================
    wire signed [16:0] msum = pcm_left + pcm_right;
    reg  [17:0] fq [0:7];
    reg  [2:0]  fq_wp, fq_rp;
    reg  [3:0]  fq_n;
    wire        fq_push = frame_start || pcm_valid;
    wire        fq_pop;
    always @(posedge clk) if (fq_push) fq[fq_wp] <= {frame_start, pcm_valid, msum[16:1]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fq_wp <= 0; fq_rp <= 0; fq_n <= 0;
        end else begin
            if (fq_push) fq_wp <= fq_wp + 3'd1;
            if (fq_pop)  fq_rp <= fq_rp + 3'd1;
            fq_n <= fq_n + (fq_push ? 4'd1 : 4'd0) - (fq_pop ? 4'd1 : 4'd0);
        end
    end
    wire [17:0] fq_head = fq[fq_rp];

    // ======================== 存储 ========================
    reg  [2:0]  est;
    reg  [3:0]  t;
    reg  [4:0]  bnd;                                 // 柱号 0~31（滤波与帧标记共用）

    // 系数 ROM（地址 {柱, j}，j: 0=B0<<G 1=B2 2=C1 3=C2），读延迟 1 拍
    wire [6:0]  ca;                                  // 组合：本拍发地址，下一拍出数据
    wire [17:0] cq_raw;
`ifdef OSD_SIM
    reg [17:0] crom [0:127];
    reg [17:0] cq_r;
    initial $readmemh("../src/user_source/hdl_source/display/spec_coef.hex", crom);
    always @(posedge clk) cq_r <= crom[ca];
    assign cq_raw = cq_r;
`else
    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(18), .ADDR_WIDTH_A(7), .DATA_DEPTH_A(128),
        .DATA_WIDTH_B(18), .ADDR_WIDTH_B(7), .DATA_DEPTH_B(128),
        .MODE("SP"), .REGMODE_A("NOREG"), .IMPLEMENT("9K"),
        .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("spec_coef.mif"), .FILL_ALL("NONE")
    ) u_coef (
        .dia(18'd0), .dib(18'd0), .addra(ca), .addrb(7'd0),
        .cea(1'b1), .ceb(1'b0), .ocea(1'b0), .oceb(1'b0),
        .clka(clk), .clkb(1'b0), .wea(1'b0), .web(1'b0),
        .bea(1'b0), .beb(1'b0), .rsta(1'b0), .rstb(1'b0),
        .doa(cq_raw), .dob()
    );
`endif
    wire signed [17:0] cq = cq_raw;

    // 状态 RAM：一个写口、一个读口，读延迟 1 拍
    reg  [17:0] smem [0:255];
    wire [7:0]  sra;
    reg  [7:0]  swa;
    reg         swe;
    reg  [17:0] swd;
    reg  [17:0] sq_r;
    integer ii;
    initial for (ii = 0; ii < 256; ii = ii + 1) smem[ii] = 18'd0;
    always @(posedge clk) begin
        if (swe) smem[swa] <= swd;
        sq_r <= smem[sra];
    end
    wire signed [17:0] sq = sq_r;

    // 显示表：{保持计数 6, 峰值 6, 柱高 6}；引擎在帧标记时读改写，其余时间显示读
    reg  [17:0] dmem [0:31];
    reg         dwe, upd;
    reg  [4:0]  dwa;                                 // 写地址和写数据一起寄存（写的时候 bnd 已经加 1）
    reg  [17:0] dwd;
    reg  [17:0] dq;
    initial for (ii = 0; ii < 32; ii = ii + 1) dmem[ii] = 18'd0;
    // 音浪显示读地址（中心对称）：段 s 里读 K(s+3)（下一段开头要用），段外读 K(2) = bar[31]（第 0 段开头要用）
    //   K(u+2)：u <= 31 -> bar[31-u]，32 <= u <= 63 -> bar[u-32]（u >= 64 是 0，不用读）
    wire [9:0] wxv  = raw_x[9:0] - (H_BLANK + 52);      // = WX0（音浪区内的 x：[9:3] 段号，[2:0] 段内）
    wire       inw0 = (wxv < 10'd536);                  // 67 段 x 8
    wire [6:0] seg0 = wxv[9:3];
    wire       t00  = (wxv[2:0] == 3'd0);
    wire [6:0] snx  = inw0 ? seg0 + 7'd1 : 7'd0;
    wire [4:0] dra = upd ? bnd : (snx[5] ? snx[4:0] : ~snx[4:0]);
    // A 口 = 写（帧标记时）/ 音浪显示读，B 口 = 全屏模式读（一块 BRAM 的两个口）。
    //   A 口写的那几拍（场消隐里）读出的是写地址的旧值：引擎只在 t3 用 dq（t2 读的，那拍不写），音浪在场消隐里不显示
    wire [4:0] da = dwe ? dwa : dra;
    always @(posedge clk) begin
        if (dwe) dmem[da] <= dwd;
        dq <= dmem[da];
    end
    always @(posedge clk) fs_rq <= dmem[fs_ra];

    // ======================== 引擎 ========================
    // 每根柱 12 拍（t = 0..11），P = 本组奇偶位：y1 在 y[P]、y2 在 y[!P]（z 同理）
    //   t0  读 B0<<G
    //   t1  读 C1、y1          乘 (B0<<G) x (x - x2)
    //   t2  读 C2、y2          乘 C1 x y1
    //   t3  读 C1、z1          乘 C2 x y2        存 y2        和 = 积
    //   t4  读 C2、z2          乘 C1 x z1                     和 += 积
    //   t5  读 B2、acc 低      乘 C2 x z2                     y = fin(和 + 积)
    //   t6  读 acc 高          乘 B2 x ((y - y2) >>> 1)       和 = 积        写 y -> y[!P]，存 acc 低
    //   t7                                                    和 += 积       存 acc 高
    //   t8                                                    z = fin(和 + 积)
    //   t9                                                    写 z -> z[!P]，算新 acc
    //   t10                                                   写 acc 低
    //   t11                                                   写 acc 高 -> 下一根；组末翻转 P
    // 帧标记（每项 4 拍）：t0 读 acc 低（显示表按 bnd 读）、t1 读 acc 高、t2 收、t3 算电平、更新柱/峰值、写显示表
    localparam [2:0] E_IDLE = 3'd0, E_MARK = 3'd1, E_HIGH = 3'd2, E_LOW = 3'd3;
    reg         pend_s, pend_low;         // 当前队列项还有样本要算 / 低组要算
    reg         php, plo;                 // 高组、低组的奇偶位
    reg  [7:0]  lowacc, lowprev;          // 节拍脉冲：本帧 bar[0..4] 新柱高之和；上一帧 bar[0..5] 之和
    reg  [3:0]  pz;                       //   脉冲 0~8
    localparam integer BEAT_T = 18;       //   低频和跳高多少算一拍（柱高单位）
    reg         fh, fl;                   // 帧标记之后高组、低组还没算过（acc 当 0）
    reg  signed [15:0] xh1, xh2, xl1, xl2, xln, xin; // xln = 本次抽取输出；低组算完才移进 xl1/xl2
    reg  signed [18:0] bx;                           // 低组：8 个样本的和
    reg  [2:0]  ph;
    wire        lowg = (est == E_LOW);
    wire        pp   = lowg ? plo : php;
    wire        fz   = (est == E_MARK) ? ((bnd < NLOW) ? fl : fh) : (lowg ? fl : fh);
    wire signed [15:0] gx  = lowg ? xln : xin;
    wire signed [15:0] gx2 = lowg ? xl2 : xh2;
    wire signed [16:0] d1 = gx - gx2;

    // 乘法器：输入寄存，积寄存
    reg  signed [17:0] ma, mb;
    reg  signed [35:0] mp;
    always @(posedge clk) mp <= ma * mb;
    reg  signed [37:0] sum;
    reg  signed [17:0] fr, ry2;                     // fr = 本节结果；ry2 = y2
    reg  [17:0] ral;
    reg  [6:0]  rah;
    reg  [24:0] accn;

    // 和：t3、t6 装载（= 积），其余累加；本节结果 = sat18(向零舍入((和 + 积) / 2^16))
    wire        ld = (t == 4'd3) || (t == 4'd6);
    wire signed [37:0] sum_p = (ld ? 38'sd0 : sum) + {{2{mp[35]}}, mp};
    wire signed [21:0] fq0 = sum_p[37:16] + {21'd0, sum_p[37] && (sum_p[15:0] != 16'd0)};
    wire signed [17:0] fin_v = (fq0 > 22'sd131071) ? 18'sd131071 : (fq0 < -22'sd131072) ? -18'sd131072 : fq0[17:0];
    wire signed [18:0] dif = fr - ry2;              // t6：fr = 新 y
    wire [17:0] za = fr[17] ? -fr : fr;             // t9：fr = 新 z（-131072 的绝对值 131072 仍放得下 18 位无符号）
    wire [24:0] acc_r = fz ? 25'd0 : {rah, ral};
    wire [25:0] acc_sum = acc_r + (za >> G);

    // 电平：8 x 最高位 + 下面 3 位（< 8 时 0）
    function [7:0] lvl8;
        input [24:0] u;
        integer i;
        reg [4:0] m;
        reg [2:0] mant;
        begin
            m = 0;
            for (i = 0; i < 25; i = i + 1)
                if (u[i]) m = i;
            mant = (m >= 3) ? (u >> (m - 3)) : 3'd0;
            lvl8 = (m < 3) ? 8'd0 : {m, mant};
        end
    endfunction
    wire [7:0]  lv8  = lvl8(acc_r);
    wire [7:0]  ofs  = (bnd < NLOW) ? LVL_LO : (LVL_HI0 - bnd);
    wire [8:0]  lvd  = {1'b0, lv8} - {1'b0, ofs};
    wire [5:0]  lv   = lvd[8] ? 6'd0 : (lvd[7:0] > HMAX) ? HMAX : lvd[5:0];
    wire [5:0]  o_bar = dq[5:0], o_cap = dq[11:6], o_ht = dq[17:12];
    wire [5:0]  n_bar = (lv > o_bar) ? lv : (o_bar != 0) ? o_bar - 6'd1 : 6'd0;
    reg  [5:0]  n_cap, n_ht;
    always @* begin                                  // 峰值帽比的是新柱高
        n_cap = o_cap; n_ht = o_ht;
        if (n_bar >= o_cap) begin n_cap = n_bar; n_ht = HOLD; end
        else if (o_ht != 0) n_ht = o_ht - 6'd1;
        else if (o_cap != 0) n_cap = o_cap - 6'd1;
    end

    // 低组抽取（取样本时一拍算完）
    wire signed [18:0] nbx = bx + {{3{fq_head[15]}}, fq_head[15:0]};

    // 读地址（组合）
    reg [1:0] cj;
    reg [2:0] sk;
    always @* begin
        cj = 2'd0; sk = 3'd4;
        if (est == E_MARK) sk = (t == 4'd0) ? 3'd4 : 3'd5;
        else case (t)
            4'd1: begin cj = 2'd2; sk = {2'b00,  pp}; end
            4'd2: begin cj = 2'd3; sk = {2'b00, !pp}; end
            4'd3: begin cj = 2'd2; sk = {2'b01,  pp}; end
            4'd4: begin cj = 2'd3; sk = {2'b01, !pp}; end
            4'd5: begin cj = 2'd1; sk = 3'd4; end
            4'd6: sk = 3'd5;
            default: ;
        endcase
    end
    assign ca  = {bnd[4:0], cj};
    assign sra = {bnd[4:0], sk};

    assign fq_pop = (est == E_IDLE) && !pend_s && !pend_low && (fq_n != 0);
    wire [7:0] sbase = {bnd[4:0], 3'd0};
    wire       last_hi = (bnd[4:0] == NB - 1), last_lo = (bnd[4:0] == NLOW - 1);

    always @(posedge clk) begin
        ma <= cq;
        mb <= (t == 4'd1) ? {d1[16], d1} : (t == 4'd6) ? dif[18:1] : sq;
        sum <= sum_p;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            est <= E_IDLE; t <= 0; bnd <= 0; pend_s <= 1'b0; pend_low <= 1'b0;
            php <= 1'b0; plo <= 1'b0; fh <= 1'b1; fl <= 1'b1;
            xh1 <= 0; xh2 <= 0; xl1 <= 0; xl2 <= 0; xln <= 0; xin <= 0;
            bx <= 0; ph <= 0;
            fr <= 0; ry2 <= 0; ral <= 0; rah <= 0; accn <= 0;
            swa <= 0; swe <= 1'b0; swd <= 0;
            dwe <= 1'b0; dwa <= 0; dwd <= 0; upd <= 1'b0;
            lowacc <= 0; lowprev <= 0; pz <= 0;
        end else begin
            swe <= 1'b0; dwe <= 1'b0;
            case (est)
            E_IDLE: begin
                t <= 0;
                if (fq_pop) begin
                    xin <= fq_head[15:0];
                    pend_s <= fq_head[16];
                    if (fq_head[16]) begin                  // 低组求和、抽取相位
                        ph <= ph + 3'd1;
                        if (ph == 3'd7) begin
                            bx <= 19'sd0;
                            xln <= nbx[18:3];
                            pend_low <= 1'b1;
                        end else
                            bx <= nbx;
                    end
                    if (fq_head[17]) begin est <= E_MARK; bnd <= 0; upd <= 1'b1; end
                end else if (pend_s) begin
                    est <= E_HIGH; bnd <= NLOW;
                end else if (pend_low) begin
                    est <= E_LOW; bnd <= 0;
                end
            end
            E_MARK: begin
                t <= t + 4'd1;
                case (t)
                4'd0: ;
                4'd1: ral <= sq;
                4'd2: rah <= sq[6:0];
                default: begin                                  // t3
                    dwe <= 1'b1; dwa <= bnd; dwd <= {n_ht, n_cap, n_bar};
                    t <= 0;
                    // 节拍脉冲：low = bar[0..5] 新柱高之和；比上一帧跳高 >= BEAT_T 且脉冲快灭了 -> 8，否则每帧 -1
                    if (bnd == 6'd0)     lowacc <= {2'd0, n_bar};
                    else if (bnd < 6'd5) lowacc <= lowacc + {2'd0, n_bar};
                    if (bnd == 6'd5) begin
                        lowprev <= lowacc + {2'd0, n_bar};
                        if ({1'b0, lowacc} + {3'd0, n_bar} >= {1'b0, lowprev} + BEAT_T && pz <= 4'd2) pz <= 4'd8;
                        else if (pz != 4'd0) pz <= pz - 4'd1;
                    end
                    if (bnd == 5'd31) begin est <= E_IDLE; upd <= 1'b0; fh <= 1'b1; fl <= 1'b1; end
                    else bnd <= bnd + 5'd1;
                end
                endcase
            end
            default: begin                                      // E_HIGH / E_LOW：一根柱 12 拍
                t <= t + 4'd1;
                case (t)
                4'd3:  ry2 <= sq;
                4'd5:  fr <= fin_v;
                4'd6:  begin swe <= 1'b1; swa <= sbase + {7'd0, !pp}; swd <= fr; ral <= sq; end
                4'd7:  rah <= sq[6:0];
                4'd8:  fr <= fin_v;
                4'd9:  begin swe <= 1'b1; swa <= sbase + {6'd1, !pp}; swd <= fr;
                             accn <= acc_sum[25] ? {ACCW{1'b1}} : acc_sum[24:0]; end
                4'd10: begin swe <= 1'b1; swa <= sbase + 8'd4; swd <= accn[17:0]; end
                4'd11: begin
                    swe <= 1'b1; swa <= sbase + 8'd5; swd <= {11'd0, accn[24:18]};
                    t <= 0;
                    if (est == E_HIGH && last_hi) begin
                        est <= E_IDLE; pend_s <= 1'b0; xh1 <= xin; xh2 <= xh1; php <= !php; fh <= 1'b0;
                    end else if (est == E_LOW && last_lo) begin
                        est <= E_IDLE; pend_low <= 1'b0; xl1 <= xln; xl2 <= xl1; plo <= !plo; fl <= 1'b0;
                    end else
                        bnd <= bnd + 5'd1;
                end
                default: ;
                endcase
            end
            endcase
        end
    end

`ifdef HDMI_AUDIO_SIM_ASSERT
    // 仿真检查：队列不能满；帧标记必须在柱区开始显示之前处理完
    reg sim_spec_ovf = 1'b0, sim_spec_late = 1'b0;
    always @(posedge clk) begin
        if (rst_n && fq_push && !fq_pop && fq_n == 4'd8 && !sim_spec_ovf) begin
            sim_spec_ovf <= 1'b1;
            $display("[ASSERT] %m: spectrum input queue overflow, t=%0t", $time);
        end
        if (rst_n && upd && raw_y >= V_BLANK + PY0 && raw_y < V_BLANK + PY1 && !sim_spec_late) begin
            sim_spec_late <= 1'b1;
            $display("[ASSERT] %m: spectrum update still running inside the panel rows, t=%0t", $time);
        end
    end
`endif

    // ======================== 显示：音浪（节拍脉冲、流动色带、中心对称） ========================
    // design/wavefx_gen.py 是逐位一致的参考模型
    //   中心对称：结点 K(j)，j = 0..67：K(2..33) = bar[31..0]，K(34..65) = bar[0..31]，其余 0（低频在中间）
    //   H(j) = (K(j-1) + 2K(j) + K(j+1)) >> 1（平滑、高 x2，0~60 行）
    //   第 s 段（x = WX0 + 8s + t）：h = (8 H(s) + (H(s+1) - H(s)) t) >> 3，用累加做（每像素 + Δ），不用乘法
    //   每段开始时从显示表取下一个 K：段 s 里显示表地址 = K(s+3) 的柱号，下一段开头那拍 dq 正好是 K(s+2)
    //   颜色：彩虹色（和全屏模式同一个色轮），hue = ((3|x - 320|) >> 3 + scroll) mod 192，scroll 每帧 +1；脉冲时往白色提亮
    localparam integer WX0 = 52, NSEG = 67, WBASE = 428;
    localparam [7:0]   HUE_N = 8'd192;

    function [23:0] rainbow;                 // 和 fullscreen_viz 一样：色相 0~191
        input [7:0] h;
        reg   [12:0] m;
        reg   [7:0]  t, q;
        begin
            m = h[4:0] * 8'd217;
            t = 8'd38 + m[12:5];
            q = 8'd255 - m[12:5];
            case (h[7:5])
                3'd0:    rainbow = {8'd255, t, 8'd38};
                3'd1:    rainbow = {q, 8'd255, 8'd38};
                3'd2:    rainbow = {8'd38, 8'd255, t};
                3'd3:    rainbow = {8'd38, q, 8'd255};
                3'd4:    rainbow = {t, 8'd38, 8'd255};
                default: rainbow = {8'd255, 8'd38, q};
            endcase
        end
    endfunction
    function [7:0] brighten;                 // c + w/16 x (255 - c)，w = 0~3（只用移位）
        input [7:0] c;
        input [1:0] w;
        reg   [7:0] e;
        begin
            e = ~c;
            brighten = c + (w[1] ? {3'd0, e[7:3]} : 8'd0) + (w[0] ? {4'd0, e[7:4]} : 8'd0);
        end
    endfunction

    // ---- 帧级状态：颜色滚动 scroll（帧起点 +1）；节拍脉冲 pz（帧标记里 bar[0..5] 更新完时算，见引擎） ----
    reg  [7:0]  scroll;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)           scroll <= 8'd0;
        else if (frame_start) scroll <= (scroll == HUE_N - 1) ? 8'd0 : scroll + 8'd1;
    end
    wire [1:0]  pw = pz[3] ? 2'd3 : pz[2:1];                 // min(3, p >> 1)：提亮档

    // ---- 第 0 拍：段号、结点、浪高累加、颜色 ----
    reg  [5:0]  ka, kb;                     // K(s)、K(s+1)（本段开头移位前）
    reg  [5:0]  hn;                         // H(s+1)
    reg  [9:0]  hacc;                       // 8 h
    reg  signed [6:0] dl;                   // H(s+1) - H(s)
    wire [5:0]  knew = (seg0 <= 7'd63) ? dq[5:0] : 6'd0;
    wire [6:0]  hsum = {1'b0, ka} + {kb, 1'b0} + {1'b0, knew};
    wire [5:0]  hnew = hsum[6:1];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ka <= 0; kb <= 0; hn <= 0; hacc <= 0; dl <= 0;
        end else if (!inw0) begin
            ka <= 0; kb <= 0; hn <= 0; hacc <= 0; dl <= 0;
        end else if (t00) begin
            ka   <= kb;
            kb   <= knew;
            hn   <= hnew;
            dl   <= $signed({1'b0, hnew}) - $signed({1'b0, hn});
            hacc <= {1'b0, hn, 3'd0};
        end else begin
            hacc <= hacc + {{3{dl[6]}}, dl};
        end
    end
    wire [9:0]  xc   = raw_x[9:0] - (H_BLANK + 320);          // x - 320（补码）
    wire [9:0]  ax   = xc[9] ? -xc : xc;
    wire [10:0] ax3  = {1'b0, ax} + {ax, 1'b0};
    wire [8:0]  hus  = {1'b0, ax3[10:3]} + {1'b0, scroll};    // 音浪区内 < 2 x 192
    wire [7:0]  hue  = (hus >= {1'b0, HUE_N}) ? hus - {1'b0, HUE_N} : hus[7:0];
    wire [23:0] rb   = rainbow(hue);
    wire [11:0] kk0 = (V_BLANK + WBASE) - raw_y;              // 底边以上第几行
    wire        iny0 = (raw_y <= V_BLANK + WBASE) && (raw_y >= V_BLANK + WBASE - 63);
    reg         w1;
    reg  [5:0]  kk1;
    reg  [23:0] bg1, cc;
    reg         hs1, vs1, de1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w1 <= 0; kk1 <= 0; bg1 <= 0; cc <= 0; hs1 <= 1; vs1 <= 1; de1 <= 0;
            hs_out <= 1; vs_out <= 1; de_out <= 0;
        end else begin
            w1  <= inw0 && iny0;
            kk1 <= kk0[5:0];
            bg1 <= rgb_in;
            cc  <= {brighten(rb[23:16], pw), brighten(rb[15:8], pw), brighten(rb[7:0], pw)};
            hs1 <= hs_in; vs1 <= vs_in; de1 <= de_in;
            hs_out <= hs1; vs_out <= vs1; de_out <= de1;
        end
    end

    // ======================== 第 1 拍：分类、颜色 ========================
    wire [5:0]  h1  = hacc[8:3];
    wire signed [7:0] d = $signed({2'b0, h1}) - $signed({2'b0, kk1});       // 浪顶以下第几行
    wire [5:0]  adl = dl[6] ? -dl[5:0] : dl[5:0];
    wire signed [7:0] ew = 8'sd1 + $signed({6'd0, adl[5:4]}) + $signed({6'd0, pz[3:2]});
    wire [8:0]  lr  = {1'b0, cc[23:16]} + 9'd255, lg = {1'b0, cc[15:8]} + 9'd255, lb = {1'b0, cc[7:0]} + 9'd255;
    wire [23:0] lc  = {lr[8:1], lg[8:1], lb[8:1]};
    wire [4:0]  ph2 = {2'b0, pz[3:1]};                                       // p >> 1
    wire [4:0]  fil = ((d[6:5] != 2'b00) ? 5'd4 : 5'd12 - {2'b0, d[4:2]}) + ph2;   // max(4, 12 - (d >> 2)) + (p >> 1)，d > 0
    reg  [23:0] col_c;
    reg  [4:0]  ab;
    always @* begin
        col_c = cc; ab = 5'd0;
        if (!w1) ab = 5'd0;
        else if (d == 8'sd0) begin col_c = lc; ab = 5'd16; end
        else if (d > 8'sd0 && d <= ew) ab = 5'd16;
        else if (d > ew) ab = fil;
        else if (d == -8'sd1) begin col_c = lc; ab = 5'd6 + ph2; end
        else if (d == -8'sd2) begin col_c = lc; ab = 5'd3 + ph2; end
        else if (d == -8'sd3 && (pz[3] || pz[2])) begin col_c = lc; ab = ph2; end
    end
    wire [9:0] af = ({5'd0, ab} * {5'd0, f16});
    wire [4:0] a  = af[8:4];
    wire signed [8:0]  er = $signed({1'b0, col_c[23:16]}) - $signed({1'b0, bg1[23:16]});
    wire signed [8:0]  eg = $signed({1'b0, col_c[15:8]})  - $signed({1'b0, bg1[15:8]});
    wire signed [8:0]  eb = $signed({1'b0, col_c[7:0]})   - $signed({1'b0, bg1[7:0]});
    wire signed [5:0]  as = $signed({1'b0, a});
    wire signed [13:0] dr = er * as;
    wire signed [13:0] dg = eg * as;
    wire signed [13:0] db = eb * as;
    reg  [11:0] sr2, sg2, sb2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sr2 <= 0; sg2 <= 0; sb2 <= 0;
        end else begin
            sr2 <= $signed({2'b0, bg1[23:16], 4'd0}) + dr;
            sg2 <= $signed({2'b0, bg1[15:8], 4'd0})  + dg;
            sb2 <= $signed({2'b0, bg1[7:0], 4'd0})   + db;
        end
    end

    // ======================== 第 2 拍：输出 ========================
    assign rgb_out = {sr2[11:4], sg2[11:4], sb2[11:4]};
endmodule
