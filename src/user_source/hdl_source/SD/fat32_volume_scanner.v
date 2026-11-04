`timescale 1ns/1ps

// fat32_volume_scanner：FAT32 卷扫描器（SD/控制时钟域）。
//   读 0 号扇区（MBR/引导扇区）定位分区和 FAT32 参数（每簇扇区数、FAT 起点、数据区起点、最大簇号），
//   然后沿 FAT 簇链扫描整个根目录，登记图片（PIC 号码 .BMP）、歌单（.WAV）和字幕（SUB.BMP），
//   最后对登记的图片逐张做文件头预检查。扇区读取通过 sector_req / sector_byte 接口由外部读卡模块完成。
//
// WAV：扫描根目录时顺便登记 .WAV 文件（歌单，最多 WAV_MAX 首，按目录顺序；默认 12 首）
//   认法：8.3 短名扩展名是 "WAV"（长文件名/中文名的文件也有这样的短名）；
//   跳过：已删除(首字节 E5)、"."/".." 、长文件名项(属性 0F)、文件夹、卷标、隐藏、系统文件、空文件。
//   歌单存在一张小表里（簇号 + 字节数），wav_rd_addr 给序号，下一拍 wav_rd_cluster/size 出数据。
//
// 图片：只认 PIC + 数字 .BMP，按数字从小到大排；取号码最小的 BMP_MAX 张好图（sd_card_bmp 给 12 = 顶层 PIC_MAX，
//   最大 16），后面的号码不再查、不认。6 块整帧缓存按需换入由 sd_card_bmp 负责。
//   认法：短名前 3 个字符是 PIC（大小写都认），后面紧跟 1~5 位数字、再往后只能是空格，数值 1~99，
//         扩展名 BMP；过滤规则和歌单一样（已删除、文件夹、卷标、隐藏、系统、空文件都不认）。
//         PIC1、PIC01、PIC001 都是 1 号；同一个号码出现两次只认目录里先出现的那个。
//         不认：PIC.BMP、PICA.BMP、PIC1A.BMP、PIC0.BMP、PIC100.BMP、"PIC1 副本.bmp"（短名 PIC1~1.BMP）等。
//   登记：按号码直接放进号码表（第 n 号放第 n-1 格），不用排序；号码可以不连续。
//   文件头预检查（目录扫完后，按号码从小到大，每张图读第 1 个扇区）：
//     规则与 bmp_scaler 的头检查一致：签名 BM、平面数 1、24/32 位、压缩 0（32 位可以是 BITFIELDS 标准掩码）、
//     宽 1~MAX_W、高 1~MAX_H、像素偏移 >= 54；另外要求 目录里的字节数 >= 像素偏移 + 行字节数 x 高（截断的文件不认）。
//     起始簇号不在 2~max_cluster 的直接算坏。通过的才进图片表，所以 bmp_count = 能正常读的张数，
//     没通过的张数在 bmp_bad_count（计入 OSD 状态条的坏图张数）。好图够 BMP_MAX 张就停（后面的号码不查，也不算坏图）。
//     查不出来的（簇链坏、内容读错）还是在加载时发现，由 sd_card_bmp 按坏图处理。
//   bmp_rd_addr 给序号（0 = 号码最小的好图），第 2 拍 bmp_rd_cluster/size 出数据（扫描结束后才有效）。
//   一张好图都没有：报 ERR_RESOURCE_MISSING(0x22)。
// 字幕：根目录里的 SUB.BMP（短名 "SUB     BMP"，大小写都认，过滤规则同上）登记起始簇号和字节数，
//   只认目录里第一个，字节数 1~32767（make_subtitle.py 生成的最大约 16.4 KB）；文件内容由 sd_card_bmp + sub_loader 读。
//   省 LUT：簇号、字节数放进号码表第 SUB_SLOT = 127 格（号码 1~99 只用 0~98 格），bmp_rd_sub = 1 时图片表读口读这一格，
//   所以读文件模块不用另外切换簇号 / 字节数；sub_found = 登记过了。号码最大 99 由上面的认名规则保证；号码上限若改大，SUB_SLOT 要避开。
//   为了把图和歌找全，一直扫到目录结束标记（或簇链结束），99 个号码和歌单都满了才提前结束。
module fat32_volume_scanner #(
    parameter [3:0] WAV_MAX = 4'd12,    // 歌单最多 12 首（wav_count 4 位）
    parameter [4:0] BMP_MAX = 5'd16,    // 图片最多几张，1~16（bmp_count 5 位；实际由 sd_card_bmp 传入 12）
    parameter integer MAX_W = 1920,     // 与 bmp_scaler 一致
    parameter integer MAX_H = 1920
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scan_start,
    output wire        scan_busy,
    output reg         scan_done,
    output reg  [7:0]  fs_error,
    output reg  [31:0] partition_lba,
    output reg  [7:0]  sectors_per_cluster,
    output reg  [31:0] fat_start_lba,
    output reg  [31:0] data_start_lba,
    output reg  [31:0] max_cluster,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error,
    // 图片表（按 PIC 号码排好序，只有通过文件头检查的）
    output reg  [4:0]  bmp_count,
    output reg  [6:0]  bmp_bad_count,   // 文件头检查没通过的张数
    input  wire [3:0]  bmp_rd_addr,
    output wire [31:0] bmp_rd_cluster,
    output wire [31:0] bmp_rd_size,
    // WAV 歌单
    output reg  [3:0]  wav_count,
    input  wire [3:0]  wav_rd_addr,
    output reg  [31:0] wav_rd_cluster,
    output reg  [31:0] wav_rd_size,
    // 字幕 SUB.BMP（扫描结束后有效）
    output reg         sub_found,      // 有 SUB.BMP
    input  wire        bmp_rd_sub      // 1 = 图片表读口读 SUB.BMP 的簇号 / 字节数（不看 bmp_rd_addr），第 2 拍出数据
);

    localparam ERR_NONE            = 8'h00;
    localparam ERR_SECTOR          = 8'h10;
    localparam ERR_BOOT_SIGNATURE  = 8'h20;
    localparam ERR_UNSUPPORTED_BPB = 8'h21;
    localparam ERR_RESOURCE_MISSING= 8'h22;

    localparam ST_IDLE       = 5'd0;
    localparam ST_READ0_REQ  = 5'd1;
    localparam ST_READ0_WAIT = 5'd2;
    localparam ST_BOOT_PARSE = 5'd3;
    localparam ST_BOOT_REQ   = 5'd4;
    localparam ST_BOOT_WAIT  = 5'd5;
    localparam ST_ROOT_REQ   = 5'd6;
    localparam ST_ROOT_WAIT  = 5'd7;
    localparam ST_DIR_SCAN   = 5'd8;
    localparam ST_FINISH     = 5'd9;
    localparam ST_BOOT_LAYOUT= 5'd10;
    localparam ST_BOOT_ROOT  = 5'd11;
    localparam ST_FAT_REQ    = 5'd12;   // 读FAT扇区取根目录下一簇
    localparam ST_FAT_WAIT   = 5'd13;
    // 号码表清空 + 文件头预检查
    localparam ST_CLEAR      = 5'd14;   // 号码表 128 格的"已登记"位清零
    localparam ST_PC_NEXT    = 5'd15;   // 下一个号码（这一拍读号码表）
    localparam ST_PC_GET     = 5'd16;   // 拿到表项：没登记 / 簇号不对 / 去读第 1 个扇区
    localparam ST_PC_REQ     = 5'd17;
    localparam ST_PC_WAIT    = 5'd18;   // 字节到达时直接解析文件头
    localparam ST_PC_CHK     = 5'd19;   // 行字节数 x 高（乘法）
    localparam ST_PC_CHK2    = 5'd20;   // 头字段 + 字节数够不够，登记或算坏
    localparam ST_PC_WAIT2   = 5'd21;   // 等登记写完
    localparam ST_LBA        = 5'd22;   // 簇号 -> 扇区号（三处共用一份，逐拍移位），算完去 lba_ret

    // 根目录簇链最多跟进的簇数(保险丝, 防坏FAT死循环)
    localparam [4:0] ROOT_CHAIN_MAX = 5'd16;

    reg [4:0] state;
    reg [7:0] sector_buffer [0:511];
    reg [31:0] root_cluster;
    reg [31:0] total_sectors;
    reg [31:0] fat_sectors;
    reg [7:0] fat_count;
    reg [15:0] reserved_sectors;
    reg [31:0] data_sector_count;
    reg [31:0] fat_span;
    reg [2:0] sectors_per_cluster_shift;
    reg [7:0] dir_name0;            // 短名第 0 个字节（过滤用）
    reg [7:0] dir_name8;            // 扩展名 3 个字节
    reg [7:0] dir_name9;
    reg [7:0] dir_name10;
    reg [15:0] dir_cluster_high;
    reg [15:0] dir_cluster_low;
    reg [31:0] dir_file_size;
    // 根目录多扇区/多簇遍历
    reg [7:0]  root_sec_cnt;        // 当前根簇内已读扇区数(0基)
    wire [7:0] root_sec_nx = root_sec_cnt + 8'd1;
    reg [31:0] root_cur_cluster;    // 当前正在扫描的根目录簇
    reg [4:0]  root_chain_steps;    // 已跟进的簇数(保险丝计数)
    reg [6:0]  fat_ent_idx;         // FAT条目在当前FAT扇区内的序号（字节偏移 = 序号 x 4）
    reg [31:0] root_next_cluster;   // 从FAT表抓到的下一根目录簇
    reg        dir_end_seen;        // 遇到0x00结束目录项(目录到此为止)

    // 目录项第 31 字节（最后一个字节）这一拍登记；字节数为 0 的空文件不登记
    wire        dir_last_byte = sector_valid && (state == ST_ROOT_WAIT) && (sector_byte_index[4:0] == 5'd31);
    wire [31:0] dir_size_now  = {sector_byte, dir_file_size[23:0]};

    // WAV 歌单
    reg        wav_match;           // 本目录项是要登记的 .WAV
    (* ram_style = "bram" *) reg [63:0] wav_tab [0:15];     // {簇号, 字节数}
    wire       wav_we = dir_last_byte && wav_match && (wav_count < WAV_MAX) && (dir_size_now != 32'd0);
    always @(posedge clk) begin
        if (wav_we)
            wav_tab[wav_count] <= {dir_cluster_high, dir_cluster_low, dir_size_now};
        {wav_rd_cluster, wav_rd_size} <= wav_tab[wav_rd_addr];
    end

    // ======================== 图片表 ========================
    // 号码表 num_tab[n-1] = {已登记, 起始簇号, 字节数}（BRAM，128 格，每次扫描先清"已登记"位）。
    // 图片表也放在同一块 BRAM 里（省 LUT，不另做 16 选 1 的触发器表）：文件头检查通过时把这张图的记录 rec_q 原样复制到
    //   第 PIC_SLOT0 + 第几张 格（100~115；号码 1~99 只用 0~98 格，SUB.BMP 在 127 格），外面读第 PIC_SLOT0 + 序号 格。
    //   读口时序：第 1 拍寄存地址（rd_q），第 2 拍 rec_q 出数据。复制用的是唯一的写口（这时不扫目录、不清表）。
    // 短名第 3~7 个字节边到边算号码 pn_v（有效数字超过两位记不合格），第 11 个字节（属性）定是否要登记；
    // 号码表读口在扫目录期间一直读 pn_v-1 这一格，到第 31 个字节时早已读出，看"已登记"位去重。
    reg        bmp_match;           // 本目录项是要登记的 PIC 号码 .BMP
    reg        sb_ok, sub_match;    // 短名到目前为止是 "SUB     "；本目录项是 SUB.BMP
    reg        pn_ok, pn_end;       // 名字到目前为止符合 PIC+数字；数字已经结束（后面只能是空格）
    reg  [6:0] pn_v;                // 号码（0~99）
    reg  [6:0] reg_cnt;             // 登记了几个号码
    reg  [6:0] pc_k;                // 清表 / 预检查用的格号
    reg        perm_we;             // 这一拍把 rec_q 复制到第 PIC_SLOT0 + bmp_count 格，下一拍 bmp_count + 1
    (* ram_style = "bram" *) reg [64:0] num_tab [0:127];
    reg  [6:0] rd_q;                // 图片表读地址（寄存一拍）
    reg  [64:0] rec_q;
    localparam [6:0] SUB_SLOT = 7'd127;   // SUB.BMP 在号码表的格号（号码 n 在第 n-1 格，号码最大 99）
    localparam [6:0] PIC_SLOT0 = 7'd100;  // 图片表第 0 张在号码表的格号（到 115，BMP_MAX <= 16）
    // 只认目录里第一个 SUB.BMP（第 127 格"已登记"位去重，和 PIC 号码一样）
    wire       sub_we = dir_last_byte && sub_match && !rec_q[64] && (dir_size_now != 32'd0) && (dir_size_now[31:15] == 17'd0);
    wire [6:0] pn_m1   = pn_v - 7'd1;
    wire       bmp_we  = dir_last_byte && bmp_match && !rec_q[64] && (dir_size_now != 32'd0);
    wire       pc_sel  = (state == ST_CLEAR) || (state >= ST_PC_NEXT);
    wire       tab_we  = bmp_we || sub_we || (state == ST_CLEAR) || perm_we;
    wire [6:0] tab_wa  = perm_we ? PIC_SLOT0 + {3'd0, bmp_count[3:0]} : (state == ST_CLEAR) ? pc_k : sub_match ? SUB_SLOT : pn_m1;
    wire [64:0] tab_wd = perm_we ? rec_q : (state == ST_CLEAR) ? 65'd0 : {1'b1, dir_cluster_high, dir_cluster_low, dir_size_now};
    wire [6:0] tab_ra  = (state == ST_ROOT_WAIT) ? (sb_ok ? SUB_SLOT : pn_m1) : pc_sel ? pc_k : rd_q;
    always @(posedge clk) begin
        if (tab_we)
            num_tab[tab_wa] <= tab_wd;
        rec_q <= num_tab[tab_ra];
        rd_q <= bmp_rd_sub ? SUB_SLOT : PIC_SLOT0 + {3'd0, bmp_rd_addr};   // SUB.BMP 在号码表第 SUB_SLOT 格
    end
    assign bmp_rd_cluster = rec_q[63:32];
    assign bmp_rd_size    = rec_q[31:0];

    // 文件头预检查：第 1 个扇区的字节到达时直接解析（规则同 bmp_scaler 的 S_HDR/S_CHK）
    reg        h_ok, h_hi0, h_hi1, h_is32, h_comp3, h_mask;
    reg [15:0] h_off, h_w, h_h;
    reg [23:0] h_need;              // 行字节数 x 高（<= 7680 x 1920）
    wire [15:0] h_abs = h_hi1 ? (~h_h + 16'd1) : h_h;
    wire [12:0] h_w3  = {2'b00, h_w[10:0]} + {1'b0, h_w[10:0], 1'b0} + 13'd3;
    wire [12:0] h_row = h_is32 ? {h_w[10:0], 2'b00} : {h_w3[12:2], 2'b00};   // 行尾补齐到 4 字节
    wire        h_fields_ok = h_ok && (h_hi0 || h_hi1) && (h_off >= 16'd54) &&
                              (h_w != 16'd0) && (h_w <= MAX_W) && (h_abs != 16'd0) && (h_abs <= MAX_H) &&
                              (!h_comp3 || (h_is32 && h_off >= 16'd66 && h_mask));
    wire [8:0]  hb = sector_byte_index;
    wire [31:0] pc_cluster = rec_q[63:32];
    wire        pc_cl_ok   = (pc_cluster >= 32'd2) && (pc_cluster <= max_cluster);

    // 簇号 -> 扇区号 = data_start_lba + ((簇号 - 2) << sectors_per_cluster_shift)
    //   根目录第 1 簇、根目录后续簇、预检查三处共用：进 ST_LBA 前把 (簇号 - 2) 装进 lba_acc，
    //   ST_LBA 每拍左移 1 位（每簇扇区数 1~128，最多 7 拍），移完写 sector_lba，转到 lba_ret
    wire [31:0] lba_src = (state == ST_PC_GET)   ? pc_cluster :
                          (state == ST_FAT_WAIT) ? root_next_cluster : root_cluster;
    wire [31:0] lba_m2  = lba_src - 32'd2;
    reg  [31:0] lba_acc;
    reg  [2:0]  lba_n;
    reg  [4:0]  lba_ret;

    assign scan_busy = (state != ST_IDLE);

    function [15:0] le16;
        input integer offset;
        begin
            le16 = {sector_buffer[offset+1], sector_buffer[offset]};
        end
    endfunction

    function [31:0] le32;
        input integer offset;
        begin
            le32 = {sector_buffer[offset+3], sector_buffer[offset+2],
                    sector_buffer[offset+1], sector_buffer[offset]};
        end
    endfunction

    function valid_spc;
        input [7:0] value;
        begin
            valid_spc = value != 0 && ((value & (value - 1'b1)) == 0);
        end
    endfunction

    function [2:0] spc_shift;
        input [7:0] value;
        begin
            case (value)
                8'd2:   spc_shift = 3'd1;
                8'd4:   spc_shift = 3'd2;
                8'd8:   spc_shift = 3'd3;
                8'd16:  spc_shift = 3'd4;
                8'd32:  spc_shift = 3'd5;
                8'd64:  spc_shift = 3'd6;
                8'd128: spc_shift = 3'd7;
                default: spc_shift = 3'd0;
            endcase
        end
    endfunction

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    // 根目录到此为止：一个号码都没登记报 0x22，否则按号码逐个查文件头
    task directory_finished;
        begin
            if (reg_cnt == 7'd0)
                finish_with_error(ERR_RESOURCE_MISSING);
            else begin
                pc_k <= 7'd0;
                state <= ST_PC_NEXT;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            scan_done <= 1'b0;
            fs_error <= ERR_NONE;
            partition_lba <= 32'd0;
            sectors_per_cluster <= 8'd0;
            fat_start_lba <= 32'd0;
            data_start_lba <= 32'd0;
            max_cluster <= 32'd0;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            root_cluster <= 32'd0;
            total_sectors <= 32'd0;
            fat_sectors <= 32'd0;
            fat_count <= 8'd0;
            reserved_sectors <= 16'd0;
            data_sector_count <= 32'd0;
            fat_span <= 32'd0;
            sectors_per_cluster_shift <= 3'd0;
            dir_name0 <= 8'd0;
            dir_name8 <= 8'd0;
            dir_name9 <= 8'd0;
            dir_name10 <= 8'd0;
            dir_cluster_high <= 16'd0;
            dir_cluster_low <= 16'd0;
            dir_file_size <= 32'd0;
            root_sec_cnt <= 8'd0;
            root_cur_cluster <= 32'd0;
            root_chain_steps <= 5'd0;
            fat_ent_idx <= 7'd0;
            root_next_cluster <= 32'd0;
            dir_end_seen <= 1'b0;
            wav_match <= 1'b0;
            wav_count <= 4'd0;
            sb_ok <= 1'b0; sub_match <= 1'b0; sub_found <= 1'b0;
            bmp_match <= 1'b0;
            bmp_count <= 5'd0;
            bmp_bad_count <= 7'd0;
            pn_ok <= 1'b0;
            pn_end <= 1'b0;
            pn_v <= 7'd0;
            reg_cnt <= 7'd0;
            pc_k <= 7'd0;
            perm_we <= 1'b0;
            h_ok <= 1'b0; h_hi0 <= 1'b0; h_hi1 <= 1'b0; h_is32 <= 1'b0; h_comp3 <= 1'b0; h_mask <= 1'b0;
            h_off <= 16'd0; h_w <= 16'd0; h_h <= 16'd0; h_need <= 24'd0;
            lba_acc <= 32'd0; lba_n <= 3'd0; lba_ret <= ST_IDLE;
        end else begin
            if (wav_we)
                wav_count <= wav_count + 4'd1;
            if (sub_we)
                sub_found <= 1'b1;
            if (bmp_we)
                reg_cnt <= reg_cnt + 7'd1;
            perm_we <= 1'b0;
            if (perm_we)
                bmp_count <= bmp_count + 5'd1;
            scan_done <= 1'b0;
            sector_req <= 1'b0;
            if (sector_valid && state != ST_ROOT_WAIT && state != ST_PC_WAIT)
                sector_buffer[sector_byte_index] <= sector_byte;

            if (sector_valid && state == ST_ROOT_WAIT) begin
                case (sector_byte_index[4:0])
                    5'd0: begin
                        dir_name0 <= sector_byte;
                        wav_match <= 1'b0;
                        bmp_match <= 1'b0;
                        sub_match <= 1'b0;
                        sb_ok     <= ((sector_byte | 8'h20) == "s");
                        pn_ok  <= ((sector_byte | 8'h20) == "p");
                        pn_end <= 1'b0;
                        pn_v   <= 7'd0;
                    end
                    5'd1: begin pn_ok <= pn_ok && ((sector_byte | 8'h20) == "i"); sb_ok <= sb_ok && ((sector_byte | 8'h20) == "u"); end
                    5'd2: begin pn_ok <= pn_ok && ((sector_byte | 8'h20) == "c"); sb_ok <= sb_ok && ((sector_byte | 8'h20) == "b"); end
                    5'd3, 5'd4, 5'd5, 5'd6, 5'd7: begin
                        sb_ok <= sb_ok && (sector_byte == " ");
                        // PIC 后面：1~5 位数字，然后只能是空格；有效数字超过两位不合格
                        if (pn_end)
                            pn_ok <= pn_ok && (sector_byte == " ");
                        else if (sector_byte >= "0" && sector_byte <= "9") begin
                            if (pn_v >= 7'd10)
                                pn_ok <= 1'b0;
                            else
                                pn_v <= {pn_v[3:0], 3'b000} + {pn_v[5:0], 1'b0} + {3'd0, sector_byte[3:0]};
                        end else if (sector_byte == " " && sector_byte_index[4:0] != 5'd3)
                            pn_end <= 1'b1;
                        else
                            pn_ok <= 1'b0;
                    end
                    5'd8:  dir_name8 <= sector_byte;
                    5'd9:  dir_name9 <= sector_byte;
                    5'd10: dir_name10 <= sector_byte;
                    5'd11: begin
                        // 普通文件：属性字节 & 1E = 0 -> 不是隐藏/系统/卷标/文件夹/长文件名(0F)，
                        //           且不是已删除(E5)、"."/".."、空项(00)
                        if ((sector_byte & 8'h1E) == 8'h00 && dir_name0 != 8'h00 &&
                            dir_name0 != 8'hE5 && dir_name0 != 8'h2E) begin
                            if (dir_name8 == "W" && dir_name9 == "A" && dir_name10 == "V")
                                wav_match <= 1'b1;
                            // PIC1~PIC99（号码重复的在第 31 字节去掉）
                            if ((dir_name8 & 8'hDF) == "B" && (dir_name9 & 8'hDF) == "M" &&
                                (dir_name10 & 8'hDF) == "P" && pn_ok && pn_v != 7'd0)
                                bmp_match <= 1'b1;
                            if ((dir_name8 & 8'hDF) == "B" && (dir_name9 & 8'hDF) == "M" &&
                                (dir_name10 & 8'hDF) == "P" && sb_ok)
                                sub_match <= 1'b1;
                        end
                        // 目录结束标记 = 首字节0x00且属性字节0x00
                        // (整项全0; 短名首字节0xE5是删除项, LFN首字节必为序号, 都不会误触发)
                        else if (sector_byte == 8'h00 && dir_name0 == 8'h00)
                            dir_end_seen <= 1'b1;
                    end
                    5'd20: dir_cluster_high[7:0] <= sector_byte;
                    5'd21: dir_cluster_high[15:8] <= sector_byte;
                    5'd26: dir_cluster_low[7:0] <= sector_byte;
                    5'd27: dir_cluster_low[15:8] <= sector_byte;
                    5'd28: dir_file_size[7:0] <= sector_byte;
                    5'd29: dir_file_size[15:8] <= sector_byte;
                    5'd30: dir_file_size[23:16] <= sector_byte;
                    5'd31: dir_file_size[31:24] <= sector_byte;
                    default: begin end
                endcase
            end

            // 预检查读文件第 1 个扇区, 边到边解析 BMP 头
            if (sector_valid && state == ST_PC_WAIT) begin
                case (hb)
                    9'd0:  h_ok <= h_ok && (sector_byte == 8'h42);           // 'B'
                    9'd1:  h_ok <= h_ok && (sector_byte == 8'h4D);           // 'M'
                    9'd10: h_off[7:0]  <= sector_byte;
                    9'd11: h_off[15:8] <= sector_byte;
                    9'd12, 9'd13, 9'd20, 9'd21, 9'd27, 9'd29, 9'd31, 9'd32, 9'd33:
                           h_ok <= h_ok && (sector_byte == 8'h00);           // 偏移/宽的高位、平面数/位深/压缩的高字节
                    9'd18: h_w[7:0]  <= sector_byte;
                    9'd19: h_w[15:8] <= sector_byte;
                    9'd22: h_h[7:0]  <= sector_byte;
                    9'd23: h_h[15:8] <= sector_byte;
                    9'd24, 9'd25: begin
                           h_hi0 <= h_hi0 && (sector_byte == 8'h00);
                           h_hi1 <= h_hi1 && (sector_byte == 8'hFF);
                    end
                    9'd26: h_ok <= h_ok && (sector_byte == 8'd1);            // 平面数 1
                    9'd28: begin                                             // 位深 24 / 32
                           h_ok   <= h_ok && ((sector_byte == 8'd24) || (sector_byte == 8'd32));
                           h_is32 <= (sector_byte == 8'd32);
                    end
                    9'd30: begin                                             // 压缩 0 / 3(BITFIELDS)
                           h_ok    <= h_ok && ((sector_byte == 8'd0) || (sector_byte == 8'd3));
                           h_comp3 <= (sector_byte == 8'd3);
                    end
                    default: ;
                endcase
                // BITFIELDS 掩码（偏移 54~65）：R=00FF0000 G=0000FF00 B=000000FF
                if (hb >= 9'd54 && hb <= 9'd65)
                    h_mask <= h_mask && (sector_byte == (((hb == 9'd56) || (hb == 9'd59) || (hb == 9'd62)) ? 8'hFF : 8'h00));
            end

            // 读FAT扇区期间, 按字节偏移抓取根目录下一簇号(4字节小端)
            if (sector_valid && state == ST_FAT_WAIT) begin
                if (sector_byte_index[8:2] == fat_ent_idx)
                    case (sector_byte_index[1:0])
                        2'd0: root_next_cluster[7:0]   <= sector_byte;
                        2'd1: root_next_cluster[15:8]  <= sector_byte;
                        2'd2: root_next_cluster[23:16] <= sector_byte;
                        default: root_next_cluster[31:24] <= sector_byte;
                    endcase
            end

            case (state)
                ST_IDLE: if (scan_start) begin
                    fs_error <= ERR_NONE;
                    partition_lba <= 32'd0;
                    bmp_count <= 5'd0;
                    bmp_bad_count <= 7'd0;
                    reg_cnt <= 7'd0;
                    wav_count <= 4'd0;
                    sub_found <= 1'b0;
                    state <= ST_READ0_REQ;
                end
                ST_READ0_REQ: if (!sector_busy) begin
                    sector_lba <= 32'd0;
                    sector_req <= 1'b1;
                    state <= ST_READ0_WAIT;
                end
                ST_READ0_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (sector_buffer[510] != 8'h55 || sector_buffer[511] != 8'haa)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else if (sector_buffer[0] == 8'heb || sector_buffer[0] == 8'he9) begin
                        partition_lba <= 32'd0;
                        state <= ST_BOOT_PARSE;
                    end else if (sector_buffer[450] == 8'h0b || sector_buffer[450] == 8'h0c) begin
                        partition_lba <= le32(454);
                        sector_lba <= le32(454);
                        state <= ST_BOOT_REQ;
                    end else begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end
                end
                ST_BOOT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_BOOT_WAIT;
                end
                ST_BOOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else if (sector_buffer[510] != 8'h55 || sector_buffer[511] != 8'haa)
                        finish_with_error(ERR_BOOT_SIGNATURE);
                    else
                        state <= ST_BOOT_PARSE;
                end
                ST_BOOT_PARSE: begin
                    if (le16(11) != 16'd512 || !valid_spc(sector_buffer[13]) ||
                        le16(14) == 0 || (sector_buffer[16] != 1 && sector_buffer[16] != 2) ||
                        le16(17) != 0 || le16(22) != 0 || le32(32) == 0 ||
                        le32(36) == 0 || le32(44) < 2) begin
                        finish_with_error(ERR_UNSUPPORTED_BPB);
                    end else begin
                        sectors_per_cluster <= sector_buffer[13];
                        reserved_sectors <= le16(14);
                        fat_count <= sector_buffer[16];
                        total_sectors <= le32(32);
                        fat_sectors <= le32(36);
                        root_cluster <= le32(44);
                        sectors_per_cluster_shift <= spc_shift(sector_buffer[13]);
                        fat_span <= sector_buffer[16] == 2 ? (le32(36) << 1) : le32(36);
                        fat_start_lba <= partition_lba + le16(14);
                        state <= ST_BOOT_LAYOUT;
                    end
                end
                ST_BOOT_LAYOUT: begin
                    data_start_lba <= partition_lba + reserved_sectors + fat_span;
                    data_sector_count <= total_sectors - reserved_sectors - fat_span;
                    state <= ST_BOOT_ROOT;
                end
                ST_BOOT_ROOT: begin
                    max_cluster <= (data_sector_count >> sectors_per_cluster_shift) + 1'b1;
                    // 初始化根目录遍历(簇内扇区计数/链保险丝/结束标记),
                    // 扫描期间累加的图片表/歌单计数在这里清零
                    bmp_count <= 5'd0;
                    bmp_bad_count <= 7'd0;
                    reg_cnt <= 7'd0;
                    wav_count <= 4'd0;
                    sub_found <= 1'b0;
                    root_sec_cnt <= 8'd0;
                    root_chain_steps <= 5'd0;
                    dir_end_seen <= 1'b0;
                    root_cur_cluster <= root_cluster;
                    lba_acc <= lba_m2;                         // root_cluster - 2
                    lba_n <= sectors_per_cluster_shift;
                    lba_ret <= ST_CLEAR;
                    pc_k <= 7'd0;
                    state <= ST_LBA;
                end
                ST_LBA: begin
                    if (lba_n != 3'd0) begin
                        lba_acc <= {lba_acc[30:0], 1'b0};
                        lba_n <= lba_n - 3'd1;
                    end else begin
                        sector_lba <= data_start_lba + lba_acc;
                        state <= lba_ret;
                    end
                end
                // 号码表清空（128 拍）
                ST_CLEAR: begin
                    pc_k <= pc_k + 7'd1;
                    if (pc_k == 7'd127)
                        state <= ST_ROOT_REQ;
                end
                ST_ROOT_REQ: if (!sector_busy) begin
                    // 注意: 不能在这里清计数, 续读下一扇区时会抹掉已找到的结果
                    sector_req <= 1'b1;
                    state <= ST_ROOT_WAIT;
                end
                ST_ROOT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else
                        state <= ST_DIR_SCAN;
                end
                ST_DIR_SCAN: begin
                    // 99 个号码、歌单都满了, 立即收工（没满就继续扫, 把 .BMP / .WAV 找全）
                    if (reg_cnt == 7'd99 && wav_count == WAV_MAX)
                        directory_finished;
                    // 遇到目录结束标记(0x00项): 后面不会再有文件了
                    else if (dir_end_seen)
                        directory_finished;
                    // 本簇还有扇区没读, 续读下一个
                    //   cnt < spc - 1 等价于 cnt + 1 != spc（0 <= cnt <= spc - 1，spc 1~128，8 位不溢出）；
                    //   这里 sector_lba 恒等于 本簇首扇区 + cnt（ST_LBA 同时写 sector_lba 和清 cnt，读扇区不改它），续读直接 + 1
                    else if (root_sec_nx != sectors_per_cluster) begin
                        root_sec_cnt <= root_sec_nx;
                        sector_lba <= sector_lba + 32'd1;
                        state <= ST_ROOT_REQ;
                    end
                    // 本簇扫完: 跟FAT链去下一个根目录簇(带保险丝)
                    else if (root_chain_steps < ROOT_CHAIN_MAX) begin
                        fat_ent_idx <= root_cur_cluster[6:0];
                        sector_lba <= fat_start_lba + (root_cur_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end
                    // 保险丝烧断: 链太长(坏FAT), 有几张算几张
                    else
                        directory_finished;
                end
                ST_FAT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_FAT_WAIT;
                end
                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else begin
                        root_chain_steps <= root_chain_steps + 5'd1;
                        if (root_next_cluster >= 32'h0ffffff8 ||
                            root_next_cluster < 32'd2 ||
                            root_next_cluster > max_cluster) begin
                            // 链尾(EOC)或坏链: 根目录到此为止
                            directory_finished;
                        end else begin
                            // 链上下一个根目录簇, 从新簇第0扇区继续扫
                            root_cur_cluster <= root_next_cluster;
                            root_sec_cnt <= 8'd0;
                            dir_end_seen <= 1'b0;
                            lba_acc <= lba_m2;                 // root_next_cluster - 2
                            lba_n <= sectors_per_cluster_shift;
                            lba_ret <= ST_ROOT_REQ;
                            state <= ST_LBA;
                        end
                    end
                end
                // 文件头预检查（按号码 1~99），好图够 BMP_MAX 张就停
                ST_PC_NEXT: begin
                    if (pc_k == 7'd99 || bmp_count == BMP_MAX) begin
                        if (bmp_count == 5'd0)
                            finish_with_error(ERR_RESOURCE_MISSING);   // 登记了但全是坏图
                        else
                            state <= ST_FINISH;
                    end else
                        state <= ST_PC_GET;                            // 这一拍读 num_tab[pc_k]
                end
                ST_PC_GET: begin
                    if (!rec_q[64]) begin                              // 这个号码没有图
                        pc_k <= pc_k + 7'd1;
                        state <= ST_PC_NEXT;
                    end else if (!pc_cl_ok) begin                      // 起始簇号不对: 坏图
                        bmp_bad_count <= bmp_bad_count + 7'd1;
                        pc_k <= pc_k + 7'd1;
                        state <= ST_PC_NEXT;
                    end else begin
                        lba_acc <= lba_m2;                             // pc_cluster - 2
                        lba_n <= sectors_per_cluster_shift;
                        lba_ret <= ST_PC_REQ;
                        h_ok <= 1'b1; h_hi0 <= 1'b1; h_hi1 <= 1'b1; h_is32 <= 1'b0; h_comp3 <= 1'b0; h_mask <= 1'b1;
                        h_off <= 16'd0; h_w <= 16'd0; h_h <= 16'd0;
                        state <= ST_LBA;
                    end
                end
                ST_PC_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_PC_WAIT;
                end
                ST_PC_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else
                        state <= ST_PC_CHK;
                end
                ST_PC_CHK: begin
                    h_need <= h_row * h_abs[10:0];                     // 13 x 11 位
                    state <= ST_PC_CHK2;
                end
                ST_PC_CHK2: begin
                    if (h_fields_ok && (rec_q[31:0] >= {8'd0, h_need} + {16'd0, h_off}))
                        perm_we <= 1'b1;                               // 下一拍复制 rec_q（= 第 pc_k 格）到图片表, 再下一拍 bmp_count + 1
                    else
                        bmp_bad_count <= bmp_bad_count + 7'd1;
                    state <= ST_PC_WAIT2;
                end
                ST_PC_WAIT2: begin
                    pc_k <= pc_k + 7'd1;
                    state <= ST_PC_NEXT;
                end
                ST_FINISH: begin
                    scan_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
