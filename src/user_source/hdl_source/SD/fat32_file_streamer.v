`timescale 1ns/1ps

// fat32_file_streamer：按 FAT32 簇链顺序读出一个文件的全部字节
//   file_start 时锁存起始簇和文件长度；逐簇算出数据扇区 LBA，逐扇区经 sd_sector_adapter 读取，
//   每个有效字节给出 file_valid / file_byte / file_byte_offset / file_remaining；
//   一簇读完后读 FAT 表取下一簇号，直到文件长度读满，最后再确认簇链确实以 EOC 结束。
//   错误码 fs_error（与 file_done 同拍有效）：0x10 扇区读错/超时，0x23 簇链成环或长度与簇链不符，
//   0x24 簇号越界，0x25 簇链提前结束，0x26 簇链长度超过 MAX_VISITED_CLUSTERS。
//   hold = 1 时暂缓请求下一个数据扇区（下游缓冲的反压）。
//   时钟域：SD 控制时钟（50MHz）。
module fat32_file_streamer #(
    parameter integer MAX_VISITED_CLUSTERS = 8192
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire        hold,               // 1 = 暂缓请求下一个数据扇区(读图的缓冲快满了); 读歌的实例接 0
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] data_start_lba,
    input  wire [31:0] max_cluster,
    output wire        file_busy,
    output reg         file_done,
    output reg         file_valid,
    output reg  [7:0]  file_byte,
    output reg  [31:0] file_byte_offset,
    output wire [31:0] file_remaining,     // 本字节之后文件还剩几个字节(与 file_valid 同拍有效)
    output reg  [7:0]  fs_error,
    output reg         sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam ERR_NONE              = 8'h00;
    localparam ERR_SECTOR            = 8'h10;
    localparam ERR_FAT_LOOP          = 8'h23;
    localparam ERR_CLUSTER_RANGE     = 8'h24;
    localparam ERR_EARLY_EOF         = 8'h25;
    localparam ERR_VISITED_EXHAUSTED = 8'h26;

    localparam ST_IDLE       = 4'd0;
    localparam ST_DATA_REQ   = 4'd1;
    localparam ST_DATA_WAIT  = 4'd2;
    localparam ST_FAT_REQ    = 4'd3;
    localparam ST_FAT_WAIT   = 4'd4;
    localparam ST_FAT_PARSE  = 4'd5;
    localparam ST_LBA        = 4'd6;   // 簇号 -> 扇区号, 每拍左移 1 位(省掉桶形移位器)
    localparam ST_FINISH     = 4'd7;

    localparam integer CHAIN_COUNT_WIDTH = $clog2(MAX_VISITED_CLUSTERS + 1);

    reg [3:0] state;
    reg [31:0] current_cluster;
    reg [31:0] next_cluster;
    reg [31:0] remaining_bytes;
    reg [7:0] sector_in_cluster;
    reg [6:0] fat_entry_idx;     // FAT 条目在扇区内的序号（字节偏移 = 序号 x 4）
    reg [7:0] fat_byte0, fat_byte1, fat_byte2, fat_byte3;
    reg [CHAIN_COUNT_WIDTH-1:0] chain_count;
    reg final_chain_check;

    assign file_busy = (state != ST_IDLE);
    assign file_remaining = remaining_bytes;

    // "簇号 x 每簇扇区数" 用移位实现（每簇扇区数一定是 2 的幂，扫描器已检查），
    // 结果与乘法相同但不占 DSP（图片、音乐两个读文件实例同时存在，DSP 不够用）。
    // 不用一拍移完的桶形移位器, 而是在 ST_LBA 里每拍左移 1 位(最多 7 拍, 每簇只做一次)
    function [2:0] spc_log2;
        input [7:0] v;
        begin
            case (v)
                8'd2:   spc_log2 = 3'd1;
                8'd4:   spc_log2 = 3'd2;
                8'd8:   spc_log2 = 3'd3;
                8'd16:  spc_log2 = 3'd4;
                8'd32:  spc_log2 = 3'd5;
                8'd64:  spc_log2 = 3'd6;
                8'd128: spc_log2 = 3'd7;
                default: spc_log2 = 3'd0;
            endcase
        end
    endfunction
    wire [2:0] spc_sh = spc_log2(sectors_per_cluster);
    reg  [2:0] sh_cnt;

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    task advance_to_next_cluster;
        input [31:0] cluster_value;
        begin
            chain_count <= chain_count + 1'b1;
            current_cluster <= cluster_value;
            sector_lba <= cluster_value - 2;
            sh_cnt <= spc_sh;
            state <= ST_LBA;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            file_done <= 1'b0;
            file_valid <= 1'b0;
            file_byte <= 8'd0;
            file_byte_offset <= 32'd0;
            fs_error <= ERR_NONE;
            sector_req <= 1'b0;
            sector_lba <= 32'd0;
            current_cluster <= 32'd0;
            next_cluster <= 32'd0;
            remaining_bytes <= 32'd0;
            sector_in_cluster <= 8'd0;
            fat_entry_idx <= 7'd0;
            fat_byte0 <= 8'd0;
            fat_byte1 <= 8'd0;
            fat_byte2 <= 8'd0;
            fat_byte3 <= 8'd0;
            chain_count <= 0;
            final_chain_check <= 1'b0;
            sh_cnt <= 3'd0;
        end else begin
            file_done <= 1'b0;
            file_valid <= 1'b0;
            sector_req <= 1'b0;

            if (state == ST_DATA_WAIT && sector_valid && remaining_bytes != 0) begin
                file_valid <= 1'b1;
                file_byte <= sector_byte;
                file_byte_offset <= file_size - remaining_bytes;
                remaining_bytes <= remaining_bytes - 1'b1;
            end

            if (state == ST_FAT_WAIT && sector_valid) begin
                if (sector_byte_index[8:2] == fat_entry_idx)
                    case (sector_byte_index[1:0])
                        2'd0: fat_byte0 <= sector_byte;
                        2'd1: fat_byte1 <= sector_byte;
                        2'd2: fat_byte2 <= sector_byte;
                        default: fat_byte3 <= sector_byte;
                    endcase
            end

            case (state)
                ST_IDLE: if (file_start) begin
                    fs_error <= ERR_NONE;
                    file_byte_offset <= 32'd0;
                    remaining_bytes <= file_size;
                    sector_in_cluster <= 8'd0;
                    chain_count <= 1;
                    final_chain_check <= 1'b0;
                    if (file_size == 0) begin
                        state <= ST_FINISH;
                    end else if (first_cluster < 2 || first_cluster > max_cluster ||
                                 sectors_per_cluster == 0) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else begin
                        current_cluster <= first_cluster;
                        sector_lba <= first_cluster - 2;
                        sh_cnt <= spc_sh;
                        state <= ST_LBA;
                    end
                end
                ST_LBA: begin
                    if (sh_cnt != 3'd0) begin
                        sector_lba <= {sector_lba[30:0], 1'b0};
                        sh_cnt <= sh_cnt - 3'd1;
                    end else begin
                        sector_lba <= sector_lba + data_start_lba;
                        state <= ST_DATA_REQ;
                    end
                end
                ST_DATA_REQ: if (!sector_busy && !hold) begin
                    sector_req <= 1'b1;
                    state <= ST_DATA_WAIT;
                end
                ST_DATA_WAIT: if (sector_done) begin
                    if (sector_error != 0) begin
                        finish_with_error(ERR_SECTOR);
                    end else if (remaining_bytes == 0 ||
                                 (remaining_bytes == 1 && sector_valid)) begin
                        final_chain_check <= 1'b1;
                        sector_in_cluster <= 8'd0;
                        fat_entry_idx <= current_cluster[6:0];
                        sector_lba <= fat_start_lba + (current_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end else if (sector_in_cluster + 1'b1 < sectors_per_cluster) begin
                        sector_in_cluster <= sector_in_cluster + 1'b1;
                        sector_lba <= sector_lba + 1'b1;
                        state <= ST_DATA_REQ;
                    end else begin
                        sector_in_cluster <= 8'd0;
                        fat_entry_idx <= current_cluster[6:0];
                        sector_lba <= fat_start_lba + (current_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end
                end
                ST_FAT_REQ: if (!sector_busy) begin
                    sector_req <= 1'b1;
                    state <= ST_FAT_WAIT;
                end
                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else
                        state <= ST_FAT_PARSE;
                end
                ST_FAT_PARSE: begin
                    next_cluster <= {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0};
                    if (final_chain_check) begin
                        final_chain_check <= 1'b0;
                        if ({fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} >= 28'hffffff8)
                            state <= ST_FINISH;
                        else if ({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} < 2 ||
                                 {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} > max_cluster)
                            finish_with_error(ERR_CLUSTER_RANGE);
                        else
                            finish_with_error(ERR_FAT_LOOP);
                    end else if ({fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} >= 28'hffffff8) begin
                        finish_with_error(ERR_EARLY_EOF);
                    end else if ({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} < 2 ||
                                 {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} > max_cluster) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else if (chain_count >= MAX_VISITED_CLUSTERS) begin
                        finish_with_error(ERR_VISITED_EXHAUSTED);
                    end else if ({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} == current_cluster) begin
                        finish_with_error(ERR_FAT_LOOP);
                    end else begin
                        advance_to_next_cluster({4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0});
                    end
                end
                ST_FINISH: begin
                    file_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule