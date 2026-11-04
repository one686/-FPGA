// ---------------------------------------------------------------------------
// sd_sector_adapter.v  —— SD 扇区读握手适配 + 超时看门狗
//
// 作用：把 sd_card_top 的"电平式扇区读"协议
//         sd_sec_read(电平,=1期间读) / sd_sec_read_addr /
//         sd_sec_read_data_valid / sd_sec_read_data / sd_sec_read_end
//       翻译成 fat32_volume_scanner / fat32_file_streamer 使用的
//       "脉冲式"握手协议：
//         sector_req(单拍脉冲, 仅当 !sector_busy 时发出)
//         sector_busy(电平)
//         sector_valid + sector_byte + sector_byte_index(0..511)
//         sector_done(单拍脉冲, 本次传输结束)
//         sector_error(8bit, 与 sector_done 同拍有效, 0=正常)
//
// 时序要点(与 sd_card_sec_read_write 控制器配合验证过)：
//   * 发起读: 把 sd_sec_read 拉高并给地址。控制器在 S_WAIT_READ_WRITE
//     状态看到电平 1 即发起 CMD17 读扇区。
//   * 结束: sd_sec_read_end 拉高 1 拍(S_READ_END 状态), 本模块同拍把
//     sd_sec_read 拉低——必须在控制器回到 S_WAIT_READ_WRITE 之前拉低,
//     否则会误触发同地址重读。控制器路径 S_READ_END -> S_WAIT_READ_WRITE
//     至少 1 拍, 同拍拉低恰好安全。
//   * 新请求: 上层在 sector_done 之后至少 1 拍才发 sector_req,
//     控制器此时已回到 S_WAIT_READ_WRITE, 看到电平 1 立即发起, 无遗漏。
//
// [超时看门狗]：
//   SD 卡被拔出/读卡失败时 sd_sec_read_end 永远不会来, 若不加处理,
//   sector_busy 永远为 1, 上层 FAT32 状态机将永久挂死,
//   上层的"拔卡恢复"也就无从触发。
//   这里在发出 sd_sec_read 后若 SECTOR_TIMEOUT_CYCLES 个时钟内
//   没有等到 sd_sec_read_end, 则:
//     1) 撤销 sd_sec_read (让 sd_card_top 复位其读状态)
//     2) 同拍给出 sector_done=1 且 sector_error=ERR_TIMEOUT
//   上层收到非零 error 后以错误码 0x10 结束本次文件读取,
//   sd_card_bmp 将其显示为细分码 1 并置 need_rescan,
//   随后走"复位SD+重扫"恢复流程(黑屏时自动, 已在显示时等按键)。
//
// 时钟域：全部在 sd_card_clk 域内(板上 50MHz, 与 sd_card_top / FAT32 模块一致)。
// rst_n 低有效, 建议由 ~(rst | sd_rst) 驱动, 保证 SD 软件复位时
// 适配器与 FAT32 模块一起清零, 不会残留半次事务。
// ---------------------------------------------------------------------------
module sd_sector_adapter #(
    parameter [31:0] SECTOR_TIMEOUT_CYCLES = 32'd50_000_000   // 默认值 = 500ms @100MHz; 顶层按 50MHz 传 25_000_000
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 对 FAT32 模块(主机侧) ----
    input  wire        sector_req,        // 单拍脉冲: 请求读一个扇区(仅当 !busy 时发出)
    input  wire [31:0] sector_addr,       // 扇区号 (LBA)
    output reg         sector_busy,       // 电平: 1 = 正在传输
    output wire        sector_valid,      // 电平跟随 sd_card_top 的 data_valid
    output wire [7:0]  sector_byte,
    output wire [8:0]  sector_byte_index, // 0..511
    output reg         sector_done,       // 单拍脉冲: 本次扇区传输结束(或超时)
    output reg  [7:0]  sector_error,      // 与 sector_done 同拍有效

    // ---- 对 sd_card_top(卡片侧) ----
    output reg         sd_sec_read,       // 电平: 读请求(到 end 为止保持为1)
    output reg  [31:0] sd_sec_read_addr,
    input  wire        sd_sec_read_end,
    input  wire        sd_sec_read_data_valid,
    input  wire [7:0]  sd_sec_read_data
);

    localparam [7:0] ERR_NONE    = 8'd0;
    localparam [7:0] ERR_TIMEOUT = 8'h10;

    reg [8:0]  byte_cnt;
    // 看门狗计数器位宽按超时拍数取, 不多占寄存器
    localparam integer TW = $clog2(SECTOR_TIMEOUT_CYCLES + 1);
    reg [TW-1:0] timeout_cnt;

    assign sector_valid      = sd_sec_read_data_valid;
    assign sector_byte       = sd_sec_read_data;
    assign sector_byte_index = byte_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            sector_busy      <= 1'b0;
            sector_done      <= 1'b0;
            sector_error     <= ERR_NONE;
            byte_cnt         <= 9'd0;
            timeout_cnt      <= 0;
        end else begin
            // 默认值: 脉冲信号每拍自动回0
            sector_done  <= 1'b0;
            sector_error <= ERR_NONE;

            if (sector_req && !sector_busy) begin
                // 发起一次新的扇区读
                sd_sec_read_addr <= sector_addr;
                sd_sec_read      <= 1'b1;         // 电平拉高, 等 sd_sec_read_end
                sector_busy      <= 1'b1;
                byte_cnt         <= 9'd0;
                timeout_cnt      <= 0;
            end else if (sector_busy) begin
                if (sd_sec_read_end) begin
                    // 正常结束: 同拍关闭读请求(防止控制器误重读),
                    // 并给出 done(正常)
                    sd_sec_read  <= 1'b0;
                    sector_busy  <= 1'b0;
                    sector_done  <= 1'b1;
                    byte_cnt     <= 9'd0;
                    timeout_cnt  <= 0;
                end else if (sd_sec_read_data_valid) begin
                    // 传输中: 逐字节计数 (仅用于产生 sector_byte_index)
                    if (byte_cnt != 9'd511)
                        byte_cnt <= byte_cnt + 9'd1;
                    // 只要有数据在流动就证明卡还活着, 看门狗清零
                    timeout_cnt <= 0;
                end else begin
                    // 空闲等待: 看门狗计数
                    if (timeout_cnt >= SECTOR_TIMEOUT_CYCLES) begin
                        // 超时: 撤销读请求, 同拍给出 done+error
                        // 注: sd_sec_read_end 与最后字节的 valid 可能同拍,
                        // 若有 valid 则走上面的正常分支, 不会误判超时。
                        sd_sec_read  <= 1'b0;
                        sector_busy  <= 1'b0;
                        sector_done  <= 1'b1;
                        sector_error <= ERR_TIMEOUT;
                        byte_cnt     <= 9'd0;
                        timeout_cnt  <= 0;
                    end else begin
                        timeout_cnt <= timeout_cnt + 1'b1;
                    end
                end
            end
        end
    end

endmodule