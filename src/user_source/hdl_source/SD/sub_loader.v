`timescale 1ns/1ps

// sub_loader：从卡上读取字幕条。SUB.BMP 字节流 -> 字幕条 RAM（osd/sub_strip_ram.v 的写口）
//   文件由 PC 工具 design/make_subtitle.py 生成：4 位调色板 BMP，高 40，宽 8~816 且是 8 的倍数，
//   每行正好 宽/2 字节（没有行尾补齐）
//   检查：签名 BM、像素偏移 64~32767、宽、高 = 40（4 个字节）、位数 = 4、压缩 = 0；
//   任一项不对就不写，ok 保持 0（显示内置默认标语，不报错）
//   调色板不读（FPGA 用固定调色板，见 osd_engine.v 的 sub_pal）
//   像素字节按文件顺序写到地址 0 起（行从下往上），写完最后一个字节 ok = 1；
//   top = 最后一行（= 画面第 0 行）的起始地址 = 39 x 行字节数，w = 宽（像素）
//   start 这一拍 ok 清零；ok = 0 时 w / top 可能在变，显示一方只在 ok 同步过去以后才用它们
//   字节在文件中的位置直接用读文件模块给的 in_off（和 in_valid 同拍），本模块不另设计数器；
//   in_byte 本身是寄存器输出，因此写口直接用输入这一拍，不另打一拍（节省 LUT）。
//   时钟域：SD 控制时钟（50MHz）。
module sub_loader (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // 开始读 SUB.BMP（单拍）
    input  wire        in_valid,
    input  wire [7:0]  in_byte,
    input  wire [14:0] in_off,         // 本字节在文件里的位置（SUB.BMP 不超过 32767 字节，扫描器已检查）
    output wire        we,
    output reg  [13:0] waddr,
    output wire [7:0]  wdata,
    output reg         ok,
    output reg  [9:0]  w,
    output reg  [13:0] top
);
    reg [15:0] pix_off;
    reg [15:0] w16;
    reg        hok;                    // 文件头到目前为止都对
    reg        pix;                    // 已到像素区（已经写过第一个像素字节）
    reg [8:0]  col;                    // 行内第几个字节
    reg [5:0]  row;                    // 第几行（文件顺序）
    wire [8:0] wb = w16[9:1];          // 每行字节数
    wire       w_ok = (w16[15:10] == 6'd0) && (w16[2:0] == 3'd0) && (w16[9:3] != 7'd0) && (w16[9:0] <= 10'd816);
    wire       at_pix = (in_off == pix_off[14:0]) && (pix_off[15] == 1'b0) && (pix_off[14:6] != 9'd0);
    assign we    = in_valid && (pix || at_pix) && hok && w_ok && !ok;
    assign wdata = in_byte;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pix_off <= 0; w16 <= 0; hok <= 1'b0; pix <= 1'b0;
            col <= 0; row <= 0; waddr <= 0;
            ok <= 1'b0; w <= 0; top <= 0;
        end else if (start) begin
            hok <= 1'b1; pix <= 1'b0; col <= 0; row <= 0; waddr <= 0; ok <= 1'b0;
        end else if (in_valid) begin
            if (!pix) begin
                case (in_off)
                    15'd0:  hok <= hok && (in_byte == "B");
                    15'd1:  hok <= hok && (in_byte == "M");
                    15'd10: pix_off[7:0]  <= in_byte;
                    15'd11: pix_off[15:8] <= in_byte;
                    15'd12, 15'd13, 15'd20, 15'd21, 15'd23, 15'd24, 15'd25, 15'd29, 15'd30, 15'd31, 15'd32, 15'd33:
                            hok <= hok && (in_byte == 8'd0);
                    15'd18: w16[7:0]  <= in_byte;
                    15'd19: w16[15:8] <= in_byte;
                    15'd22: hok <= hok && (in_byte == 8'd40);
                    15'd28: hok <= hok && (in_byte == 8'd4);
                    default: begin end
                endcase
            end
            if (we) begin
                pix   <= 1'b1;
                waddr <= waddr + 14'd1;
                if (!pix) w <= w16[9:0];
                if (row == 6'd39 && col == 9'd0) top <= waddr;
                if (col == wb - 9'd1) begin
                    col <= 0;
                    row <= row + 6'd1;
                    if (row == 6'd39) ok <= 1'b1;               // 最后一个字节
                end else
                    col <= col + 9'd1;
            end
        end
    end
endmodule
