// seg_scan：8 位数码管动态扫描（左起第 1~8 个 = seg_sel[0]~seg_sel[7] = SEL0~SEL7）
//   1~2  当前第几张图（1~99，高位的 0 不亮；0 = 没有图，两位都熄灭）
//   3~4  好图总张数（同上）
//   5~6  当前第几首歌（只在播 WAV 时亮）
//   7    错误码（正常熄灭），同时有几个错误时按优先级只显示一个，显示方式区分来源：
//          读卡/FAT 细分码    数字常亮
//          显示通路错误码     数字 + 小数点
//          HDMI 音频错误码    数字 + 小数点闪烁
//          音乐（WAV）错误码  数字闪烁
//   8    状态字母，优先级 L 正在加载 > A 自动轮播 > P 音乐暂停；显示 L/A 时音乐也暂停则小数点亮
// 段码低电平点亮（{点, g~a}）。位选：手册写高电平有效，但本板实测低电平选通，
//   由 SEL_ON 参数决定。每位开头先全灭一小段（BLANK_CYCLES），避免换位时串影。
// 三组两位数分时共用一个"除以 10"和一个译码器；输入都是慢信号，video_clk 域来的错误码是粘滞电平，直接用。
module seg_scan(
	input            clk,
	input            rst_n,
	output reg [7:0] seg_sel,      // 位选，[0] = 左起第 1 个
	output reg [7:0] seg_data,     // 段码，[7] = 小数点
	input      [6:0] pic_idx,      // 第几张图 1~99，0 = 没有
	input      [6:0] pic_total,    // 好图张数 0~99
	input            song_on,      // 正在播 WAV
	input      [6:0] song_idx,     // 第几首 1~99
	input      [3:0] fs_err,       // 读卡/FAT 细分码，0 = 没有
	input      [3:0] video_err,    // 显示通路错误码
	input      [3:0] audio_err,    // HDMI 音频错误码
	input      [2:0] mus_err,      // 音乐错误码
	input            loading,
	input            auto_play,
	input            paused
);
parameter SCAN_FREQ    = 200;        // 8 位扫一遍的频率
parameter CLK_FREQ     = 50000000;
parameter BLANK_CYCLES = 1000;       // 每位开头全灭 20us@50MHz
parameter BLINK_FRAMES = 50;         // 闪烁：每 50 遍（0.25s）翻转一次
parameter [0:0] SEL_ON = 1'b0;

localparam integer SCAN_COUNT = CLK_FREQ / (SCAN_FREQ * 8) - 1;
localparam integer STW = $clog2(SCAN_COUNT + 1);

reg [STW-1:0] scan_timer;
reg [2:0]     scan_sel;
reg [5:0]     blink_cnt;
reg           blink;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		scan_timer <= 0;
		scan_sel   <= 3'd0;
		blink_cnt  <= 6'd0;
		blink      <= 1'b0;
	end else if (scan_timer == SCAN_COUNT) begin       // scan_timer 从 0 计到 SCAN_COUNT 即回零，不会超过，用 == 比较即可
		scan_timer <= 0;
		scan_sel   <= scan_sel + 3'd1;
		if (scan_sel == 3'd7) begin
			if (blink_cnt == BLINK_FRAMES - 1) begin
				blink_cnt <= 6'd0;
				blink     <= ~blink;
			end else
				blink_cnt <= blink_cnt + 6'd1;
		end
	end else
		scan_timer <= scan_timer + 1'b1;
end

// ---- 两位数：按位置选数，超过 99 显示 99 ----
reg  [6:0] num;
always @* begin
	case (scan_sel[2:1])
		2'd0:    num = pic_idx;
		2'd1:    num = pic_total;
		default: num = song_on ? song_idx : 7'd0;
	endcase
end
wire [6:0]  num_c = (num > 7'd99) ? 7'd99 : num;
// 二进制转 BCD 用"移位加 3"（double dabble），只用加法和比较，不占乘法器（DSP）；0~99 已穷举验证
function [7:0] dd7;               // 7 位 -> BCD（只用到 0~99）
	input [6:0] v;
	integer k;
	reg [14:0] t;
	begin
		t = {8'd0, v};
		for (k = 0; k < 7; k = k + 1) begin
			if (t[10:7]  >= 4'd5) t[10:7]  = t[10:7]  + 4'd3;
			if (t[14:11] >= 4'd5) t[14:11] = t[14:11] + 4'd3;
			t = t << 1;
		end
		dd7 = t[14:7];
	end
endfunction
wire [7:0]  num_b = dd7(num_c);
wire [3:0]  tens  = num_b[7:4];
wire [3:0]  ones  = num_b[3:0];

// ---- 错误码：按优先级选一个 ----
reg  [3:0] err_v;
reg        err_dp, err_blank;
always @* begin
	err_v = fs_err; err_dp = 1'b0; err_blank = 1'b0;
	if (fs_err != 4'd0) begin
	end else if (video_err != 4'd0) begin
		err_v = video_err; err_dp = 1'b1;
	end else if (audio_err != 4'd0) begin
		err_v = audio_err; err_dp = blink;
	end else if (mus_err != 3'd0) begin
		err_v = {1'b0, mus_err}; err_blank = blink;
	end
end
wire err_any = (fs_err != 4'd0) || (video_err != 4'd0) || (audio_err != 4'd0) || (mus_err != 3'd0);

// ---- 一个译码器 ----
reg  [3:0] dec_in;
always @* begin
	case (scan_sel)
		3'd0, 3'd2, 3'd4: dec_in = tens;
		3'd6:             dec_in = err_v;
		default:          dec_in = ones;
	endcase
end
wire [6:0] dec_seg;
seg_decoder u_dec(.bin_data(dec_in), .seg_data(dec_seg));

localparam [6:0] SEG_OFF = 7'b111_1111,
                 SEG_L   = 7'b100_0111,
                 SEG_A   = 7'b000_1000,
                 SEG_P   = 7'b000_1100;

reg [7:0] seg_next;
always @* begin
	case (scan_sel)
		3'd0, 3'd2, 3'd4: seg_next = {1'b1, (tens == 4'd0) ? SEG_OFF : dec_seg};
		3'd1, 3'd3, 3'd5: seg_next = {1'b1, (num_c == 7'd0) ? SEG_OFF : dec_seg};
		3'd6:             seg_next = (!err_any || err_blank) ? 8'hff : {~err_dp, dec_seg};
		default:          seg_next = loading   ? {~paused, SEG_L} :
		                             auto_play ? {~paused, SEG_A} :
		                             paused    ? {1'b1, SEG_P}    : 8'hff;
	endcase
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		seg_sel  <= {8{~SEL_ON}};
		seg_data <= 8'hff;
	end else begin
		seg_data <= seg_next;
		seg_sel  <= (scan_timer < BLANK_CYCLES) ? {8{~SEL_ON}} :
		            ({8{~SEL_ON}} ^ (8'd1 << scan_sel));
	end
end

endmodule
