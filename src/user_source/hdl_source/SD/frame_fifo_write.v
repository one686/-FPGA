`timescale 1ns/1ps
// frame_fifo_write：整帧缓存写 burst 控制（mem_clk 域）。
//   收到 write_req 后清写 FIFO、锁存块号/缩略图格号，然后在写 FIFO 攒够一个 burst 时发一次 SDRAM 写 burst；
//   可选上下翻转（写地址按行倒排）；同时把缩略图 FIFO 里的字以 burst 写到缩略图区（缩略图优先）。
//   读 burst 进行中（App_rd_busy）不开始写 burst；O_wr_busy 告诉读侧写 burst 进行中或即将开始。
//   整帧和缩略图都写完后给一拍 write_finish。
module frame_fifo_write
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter BURST_SIZE             = 128,
	parameter WRITE_V_FLIP           = 1,      // 1: write image rows upside-down to cancel BMP bottom-up storage
	                                           // 实际是否翻转 = WRITE_V_FLIP & write_v_flip(每张图一个, 倒存的 BMP 不翻)
	parameter FRAME_WIDTH            = 640,
	parameter FRAME_HEIGHT           = 480,
	// 两次写 burst 之间至少隔这么多拍(mem_clk)。默认 8000 拍 = 200us@40MHz。
	//   原因: 缩放后像素一行一行地成批进 FIFO, 若写 burst 一个接一个几乎没有空隙, 板上实测会出现整块 128 字
	//   没写对(白线)和显示读下溢。SDRAM 控制器是加密 IP, 内部不可见; 写 burst 间隔较大时板上一直正常,
	//   这里把最密的情况限制到 200us 一次; 若仍有问题可加大到 32000(800us)。
	//   一帧 1200 个 burst x 200us = 0.24s, 远小于读卡时间(大图几秒), 不影响加载速度。
	parameter BURST_GAP_CYCLES       = 8000,
	// 缩略图通道。第 k 张图(图片序号)的缩略图放在 THUMB_BASE + k x THUMB_WORDS, 按缩略图 FIFO 送来的顺序连续存
	//   (不翻转); 和整帧共用写口: 缩略图 FIFO 攒够一个 burst 就插一个缩略图 burst(优先), burst 长度、间隔和整帧一样。
	//   THUMB_WORDS 必须是 BURST_SIZE 的整数倍(板上 128 x 84 = 42 x 256)。整帧和缩略图都写完才报 write_finish。
	//   THUMB_WORDS = 0: 不写缩略图。
	parameter THUMB_BASE             = 1843200,
	parameter THUMB_WORDS            = 10752
)               
(
	input                            rst,                  
	input                            mem_clk,                    // external memory controller user interface clock
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
	input							 App_rd_busy,
    output							 O_wr_busy,
	
	output 							 App_wr_en,
	output  [ADDR_BITS - 1:0]	 	App_wr_addr,
	/*
    output reg                       wr_burst_req,               // to external memory controller,send out a burst write request  
	output reg[BURST_BITS - 1:0]     wr_burst_len,               // to external memory controller,data length of the burst write request, not bytes 
	output reg[ADDR_BITS - 1:0]      wr_burst_addr,              // to external memory controller,base address of the burst write request 
	input                            wr_burst_data_req,          // from external memory controller,write data request ,before data 1 clock 
	input                            wr_burst_finish,            // from external memory controller,burst write finish
	*/
	input                            write_req,                  // data write module write request,keep '1' until read_req_ack = '1'
	output reg                       write_req_ack,              // data write module write request response
	output                           write_finish,               // data write module write request finish
	input[2:0]                       write_addr_index,           // 写哪块整帧缓存(0~5), 基地址 = 块号 x 一帧字数(FRAME_WIDTH x FRAME_HEIGHT)
	input[3:0]                       write_thumb_index,          // 缩略图写第几格(= 图片序号 0~15), 和 write_addr_index 一样在写请求前就稳定
	input                            write_v_flip,               // 本张图是否上下翻转(写时钟域电平, 读完 BMP 头才知道, 第一个 burst 前一定已稳定)
	input[ADDR_BITS - 1:0]           write_len,                  // data write module write request data length
	output reg                       fifo_aclr,                  // to fifo asynchronous clear
	input[9:0]                      rdusedw,                    // from fifo read used words
	input[9:0]                       thumb_level,                // 缩略图 FIFO 里的字数
	output                           thumb_rd_en,                // 读缩略图 FIFO(= 缩略图 burst 里的 App_wr_en)
	output reg                       thumb_sel                   // 1 = 这个 burst 写缩略图(写数据从缩略图 FIFO 取)
);
localparam ONE                       = 256'd1;                   //256 bit '1'   you can use ONE[n-1:0] for n bit '1'
localparam ZERO                      = 256'd0;                   //256 bit '0'

// Vertical flip address mapping for 640x480 frame buffer.
// Input stream order remains unchanged; only SDRAM write addresses are remapped:
// stream row 0 -> memory row FRAME_HEIGHT-1, stream row 1 -> memory row FRAME_HEIGHT-2, ...
localparam [ADDR_BITS - 1:0] VFLIP_FIRST_ADDR_OFFSET = (FRAME_HEIGHT - 1) * FRAME_WIDTH;
localparam [ADDR_BITS - 1:0] VFLIP_LINE_JUMP         = (FRAME_WIDTH * 2) - 1;
localparam integer           FRAME_WORDS             = FRAME_WIDTH * FRAME_HEIGHT;   // 一块整帧缓存的字数
// 计数器按实际范围取位宽:
//   帧长就是 FRAME_WORDS(顶层和各 tb 的 write_len 都等于宽 x 高), 写计数用"剩余整帧 burst 数" fb_rem 倒计数,
//   要求 FRAME_WORDS 是 BURST_SIZE 的整数倍(640x480 / 256 = 1200)。write_len 端口保留, 只在仿真里检查。
localparam integer           FRAME_BURSTS            = FRAME_WORDS / BURST_SIZE;
localparam integer           RW                      = $clog2(FRAME_BURSTS + 1);
localparam integer           XW                      = (FRAME_WIDTH > 1) ? $clog2(FRAME_WIDTH) : 1;
localparam integer           TCW                     = (THUMB_WORDS > 0) ? $clog2(THUMB_WORDS + 1) : 1;
localparam [XW-1:0]          VFLIP_LINE_LAST_X       = FRAME_WIDTH - 1;

//write state machine code
localparam S_IDLE                    = 0;                        //idle state,waiting for write
localparam S_ACK                     = 1;                        //written request response
localparam S_CHECK_FIFO              = 2;                        //check the FIFO status, ensure that there is enough space to burst write
localparam S_WRITE_BURST             = 3;                        //begin a burst write
localparam S_WRITE_BURST_END         = 4;                        //a burst write complete
localparam S_END                     = 5;                        //a frame of data is written to complete


reg                                 write_req_d0;                //asynchronous write request, synchronize to 'mem_clk' clock domain,first beat
reg                                 write_req_d1;                //the second
reg                                 write_req_d2;                //third,Why do you need 3 ? Here's the design habit
reg[RW - 1:0]                       fb_rem;                      // 这一帧还要写几个整帧 burst
reg[2:0]                            write_addr_index_d0;
reg[2:0]                            write_addr_index_d1;
reg[3:0]                            write_thumb_index_d0;
reg[3:0]                            write_thumb_index_d1;
reg[3:0]                            state;                       //state machine
reg [ADDR_BITS - 1:0]	 App_wr_addr_r;
reg [XW - 1:0]                        wr_x;                       // current x position in one input row
// 是否翻转要读完 BMP 头才知道(比写请求晚), 所以起始地址不在 S_ACK 定, 而在第一个 burst 开始时定。
// 第一个 burst 要 FIFO 里攒够 256 个像素, 那时头早就读完了, write_v_flip 已稳定了几千拍。
reg [ADDR_BITS - 1:0]                 base_addr;                  // S_ACK 锁存的整帧缓存块基地址
reg [2:0]                             vflip_sync;
reg                                   vflip_r;                    // 本帧锁存的翻转标志
wire                                  first_burst;                // 见 into_burst 后面

reg [BURST_BITS - 1:0]				burst_cnt;
wire								wr_burst_finish;

reg App_wr_en_r;
reg App_wr_en_d0;

reg [15:0] gap_cnt;                                             // 写 burst 间隔倒计时
// 缩略图通道
reg [TCW - 1:0]                       thumb_cnt;                  // 已写的缩略图字数
reg [ADDR_BITS - 1:0]                 t_addr;                     // 缩略图写地址(S_ACK 定起点, 之后每写一个字加 1)
wire frame_go;                                                  // 整帧还没写完, FIFO 够一个 burst(或剩下的全到齐)
wire thumb_go;                                                  // 缩略图还没写完, 缩略图 FIFO 够一个 burst
wire into_burst;
// 开始整帧 burst 的条件: FIFO 里多于一个 burst 的字; 只剩最后 1 个 burst 时恰好够一个 burst 即可
assign frame_go = (fb_rem != 0) && ((fb_rem == 1 && rdusedw >= BURST_SIZE) || rdusedw > BURST_SIZE);
assign thumb_go = (THUMB_WORDS != 0) && (thumb_cnt != THUMB_WORDS) && (thumb_level >= BURST_SIZE);
assign into_burst = ((frame_go || thumb_go) && ~App_rd_busy
                     && (gap_cnt == 16'd0));  // 离上一次写 burst 结束不够 BURST_GAP_CYCLES 也不进入
// 读侧在突发时不进入写 burst
assign first_burst = (state == S_CHECK_FIFO) && into_burst && !thumb_go && (fb_rem == FRAME_BURSTS)
                     && (write_req_d2 == 1'b0);                  // 与状态机"进入第一个整帧 burst"的条件相同

assign App_wr_addr = thumb_sel ? t_addr : App_wr_addr_r;          // 缩略图 burst 用缩略图地址
// 缩略图地址(+1)和整帧地址(+1 / 翻转时行尾跳回上一行开头 -VFLIP_LINE_JUMP)共用一个加法器,
//   两种 burst 互斥(thumb_sel 在一个 burst 内不变)
wire [ADDR_BITS - 1:0] addr_step = (!thumb_sel && vflip_r && wr_x == VFLIP_LINE_LAST_X) ? (~VFLIP_LINE_JUMP + 1'b1) : ONE[ADDR_BITS - 1:0];
wire [ADDR_BITS - 1:0] addr_nx   = App_wr_addr + addr_step;
//assign O_wr_busy = (state != S_IDLE || (S_IDLE && write_req_d2));
assign O_wr_busy = (state == S_WRITE_BURST || (state == S_CHECK_FIFO && into_burst));//在突发，或者将要突发，则显示busy
assign wr_burst_finish = (burst_cnt >= BURST_SIZE);
assign write_finish = (state == S_END) ? 1'b1 : 1'b0;            //write finish at state 'S_END'
assign App_wr_en = App_wr_en_d0;
assign thumb_rd_en = App_wr_en_d0 && thumb_sel;


always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_req_d0    <=  1'b0;
		write_req_d1    <=  1'b0;
		write_req_d2    <=  1'b0;
		write_addr_index_d0    <=  3'd0;
		write_addr_index_d1    <=  3'd0;
		write_thumb_index_d0   <=  4'd0;
		write_thumb_index_d1   <=  4'd0;
		vflip_sync             <=  3'b000;
	end
	else
	begin
		vflip_sync      <=  {vflip_sync[1:0], write_v_flip};
		write_req_d0    <=  write_req;
		write_req_d1    <=  write_req_d0;
		write_req_d2    <=  write_req_d1;
		write_addr_index_d0 <= write_addr_index;
		write_addr_index_d1 <= write_addr_index_d0;
		write_thumb_index_d0 <= write_thumb_index;
		write_thumb_index_d1 <= write_thumb_index_d0;
	end 
end
always @(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		burst_cnt <= ZERO[BURST_BITS - 1:0];
		App_wr_addr_r <= ZERO[ADDR_BITS - 1:0];
		App_wr_en_d0 <= 1'b0;
		wr_x <= {XW{1'b0}};
		base_addr <= ZERO[ADDR_BITS - 1:0];
		vflip_r <= 1'b0;
		t_addr <= ZERO[ADDR_BITS - 1:0];
	end
	else begin
	
		if(state == S_CHECK_FIFO)
			burst_cnt <= ZERO[BURST_BITS - 1:0];
		else if(App_wr_en)
			burst_cnt <= burst_cnt + 1'b1;
		else
			burst_cnt <= burst_cnt;
		//
		// 缩略图地址
		if(state == S_ACK)
			t_addr <= THUMB_BASE + write_thumb_index_d1 * THUMB_WORDS;  // 第 k 张图(图片序号)的缩略图格
		else if(App_wr_en && thumb_sel)
			t_addr <= addr_nx;
		//
		if(state == S_ACK)
			begin
				wr_x <= {XW{1'b0}};
				base_addr <= write_addr_index_d1 * FRAME_WORDS;   // 块 k 基地址 = k x 一帧字数
			end
		else if(first_burst)
			begin
				// 第一个 burst 开始前一拍(App_wr_en 还要再过 2 拍才出来): 定起始地址和翻转
				vflip_r       <= WRITE_V_FLIP && vflip_sync[2];
				App_wr_addr_r <= (WRITE_V_FLIP && vflip_sync[2]) ? (base_addr + VFLIP_FIRST_ADDR_OFFSET) : base_addr;
			end
		else if(App_wr_en && !thumb_sel)
			begin
				if(vflip_r)
				begin
					if(wr_x == VFLIP_LINE_LAST_X)
					begin
						wr_x <= {XW{1'b0}};
						App_wr_addr_r <= addr_nx;
					end
					else
					begin
						wr_x <= wr_x + 1'b1;
						App_wr_addr_r <= addr_nx;
					end
				end
				else
					App_wr_addr_r <= addr_nx;
			end
		else
			App_wr_addr_r <= App_wr_addr_r;
		//
		// 每个 burst 恰好 BURST_SIZE 个字(帧长是 BURST_SIZE 的整数倍), 不需要再和剩余长度比较
		if(App_wr_en_r && burst_cnt + App_wr_en < BURST_SIZE)begin
            App_wr_en_d0 <= 1'b1;
        end
		else
			App_wr_en_d0 <= 1'b0;
	
	end		
end
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		
		//wr_burst_addr <= ZERO[ADDR_BITS - 1:0];
		//wr_burst_req <= 1'b0;
		App_wr_en_r <= 1'b0;
		
		fb_rem <= {RW{1'b0}};
		fifo_aclr <= 1'b0;
		write_req_ack <= 1'b0;
		//wr_burst_len <= ZERO[BURST_BITS - 1:0];
		gap_cnt <= 16'd0;
		thumb_cnt <= {TCW{1'b0}};
		thumb_sel <= 1'b0;
	end
	else begin
		// 写 burst 结束时装入间隔, 之后每拍减 1
		if (state == S_WRITE_BURST && wr_burst_finish == 1'b1)
			gap_cnt <= BURST_GAP_CYCLES;
		else if (gap_cnt != 16'd0)
			gap_cnt <= gap_cnt - 16'd1;
		case(state)
			//idle state,waiting for write write_req_d2 == '1' goto the 'S_ACK'
			S_IDLE:
			begin
				if(write_req_d2 == 1'b1 && Sdr_init_done)
				begin
					state <= S_ACK;
				end
				write_req_ack <= 1'b0;
			end
			//'S_ACK' state completes the write request response, the FIFO reset, the address latch, and the data length latch
			S_ACK:
			begin
				//after write request revocation(write_req_d2 == '0'),goto 'S_CHECK_FIFO',write_req_ack goto '0'
				if(write_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					write_req_ack <= 1'b0;
				end
				else
				begin
					//write request response
					write_req_ack <= 1'b1;
					//FIFO reset
					fifo_aclr <= 1'b1;
					//base address is computed from write_addr_index_d1 (see base_addr above); legacy code below kept commented out
					/*
					if(write_addr_index_d1 == 2'd0)
						wr_burst_addr <= write_addr_0;
					else if(write_addr_index_d1 == 2'd1)
						wr_burst_addr <= write_addr_1;
					else if(write_addr_index_d1 == 2'd2)
						wr_burst_addr <= write_addr_2;
					else if(write_addr_index_d1 == 2'd3)
						wr_burst_addr <= write_addr_3;
					*/
				end
				//write counters reset
				fb_rem <= FRAME_BURSTS;
				thumb_cnt <= {TCW{1'b0}};
				thumb_sel <= 1'b0;
			end
			S_CHECK_FIFO:
			begin
				//if there is a write request at this time, enter the 'S_ACK' state
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the FIFO space is a burst write request, goto burst write state
				else if(into_burst)
				begin
					state <= S_WRITE_BURST;
					//wr_burst_len <= BURST_SIZE[BURST_BITS - 1:0];
					//wr_burst_req <= 1'b1;
					App_wr_en_r <= 1'b1;
					thumb_sel <= thumb_go;                       // 缩略图优先
				end
			end 
			
			S_WRITE_BURST:
			begin
				//burst finish
				if(wr_burst_finish == 1'b1)
				begin
					App_wr_en_r <= 1'b0;
					state <= S_WRITE_BURST_END;
					//write counter + burst length
					if(thumb_sel)
						thumb_cnt <= thumb_cnt + BURST_SIZE;
					else
						fb_rem <= fb_rem - 1'b1;
					//the next burst write address is generated
					//wr_burst_addr <= wr_burst_addr + BURST_SIZE[ADDR_BITS - 1:0];
				end     
			end
			S_WRITE_BURST_END:
			begin
				//if there is a write request at this time, enter the 'S_ACK' state
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				//if the write counter value is less than the frame length, continue writing,
				//otherwise the writing is complete
				else if(fb_rem != 0 || thumb_cnt != THUMB_WORDS)   // 缩略图也写完才结束
				begin
					state <= S_CHECK_FIFO;
				end
				else
				begin
					state <= S_END;
				end
			end
			S_END:
			begin
				state <= S_IDLE;
			end
			default:
				state <= S_IDLE;
		endcase
	end
end
`ifdef OSD_SIM
// fb_rem 倒计数的前提检查(仿真): 帧长 = 宽 x 高, 且是 burst 长度的整数倍
initial begin
	if (FRAME_WORDS % BURST_SIZE != 0) begin
		$display("[ASSERT] %m: FRAME_WIDTH x FRAME_HEIGHT 不是 BURST_SIZE 的整数倍");
		$finish;
	end
end
always @(posedge mem_clk)
	if (!rst && state == S_ACK && write_req_d2 && write_len != FRAME_WORDS)
		$display("[ASSERT] %m: write_len = %0d, 应等于 FRAME_WIDTH x FRAME_HEIGHT = %0d, t=%0t", write_len, FRAME_WORDS, $time);
`endif
endmodule
