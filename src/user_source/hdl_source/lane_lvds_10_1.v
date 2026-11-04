`timescale 1ns/1ps




// ===========================================================================
// TMDS 10:1 串行器（像素时钟 25MHz -> 串行时钟 125MHz，ODDR 双沿输出，每拍 2 位）
//
// tmds_load_gen：产生"把 10 位像素数据装进移位寄存器"的串行时钟域单拍信号，4 条通道共用。
//   像素时钟每拍翻转 S_tog，串行时钟（125MHz，相位 90°，上升沿在 2/10/18/26/34ns）检测到翻转后再等 2 拍装数据。
//   像素时钟与串行时钟在约束里是互斥的，这条跨域路径不做时序检查，因此装载时刻必须对布线延迟不敏感：
//     2ns 沿检测到 -> 26ns 装，10ns 沿检测到 -> 34ns 装，都在本像素稳定区内（下一个像素 40ns 才变）；
//     翻转信号布线延迟 < 约 9ns 都正确（约束里另外限制这段线 <= 6ns）。
//   若装载时刻落在像素边沿附近，装进去的 10 位会一半新一半旧 -> TMDS 码随机出错 -> 画面抖动、音频包校验失败。
// 设计要点：
//   1) 翻转检测与装载信号只做一份，4 条通道共用 -> 4 条通道永远同一拍装数据
//   2) 翻转寄存器加 keep，防止 TD 把它与其他同样每拍翻转的寄存器合并后放到远处，使布线延迟变长
// 定义 SER_SIM 时可给翻转信号和数据加布线延迟，用于仿真扫描验证装载时刻的裕量。
// ===========================================================================
module tmds_load_gen (
    input  wire I_pixel_clk,
    input  wire I_serial_clk,
    input  wire I_rst,
    output reg  O_load          // 串行时钟域单拍：本拍把像素数据装进移位寄存器
);
`ifdef SER_SIM
    parameter real TOG_DLY = 0.0;   // 仿真用：翻转信号到串行时钟域的布线延迟（ns）
`endif
    (* keep = "true" *) reg S_tog /* synthesis keep */;
    always @(posedge I_pixel_clk or posedge I_rst) begin
        if (I_rst) S_tog <= 1'b0;
        else       S_tog <= ~S_tog;
    end
    wire S_tog_w;
`ifdef SER_SIM
    assign #(TOG_DLY) S_tog_w = S_tog;
`else
    assign S_tog_w = S_tog;
`endif
    (* keep = "true" *) reg S_tog_1d /* synthesis keep */;
    reg S_tog_2d;
    reg S_det;
    always @(posedge I_serial_clk) begin
        S_tog_1d <= S_tog_w;
        S_tog_2d <= S_tog_1d;
        S_det    <= S_tog_1d ^ S_tog_2d;   // 检测到翻转的下一拍
        O_load   <= S_det;                 // 再下一拍装数据
    end
endmodule

module lane_lvds_10_1 #(
`ifdef SER_SIM
    parameter real DATA_DLY0 = 0.0,      // 仿真用：数据第 0 位布线延迟（ns），第 i 位 = DATA_DLY0 + i*DATA_DLY_STEP
    parameter real DATA_DLY_STEP = 0.0,
`endif
    parameter DEVICE = "EG"
    )(
    input wire      I_pixel_clk,
    input wire      I_serial_clk,
    input wire      I_rst,
    input wire      I_load,         // 共用的装数据信号（来自 tmds_load_gen）

    input wire[9:0] I_data_in,

    output wire     O_serial_out
);

`ifdef SER_SIM
    wire [9:0] S_data_w;
    genvar gi;
    generate for (gi = 0; gi < 10; gi = gi + 1) begin : g_dly
        assign #(DATA_DLY0 + gi * DATA_DLY_STEP) S_data_w[gi] = I_data_in[gi];
    end endgenerate
`else
    wire [9:0] S_data_w = I_data_in;
`endif
    reg[9:0] S_shift_data;   
    reg[1:0] S_oddr_data;    

    always @(posedge I_serial_clk or posedge I_rst) begin
        if(I_rst)
            S_shift_data <= 'd0;
        else
            if(I_load)
                S_shift_data <= S_data_w;
            else 
                S_shift_data <= S_shift_data << 2;
    end

    always @(posedge I_serial_clk) begin
        S_oddr_data <= S_shift_data[9:8];
    end

    generate
        if(DEVICE == "EF2")
            begin
                EF2_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF3")
            begin
                EF3_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF4")
            begin
                EF4_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "SF1")
            begin
                SF1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EG")
            begin
                EG_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1A")
            begin
                PH1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1P")
            begin
                PH1P_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "DR1")
            begin
                DR1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
    endgenerate
    
endmodule