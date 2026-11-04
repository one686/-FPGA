`timescale 1ns/1ps
// HDMI 单通道 TMDS 编码：按 mode 输出视频（8b/10b 直流平衡）、视频保护带、数据岛（TERC4）、数据岛保护带或控制符号。
//   省 LUT 写法：disparity 恒为偶数，只存一半（dh）；三个分支合成一个加减；
//   q_m 用 XOR 前缀再按 use_xnor 异或交替掩码得到（XNOR 链 = XOR 链 ^ 10101010），结果与标准算法逐符号相同（已做等价仿真）。
module hdmi_tmds_channel_encoder #(
    parameter integer CHANNEL = 0
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [2:0] mode,
    input  wire [7:0] video_data,
    input  wire [3:0] data_island_data,
    input  wire [1:0] control_data,
    output reg  [9:0] symbol
);
    localparam [2:0] MODE_VIDEO       = 3'd1;
    localparam [2:0] MODE_VIDEO_GUARD = 3'd2;
    localparam [2:0] MODE_DATA_ISLAND = 3'd3;
    localparam [2:0] MODE_DATA_GUARD  = 3'd4;
    reg signed [4:0] dh;          // disparity / 2
    wire [9:0] terc4_symbol;
    hdmi_terc4_encoder u_terc4 (.data(data_island_data), .symbol(terc4_symbol));
    reg [3:0] n1, n2;
    reg use_xnor, inv, q8;
    reg [7:0] x, q;
    reg signed [4:0] b, nd;
    integer k;
    always @* begin
        n1 = 4'd0;
        for (k = 0; k < 8; k = k + 1) n1 = n1 + video_data[k];
        use_xnor = (n1 > 4'd4) || (n1 == 4'd4 && !video_data[0]);
        x[0] = video_data[0];
        for (k = 1; k < 8; k = k + 1) x[k] = x[k-1] ^ video_data[k];
        q = x ^ (use_xnor ? 8'b10101010 : 8'b0);
        q8 = !use_xnor;
        n2 = 4'd0;
        for (k = 0; k < 8; k = k + 1) n2 = n2 + q[k];
        b = $signed({1'b0, n2}) - 5'sd4;
        if (dh == 0 || b == 0) inv = !q8;
        else inv = (dh[4] == b[4]);
        nd = inv ? (dh - b + {4'd0, q8}) : (dh + b - {4'd0, !q8});
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            symbol <= 10'b1101010100;
            dh <= 5'sd0;
        end else begin
            case (mode)
                MODE_VIDEO: begin
                    symbol <= {inv, q8, inv ? ~q : q};
                    dh <= nd;
                end
                MODE_VIDEO_GUARD: begin
                    symbol <= (CHANNEL == 1) ? 10'b0100110011 : 10'b1011001100;
                    dh <= 5'sd0;
                end
                MODE_DATA_ISLAND: begin
                    symbol <= terc4_symbol;
                    dh <= 5'sd0;
                end
                MODE_DATA_GUARD: begin
                    if (CHANNEL == 0)
                        case (control_data)
                            2'b00: symbol <= 10'b1010001110;
                            2'b01: symbol <= 10'b1001110001;
                            2'b10: symbol <= 10'b0101100011;
                            default: symbol <= 10'b1011000011;
                        endcase
                    else
                        symbol <= 10'b0100110011;
                    dh <= 5'sd0;
                end
                default: begin
                    case (control_data)
                        2'b00: symbol <= 10'b1101010100;
                        2'b01: symbol <= 10'b0010101011;
                        2'b10: symbol <= 10'b0101010100;
                        default: symbol <= 10'b1010101011;
                    endcase
                    dh <= 5'sd0;
                end
            endcase
        end
    end
endmodule
