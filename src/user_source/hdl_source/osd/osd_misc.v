`timescale 1ns/1ps

// OSD 辅助模块（像素时钟域）
//   osd_runtime   ：运行时间 BCD 计数 hh:mm:ss（SEC_CYCLES 个时钟 = 1 秒，25MHz 精确）
//   osd_status_sync：sd_card_clk 域的状态位逐位两级同步，连续两拍一致才采用
module osd_runtime #(
    parameter integer SEC_CYCLES = 25_000_000
) (
    input  wire        clk,
    input  wire        rst_n,
    output reg  [23:0] bcd,         // h1 h0 m1 m0 s1 s0
    output wire        sec_tick     // 1 秒节拍（定时关机共用）
);
    reg [24:0] div;
    wire tick = (div == SEC_CYCLES - 1);
    assign sec_tick = tick;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div <= 0;
            bcd <= 24'd0;
        end else begin
            div <= tick ? 25'd0 : div + 25'd1;
            if (tick) begin
                if (bcd[3:0] != 4'd9)        bcd[3:0] <= bcd[3:0] + 4'd1;
                else begin
                    bcd[3:0] <= 4'd0;
                    if (bcd[7:4] != 4'd5)    bcd[7:4] <= bcd[7:4] + 4'd1;
                    else begin
                        bcd[7:4] <= 4'd0;
                        if (bcd[11:8] != 4'd9)   bcd[11:8] <= bcd[11:8] + 4'd1;
                        else begin
                            bcd[11:8] <= 4'd0;
                            if (bcd[15:12] != 4'd5)  bcd[15:12] <= bcd[15:12] + 4'd1;
                            else begin
                                bcd[15:12] <= 4'd0;
                                if (bcd[19:16] != 4'd9)  bcd[19:16] <= bcd[19:16] + 4'd1;
                                else begin
                                    bcd[19:16] <= 4'd0;
                                    bcd[23:20] <= (bcd[23:20] == 4'd9) ? 4'd0 : bcd[23:20] + 4'd1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
endmodule

module osd_status_sync #(
    parameter integer N = 19
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [N-1:0] async_in,
    output reg  [N-1:0] sync_out
);
    (* async_reg = "true" *) reg [N-1:0] m1, m2;
    reg [N-1:0] m3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m1 <= 0; m2 <= 0; m3 <= 0; sync_out <= 0;
        end else begin
            m1 <= async_in; m2 <= m1; m3 <= m2;
            if (m2 == m3) sync_out <= m2;
        end
    end
endmodule
