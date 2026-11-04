`timescale 1ns/1ps

// PCM 接口适配：音源（media_audio_source）与 HDMI 音频打包链之间的 valid/ready 边界，数据直通。
//   约定：valid=1 且 ready=0 时左右声道数据必须保持不变。该检查不影响输出，只放在仿真断言里
//   （+define+HDMI_AUDIO_SIM_ASSERT 时编进）；板上 contract_error 恒为 0，端口保留以兼容例化。
module m06_pcm_contract_adapter (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               source_valid,
    output wire               source_ready,
    input  wire signed [15:0] source_left,
    input  wire signed [15:0] source_right,
    output wire               pcm_valid,
    input  wire               pcm_ready,
    output wire signed [15:0] pcm_left,
    output wire signed [15:0] pcm_right,
    output wire               contract_error
);
    assign source_ready = pcm_ready;
    assign pcm_valid = source_valid;
    assign pcm_left = source_left;
    assign pcm_right = source_right;
    assign contract_error = 1'b0;

`ifdef HDMI_AUDIO_SIM_ASSERT
    // 音源在 valid=1、ready=0 期间必须保持左右声道不变。
    reg hold_active = 1'b0;
    reg signed [15:0] held_left;
    reg signed [15:0] held_right;
    reg sim_contract_error = 1'b0;
    always @(posedge clk) begin
        if (!rst_n) begin
            hold_active <= 1'b0;
        end else if (source_valid && !source_ready) begin
            if (hold_active &&
                (source_left !== held_left || source_right !== held_right)) begin
                if (!sim_contract_error)
                    $display("[ASSERT] %m: PCM changed while valid && !ready, t=%0t", $time);
                sim_contract_error <= 1'b1;
            end
            if (!hold_active) begin
                held_left <= source_left;
                held_right <= source_right;
                hold_active <= 1'b1;
            end
        end else begin
            hold_active <= 1'b0;
        end
    end
`endif
endmodule
