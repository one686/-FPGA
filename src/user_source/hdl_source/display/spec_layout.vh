// 由 design/spec_gen.py 自动生成，不要手改

    localparam integer PX0 = 44, PX1 = 596, PY0 = 380, PY1 = 428, PANEL_R = 6;
    localparam integer SX0 = 54, SX1 = 562, BARW = 12, BASE = 414, NREF = 10, HMAX = 30;
    localparam integer VUL_X0 = 572, VUR_X0 = 582, VU_W = 6, VU_Y0 = 384, VU_N = 40;
    localparam integer NB = 32, NLOW = 17, G = 2, CF = 16, SB = 18, HOLD = 20, HOLD_FRAMES = 40, LVL_OFS_VU = 19;
    localparam integer LVL_LO = 115, LVL_HI0 = 155, ACCW = 25;
    localparam [23:0] C_PANEL = 24'h080e1e; localparam [4:0] A_PANEL = 5'd10;
    localparam [23:0] C_CAP = 24'hebebff; localparam [4:0] A_CAP = 5'd16;
    localparam [23:0] C_DIM = 24'h283244; localparam [4:0] A_DIM = 5'd12;
    localparam [23:0] C_HOLD = 24'hfafafa; localparam [4:0] A_HOLD = 5'd16;

    function [23:0] bar_rgb;
        input [4:0] k;
        begin
            case (k)
                5'd0: bar_rgb = 24'd2678015;
                5'd1: bar_rgb = 24'd3004159;
                5'd2: bar_rgb = 24'd3330047;
                5'd3: bar_rgb = 24'd3721727;
                5'd4: bar_rgb = 24'd4047871;
                5'd5: bar_rgb = 24'd4373759;
                5'd6: bar_rgb = 24'd4699903;
                5'd7: bar_rgb = 24'd5026047;
                5'd8: bar_rgb = 24'd5351935;
                5'd9: bar_rgb = 24'd5743615;
                5'd10: bar_rgb = 24'd6069759;
                5'd11: bar_rgb = 24'd6395647;
                5'd12: bar_rgb = 24'd6721791;
                5'd13: bar_rgb = 24'd7047935;
                5'd14: bar_rgb = 24'd7373823;
                5'd15: bar_rgb = 24'd7765503;
                5'd16: bar_rgb = 24'd8091647;
                5'd17: bar_rgb = 24'd8417791;
                5'd18: bar_rgb = 24'd8940796;
                5'd19: bar_rgb = 24'd9660663;
                5'd20: bar_rgb = 24'd10380787;
                5'd21: bar_rgb = 24'd11100910;
                5'd22: bar_rgb = 24'd11820777;
                5'd23: bar_rgb = 24'd12475364;
                5'd24: bar_rgb = 24'd13195232;
                5'd25: bar_rgb = 24'd13915355;
                5'd26: bar_rgb = 24'd14635222;
                5'd27: bar_rgb = 24'd15289809;
                5'd28: bar_rgb = 24'd16009677;
                5'd29: bar_rgb = 24'd16729800;
                default: bar_rgb = 24'd0;
            endcase
        end
    endfunction

    function [23:0] vu_rgb;
        input [5:0] k;
        begin
            case (k)
                6'd1: vu_rgb = 24'd2676570;
                6'd2: vu_rgb = 24'd2676570;
                6'd3: vu_rgb = 24'd2676570;
                6'd4: vu_rgb = 24'd2676570;
                6'd5: vu_rgb = 24'd2676570;
                6'd6: vu_rgb = 24'd2676570;
                6'd7: vu_rgb = 24'd2676570;
                6'd8: vu_rgb = 24'd2676570;
                6'd9: vu_rgb = 24'd2676570;
                6'd10: vu_rgb = 24'd2676570;
                6'd11: vu_rgb = 24'd2676570;
                6'd12: vu_rgb = 24'd2676570;
                6'd13: vu_rgb = 24'd2676570;
                6'd14: vu_rgb = 24'd2676570;
                6'd15: vu_rgb = 24'd2676570;
                6'd16: vu_rgb = 24'd2676570;
                6'd17: vu_rgb = 24'd2676570;
                6'd18: vu_rgb = 24'd2676570;
                6'd19: vu_rgb = 24'd4183892;
                6'd20: vu_rgb = 24'd5756751;
                6'd21: vu_rgb = 24'd7264073;
                6'd22: vu_rgb = 24'd8771396;
                6'd23: vu_rgb = 24'd10344254;
                6'd24: vu_rgb = 24'd11851577;
                6'd25: vu_rgb = 24'd13358899;
                6'd26: vu_rgb = 24'd14931758;
                6'd27: vu_rgb = 24'd16500008;
                6'd28: vu_rgb = 24'd16495400;
                6'd29: vu_rgb = 24'd16556584;
                6'd30: vu_rgb = 24'd16551976;
                6'd31: vu_rgb = 24'd16612904;
                6'd32: vu_rgb = 24'd16608296;
                6'd33: vu_rgb = 24'd16669480;
                6'd34: vu_rgb = 24'd16664872;
                6'd35: vu_rgb = 24'd16725800;
                6'd36: vu_rgb = 24'd16725800;
                6'd37: vu_rgb = 24'd16725800;
                6'd38: vu_rgb = 24'd16725800;
                6'd39: vu_rgb = 24'd16725800;
                6'd40: vu_rgb = 24'd16725800;
                default: vu_rgb = 24'd0;
            endcase
        end
    endfunction

    function [0:0] corner_in;
        input [5:0] k;
        begin
            case (k)
                6'd0: corner_in = 1'd0;
                6'd1: corner_in = 1'd0;
                6'd2: corner_in = 1'd0;
                6'd3: corner_in = 1'd0;
                6'd4: corner_in = 1'd1;
                6'd5: corner_in = 1'd1;
                6'd8: corner_in = 1'd0;
                6'd9: corner_in = 1'd0;
                6'd10: corner_in = 1'd1;
                6'd11: corner_in = 1'd1;
                6'd12: corner_in = 1'd1;
                6'd13: corner_in = 1'd1;
                6'd16: corner_in = 1'd0;
                6'd17: corner_in = 1'd1;
                6'd18: corner_in = 1'd1;
                6'd19: corner_in = 1'd1;
                6'd20: corner_in = 1'd1;
                6'd21: corner_in = 1'd1;
                6'd24: corner_in = 1'd0;
                6'd25: corner_in = 1'd1;
                6'd26: corner_in = 1'd1;
                6'd27: corner_in = 1'd1;
                6'd28: corner_in = 1'd1;
                6'd29: corner_in = 1'd1;
                6'd32: corner_in = 1'd1;
                6'd33: corner_in = 1'd1;
                6'd34: corner_in = 1'd1;
                6'd35: corner_in = 1'd1;
                6'd36: corner_in = 1'd1;
                6'd37: corner_in = 1'd1;
                6'd40: corner_in = 1'd1;
                6'd41: corner_in = 1'd1;
                6'd42: corner_in = 1'd1;
                6'd43: corner_in = 1'd1;
                6'd44: corner_in = 1'd1;
                6'd45: corner_in = 1'd1;
                default: corner_in = 1'd1;
            endcase
        end
    endfunction

