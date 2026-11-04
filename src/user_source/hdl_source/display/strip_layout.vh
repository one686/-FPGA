// 胶片条版面参数（有效区坐标：x 0~639，y 0~479）
// 版面数据的唯一来源：design/osd_gen.py（OSD 文字位置）和 sim/strip_model.py（仿真参考模型）都从这里读数，
// 改这里两边自动跟着变（每行一个 localparam，"名字 = 数" 的写法不要改）。
//
//   y 354~479 胶片条底板（不透明，这些行不读大图），顶上 2 行细线
//   焦点缩略图 112x84（原尺寸）在中间，外面 3 像素金框；两边各 2 张缩成 84x63（横竖都 4 取 3）
//   x:  64 [左2] 148  164 [左1] 248   264 [焦点] 376   392 [右1] 476  492 [右2] 576
//   两边箭头：左边尖在 x=24，右边尖在 x=615，宽 12，中心 y=432（= 焦点、两边缩略图的中心）
localparam integer SP_Y0     = 354;
localparam integer SP_EDGE_H = 2;
localparam integer SP_FX0    = 264;
localparam integer SP_FY0    = 390;
localparam integer SP_SY0    = 401;
localparam integer SP_SX0    = 64;
localparam integer SP_SX1    = 164;
localparam integer SP_SX3    = 392;
localparam integer SP_SX4    = 492;
localparam integer SP_TW     = 112;
localparam integer SP_TH     = 84;
localparam integer SP_SW     = 84;
localparam integer SP_SH     = 63;
localparam integer SP_FRAME  = 3;
localparam integer SP_AYC    = 432;
localparam integer SP_ALX    = 24;
localparam integer SP_ARX    = 615;
localparam integer SP_AW     = 12;
localparam integer SP_DIM    = 176;
localparam [23:0]  SP_C_PANEL = 24'h12141a;
localparam [23:0]  SP_C_EDGE  = 24'h6e5c28;
localparam [23:0]  SP_C_GOLD  = 24'hffc430;
localparam [23:0]  SP_C_PH    = 24'h363a46;
localparam [23:0]  SP_C_ARROW = 24'hd2d2d2;
