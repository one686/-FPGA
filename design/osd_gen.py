# -*- coding: utf-8 -*-
"""第9课 OSD：字库 / 版面生成器 + 与 RTL 逐位一致的 Python 参考模型。
第10课 WAV：右上角第二行加"音乐信息"（♪ 第几首/共几首  已播放/总时长，没有 WAV 时显示"♪ 琶音"），
           左侧状态行加"音乐错误 n"（出错 5 秒，自动淡出）。
第11课 SW4 分层：右侧音乐行下面加"提示"行 T（切换 SW4 / 暂停 / 循环模式时淡入 2 秒：图片模式、音乐模式、
           暂停播放、继续播放、单曲循环、列表循环）；音乐行的 ♪ 暂停时变成 ⏸，单曲循环时行尾显示"单"；
           字幕压缩到字高 40，放在 y=434~474（给第2步的波形让出 y=380~428）。
第14课 第3步 胶片条：屏幕下方胶片条上两行文字（底板、缩略图、金框由 RTL 的 filmstrip 画，不在 OSD 里）：
           "n/N"（第几张 / 共几张，和状态条同样的字段）居中在胶片条顶上；每格中间一行"加载中"或"坏图片"
           （格的状态由 filmstrip 给，标签格的源码 = LBL_BASE + 格号*4 + 第几个字）。几何从 display/strip_layout.vh 读。
           胶片条开着或还看得见时字幕淡出（st_hide）。
finish_inc_2_6 定时关机：菜单加第 6 行"定时 关/1/15/30/60/90 分"（菜单整体上移 44 像素，y=146~368）；
           B 行（"正在加载"，一直不显示）改成右侧第 4 行的"关机 mm:ss"（定时开着时显示，受 KEY3 显示模式影响）；
           最后 10 秒画面中间弹出提示框（"n 秒后关机" + 10 格进度条 + "按任意键继续"，借用菜单第 2、3 行的 y 和 b_sel）；
           关机淡出时状态条、字幕、提示行一起淡出（sleep）。
第13课 画面调节菜单（面板 B 样式）：左侧 y=190~376 一个圆角半透明面板，5 行
           "亮度 / 对比度 / 饱和度 / 色温 / 轮播"，当前行有蓝色高亮条、金色字，前 4 行带 11 格进度条；
           提示行加"恢复默认"；状态条的"自动 N秒"随轮播周期变化。
           字形编号从 6 位扩到 7 位（最多 127 个，127 = 空白），字段编码从 64 起改为 128 起。

用法：
  python osd_gen.py            生成 RTL 用的字库和版面文件，并输出预览图
  （被 sim/check_osd.py import，作为仿真比对的标准答案）

生成：
  src/user_source/hdl_source/osd/osd_font.mif   字库 ROM 初值（TD 综合用，16 位宽 = 一行 2 个字节）
  src/user_source/hdl_source/osd/osd_font.hex   同一份数据（ModelSim 用 $readmemh，每行 4 位十六进制 = 2 字节）
  src/user_source/hdl_source/osd/osd_layout.vh  版面表：字形表、各行字符格、底板、颜色、渐变
  design/preview_*.png                          效果预览（就是 RTL 将输出的像素）

字体：思源黑体 Source Han Sans CN（Adobe，SIL Open Font License，见 design/fonts/LICENSE.txt）
字库格式（第13课起压缩，ROM_DEPTH 不变、更省空间）：
  状态栏字形（无描边，g == o）：每像素只存 4 位的 g，一个字节存相邻 2 个像素（偶数列在低 4 位，奇数列在高 4 位）；
    RTL 读出后令 g = o = 该 4 位，与压缩前逐位相同（design/font_pack_compare.py 验证过 0 像素差异）
  字幕字形（有描边/投影，g != o）：仍为每像素 1 字节 [7:4] = g，[3:0] = o

混合算法（全部整数，RTL 与本模型完全相同）：
  a9(v)  = v*17 + (v==15)                         4 位覆盖度 -> 0~256 的透明度
  C      = (tc*a9(g) >> 8) + (Cp*(256-a9(o)) >> 8) 预乘颜色：文字 over 底板（描边是黑色，只占透明度）
  A      = a9(o) + (Ap*(256-a9(o)) >> 8)
  C, A   = (C*f) >> 8, (A*f) >> 8                  淡入淡出系数 f（0~256）
  状态栏字形取像素：像素序号 n = 行*宽 + 列（整个字形连续编号，跨行也接着排），
    字节地址 = base + n >> 1，4 位值 = n 是奇数 ? 字节[7:4] : 字节[3:0]，令 g = o = 该值
  字幕字形取像素：地址 = base + 行*宽 + 列，高 4 位是 g、低 4 位是 o
  out    = min(255, C + (bg*(256-A) >> 8))         与图片混合；没有 OSD 的像素 A=0，输出与原图逐位相同
"""
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_HDL = os.path.normpath(os.path.join(HERE, "..", "src", "user_source", "hdl_source", "osd"))
FONT_REG = os.path.join(HERE, "fonts", "SourceHanSansSC-Regular.otf")
FONT_BOLD = os.path.join(HERE, "fonts", "SourceHanSansSC-Bold.otf")

W, H = 640, 480
ROM_DEPTH = 32768

# ---------------- 字形 ----------------
STATUS_CJK = "第张自动手秒音量运行正在加载读卡错误坏图乐琶♪片模式单曲循环列表暂停播放继续⏸亮度对比饱和色温轮恢复默认中定时关机后分按任意键"
# 第10课加 乐 琶 ♪；第11课加提示行用字和 ⏸；第13课加菜单用字 亮度对比饱和色温轮 和提示"恢复默认"；第14课加"中"（加载中）
# finish_inc_2_6 定时关机加 定时关机后分按任意键
STATUS_ASCII = "0123456789/:%-+"                  # 第13课加 +
SUBTITLE = "天下无敌"
ST_H, ST_CJK_W, ST_ASC_W, ST_SIZE, ST_BASE = 24, 22, 11, 20, 19


def _ascii_width(ch):
    """半角字形格宽：至少 11，字形本身更宽就加宽（左右各留 1 像素），保持偶数居中。"""
    f = ImageFont.truetype(FONT_REG, ST_SIZE)
    x0, _, x1, _ = f.getbbox(ch, anchor="ms")
    need = int(np.ceil(x1 - x0)) + 3
    return max(ST_ASC_W, need)


ASCII_W = {}               # 在 glyph_list() 里按实际字形填
SB_H, SB_W, SB_SIZE, SB_BASE, SB_STROKE = 40, 40, 29, 30, 3          # 第11课：字高 58 -> 40
SB_DX = -1                 # 字形在格子里左移 1 像素，给右下的投影留地方
SB_SHADOW_DX, SB_SHADOW_DY, SB_SHADOW_BLUR, SB_SHADOW_K = 2, 2, 2, 0.55   # 柔和投影
SHOW_LOADING = False      # 用户决定：不显示"正在加载"行

# ---------------- 版面 ----------------
BAND_H = 30            # 状态行底板高度
TEXT_DY = 3            # 文字格在底板内的下移量
SLOT_PITCH = 36        # 状态行之间的间距
PANEL_R = 6            # 圆角半径
SUB_BAND_Y0, SUB_BAND_Y1 = 434, 474   # 字幕区 [y0, y1)（方案A：无底板，只是文字的范围）；第11课下移+压缩
SUB_TEXT_Y0 = 434
SUB_SPEED = 1          # 每帧滚动像素数（60 帧/秒 => 约 60 像素/秒）
SUB_TOTAL = SB_W * len(SUBTITLE)

# 各状态行颜色：(底板 RGB, 底板透明度 0~256, 文字 RGB)
BAND_STYLE = {
    "A": ((0, 0, 0), 141, (255, 255, 255)),
    "B": ((0, 0, 0), 141, (255, 215, 90)),
    "C": ((110, 0, 0), 160, (255, 96, 96)),
    "D": ((90, 40, 0), 150, (255, 160, 60)),
    "E": ((90, 0, 70), 160, (255, 130, 225)),     # 第10课 音乐错误（紫红）
    "M": ((0, 0, 0), 141, (170, 230, 255)),       # 第10课 音乐信息（浅蓝字）
    "P": ((0, 0, 0), 141, (170, 230, 255)),       # 第10课 琶音
    "T": ((0, 45, 90), 170, (255, 225, 120)),     # 第11课 提示行（深蓝底、金色字）
}
SUB_PANEL_ALPHA = 0        # 方案A：不要字幕底板，只靠描边+投影保证可读
# 字幕蓝色渐变：上浅下深
SUB_TOP, SUB_BOTTOM = (170, 230, 255), (25, 85, 235)

# 状态行模板：{x} 是动态字段；空格只留空不占字形
# 字段：IDX 当前第几张  TOT 总张数  M1/M2 自动|手动  PT/PO 轮播秒数十位/个位
#       VH/VT/VO 音量百分比  H1 H0 M1 M0 S1 S0 运行时间  ERR 错误码  BAD 坏图数
# 第14课：最多 99 张图，IDX（右对齐）/TOT、BAD（左对齐）改成两位：X1 = 十位格，X0 = 个位格；
#        "正在加载"行不再带张数（按需加载后"第几张/共几张"没有意义；该行本来就不显示，SHOW_LOADING）
TEMPLATES = {
    "A_L": "第 {IDX1}{IDX0}/{TOT1}{TOT0} 张  {MA}{MB}{PT}{PO}秒  音量{VH}{VT}{VO}%",
    "A_R": "运行 {H1}{H0}:{N1}{N0}:{S1}{S0}",
    "B":   "关机 {ZM1}{ZM0}:{ZS1}{ZS0}",           # finish_inc_2_6：原"正在加载"（从来不显示）改成定时关机剩余时间
    "C":   "读卡错误 {ERR}",
    "D":   "坏图 {BAD1}{BAD0} 张",
    "E":   "音乐错误 {ME}",
    "M":   "{MI} {SI1}{SI0}/{SC1}{SC0}  {EM1}{EM0}:{ES1}{ES0}/{TM1}{TM0}:{TS1}{TS0} {LP}",
    "P":   "{MI} 琶音",
    "T":   "{T1}{T2}{T3}{T4}",
    # 第13课 调节菜单（数值都从同一个 x 开始：标签区 66 像素 + 1 个空格）
    "N0":  "亮度   {MS0}{MD0}",
    "N1":  "对比度 {MS1}{MD1}",
    "N2":  "饱和度 {MS2}{MD2}",
    "N3":  "色温   {MS3}{MD3}",
    "N4":  "轮播   {PT}{PO}秒 {MA}{MB}",
    "N5":  "定时   {SVA}{SVB}{SVC}",               # finish_inc_2_6：关 / 1 / 15 / 30 / 60 / 90 分（"关"在"分"的位置）
    # finish_inc_2_6 最后 10 秒提示框的两行（居中）
    "W1":  "{WS1}{WS0} 秒后关机",
    "W2":  "按任意键继续",
}
# 第11课：MI = ♪ / ⏸（暂停）  LP = "单"（单曲循环）/ 空  T1~T4 = 提示行的 4 个字
TOAST = ["图片模式", "音乐模式", "暂停播放", "继续播放", "单曲循环", "列表循环", "恢复默认"]
TOAST_FRAMES = 120         # 提示显示 2 秒（60 帧/秒），之后淡出
# 第10课：SI 第几首（十位不足时空）  SC 共几首（左对齐）  EM/ES 已播放分:秒  TM/TS 总时长  ME 音乐错误码
FIELDS = ["IDX1", "IDX0", "TOT1", "TOT0", "MA", "MB", "PT", "PO", "VH", "VT", "VO",
          "H1", "H0", "N1", "N0", "S1", "S0", "ERR", "BAD1", "BAD0",
          "SI1", "SI0", "SC1", "SC0", "EM1", "EM0", "ES1", "ES0", "TM1", "TM0", "TS1", "TS0", "ME",
          "MI", "LP", "T1", "T2", "T3", "T4",
          "MS0", "MS1", "MS2", "MS3", "MD0", "MD1", "MD2", "MD3",
          "ZM1", "ZM0", "ZS1", "ZS0", "SVA", "SVB", "SVC", "WS1", "WS0"]      # finish_inc_2_6 定时关机
# 第13课：字形编号 0~126（127 = 空白），字段编码 128 起
FIELD_BASE = 128
FIELD_CODE = {f: FIELD_BASE + i for i, f in enumerate(FIELDS)}
FIELD_CJK = {"MA", "MB", "MI", "LP", "T1", "T2", "T3", "T4", "SVC"}   # 这些字段是汉字宽度
LEFT_X, RIGHT_X1 = 8, 632
PAD = 8
R_Y = 8 + 36            # 第10课：右侧音乐行的 y（A 行下面一格）
T_Y = R_Y + 36          # 第11课：右侧提示行的 y（音乐行下面一格）
Z_Y = T_Y + 36          # finish_inc_2_6：右侧"关机 mm:ss"的 y（= 左侧第 3 个槽，RTL 用 sl_hit && sl_k == 3）
SLEEP_PRESET = [0, 0x0100, 0x1500, 0x3000, 0x6000, 0x9000]   # 定时档位 0~5 -> 剩余时间 BCD mm:ss（0 = 关）

# 第13课 调节菜单（面板 B）：几何、颜色
MENU_ROWS = ["N0", "N1", "N2", "N3", "N4", "N5"]
MX0, MY_ROW0, MPITCH = 8, 152, 36          # 面板左边、第一行 y、行距（finish_inc_2_6：6 行，上移 44 像素，避开底部频谱面板）
MPAD_X, MPAD_Y = 10, 6                     # 文字离面板左边、面板上下留边
MTEXT_X0 = MX0 + MPAD_X                    # 文字起点
MVAL_DX = 66 + 11                          # 数值相对文字起点
MBAR_X0 = MTEXT_X0 + MVAL_DX + 22 + 10     # 进度条起点
MSEG_P, MSEG_W, MSEG_N = 8, 6, 11          # 每格间距、宽、格数（档位 0~10 -> 亮 1~11 格）
MSEG_Y0, MSEG_Y1 = 9, 21                   # 进度条在行内的 y 范围
MX1 = MBAR_X0 + MSEG_N * MSEG_P + 10       # 面板右边（不含）
MY0 = MY_ROW0 - MPAD_Y                     # 面板上边
MY1 = MY_ROW0 + 5 * MPITCH + 30 + MPAD_Y   # 面板下边（不含）
SBAR_X0, SSEG_P, SSEG_W, SSEG_N = 151, 14, 11, 5   # finish_inc_2_6 第 6 行"定时"的 5 格（档位 1~5 亮 1~5 格）
MHL_M = 4                                  # 高亮条离面板左右边
M_PANEL = ((10, 16, 32), 176)              # 面板：颜色、透明度（0~256）
M_HL = ((30, 90, 180), 210)                # 高亮条
M_OFF = ((60, 70, 92), 220)                # 进度条灭的格
M_ON = (120, 210, 255)                     # 进度条亮的格（不透明）
M_ON_SEL = (255, 215, 90)                  # 当前行亮的格（金色）
M_TC, M_TC_SEL = (235, 240, 255), (255, 225, 120)   # 文字颜色：其他行 / 当前行
MENU_STEP = 16                             # 菜单淡入淡出：每帧 16/256（16 帧约 0.27 秒）
# finish_inc_2_6 最后 10 秒提示框：x [WX0, WX1)、y [WY0, WY1)；两行字借用菜单第 2、3 行的 y（188、224），中间 10 格进度条
WX0, WX1, WY0, WY1 = 212, 428, 180, 260
WBAR_X0, WBAR_Y0, WBAR_Y1, WSEG_P, WSEG_W, WSEG_N = 241, 218, 224, 16, 14, 10
W_PANEL = ((0, 45, 90), 200)
W_OFF = ((60, 70, 92), 230)
W_ON = (255, 215, 90)
W_TC1, W_TC2 = (255, 225, 120), (200, 215, 235)
WARN_ROWS = (1, 2)                         # 用菜单第几行的 y


def _over(top, top_a, under_c, under_a):
    """预乘颜色合成：top(颜色, 透明度) 盖在 (预乘颜色 under_c, 透明度 under_a) 上 -> (预乘颜色, 透明度)"""
    c = tuple(((t * top_a) >> 8) + ((u * (256 - top_a)) >> 8) for t, u in zip(top, under_c))
    return c, top_a + ((under_a * (256 - top_a)) >> 8)


def menu_regions():
    """菜单区域编码 -> (预乘颜色, 透明度)：0 无 1 面板 2 面板+高亮 3 灭格(面板) 4 灭格(面板+高亮) 5 亮格 6 亮格(当前行)"""
    pc = tuple((c * M_PANEL[1]) >> 8 for c in M_PANEL[0])
    p = (pc, M_PANEL[1])
    ph = _over(M_HL[0], M_HL[1], *p)
    return [((0, 0, 0), 0), p, ph, _over(M_OFF[0], M_OFF[1], *p), _over(M_OFF[0], M_OFF[1], *ph),
            (M_ON, 256), (M_ON_SEL, 256)]


MENU_REG = menu_regions()


def warn_regions():
    """finish_inc_2_6 提示框区域编码 -> (预乘颜色, 透明度)：0 无 1 底板 2 灭格 3 亮格"""
    p = (tuple((c * W_PANEL[1]) >> 8 for c in W_PANEL[0]), W_PANEL[1])
    return [((0, 0, 0), 0), p, _over(W_OFF[0], W_OFF[1], *p), (W_ON, 256)]


WARN_REG = warn_regions()


def strip_layout():
    """第14课 第3步：读 display/strip_layout.vh 里的 localparam（RTL filmstrip 用的同一份数）"""
    import re
    p = os.path.join(OUT_HDL, "..", "display", "strip_layout.vh")
    d = {}
    for line in open(p, encoding="utf-8"):
        m = re.match(r"\s*localparam\s+(?:integer|\[23:0\])\s+(\w+)\s*=\s*(24'h[0-9a-fA-F]+|\d+)\s*;", line)
        if m:
            v = m.group(2)
            d[m.group(1)] = int(v[4:], 16) if v.startswith("24'h") else int(v)
    return d


SP = strip_layout()
SP_SLOT_X = [SP["SP_SX0"], SP["SP_SX1"], SP["SP_FX0"], SP["SP_SX3"], SP["SP_SX4"]]
SP_SLOT_W = [SP["SP_SW"], SP["SP_SW"], SP["SP_TW"], SP["SP_SW"], SP["SP_SW"]]
SN_Y = SP["SP_Y0"] + SP["SP_EDGE_H"]       # "n/N" 行（BAND_H 高，文字在 TEXT_DY 下面）
SL_Y = SP["SP_AYC"] - BAND_H // 2          # 标签行：文字竖直居中在缩略图中心
LBL_BASE = 192                             # 标签格源码 = LBL_BASE + 格号*4 + 第几个字（0~2）
LBL_LOAD, LBL_BAD = "加载中", "坏图片"
S_TC, L_TC = (255, 236, 180), (210, 214, 224)   # "n/N"、标签的文字颜色
PERIODS = [2, 3, 5, 8, 10, 15, 30, 60]     # 轮播周期档位 0~7（秒），默认档位 1 = 3 秒
SPLIT_X = 320           # 右侧音乐行所在的行里：x >= SPLIT_X 归音乐行，左边归左侧状态行


def glyph_list():
    # 第14课（省 LUT）：ASCII 排在最前面，数字 0 = 字形 0（16 的倍数），RTL 里"G_DIGIT0 + 数字"就是直接拼位，不用加法器
    g = []
    for ch in STATUS_ASCII:
        ASCII_W[ch] = _ascii_width(ch)
        g.append((ch, "st", ASCII_W[ch], ST_H))
    for ch in STATUS_CJK:
        g.append((ch, "st", ST_CJK_W, ST_H))
    for ch in SUBTITLE:
        g.append((ch, "sb", SB_W, SB_H))
    return g


GLYPHS = glyph_list()
GID = {}
for i, (ch, kind, w, h) in enumerate(GLYPHS):
    GID[(ch, kind)] = i
# 第14课：osd_engine.v 的 dg(d) = {G_DIGIT0[6:4], d}，要求数字 0 的字形号是 16 的倍数、0~9 连续
assert GID[("0", "st")] % 16 == 0
assert all(GID[(str(d), "st")] == GID[("0", "st")] + d for d in range(10))


def q4(arr):
    return ((arr.astype(np.int32) * 15 + 127) // 255).astype(np.int32)


# ======== finish_u_2 字幕从卡读取：字幕条模型（RTL 的 osd_engine 读 sub_strip_ram）========
#   像素 0~15 按固定调色板查 {g, o}（和 make_subtitle.py 的 FIXED_PAL、osd_engine.v 的 sub_pal 一样）
#   默认 = osd/sub_default.hex（"热烈欢迎"，宽 144）；仿真 SUB.BMP 时用 set_strip_bmp() 换成卡里的
SUB_FIXED_PAL = [(0, 0), (15, 15), (0, 15), (0, 1), (0, 2), (0, 3), (0, 6), (0, 5),
                 (0, 4), (0, 8), (12, 15), (1, 15), (7, 15), (4, 15), (0, 12), (2, 15)]
SUB_DEF_W = 144
_strip = {}


def strip_from_bytes(data, w):
    """BMP 顺序（从下往上、偶数列在高 4 位）的像素字节 -> g、o（SB_H x w）。"""
    wb = w // 2
    g = np.zeros((SB_H, w), np.int64)
    o = np.zeros((SB_H, w), np.int64)
    for r in range(SB_H):
        base = (SB_H - 1 - r) * wb
        for x in range(w):
            b = data[base + x // 2]
            k = (b >> 4) if x % 2 == 0 else (b & 15)
            g[r, x], o[r, x] = SUB_FIXED_PAL[k]
    return g, o


def strip_default():
    if "def" not in _strip:
        hexp = os.path.join(OUT_HDL, "sub_default.hex")
        data = [int(ln, 16) for ln in open(hexp) if ln.strip()]
        _strip["def"] = strip_from_bytes(data, SUB_DEF_W)
    return _strip["def"]


def set_strip_bmp(path):
    """仿真卡里有 SUB.BMP 的场景：以后 render() 用这个文件的字幕条（None = 回到默认）。"""
    if path is None:
        _strip.pop("card", None)
        return
    d = open(path, "rb").read()
    off = int.from_bytes(d[10:14], "little")
    w = int.from_bytes(d[18:22], "little")
    _strip["card"] = strip_from_bytes(d[off:off + SB_H * w // 2], w)


def strip_now():
    return _strip.get("card") or strip_default()


def render_glyph(ch, kind, w, h):
    if kind == "st":
        if ch == "⏸":          # 暂停图标：两根竖条（4 倍分辨率画再缩小 = 抗锯齿），和 ♪ 一样大
            k = 4
            im = Image.new("L", (w * k, h * k), 0)
            d = ImageDraw.Draw(im)
            for x0 in (5, 12):
                d.rounded_rectangle((x0 * k, 5 * k, (x0 + 5) * k - 1, 21 * k - 1), radius=k, fill=255)
            im = im.resize((w, h), Image.LANCZOS)
        else:
            font = ImageFont.truetype(FONT_REG, ST_SIZE)
            im = Image.new("L", (w, h), 0)
            ImageDraw.Draw(im).text((w / 2, ST_BASE), ch, font=font, fill=255, anchor="ms")
        g = q4(np.array(im))
        return g, g.copy()
    font = ImageFont.truetype(FONT_BOLD, SB_SIZE)
    t = Image.new("L", (w, h), 0)
    ImageDraw.Draw(t).text((w / 2 + SB_DX, SB_BASE), ch, font=font, fill=255, anchor="ms")
    o = Image.new("L", (w, h), 0)
    ImageDraw.Draw(o).text((w / 2 + SB_DX, SB_BASE), ch, font=font, fill=255, anchor="ms",
                           stroke_width=SB_STROKE, stroke_fill=255)
    # 投影：描边轮廓向右下平移再模糊，按 SB_SHADOW_K 变淡；和描边一样是黑色，只占透明度通道
    sh = Image.new("L", (w, h), 0)
    sh.paste(o, (SB_SHADOW_DX, SB_SHADOW_DY))
    sh = sh.filter(ImageFilter.GaussianBlur(SB_SHADOW_BLUR))
    shadow = (np.array(sh).astype(np.float64) * SB_SHADOW_K).astype(np.int32)
    g = q4(np.array(t))
    oo = np.maximum(np.maximum(q4(np.array(o)), q4(shadow)), g)
    return g, oo


def build_rom():
    rom = np.zeros(ROM_DEPTH, dtype=np.int32)
    table = []          # (base, 宽, 高, 压缩?)
    addr = 0
    data = {}
    for ch, kind, w, h in GLYPHS:
        g, o = render_glyph(ch, kind, w, h)
        # 检查字形没有被格子裁掉（边上一圈应基本为空）
        assert o[0, :].max() <= 2 and o[-1, :].max() <= 2, (ch, "上下被裁")
        assert o[:, 0].max() <= 2 and o[:, -1].max() <= 2, (ch, "左右被裁")
        if kind == "st":
            # 状态栏字形没有描边：g == o，压成一字节 2 个像素
            assert (g == o).all(), ch
            table.append((addr, w, h, True))
            flat = g.reshape(-1)
            if len(flat) % 2:
                flat = np.append(flat, 0)
            rom[addr:addr + len(flat) // 2] = flat[0::2] | (flat[1::2] << 4)
            addr += len(flat) // 2
        else:
            table.append((addr, w, h, False))
            rom[addr:addr + w * h] = ((g << 4) | o).reshape(-1)
            addr += w * h
        data[(ch, kind)] = (g, o)
    assert addr <= ROM_DEPTH, addr
    return rom, table, addr


ROM, GTABLE, ROM_USED = build_rom()


def parse_template(t):
    """模板 -> [(相对 x, 宽, 源码)]，源码 <64 为固定字形，>=64 为字段。"""
    cells, x, i = [], 0, 0
    while i < len(t):
        if t[i] == "{":
            j = t.index("}", i)
            f = t[i + 1:j]
            w = ST_CJK_W if f in FIELD_CJK else ST_ASC_W
            cells.append((x, w, FIELD_CODE[f]))
            x += w
            i = j + 1
            continue
        ch = t[i]
        if ch == " ":
            x += ST_ASC_W
        elif ch in STATUS_CJK:
            cells.append((x, ST_CJK_W, GID[(ch, "st")]))
            x += ST_CJK_W
        else:
            w = ASCII_W.get(ch, ST_ASC_W)
            cells.append((x, w, GID[(ch, "st")]))
            x += w
        i += 1
    return cells, x


def build_layout():
    bands = {}
    # 第14课 第3步 胶片条："n/N" 居中；标签每格 3 个汉字格，居中在格中间（状态 2 = 加载中，3 = 坏图片，其他不显示）
    c, wb = parse_template("{IDX1}{IDX0}/{TOT1}{TOT0}")
    x0 = W // 2 - wb // 2
    bands["S"] = ([(x0 + x, w, s) for x, w, s in c], [])
    cells = []
    for j in range(5):
        cx = SP_SLOT_X[j] + SP_SLOT_W[j] // 2
        for k in range(3):
            cells.append((cx - 3 * ST_CJK_W // 2 + k * ST_CJK_W, ST_CJK_W, LBL_BASE + j * 4 + k))
    bands["L"] = (cells, [])
    cl, wl = parse_template(TEMPLATES["A_L"])
    cr, wr = parse_template(TEMPLATES["A_R"])
    xl0 = LEFT_X + PAD
    xr0 = RIGHT_X1 - PAD - wr
    cells = [(xl0 + x, w, s) for x, w, s in cl] + [(xr0 + x, w, s) for x, w, s in cr]
    panels = [(LEFT_X, xl0 + wl + PAD), (xr0 - PAD, RIGHT_X1)]
    bands["A"] = (cells, panels)
    for b in "CDE":
        c, wb = parse_template(TEMPLATES[b])
        bands[b] = ([(LEFT_X + PAD + x, w, s) for x, w, s in c], [(LEFT_X, LEFT_X + PAD + wb + PAD)])
    for b in "BMPT":                                 # 右对齐到 RIGHT_X1（finish_inc_2_6：B = 右侧"关机 mm:ss"）
        c, wb = parse_template(TEMPLATES[b])
        x0 = RIGHT_X1 - PAD - wb
        assert x0 - PAD >= SPLIT_X
        bands[b] = ([(x0 + x, w, s) for x, w, s in c], [(x0 - PAD, RIGHT_X1)])
    for b in "CDE":
        assert max(px1 for _, px1 in bands[b][1]) <= SPLIT_X - 2
    for b in MENU_ROWS:                              # 第13课 菜单行：没有自己的底板（面板单独画）
        c, wb = parse_template(TEMPLATES[b])
        bands[b] = ([(MTEXT_X0 + x, w, s) for x, w, s in c], [])
        assert MTEXT_X0 + wb <= MX1 - 4
    assert MX1 < SPLIT_X and MY1 <= 380
    assert SBAR_X0 >= MTEXT_X0 + MVAL_DX + 3 * ST_ASC_W + 1 and SBAR_X0 + (SSEG_N - 1) * SSEG_P + SSEG_W <= MX1 - MHL_M
    # finish_inc_2_6 提示框两行：居中；cell 表里放在菜单第 2、3 行（b_sel 9、10）的"第二套"（ckey 第 5 位 = 1）
    for k, b in zip(WARN_ROWS, ("W1", "W2")):
        c, wb = parse_template(TEMPLATES[b])
        x0 = W // 2 - wb // 2
        bands[b] = ([(x0 + x, w, s) for x, w, s in c], [])
        assert WX0 + 8 <= x0 and x0 + wb <= WX1 - 8
    assert WY0 + 6 <= MY_ROW0 + WARN_ROWS[0] * MPITCH and MY_ROW0 + WARN_ROWS[1] * MPITCH + BAND_H + 6 <= WY1
    assert MY_ROW0 + WARN_ROWS[0] * MPITCH + BAND_H <= WBAR_Y0 < WBAR_Y1 <= MY_ROW0 + WARN_ROWS[1] * MPITCH
    assert WBAR_X0 + WSEG_N * WSEG_P <= WX1 - 8 and WSEG_P == 16
    return bands


BANDS = build_layout()
BAND_ORDER = ["A", "B", "C", "D", "E", "M", "P", "T"] + MENU_ROWS + ["S", "L"]   # 编号 0~7 状态行，8~12 菜单行，13/14 胶片条


def corner_inside(dx, dy):
    r = PANEL_R
    return (r - (dx + 0.5)) ** 2 + (r - (dy + 0.5)) ** 2 <= r * r


def sub_grad(row):
    t = row / (SB_H - 1)
    return tuple(int(round(a + (b - a) * t)) for a, b in zip(SUB_TOP, SUB_BOTTOM))


def a9(v):
    return v * 17 + (1 if v == 15 else 0)


# ---------------- 帧级状态（与 RTL 帧起点逻辑一致） ----------------
GROUP_STEP, BAND_STEP = 16, 32


class OsdState:
    """每帧起点更新一次。输入 status（字典）在 RTL 里也是在帧起点锁存的值。"""

    def __init__(self, xs_init=W, speed=SUB_SPEED):
        self.mode = 0
        self.f_stat = 0
        self.f_sub = 0
        self.f_band = {b: 0 for b in "BCDE"}
        self.xs = xs_init
        self.speed = speed
        self.band_y = {"A": 8, "B": None, "C": None, "D": None, "E": None, "M": None, "P": None, "T": None}
        for k, b in enumerate(MENU_ROWS):
            self.band_y[b] = None
        # 第13课 菜单
        self.f_menu = 0
        self.mvis = False
        self.msel = 0
        self.mlv = [5, 5, 5, 5]
        self.f_eff = {b: 0 for b in BAND_ORDER}
        self.f_sub_eff = 0
        self.fields = {}
        self.disp = {}          # 正在淡出的行沿用消失前最后一帧的字段，避免出现"错误 0""加载 5/4"
        # 第11课 提示行：上一帧的 SW4 / 暂停 / 循环，剩余帧数、淡入淡出
        self.q = {"layer": 0, "pause": 0, "loop": 0, "rst": 0}
        self.t_cnt = 0
        self.f_t = 0
        # 第14课 第3步 胶片条（本帧的值，RTL 里由 media_display_path 在帧起点给出，一帧内不变）
        self.strip, self.sf, self.sst = 0, 0, [0] * 5
        # finish_inc_2_6 定时关机：提示框淡化、秒数（不显示时冻结）、定时档位
        self.f_w = 0
        self.wvis = False
        self.wsec = 0
        self.slv = 0
        self.band_y["S"] = self.band_y["L"] = None       # 胶片条两行在 render() 最后单独画

    @staticmethod
    def step(v, target, step):
        return min(256, v + step) if target else max(0, v - step)

    def frame_start(self, st):
        """st: dict(idx_code, live, auto, loading, loaded, err, bad, vol, period_s, h, m, s)"""
        sleep = st.get("sleep", 0)                                    # finish_inc_2_6：关机淡出 / 关着 / 唤醒前
        stat_on = self.mode in (0, 2) and not sleep
        sub_on = self.mode in (0, 1) and not st.get("hide", 0) and not sleep   # 第14课 第3步：胶片条时字幕淡出
        self.strip, self.sf, self.sst = st.get("strip", 0), st.get("sf", 0), list(st.get("sst", [0] * 5))
        self.f_stat = self.step(self.f_stat, stat_on, GROUP_STEP)
        self.f_sub = self.step(self.f_sub, sub_on, GROUP_STEP)
        warn = bool(st.get("warn", 0))
        tgt = {"B": bool(st.get("son", 0)), "C": st["err"] != 0, "D": st["bad"] != 0,
               "E": st["merr"] != 0, "W": warn}
        new = field_glyphs(st)
        # B/C/D/E 行的字段只在该行"应当显示"时更新，淡出期间冻结（finish_inc_2_6：B = 剩余时间，W = 提示框秒数）
        hold = {"B": ["ZM1", "ZM0", "ZS1", "ZS0"], "C": ["ERR"], "D": ["BAD1", "BAD0"], "E": ["ME"], "W": ["WS1", "WS0"]}
        for f, v in new.items():
            owner = next((b for b, fs in hold.items() if f in fs), None)
            if owner is None or tgt[owner]:
                self.disp[f] = v
            elif f not in self.disp:
                self.disp[f] = BLANK        # RTL 复位值：该行从未显示过时字段为空
        for b in "BCDE":
            self.f_band[b] = self.step(self.f_band[b], tgt[b], BAND_STEP)
        # 第11课 提示行：SW4 / 暂停 / 循环 有变化就换提示（同一帧多个变化时按这个优先级），显示 TOAST_FRAMES 帧
        ev = None
        if st["layer"] != self.q["layer"]:
            ev = 1 if st["layer"] else 0
        elif st["pause"] != self.q["pause"]:
            ev = 2 if st["pause"] else 3
        elif st["loop"] != self.q["loop"]:
            ev = 4 if st["loop"] else 5
        elif st["rst"] != self.q["rst"]:
            ev = 6                                       # 第13课：恢复默认
        self.q = {k: st[k] for k in self.q}
        self.f_t = self.step(self.f_t, self.t_cnt != 0 and not sleep, BAND_STEP)
        if ev is not None:
            self.t_cnt = TOAST_FRAMES
            for i, ch in enumerate(TOAST[ev]):
                self.disp["T%d" % (i + 1)] = GID[(ch, "st")]
        elif self.t_cnt:
            self.t_cnt -= 1
        for i in range(4):
            self.disp.setdefault("T%d" % (i + 1), BLANK)
        self.band_y["T"] = T_Y if (self.t_cnt != 0 or self.f_t != 0) else None
        self.f_eff["T"] = self.f_t
        # 第13课 菜单：淡入淡出（不受显示模式影响），当前行、各档位在帧起点锁存
        self.f_menu = self.step(self.f_menu, st["menu"], MENU_STEP)
        self.mvis = bool(st["menu"]) or self.f_menu != 0
        self.msel = st["msel"]
        self.mlv = list(st["mlv"])
        self.slv = st.get("slv", 0)
        # finish_inc_2_6 提示框：淡入淡出（不受显示模式影响），秒数只在显示时更新
        self.f_w = self.step(self.f_w, warn, BAND_STEP)
        self.wvis = warn or self.f_w != 0
        if warn:
            r = st.get("rem", 0) & 0xFF
            self.wsec = (r >> 4) * 10 + (r & 15)
        for k, b in enumerate(MENU_ROWS):
            self.band_y[b] = (MY_ROW0 + k * MPITCH) if self.mvis else None
            self.f_eff[b] = self.f_menu
        y = 8 + SLOT_PITCH
        self.band_y["B"] = Z_Y if (tgt["B"] or self.f_band["B"] > 0) else None   # finish_inc_2_6：右侧固定位置，不占左侧槽
        for b in "CDE":
            if tgt[b] or self.f_band[b] > 0:
                self.band_y[b] = y
                y += SLOT_PITCH
            else:
                self.band_y[b] = None
        # 右侧音乐行：有 WAV 显示 M，否则 P（琶音）
        self.band_y["M"] = R_Y if st["wav"] else None
        self.band_y["P"] = None if st["wav"] else R_Y
        self.f_eff["A"] = self.f_stat
        self.f_eff["M"] = self.f_stat
        self.f_eff["P"] = self.f_stat
        for b in "BCDE":
            self.f_eff[b] = (self.f_band[b] * self.f_stat) >> 8
        self.f_sub_eff = self.f_sub
        self.xs -= self.speed
        if self.xs <= -strip_now()[0].shape[1]:       # finish_u_2：按字幕条实际宽度回绕
            self.xs = W
        self.fields = dict(self.disp)

    def press_key3(self):
        self.mode = (self.mode + 1) & 3


BLANK = -1


def right2(n):
    """两位数右对齐：(十位格, 个位格)，一位数时十位格空"""
    d = lambda v: GID[(str(v), "st")]
    return (d(n // 10) if n >= 10 else BLANK), d(n % 10)


def left2(n):
    """两位数左对齐：一位数时数字在前一格、后一格空"""
    d = lambda v: GID[(str(v), "st")]
    return (d(n // 10), d(n % 10)) if n >= 10 else (d(n), BLANK)


def field_glyphs(st):
    d = lambda n: GID[(str(n), "st")]
    dash = GID[("-", "st")]
    f = {}
    # 第14课：两位数，第几张右对齐（十位格不足时空）、共几张左对齐（个位格不足时空）
    if st["live"] == 0:
        f["IDX1"], f["IDX0"], f["TOT1"], f["TOT0"] = BLANK, dash, dash, BLANK
    else:
        f["IDX1"], f["IDX0"] = right2(st["idx"])
        f["TOT1"], f["TOT0"] = left2(st["live"])
    f["MA"] = GID[("自" if st["auto"] else "手", "st")]
    f["MB"] = GID[("动", "st")]
    p = st["period_s"]
    f["PT"] = d(p // 10) if p >= 10 else BLANK
    f["PO"] = d(p % 10)
    pct = (st["vol"] * 101) >> 8
    f["VH"] = d(1) if pct >= 100 else BLANK
    f["VT"] = d((pct % 100) // 10) if pct >= 10 else BLANK
    f["VO"] = d(pct % 10)
    for k, v in zip(["H1", "H0", "N1", "N0", "S1", "S0"],
                    [st["h"] // 10, st["h"] % 10, st["m"] // 10, st["m"] % 10, st["s"] // 10, st["s"] % 10]):
        f[k] = d(v)
    f["ERR"] = d(st["err"])
    f["BAD1"], f["BAD0"] = left2(st["bad"])
    # 第10课 音乐
    mi, mc = st["midx"], st["mcnt"]
    f["SI1"] = d(mi // 10) if mi >= 10 else BLANK
    f["SI0"] = d(mi % 10)
    f["SC1"] = d(mc // 10) if mc >= 10 else d(mc % 10)
    f["SC0"] = d(mc % 10) if mc >= 10 else BLANK
    for k, v in zip(["EM1", "EM0", "ES1", "ES0"], st["mel"]):
        f[k] = d(v) if st["mknown"] else dash
    for k, v in zip(["TM1", "TM0", "TS1", "TS0"], st["mtot"]):
        f[k] = d(v) if st["mknown"] else dash
    f["ME"] = d(st["merr"])
    # 第11课
    f["MI"] = GID[("⏸" if st["pause"] else "♪", "st")]
    f["LP"] = GID[("单", "st")] if st["loop"] else BLANK
    # 第13课 菜单数值：档位 0~10 -> -5..+5（符号 + 一位数字，0 没有符号）
    for k in range(4):
        dv = st["mlv"][k] - 5
        f["MS%d" % k] = GID[("+", "st")] if dv > 0 else GID[("-", "st")] if dv < 0 else BLANK
        f["MD%d" % k] = d(abs(dv))
    # finish_inc_2_6 定时关机：剩余时间 mm:ss、菜单档位（右对齐，"关"在"分"的位置）、提示框秒数（十位为 0 时空）
    r = st.get("rem", 0)
    f["ZM1"], f["ZM0"], f["ZS1"], f["ZS0"] = d(r >> 12), d((r >> 8) & 15), d((r >> 4) & 15), d(r & 15)
    m = SLEEP_PRESET[st.get("slv", 0)] >> 8
    f["SVA"] = d(m >> 4) if m >= 0x10 else BLANK
    f["SVB"] = d(m & 15) if m else BLANK
    f["SVC"] = GID[("分" if m else "关", "st")]
    f["WS1"] = d(1) if (r & 0xF0) else BLANK
    f["WS0"] = d(r & 15)
    return f


# ---------------- 逐像素渲染（参考模型） ----------------
def render(bg, state):
    """bg: (H,W,3) uint8 -> 叠加 OSD 后的 (H,W,3) uint8，与 RTL 逐位一致。"""
    out = bg.astype(np.int64).copy()
    # 覆盖层：tc*g9 部分与 Cp/Ap，先逐像素算 C、A
    C = np.zeros((H, W, 3), np.int64)
    A = np.zeros((H, W), np.int64)
    rom = ROM

    def put(y, x0, x1, tc, g9, o9, Cp, Ap, f):
        o9c = 256 - o9
        for ch in range(3):
            tcc = tc[ch] if np.ndim(tc) == 1 or isinstance(tc, tuple) else tc[:, ch]
            c = ((tcc * g9) >> 8) + ((Cp[ch] * o9c) >> 8)
            C[y, x0:x1, ch] = (c * f) >> 8
        a = o9 + ((Ap * o9c) >> 8)
        A[y, x0:x1] = (a * f) >> 8

    # 状态行（右侧音乐行最后画，只覆盖 x >= SPLIT_X，与 RTL 按 x 选行一致；finish_inc_2_6：B 是右侧行，放到最后）
    for b in [x for x in BAND_ORDER if x != "B"] + ["B"]:
        y0 = state.band_y[b]
        if y0 is None:
            continue
        xa = SPLIT_X if b in ("B", "M", "P", "T") else 0
        if b in MENU_ROWS or b in ("W1", "W2"):
            continue                                     # 菜单行在下面单独画
        f = state.f_eff[b]
        pc, pa, tc = BAND_STYLE[b]
        Cp = [(c * pa) >> 8 for c in pc]
        cells, panels = BANDS[b]
        for ry in range(BAND_H):
            y = y0 + ry
            ap = np.zeros(W, np.int64)
            for (px0, px1) in panels:
                for x in range(px0, px1):
                    dx = min(x - px0, px1 - 1 - x)
                    dy = min(ry, BAND_H - 1 - ry)
                    if dx < PANEL_R and dy < PANEL_R and not corner_inside(dx, dy):
                        continue
                    ap[x] = pa
            g = np.zeros(W, np.int64)
            o = np.zeros(W, np.int64)
            row = ry - TEXT_DY
            if 0 <= row < ST_H:
                for (cx, cw, src) in cells:
                    gid = src if src < FIELD_BASE else state.fields[FIELDS[src - FIELD_BASE]]
                    if gid == BLANK:
                        continue
                    base, gw, gh, packed = GTABLE[gid]
                    if packed:
                        idx = row * gw + np.arange(cw)
                        v = rom[base + (idx >> 1)]
                        h4 = np.where(idx & 1, v >> 4, v & 15)
                        g[cx:cx + cw] = h4
                        o[cx:cx + cw] = h4
                    else:
                        v = rom[base + row * gw: base + row * gw + cw]
                        g[cx:cx + cw] = v >> 4
                        o[cx:cx + cw] = v & 15
            g9 = np.array([a9(int(v)) for v in g])
            o9 = np.array([a9(int(v)) for v in o])
            apm = np.where(ap > 0, ap, 0)
            Cpm = [np.where(ap > 0, c, 0) for c in Cp]
            o9c = 256 - o9
            for ch in range(3):
                c = ((tc[ch] * g9) >> 8) + ((Cpm[ch] * o9c) >> 8)
                C[y, xa:, ch] = ((c * f) >> 8)[xa:]
            A[y, xa:] = (((o9 + ((apm * o9c) >> 8)) * f) >> 8)[xa:]
    # 第13课 调节菜单（与 RTL 的区域判断一一对应）
    if state.mvis:
        f = state.f_menu
        for y in range(MY0, MY1):
            k = next((r for r in range(6) if MY_ROW0 + r * MPITCH <= y < MY_ROW0 + r * MPITCH + BAND_H), None)
            rr = y - (MY_ROW0 + k * MPITCH) if k is not None else 0
            sel = (k == state.msel)
            tc = M_TC_SEL if sel else M_TC
            # 本行文字：每列的 g/o
            g = np.zeros(W, np.int64)
            o = np.zeros(W, np.int64)
            row = rr - TEXT_DY
            if k is not None and 0 <= row < ST_H:
                for (cx, cw, src) in BANDS[MENU_ROWS[k]][0]:
                    gid = src if src < FIELD_BASE else state.fields[FIELDS[src - FIELD_BASE]]
                    if gid == BLANK:
                        continue
                    base, gw, gh, packed = GTABLE[gid]
                    idx = row * gw + np.arange(cw)
                    v = rom[base + (idx >> 1)]
                    h4 = np.where(idx & 1, v >> 4, v & 15)
                    g[cx:cx + cw] = h4
                    o[cx:cx + cw] = h4
            for x in range(MX0, MX1):
                dx, dy = min(x - MX0, MX1 - 1 - x), min(y - MY0, MY1 - 1 - y)
                if dx < PANEL_R and dy < PANEL_R and not corner_inside(dx, dy):
                    continue
                code = 1
                if k is not None:
                    hx0, hx1 = MX0 + MHL_M, MX1 - MHL_M
                    hl = False
                    if sel and hx0 <= x < hx1:
                        hdx, hdy = min(x - hx0, hx1 - 1 - x), min(rr, BAND_H - 1 - rr)
                        hl = not (hdx < PANEL_R and hdy < PANEL_R and not corner_inside(hdx, hdy))
                    bo = x - MBAR_X0
                    seg = k < 4 and MSEG_Y0 <= rr < MSEG_Y1 and 0 <= bo < MSEG_N * MSEG_P and (bo % MSEG_P) < MSEG_W
                    on = seg and (bo // MSEG_P) <= state.mlv[k]
                    so = x - SBAR_X0                     # finish_inc_2_6 第 6 行"定时"的 5 格
                    if k == 5 and MSEG_Y0 <= rr < MSEG_Y1 and 0 <= so < SSEG_N * SSEG_P and (so % SSEG_P) < SSEG_W:
                        seg, on = True, (so // SSEG_P) < state.slv
                    if seg:
                        code = (6 if sel else 5) if on else (4 if hl else 3)
                    elif hl:
                        code = 2
                (Cp, Ap) = MENU_REG[code]
                g9, o9 = a9(int(g[x])), a9(int(o[x]))
                o9c = 256 - o9
                for ch in range(3):
                    c = ((tc[ch] * g9) >> 8) + ((Cp[ch] * o9c) >> 8)
                    C[y, x, ch] = (c * f) >> 8
                A[y, x] = ((o9 + ((Ap * o9c) >> 8)) * f) >> 8
    # finish_inc_2_6 最后 10 秒提示框（菜单关着才画；与菜单互斥：倒计时只在菜单关着时走，提示时的按键会被吞掉）
    if state.wvis and not state.mvis:
        f = state.f_w
        for y in range(WY0, WY1):
            k = next((r for r in WARN_ROWS if MY_ROW0 + r * MPITCH <= y < MY_ROW0 + r * MPITCH + BAND_H), None)
            rr = y - (MY_ROW0 + k * MPITCH) if k is not None else 0
            tc = (W_TC1 if k == WARN_ROWS[0] else W_TC2) if k is not None else (0, 0, 0)
            g = np.zeros(W, np.int64)
            row = rr - TEXT_DY
            if k is not None and 0 <= row < ST_H:
                for (cx, cw, src) in BANDS["W1" if k == WARN_ROWS[0] else "W2"][0]:
                    gid = src if src < FIELD_BASE else state.fields[FIELDS[src - FIELD_BASE]]
                    if gid == BLANK:
                        continue
                    base, gw, gh, packed = GTABLE[gid]
                    idx = row * gw + np.arange(cw)
                    v = rom[base + (idx >> 1)]
                    g[cx:cx + cw] = np.where(idx & 1, v >> 4, v & 15)
            for x in range(WX0, WX1):
                dx, dy = min(x - WX0, WX1 - 1 - x), min(y - WY0, WY1 - 1 - y)
                if dx < PANEL_R and dy < PANEL_R and not corner_inside(dx, dy):
                    continue
                code = 1
                bx = x - WBAR_X0
                if WBAR_Y0 <= y < WBAR_Y1 and 0 <= bx < WSEG_N * WSEG_P and (bx % WSEG_P) < WSEG_W:
                    code = 3 if (bx // WSEG_P) < state.wsec else 2
                (Cp, Ap) = WARN_REG[code]
                g9 = a9(int(g[x]))
                o9c = 256 - g9
                for ch in range(3):
                    c = ((tc[ch] * g9) >> 8) + ((Cp[ch] * o9c) >> 8)
                    C[y, x, ch] = (c * f) >> 8
                A[y, x] = ((g9 + ((Ap * o9c) >> 8)) * f) >> 8
    # 字幕带
    f = state.f_sub_eff
    xs = state.xs
    for y in range(SUB_BAND_Y0, SUB_BAND_Y1):
        g = np.zeros(W, np.int64)
        o = np.zeros(W, np.int64)
        row = y - SUB_TEXT_Y0
        tc = sub_grad(row) if 0 <= row < SB_H else (0, 0, 0)
        if 0 <= row < SB_H:                        # finish_u_2：字幕条（卡里的 SUB.BMP 或默认标语）
            sg, so = strip_now()
            for col in range(sg.shape[1]):
                x = xs + col
                if 0 <= x < W:
                    g[x] = sg[row, col]
                    o[x] = so[row, col]
        g9 = np.array([a9(int(v)) for v in g])
        o9 = np.array([a9(int(v)) for v in o])
        o9c = 256 - o9
        for ch in range(3):
            c = ((tc[ch] * g9) >> 8) + ((0 * o9c) >> 8)
            C[y, :, ch] = (c * f) >> 8
        A[y, :] = ((o9 + ((SUB_PANEL_ALPHA * o9c) >> 8)) * f) >> 8
    # 第14课 第3步 胶片条两行文字：这两行整行归胶片条（RTL 里这两行 b_hit 最优先，没有底板），盖掉别的
    if state.strip:
        lgid = {2: [GID[(ch, "st")] for ch in LBL_LOAD], 3: [GID[(ch, "st")] for ch in LBL_BAD]}
        for b, y0, tc in (("S", SN_Y, S_TC), ("L", SL_Y, L_TC)):
            for ry in range(BAND_H):
                y = y0 + ry
                g = np.zeros(W, np.int64)
                row = ry - TEXT_DY
                if 0 <= row < ST_H:
                    for (cx, cw, src) in BANDS[b][0]:
                        if src >= LBL_BASE:
                            sj, sk = (src - LBL_BASE) >> 2, (src - LBL_BASE) & 3
                            gid = lgid[state.sst[sj]][sk] if state.sst[sj] in lgid else BLANK
                        elif src >= FIELD_BASE:
                            gid = state.fields[FIELDS[src - FIELD_BASE]]
                        else:
                            gid = src
                        if gid == BLANK:
                            continue
                        base, gw, gh, packed = GTABLE[gid]
                        idx = row * gw + np.arange(cw)
                        v = rom[base + (idx >> 1)]
                        g[cx:cx + cw] = np.where(idx & 1, v >> 4, v & 15)
                g9 = np.array([a9(int(v)) for v in g])
                for ch in range(3):
                    C[y, :, ch] = (((tc[ch] * g9) >> 8) * state.sf) >> 8
                A[y, :] = (g9 * state.sf) >> 8
    res = C + ((out * (256 - A)[:, :, None]) >> 8)
    return np.minimum(res, 255).astype(np.uint8)


# ---------------- 输出 RTL 文件 ----------------
def write_rom_files():
    os.makedirs(OUT_HDL, exist_ok=True)
    with open(os.path.join(OUT_HDL, "osd_font.hex"), "w") as f:      # 16 位一行，偶数在低字节
        f.write("\n".join("%02x%02x" % (ROM[i + 1], ROM[i]) for i in range(0, ROM_DEPTH, 2)) + "\n")
    with open(os.path.join(OUT_HDL, "osd_font.mif"), "w") as f:
        f.write("WIDTH=16;\nDEPTH=%d;\nADDRESS_RADIX=HEX;\nDATA_RADIX=HEX;\nCONTENT BEGIN\n" % (ROM_DEPTH // 2))
        for i in range(0, ROM_DEPTH, 2):
            f.write("%X : %02X%02X;\n" % (i // 2, ROM[i + 1], ROM[i]))
        f.write("END;\n")


def vcase(name, width, keyw, entries, default):
    s = "    function [%d:0] %s;\n        input [%d:0] k;\n        begin\n            case (k)\n" % (width - 1, name, keyw - 1)
    for k, v in entries:
        s += "                %d'd%d: %s = %d'd%d;\n" % (keyw, k, name, width, v)
    s += "                default: %s = %d'd%d;\n            endcase\n        end\n    endfunction\n\n" % (name, width, default)
    return s


def write_layout():
    L = []
    L.append("// 由 design/osd_gen.py 自动生成，不要手改\n")
    L.append("// 字体：思源黑体 Source Han Sans CN（SIL OFL）。字库 %d 像素 / %d\n\n" % (ROM_USED, ROM_DEPTH))
    gid_entries = [(i, b) for i, (b, w, h, _) in enumerate(GTABLE)]
    assert len(GTABLE) <= 127
    L.append(vcase("glyph_base", 16, 7, gid_entries, 0))
    L.append(vcase("glyph_w", 6, 7, [(i, w) for i, (b, w, h, _) in enumerate(GTABLE)], 1))
    L.append(vcase("glyph_packed", 1, 7, [(i, 1 if p else 0) for i, (b, w, h, p) in enumerate(GTABLE)], 0))
    L.append("    localparam integer N_FIELDS = %d;\n" % len(FIELDS))
    for f in FIELDS:
        L.append("    localparam [7:0] F_%s = 8'd%d;\n" % (f, FIELD_CODE[f]))
    L.append("    localparam [6:0] G_DIGIT0 = 7'd%d, G_DASH = 7'd%d, G_ZI = 7'd%d, G_SHOU = 7'd%d, G_DONG = 7'd%d;\n"
             % (GID[("0", "st")], GID[("-", "st")], GID[("自", "st")], GID[("手", "st")], GID[("动", "st")]))
    sb = [GID[(c, "sb")] for c in SUBTITLE]
    L.append("    localparam integer SUB_N = %d, SUB_W = %d, SUB_H = %d, SUB_TOTAL = %d, SUB_SPEED = %d;\n"
             % (len(SUBTITLE), SB_W, SB_H, SUB_TOTAL, SUB_SPEED))
    L.append("    localparam integer SUB_BAND_Y0 = %d, SUB_BAND_Y1 = %d, SUB_TEXT_Y0 = %d, SUB_PANEL_ALPHA = %d;\n"
             % (SUB_BAND_Y0, SUB_BAND_Y1, SUB_TEXT_Y0, SUB_PANEL_ALPHA))
    L.append("    localparam integer BAND_H = %d, TEXT_DY = %d, SLOT_PITCH = %d, PANEL_R = %d, ST_H = %d;\n"
             % (BAND_H, TEXT_DY, SLOT_PITCH, PANEL_R, ST_H))
    L.append("    localparam integer GROUP_STEP = %d, BAND_STEP = %d;\n" % (GROUP_STEP, BAND_STEP))
    L.append("    localparam SHOW_LOADING = 1'b%d;\n" % (1 if SHOW_LOADING else 0))
    # finish_inc_2_6 定时关机
    L.append("    localparam integer WX0 = %d, WX1 = %d, WY0 = %d, WY1 = %d, WBAR_X0 = %d, WBAR_Y0 = %d, WBAR_Y1 = %d, WSEG_N = %d, WSEG_W = %d;\n"
             % (WX0, WX1, WY0, WY1, WBAR_X0, WBAR_Y0, WBAR_Y1, WSEG_N, WSEG_W))
    L.append("    localparam integer SBAR_X0 = %d;\n" % SBAR_X0)
    L.append("    localparam [6:0] G_FEN = 7'd%d, G_GUAN = 7'd%d;\n" % (GID[("分", "st")], GID[("关", "st")]))
    se = []
    for so in range(SSEG_N * SSEG_P):
        if so % SSEG_P < SSEG_W:
            se.append((so, 8 | (so // SSEG_P)))
    L.append(vcase("sl_seg", 4, 7, se, 0))
    L.append("    localparam integer R_Y = %d, SPLIT_X = %d, T_Y = %d, TOAST_FRAMES = %d;\n" % (R_Y, SPLIT_X, T_Y, TOAST_FRAMES))
    L.append("    localparam [6:0] G_PLUS = 7'd%d;\n" % GID[("+", "st")])
    L.append("    localparam integer FIELD_BASE = %d;\n" % FIELD_BASE)
    # 第13课 菜单
    L.append("    localparam integer MX0 = %d, MX1 = %d, MY0 = %d, MY1 = %d, MY_ROW0 = %d, MPITCH = %d, MHL_M = %d;\n"
             % (MX0, MX1, MY0, MY1, MY_ROW0, MPITCH, MHL_M))
    L.append("    localparam integer MBAR_X0 = %d, MSEG_P = %d, MSEG_W = %d, MSEG_N = %d, MSEG_Y0 = %d, MSEG_Y1 = %d, MENU_STEP = %d;\n"
             % (MBAR_X0, MSEG_P, MSEG_W, MSEG_N, MSEG_Y0, MSEG_Y1, MENU_STEP))
    L.append("    localparam [23:0] M_TC = 24'h%06x, M_TC_SEL = 24'h%06x;\n"
             % ((M_TC[0] << 16) | (M_TC[1] << 8) | M_TC[2], (M_TC_SEL[0] << 16) | (M_TC_SEL[1] << 8) | M_TC_SEL[2]))
    L.append(vcase("menu_cp", 24, 3, [(i, (c[0] << 16) | (c[1] << 8) | c[2]) for i, (c, a) in enumerate(MENU_REG)], 0))
    L.append(vcase("menu_ap", 9, 3, [(i, a) for i, (c, a) in enumerate(MENU_REG)], 0))
    L.append(vcase("period_s", 7, 3, list(enumerate(PERIODS)), 3))
    L.append("    localparam [6:0] G_NOTE = 7'd%d, G_PAUSE = 7'd%d, G_DAN = 7'd%d;\n\n"
             % (GID[("♪", "st")], GID[("⏸", "st")], GID[("单", "st")]))
    # 第14课 第3步 胶片条：标签字形（第几个字 -> 字形号）
    L.append(vcase("lbl_load", 7, 2, [(k, GID[(ch, "st")]) for k, ch in enumerate(LBL_LOAD)], 127))
    L.append(vcase("lbl_bad", 7, 2, [(k, GID[(ch, "st")]) for k, ch in enumerate(LBL_BAD)], 127))
    # 第11课 提示行：{种类[2:0], 第几个字[1:0]} -> 字形号
    L.append(vcase("toast_gid", 7, 5, [((k << 2) | i, GID[(ch, "st")]) for k, t in enumerate(TOAST) for i, ch in enumerate(t)], 127))
    L.append(vcase("sub_gid", 7, 3, list(enumerate(sb)), sb[0]))
    # 字幕渐变色
    L.append(vcase("sub_grad", 24, 6, [(r, (lambda c: (c[0] << 16) | (c[1] << 8) | c[2])(sub_grad(r))) for r in range(SB_H)], 0))
    # 状态行：字符格表（band 0..3 = A..D），每行按 x 升序，结尾 x0=1023
    maxc = max(len(BANDS[b][0]) for b in BAND_ORDER)
    L.append("    localparam integer MAX_CELLS = %d;\n" % (maxc + 1))
    ent_x, ent_w, ent_s = [], [], []
    alt = {BAND_ORDER.index(MENU_ROWS[k]): b for k, b in zip(WARN_ROWS, ("W1", "W2"))}
    for bi, b in enumerate(BAND_ORDER):
        for sub, bb in ((0, b), (32, alt.get(bi))):
            if bb is None:
                continue
            cells = sorted(BANDS[bb][0])
            assert len(cells) + 1 <= 32, bb      # finish_inc_2_6：ckey 第 5 位用来选第二套，每行最多 31 格
            for ci, (x, w, s) in enumerate(cells + [(1023, 1, 0)]):
                key = (bi << 6) | sub | ci
                ent_x.append((key, x)); ent_w.append((key, w)); ent_s.append((key, s))
    # finish_u_3_1（省 LUT）：字符格表不再生成组合函数 cell_x0 / cell_w / cell_src，改成 ROM 初值
    #   osd_cell.mif / .hex：1024 x 23 位 = {x0[9:0], w[4:0], src[7:0]}，地址 = ckey（没列出的格 = 结尾 1023 / 1 / 0）
    #   osd_engine.v 用一块 BRAM32K 同步读（提前一个像素算好下一拍的 ckey，见那里的说明）
    cw = [(1023 << 13) | (1 << 8) | 0] * 1024
    for (k, x), (_, w), (_, s) in zip(ent_x, ent_w, ent_s):
        cw[k] = (x << 13) | (w << 8) | s
    with open(os.path.join(OUT_HDL, "osd_cell.hex"), "w") as f:
        f.write("\n".join("%06x" % v for v in cw) + "\n")
    with open(os.path.join(OUT_HDL, "osd_cell.mif"), "w") as f:
        f.write("WIDTH=23;\nDEPTH=1024;\nADDRESS_RADIX=HEX;\nDATA_RADIX=HEX;\nCONTENT BEGIN\n")
        for i, v in enumerate(cw):
            f.write("%x : %06x;\n" % (i, v))
        f.write("END;\n")
    # 底板：每行最多 2 块
    pe0, pe1, pv = [], [], []
    for bi, b in enumerate(BAND_ORDER):
        ps = BANDS[b][1]
        for pi in range(2):
            key = (bi << 1) | pi
            if pi < len(ps):
                pe0.append((key, ps[pi][0])); pe1.append((key, ps[pi][1]))
            else:
                pe0.append((key, 1023)); pe1.append((key, 1023))
    L.append(vcase("panel_x0", 10, 5, pe0, 1023))
    L.append(vcase("panel_x1", 10, 5, pe1, 1023))
    # 颜色：底板预乘色、底板透明度、文字色
    ST_ORDER = BAND_ORDER[:8]                    # 颜色表只给状态行（菜单行的颜色在 RTL 里单独选）
    L.append(vcase("band_cp", 24, 3, [(bi, (lambda pc, pa: ((pc[0] * pa >> 8) << 16) | ((pc[1] * pa >> 8) << 8) | (pc[2] * pa >> 8))(*BAND_STYLE[b][:2])) for bi, b in enumerate(ST_ORDER)], 0))
    L.append(vcase("band_ap", 9, 3, [(bi, BAND_STYLE[b][1]) for bi, b in enumerate(ST_ORDER)], 0))
    L.append(vcase("band_tc", 24, 3, [(bi, (lambda c: (c[0] << 16) | (c[1] << 8) | c[2])(BAND_STYLE[b][2])) for bi, b in enumerate(ST_ORDER)], 0))
    # 圆角查表：{dx[2:0], dy[2:0]} -> 在内
    L.append(vcase("corner_in", 1, 6, [((dx << 3) | dy, 1 if corner_inside(dx, dy) else 0)
                                        for dx in range(PANEL_R) for dy in range(PANEL_R)], 1))
    # 第14课准备（省 LUT）：颜色不在 S0 里按值选，而是选一个"颜色号"，S0B 用颜色号查表（输出寄存，TD 推成 BRAM ROM）
    #   文字色 tc_rom（6 位号）：0 无，1~8 状态行 0~7 的文字色，9/10 菜单文字（普通/当前行），16~16+SUB_H-1 字幕渐变
    #   底板 cpap_rom（5 位号）：{预乘色 24 位, 透明度 9 位}；0 无，1~8 状态行 0~7，9~15 菜单区域码 0~6，16 字幕底板
    assert 16 + SB_H <= 64
    rgb = lambda c: (c[0] << 16) | (c[1] << 8) | c[2]
    tce = [(1 + bi, rgb(BAND_STYLE[b][2])) for bi, b in enumerate(ST_ORDER)]
    tce += [(9, rgb(M_TC)), (10, rgb(M_TC_SEL)), (11, rgb(S_TC)), (12, rgb(L_TC))]   # 第14课 第3步：胶片条 "n/N" / 标签
    tce += [(13, rgb(W_TC1)), (14, rgb(W_TC2))]                                        # finish_inc_2_6 提示框两行
    tce += [(16 + r, rgb(sub_grad(r))) for r in range(SB_H)]
    L.append(vcase("tc_rom", 24, 6, tce, 0))
    cpe = [(1 + bi, (((lambda pc, pa: ((pc[0] * pa >> 8) << 16) | ((pc[1] * pa >> 8) << 8) | (pc[2] * pa >> 8))(*BAND_STYLE[b][:2])) << 9)
            | BAND_STYLE[b][1]) for bi, b in enumerate(ST_ORDER)]
    cpe += [(9 + i, (rgb(c) << 9) | a) for i, (c, a) in enumerate(MENU_REG)]
    cpe += [(16, SUB_PANEL_ALPHA)]
    cpe += [(16 + i, (rgb(c) << 9) | a) for i, (c, a) in enumerate(WARN_REG) if i > 0]   # finish_inc_2_6 提示框 17~19
    assert len(MENU_REG) <= 7
    L.append(vcase("cpap_rom", 33, 5, cpe, 0))
    # 音量百分比 -> 十位/个位（0~100）
    L.append(vcase("pct_tens", 4, 7, [(p, (p % 100) // 10) for p in range(101)], 0))
    L.append(vcase("pct_ones", 4, 7, [(p, p % 10) for p in range(101)], 0))
    # 第14课（省 LUT）：两位数 0~99 -> 十位 / 个位数字的字形号（= G_DIGIT0 + 数字，查表里直接给出，不用再加）
    g0 = GID[("0", "st")]
    L.append(vcase("dec_tg", 7, 7, [(v, g0 + v // 10) for v in range(100)], g0))
    L.append(vcase("dec_og", 7, 7, [(v, g0 + v % 10) for v in range(100)], g0))
    L.append(row_rom_vcase())
    with open(os.path.join(OUT_HDL, "osd_layout.vh"), "w", encoding="utf-8") as f:
        f.write("".join(L))


def row_rom_vcase():
    """第14课（省 LUT）：OSD 按行的表 row_rom（9 位行号 -> 35 位），算法和 osd_engine.v 原来的组合逻辑逐位相同。
    第14课 第3步：高 7 位加胶片条两行 {"n/N" 行, 标签行, 行内第几行}（和别的带分开记行内序号，所以可以和菜单 / 字幕的行重叠）"""
    M11 = 0x7FF

    def near3(dl, dr):
        nl, nr = dl < PANEL_R, dr < PANEL_R
        lo = (min(dl & 7, dr & 7) if (nl and nr) else (dl & 7) if nl else (dr & 7))
        return ((1 if (nl or nr) else 0) << 3) | lo

    ents = []
    for ay in range(512):
        offs = set()
        sl_hit, sl_k = 0, 0
        for i in range(5):
            y0 = 8 + i * SLOT_PITCH
            if y0 <= ay < y0 + BAND_H:
                sl_hit, sl_k = 1, i
                offs.add(ay - y0)
        r_row = 1 if R_Y <= ay < R_Y + BAND_H else 0
        t_row = 1 if T_Y <= ay < T_Y + BAND_H else 0
        if r_row:
            offs.add(ay - R_Y)
        if t_row:
            offs.add(ay - T_Y)
        m_yin = 1 if MY0 <= ay < MY1 else 0
        m_row_ok, m_r, m_ry = 0, 0, 0
        for i in range(6):
            y0 = MY_ROW0 + i * MPITCH
            if y0 <= ay < y0 + BAND_H:
                m_row_ok, m_r, m_ry = 1, i, ay - y0
        if m_row_ok:
            offs.add(m_ry)
        hdy = m_ry if m_ry < BAND_H - 1 - m_ry else BAND_H - 1 - m_ry
        h_ny = 1 if hdy < PANEL_R else 0
        m_segy = 1 if (m_row_ok and m_r != 4 and MSEG_Y0 <= m_ry < MSEG_Y1) else 0   # finish_inc_2_6：第 6 行也有格
        mny = near3((ay - MY0) & M11, (MY1 - 1 - ay) & M11)
        sub_band = 1 if SUB_BAND_Y0 <= ay < SUB_BAND_Y1 else 0
        sub_row_ok = 1 if SUB_TEXT_Y0 <= ay < SUB_TEXT_Y0 + SB_H else 0
        if sub_row_ok:
            offs.add(ay - SUB_TEXT_Y0)
        assert len(offs) <= 1, (ay, offs)            # 同一行里各个带的行内序号必须相同
        roff = offs.pop() if offs else 0
        assert roff < 64 and (not m_row_ok or m_ry < 32)
        s_nrow = 1 if SN_Y <= ay < SN_Y + BAND_H else 0
        s_lrow = 1 if SL_Y <= ay < SL_Y + BAND_H else 0
        assert not (s_nrow and s_lrow)
        soff = (ay - SN_Y) if s_nrow else (ay - SL_Y) if s_lrow else 0
        v = 0
        for w, x in [(1, s_nrow), (1, s_lrow), (5, soff),
                     (1, sl_hit), (3, sl_k), (1, r_row), (1, t_row), (1, m_yin), (1, m_row_ok), (3, m_r),
                     (1, h_ny), (3, hdy & 7), (1, m_segy), (4, mny), (1, sub_band), (1, sub_row_ok), (6, roff)]:
            assert 0 <= x < (1 << w)
            v = (v << w) | x
        if v:
            ents.append((ay, v))
    return vcase("row_rom", 35, 9, ents, 0)


def demo_background():
    y = np.linspace(0, 1, H)[:, None]
    x = np.linspace(0, 1, W)[None, :]
    r = 90 + 120 * y + 60 * x
    g = 150 + 60 * y - 40 * x
    b = 230 - 150 * y
    img = np.dstack([r + 0 * x, g + 0 * x, b + 0 * x]).clip(0, 255).astype(np.uint8)
    img[300:, :] = (70, 120, 50)
    img[330:420, 60:260] = (245, 245, 240)
    img[60:200, 420:600] = (250, 220, 90)
    return img


DEMO_STATUS = dict(idx=7, live=23, auto=1, loading=0, loaded=4, err=0, bad=0, vol=255, period_s=3, h=0, m=12, s=34,
                   wav=1, midx=2, mcnt=3, mknown=1, mel=(0, 1, 2, 3), mtot=(0, 3, 4, 5), merr=0,
                   layer=0, pause=0, loop=0, rst=0, menu=0, msel=0, mlv=(5, 5, 5, 5),
                   slv=0, rem=0, son=0, warn=0, sleep=0)


def run_frames(st_list, key3_frames=()):
    """依次执行若干帧起点，返回最后的状态。"""
    s = OsdState()
    for i, st in enumerate(st_list):
        if i in key3_frames:
            s.press_key3()
        s.frame_start(st)
    return s


def main():
    write_rom_files()
    write_layout()
    print("字库：%d 个字形，%d / %d 像素；版面：A 行 %d 格" % (len(GLYPHS), ROM_USED, ROM_DEPTH, len(BANDS["A"][0])))
    bg = demo_background()
    # 预览 1：正常状态，淡入完成，字幕滚到中间
    s = run_frames([DEMO_STATUS] * 20)
    s.xs = 240
    Image.fromarray(render(bg, s)).save(os.path.join(HERE, "preview_1_normal.png"))
    # 预览 2：加载中 + 读卡错误 + 坏图
    st = dict(DEMO_STATUS, idx=12, loading=1, loaded=2, err=1, bad=12, merr=3, midx=12, mcnt=35)
    s = run_frames([st] * 20)
    s.xs = 380
    Image.fromarray(render(bg, s)).save(os.path.join(HERE, "preview_2_loading_error.png"))
    # 预览 3：上电黑屏时正在加载
    st = dict(DEMO_STATUS, idx=1, loading=1, loaded=0, h=0, m=0, s=1, wav=0)
    s = run_frames([st] * 20)
    s.xs = 520
    Image.fromarray(render(np.zeros_like(bg), s)).save(os.path.join(HERE, "preview_3_powerup_black.png"))
    # 放大看抗锯齿
    big = Image.open(os.path.join(HERE, "preview_1_normal.png")).crop((0, 0, 360, 44)).resize((1080, 132), Image.NEAREST)
    big.save(os.path.join(HERE, "preview_4_zoom_status.png"))
    big = Image.open(os.path.join(HERE, "preview_1_normal.png")).crop((220, 428, 440, 480)).resize((660, 156), Image.NEAREST)
    big.save(os.path.join(HERE, "preview_5_zoom_subtitle.png"))
    big = Image.open(os.path.join(HERE, "preview_2_loading_error.png")).crop((300, 0, 640, 120)).resize((1020, 360), Image.NEAREST)
    big.save(os.path.join(HERE, "preview_6_zoom_music.png"))
    # 第11课 预览 8：拨到音乐层（提示"音乐模式"）+ 暂停 + 单曲循环
    st0 = dict(DEMO_STATUS)
    st1 = dict(DEMO_STATUS, layer=1, pause=1, loop=1)
    s = run_frames([st0] * 10 + [dict(st0, layer=1)] * 2 + [dict(st0, layer=1, loop=1)] * 2 + [st1] * 12)
    s.xs = 240
    Image.fromarray(render(bg, s)).save(os.path.join(HERE, "preview_8_toast_pause_loop.png"))
    big = Image.open(os.path.join(HERE, "preview_8_toast_pause_loop.png")).crop((300, 0, 640, 120)).resize((1020, 360), Image.NEAREST)
    big.save(os.path.join(HERE, "preview_8_zoom.png"))
    # 第13课 预览 10：调节菜单（亮度 +2、饱和度 +1、色温 -3，当前在亮度，轮播 15 秒）
    import color_gen as cg
    stm = dict(DEMO_STATUS, menu=1, msel=0, mlv=(7, 5, 6, 2), period_s=15)
    s = run_frames([stm] * 20)
    s.xs = 240
    img = cg.apply(cg.test_card(), cg.ColorState().tables())
    Image.fromarray(render(img, s)).save(os.path.join(HERE, "preview_10_menu.png"))
    Image.open(os.path.join(HERE, "preview_10_menu.png")).crop((0, 180, 240, 390)).resize((720, 630), Image.NEAREST).save(
        os.path.join(HERE, "preview_10_menu_zoom.png"))
    # finish_inc_2_6 预览 12：菜单第 6 行"定时 30 分" + 右侧剩余时间；预览 13：最后 10 秒提示框
    st12 = dict(DEMO_STATUS, menu=1, msel=5, slv=3, son=1, rem=0x2959)
    s = run_frames([st12] * 20)
    s.xs = 240
    Image.fromarray(render(bg, s)).save(os.path.join(HERE, "preview_12_sleep_menu.png"))
    st13 = dict(DEMO_STATUS, slv=1, son=1, rem=0x0007, warn=1)
    s = run_frames([st13] * 20)
    s.xs = 240
    Image.fromarray(render(bg, s)).save(os.path.join(HERE, "preview_13_sleep_warn.png"))
    print("预览图已生成")


if __name__ == "__main__":
    sys.exit(main())
