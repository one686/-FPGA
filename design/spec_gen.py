# -*- coding: utf-8 -*-
"""音频频谱（32 根柱，A′ 四阶恒 Q 滤波器组）—— 版面 / 系数生成器 + 与 RTL 逐位一致的 Python 参考模型。

用法：
  python spec_gen.py            生成 src/user_source/hdl_source/display/spec_layout.vh 和预览图
  python spec_gen.py --test     定点检查（SONG1 + 满幅扫频 + 静音，和浮点比柱高；看静音能不能回 0）
  （被 sim/check_osd.py、sim/check_spec.py import，作为仿真比对的标准答案）

信号处理（每个送进 HDMI 的音频样本，音量调节之后）：
  m = (L + R) >>> 1                                          单声道 16 位
  高组：第 17~31 根柱（1182 Hz~16 kHz），采样率 48 kHz，输入 m
  低组：第 0~16 根柱（50~982 Hz），8 个样本求和再 >>> 3（一级 CIC = 滑动平均抽 8）-> 6 kHz，输入 xl（16 位）
        （混叠抑制：300 Hz 以下 > 25 dB，1 kHz 约 14 dB；比三级 CIC 省约 250 LUT）
  每根柱两个相同的二阶带通节串联（四阶），节 1：
     y = sat18( trunc0( (B0*(x - x2) << G  +  C1*y1  +  C2*y2) / 2^CF ) )     trunc0 = 向零舍入
  节 2（输入是节 1 的输出 y，差值 19 位，先 >>> 1，用 2*B0）：
     z = sat18( trunc0( (B2*((y - y2) >>> 1)  +  C1*z1  +  C2*z2) / 2^CF ) )
  累加：acc += |z| >>> G（高组 806 个样本左右/帧，低组 1/8；低组电平 +12 dB 补回）
  帧标记：电平 lv = clamp(level(acc) - LVL_OFS[b], 0, HMAX)（level = 8 x 最高位位置 + 下面 3 位，约 0.75 dB/级）
         柱子：上升立即，下降每帧 1；峰值帽：>= 时更新并保持 HOLD 帧，然后每帧落 1；acc 清零

  帧标记和样本的先后：帧起点那一拍若同时有样本，先处理帧标记，样本算进新一帧（和波形一致）。

画面（有效区坐标）：底板 x 44~595, y 380~427（和原来波形一样，圆角 6）
  32 根柱：第 i 根 x = 54 + 16i ~ +11（宽 12），底边 BASE = 414，往上长 0~HMAX 行；
  峰值帽：白线，在 y = BASE - 1 - cap（cap > 0 才画）
  倒影：y = BASE + 1 + k（k = 0..9），k*4 < h*3 时画，透明度 7 - k/2（最小 2）/16
  柱子顶上一行的左右两个像素透明度 8（圆头）
  VU 表保持原样（x 572~587）
"""
import os
import sys
import wave

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_VH = os.path.normpath(os.path.join(HERE, "..", "src", "user_source", "hdl_source", "display", "spec_layout.vh"))

# ---------------- 滤波器组 ----------------
FS = 48000
NB = 32
F_LO, F_HI = 50.0, 16000.0
RATIO = (F_HI / F_LO) ** (1.0 / (NB - 1))
FC = F_LO * RATIO ** np.arange(NB)
BW_OCT = np.log2(RATIO) * 1.55          # 每节带宽；两节串联后 -3 dB 带宽约 1/4 倍频程
NLOW = int((FC < 1000).sum())           # 17：第 0~16 根在 6 kHz 算
DECIM = 8
G = 2                                   # 状态里多留的小数位（满幅正弦也不溢出 18 位）
CF = 16                                 # 系数小数位
SB = 18                                 # 状态位宽（饱和）
CICW = 25
ACCW = 25                               # 每帧累加器位宽（饱和）
HOLD = 20
HMAX = 30                               # 柱高最大 30 行（0.75 dB/行 -> 约 22 dB）

# ---------------- 几何 ----------------
PX0, PX1, PY0, PY1 = 44, 596, 380, 428
PANEL_R = 6
SX0, PITCH, BARW = 54, 16, 12           # 第 i 根柱 [SX0+16i, SX0+16i+12)
SX1 = SX0 + (NB - 1) * PITCH + BARW     # 562
BASE = 414
NREF = 10
VUL_X0, VUR_X0, VU_W = 572, 582, 6
VU_Y0, VU_N = 384, 40
HOLD_FRAMES = 40                        # VU 的峰值保持（和原来一样）
LVL_OFS_VU = 19

# ---------------- 颜色 ----------------
C_PANEL = ((8, 14, 30), 10)
C_CAP = ((235, 235, 255), 16)
C_DIM = ((40, 50, 68), 12)
C_HOLD = ((250, 250, 250), 16)


def bar_color(k):
    """柱子第 k 行（0 = 最下）：青 -> 紫（60%）-> 粉"""
    stops = [(0.0, (40, 220, 255)), (0.6, (130, 110, 255)), (1.0, (255, 70, 200))]
    t = k / (HMAX - 1)
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            return tuple(int(round(c0[j] + (c1[j] - c0[j]) * u)) for j in range(3))
    return stops[-1][1]


def vu_color(b):
    g, y, r = np.array((40, 215, 90)), np.array((250, 215, 40)), np.array((255, 55, 40))
    if b < 18:
        c = g
    elif b < 26:
        c = g + (y - g) * (b - 17) / 9
    elif b < 34:
        c = y + (r - y) * (b - 25) / 9
    else:
        c = r
    return tuple(int(round(v)) for v in c)


def corner_inside(dx, dy):
    r = PANEL_R
    return (r - (dx + 0.5)) ** 2 + (r - (dy + 0.5)) ** 2 <= r * r


def ref_alpha(k):
    return max(2, 7 - (k >> 1))


# ---------------- 系数 ----------------
def coef_float(f0, fs):
    w0 = 2 * np.pi * f0 / fs
    al = np.sin(w0) * np.sinh(np.log(2) / 2 * BW_OCT * w0 / np.sin(w0))
    a0 = 1 + al
    return al / a0, 2 * np.cos(w0) / a0, -(1 - al) / a0


def coefs():
    """每根柱 (B0, B2, C1, C2)，整数，CF 位小数"""
    out = []
    for b in range(NB):
        fs = FS / DECIM if b < NLOW else FS
        b0, c1, c2 = coef_float(FC[b], fs)
        B0 = int(round(b0 * (1 << CF)))
        C1 = int(round(c1 * (1 << CF)))
        C2 = int(round(c2 * (1 << CF)))
        out.append((B0, 2 * B0, C1, C2))
    for c in out:
        for v in c:
            assert -(1 << 17) <= v < (1 << 17), c          # 18 位有符号
    return out


COEF = coefs()


def level(u):
    """非负整数 -> 8 x 最高位位置 + 下面 3 位（u < 8 时 0），约 0.75 dB/级"""
    if u < 8:
        return 0
    m = u.bit_length() - 1
    return 8 * m + ((u >> (m - 3)) & 7)


def level_vu(u):
    if u <= 0:
        return 0
    m = u.bit_length() - 1
    if m < 2:
        return 0
    return max(0, 4 * m + ((u >> (m - 2)) & 3) - LVL_OFS_VU)


# 每根柱的电平偏移（显示 = level(acc) - LVL_OFS[b]，0.75 dB/行，30 行约 22 dB）：
#   低组只累加 1/8 的样本 -> 少 3 位 = 24 级；
#   高组（1 kHz 以上）每往右一根补 1 级（0.75 dB，= 3 dB/倍频程的"斜率补偿"，让高音柱也跳得起来）；
#   LVL_BASE 用 SONG1 定：整首 99% 的 (柱, 帧) 不超过 30 行（design/spec_gen.py --calib 可重算）
LVL_BASE = 139
LVL_OFS = [LVL_BASE - 24 if b < NLOW else LVL_BASE - (b - NLOW + 1) for b in range(NB)]


def sat(v, bits=SB):
    lim = 1 << (bits - 1)
    return lim - 1 if v >= lim else (-lim if v < -lim else v)


def wrap(v, bits=CICW):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def sabs(s):
    return 32767 if s == -32768 else abs(s)


class SpecState:
    def __init__(self):
        self.st = [[0, 0, 0, 0] for _ in range(NB)]     # y1 y2 z1 z2
        self.acc = [0] * NB
        self.xh = [0, 0]                                # 高组 x1 x2
        self.xl = [0, 0]                                # 低组 x1 x2
        self.bx = 0                                     # 8 个样本的和
        self.ph = 0                                     # 抽取相位 0~7
        self.bar = [0] * NB
        self.cap = [0] * NB
        self.ht = [0] * NB
        # VU（和原来波形模块一样）
        self.pk = [0, 0]
        self.vbar = [0, 0]
        self.vhold = [0, 0]
        self.vht = [0, 0]
        self.f16 = 0
        self.max_state = 0
        self.n_sat = 0

    # ---- 一个二阶节 ----
    def _sec(self, B, C1, C2, d, s1, s2, shift_in):
        p = B * d
        if shift_in:
            p <<= G
        acc = p + C1 * s1 + C2 * s2
        v = -((-acc) >> CF) if acc < 0 else acc >> CF      # 向零舍入（消除静音时的极限环）
        self.max_state = max(self.max_state, abs(v))
        o = sat(v)
        if o != v:
            self.n_sat += 1
        return o

    def _band(self, b, x, x2):
        B0, B2, C1, C2 = COEF[b]
        y1, y2, z1, z2 = self.st[b]
        y = self._sec(B0, C1, C2, x - x2, y1, y2, True)
        z = self._sec(B2, C1, C2, (y - y2) >> 1, z1, z2, False)
        self.st[b] = [y, y1, z, z1]
        self.acc[b] = min(self.acc[b] + (abs(z) >> G), (1 << ACCW) - 1)

    def sample(self, l, r):
        self.pk[0] = max(self.pk[0], level_vu(sabs(l)))
        self.pk[1] = max(self.pk[1], level_vu(sabs(r)))
        m = (l + r) >> 1
        for b in range(NLOW, NB):
            self._band(b, m, self.xh[1])
        self.xh = [m, self.xh[0]]
        # 8 个样本求和抽取
        nb = self.bx + m
        self.bx = nb
        if self.ph == DECIM - 1:
            xl = nb >> 3
            self.bx = 0
            for b in range(NLOW):
                self._band(b, xl, self.xl[1])
            self.xl = [xl, self.xl[0]]
        self.ph = (self.ph + 1) % DECIM

    def frame(self, show):
        for b in range(NB):
            lv = min(HMAX, max(0, level(self.acc[b]) - LVL_OFS[b]))
            self.bar[b] = lv if lv > self.bar[b] else max(self.bar[b] - 1, 0)
            if self.bar[b] >= self.cap[b]:
                self.cap[b], self.ht[b] = self.bar[b], HOLD
            elif self.ht[b]:
                self.ht[b] -= 1
            elif self.cap[b]:
                self.cap[b] -= 1
            self.acc[b] = 0
        for c in range(2):
            lv = self.pk[c]
            self.vbar[c] = lv if lv > self.vbar[c] else max(self.vbar[c] - 1, 0)
            if lv >= self.vhold[c]:
                self.vhold[c], self.vht[c] = lv, HOLD_FRAMES
            elif self.vht[c]:
                self.vht[c] -= 1
            elif self.vhold[c]:
                self.vhold[c] -= 1
            self.pk[c] = 0
        self.f16 = min(16, self.f16 + 1) if show else max(0, self.f16 - 1)

    def summary(self):
        return (self.f16, self.vbar[0], self.vbar[1], self.vhold[0], self.vhold[1],
                sum(b << (5 * (i % 6)) for i, b in enumerate(self.bar)) & 0xffffffff,
                sum(self.bar), sum(self.cap))


# ---------------- 渲染（与 RTL 逐位一致） ----------------
def render(img, ss):
    out = img.astype(np.int64).copy()
    if ss.f16 == 0:
        return img.copy()
    for y in range(PY0, PY1):
        b = VU_Y0 + VU_N - 1 - y
        for x in range(PX0, PX1):
            dx, dy = min(x - PX0, PX1 - 1 - x), min(y - PY0, PY1 - 1 - y)
            if dx < PANEL_R and dy < PANEL_R and not corner_inside(dx, dy):
                continue
            col, ab = C_PANEL
            sx = x - SX0
            band, bx = sx >> 4, sx & 15
            in_bar = 0 <= sx and band < NB and bx < BARW
            vul = VUL_X0 <= x < VUL_X0 + VU_W
            vur = VUR_X0 <= x < VUR_X0 + VU_W
            vuy = VU_Y0 <= y < VU_Y0 + VU_N
            if in_bar:
                h, cp = ss.bar[band], ss.cap[band]
                k = BASE - 1 - y               # 柱内行号（0 = 最下）
                rk = y - BASE - 1              # 倒影行号
                if cp and k == cp:
                    col, ab = C_CAP
                elif 0 <= k < h:
                    col, ab = bar_color(k), (8 if (k == h - 1 and (bx == 0 or bx == BARW - 1)) else 16)
                elif 0 <= rk < NREF and rk * 4 < h * 3:
                    col, ab = bar_color(rk), ref_alpha(rk)
            elif (vul or vur) and vuy:
                c = 0 if vul else 1
                if ss.vhold[c] and b == ss.vhold[c] - 1:
                    col, ab = C_HOLD
                elif (b & 3) == 3:
                    pass
                elif b < ss.vbar[c]:
                    col, ab = vu_color(b), 16
                else:
                    col, ab = C_DIM
            a = (ab * ss.f16) >> 4
            for ch in range(3):
                out[y, x, ch] = (col[ch] * a + int(img[y, x, ch]) * (16 - a)) >> 4
    return out.astype(np.uint8)


# ---------------- 输出 RTL 版面 / 系数文件 ----------------
def rgb24(c):
    return (c[0] << 16) | (c[1] << 8) | c[2]


def s18(v):
    return "18'sd%d" % v if v >= 0 else "-18'sd%d" % (-v)


def vcase(name, width, keyw, entries, default, signed=False):
    s = "    function %s[%d:0] %s;\n        input [%d:0] k;\n        begin\n            case (k)\n" % (
        "signed " if signed else "", width - 1, name, keyw - 1)
    for k, v in entries:
        val = (s18(v) if signed else "%d'd%d" % (width, v))
        s += "                %d'd%d: %s = %s;\n" % (keyw, k, name, val)
    s += "                default: %s = %d'd%d;\n            endcase\n        end\n    endfunction\n\n" % (name, width, default)
    return s


def write_vh():
    L = ["// 由 design/spec_gen.py 自动生成，不要手改\n\n"]
    L.append("    localparam integer PX0 = %d, PX1 = %d, PY0 = %d, PY1 = %d, PANEL_R = %d;\n" % (PX0, PX1, PY0, PY1, PANEL_R))
    L.append("    localparam integer SX0 = %d, SX1 = %d, BARW = %d, BASE = %d, NREF = %d, HMAX = %d;\n" % (SX0, SX1, BARW, BASE, NREF, HMAX))
    L.append("    localparam integer VUL_X0 = %d, VUR_X0 = %d, VU_W = %d, VU_Y0 = %d, VU_N = %d;\n" % (VUL_X0, VUR_X0, VU_W, VU_Y0, VU_N))
    L.append("    localparam integer NB = %d, NLOW = %d, G = %d, CF = %d, SB = %d, HOLD = %d, HOLD_FRAMES = %d, LVL_OFS_VU = %d;\n"
             % (NB, NLOW, G, CF, SB, HOLD, HOLD_FRAMES, LVL_OFS_VU))
    assert all(LVL_OFS[b] == LVL_BASE - 24 for b in range(NLOW))
    assert all(LVL_OFS[b] == LVL_BASE + NLOW - 1 - b for b in range(NLOW, NB))
    L.append("    localparam integer LVL_LO = %d, LVL_HI0 = %d, ACCW = %d;\n" % (LVL_BASE - 24, LVL_BASE + NLOW - 1, ACCW))
    for n, (c, a) in [("PANEL", C_PANEL), ("CAP", C_CAP), ("DIM", C_DIM), ("HOLD", C_HOLD)]:
        L.append("    localparam [23:0] C_%s = 24'h%06x; localparam [4:0] A_%s = 5'd%d;\n" % (n, rgb24(c), n, a))
    L.append("\n")
    L.append(vcase("bar_rgb", 24, 5, [(k, rgb24(bar_color(k))) for k in range(HMAX)], 0))
    # v2：VU 颜色表按 q = b + 1 编址（RTL 里 VU 和频谱共用行号 q）
    L.append(vcase("vu_rgb", 24, 6, [(b + 1, rgb24(vu_color(b))) for b in range(VU_N)], 0))
    L.append(vcase("corner_in", 1, 6, [((dx << 3) | dy, 1 if corner_inside(dx, dy) else 0)
                                        for dx in range(PANEL_R) for dy in range(PANEL_R)], 1))
    with open(OUT_VH, "w", encoding="utf-8") as f:
        f.write("".join(L))
    # 系数 ROM：地址 {柱号, j}（j: 0=B0<<G 1=B2 2=C1 3=C2），18 位补码
    #   v2：第一节的 B0 预先乘 2^G，RTL 的累加器装载就不用再移位（结果逐位相同）
    def word(a):
        v = COEF[a >> 2][a & 3]
        if (a & 3) == 0:
            v <<= G
        assert -(1 << 17) <= v < (1 << 17)
        return v & 0x3ffff
    words = [word(a) for a in range(NB * 4)]
    d = os.path.dirname(OUT_VH)
    NL = chr(10)
    with open(os.path.join(d, "spec_coef.mif"), "w") as f:
        f.write("WIDTH=18;" + NL + "DEPTH=128;" + NL + "ADDRESS_RADIX=HEX;" + NL + "DATA_RADIX=HEX;" + NL + "CONTENT BEGIN" + NL)
        for a, w in enumerate(words):
            f.write("%X : %05X;" % (a, w) + NL)
        f.write("END;" + NL)
    with open(os.path.join(d, "spec_coef.hex"), "w") as f:
        for w in words:
            f.write("%05x" % w + NL)


# ---------------- 测试 / 预览 ----------------
SPF = 806


def read_song():
    w = wave.open(os.path.join(HERE, "tf_card_music", "SONG1.WAV"))
    return np.frombuffer(w.readframes(w.getnframes()), "<i2").astype(np.int64).reshape(-1, 2)


def run_frames(lr, nfr=None):
    ss = SpecState()
    bars = []
    for n in range(len(lr)):
        ss.sample(int(lr[n, 0]), int(lr[n, 1]))
        if n % SPF == SPF - 1:
            ss.frame(True)
            bars.append(list(ss.bar))
    return ss, np.array(bars)


def float_levels(lr):
    """浮点理想：同样的四阶滤波器（48 kHz 直接算，不抽取），每帧 sum|z|，换成 1.5 dB 级"""
    m = (lr[:, 0] + lr[:, 1]) / 2.0
    nfr = len(m) // SPF
    out = np.zeros((nfr, NB))
    b0, c1, c2 = np.array([coef_float(f, FS) for f in FC]).T
    y1 = np.zeros(NB); y2 = np.zeros(NB); z1 = np.zeros(NB); z2 = np.zeros(NB)
    x1 = x2 = 0.0
    acc = np.zeros(NB)
    for n in range(nfr * SPF):
        x = m[n]
        y = b0 * (x - x2) + c1 * y1 + c2 * y2
        z = b0 * (y - y2) + c1 * z1 + c2 * z2
        x2, x1 = x1, x
        y2, y1, z2, z1 = y1, y, z1, z
        acc += np.abs(z)
        if n % SPF == SPF - 1:
            out[n // SPF] = 4 * np.log2(np.maximum(acc * (1 << 0), 1e-9))
            acc[:] = 0
    return out


def test():
    song = read_song()
    sil = np.zeros((FS // 2, 2), dtype=np.int64)
    T = 4.0
    t = np.arange(int(T * FS)) / FS
    k = np.log(F_HI / F_LO)
    sw = np.round(32000 * np.sin(2 * np.pi * F_LO * T / k * (np.exp(t / T * k) - 1))).astype(np.int64)
    sweep = np.stack([sw, sw], axis=1)
    for name, sig in [("SONG1 前 3 秒", song[:3 * FS]), ("满幅扫频 4 秒", sweep)]:
        lr = np.concatenate([sig, sil])
        ss, bars = run_frames(lr)
        fl = float_levels(lr)
        nsig = len(sig) // SPF
        # 定点柱高 vs 浮点（同样的偏移、未做快升慢落的电平）
        fx = SpecState()
        lv_fx = []
        for n in range(len(lr)):
            fx.sample(int(lr[n, 0]), int(lr[n, 1]))
            if n % SPF == SPF - 1:
                lv_fx.append([min(HMAX, max(0, level(fx.acc[b]) - LVL_OFS[b])) for b in range(NB)])
                fx.frame(True)
        lv_fx = np.array(lv_fx)
        lv_fl = np.clip(np.floor(2 * fl) - np.array(LVL_OFS) - np.where(np.arange(NB) < NLOW, 24, 0), 0, HMAX)
        d = np.abs(lv_fx[:nsig] - lv_fl[:nsig])
        print("%s：状态最大 %d（18 位上限 131071），饱和 %d 次；电平和浮点比 平均差 %.2f 级，95%% 分位 %.0f 级，最大 %.0f 级"
              % (name, fx.max_state, fx.n_sat, d.mean(), np.percentile(d, 95), d.max()))
        print("   每根柱平均电平（定点）：%s" % np.round(lv_fx[:nsig].mean(axis=0)).astype(int).tolist())
        print("   每根柱平均电平（浮点）：%s" % np.round(lv_fl[:nsig].mean(axis=0)).astype(int).tolist())
        print("   静音 0.5 秒后最后一帧：电平 %s，acc %s" % (lv_fx[-1].tolist(), fx.acc))
        print("   静音末状态（非 0 的柱）：%s" % [(b, s) for b, s in enumerate(fx.st) if any(s)])


def demo():
    import osd_gen as og
    song = read_song()
    ss = SpecState()
    for n in range(int(2.0 * FS)):
        ss.sample(int(song[n, 0]), int(song[n, 1]))
        if n % SPF == SPF - 1:
            ss.frame(True)
    st = og.run_frames([og.DEMO_STATUS] * 20)
    st.xs = 240
    img = og.render(og.demo_background(), st)
    img = render(img, ss)
    Image.fromarray(img).save(os.path.join(HERE, "preview_11_spectrum.png"))
    Image.fromarray(img).crop((30, 370, 610, 480)).resize((1160, 220), Image.NEAREST).save(
        os.path.join(HERE, "preview_11_spectrum_zoom.png"))


if __name__ == "__main__":
    if "--test" in sys.argv:
        test()
    else:
        write_vh()
        print("spec_layout.vh 已生成")
        demo()
        print("预览图已生成")
