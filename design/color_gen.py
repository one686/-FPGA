# -*- coding: utf-8 -*-
"""第13课：查表调色（亮度 / 对比度 / 色温）+ 饱和度 —— 与 RTL color_adjust.v 逐位一致的参考模型 + 效果预览。

用法：
  python color_gen.py        输出 design/color_preview/ 下各档位的效果图（用卡里的图片）
  （被 sim/check_osd.py import，作为仿真比对的标准答案）

公式（全部整数）：
  档位 l = 0~10，d = l - 5
  gb = 256 + 26d（亮度）  gk = 256 + 26d（对比度）  色温：gr = 256 + 10d，gg = 256，gbl = 256 - 10d
  v0 = (x * gb) >> 8;  v1 = (v0 * gc) >> 8;  v2 = (((v1 - 128) * gk) >> 8) + 128;  out = clamp(v2, 0, 255)
  （Python 的 >> 对负数就是算术右移，与 Verilog 的 >>> 一致）
平滑：每个帧起点，当前增益向目标增益走 STEP，到了就停。
饱和度（查表之后）：Y = (77R+150G+29B)>>8，out = clamp(Y + ((c-Y)*m)>>5)，m = gs/8，
  gs 档位表 GS_TAB（8 的倍数，0 档 256 = 不变，-5 = 黑白）。
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# 第2步临时测试用的 7 个预设（picture 层长按 KEY3 依次切换，数码管显示序号）：(亮度, 对比度, 色温) 档位
# 预览用的几组 (亮度, 对比度, 色温, 饱和度) 档位
PRESETS = [(5, 5, 5, 5), (10, 5, 5, 5), (0, 5, 5, 5), (5, 10, 5, 5), (5, 0, 5, 5), (5, 5, 10, 5), (5, 5, 0, 5),
           (5, 5, 5, 0), (5, 5, 5, 10)]
PRESET_NAMES = ["原图", "亮度+5", "亮度-5", "对比度+5", "对比度-5", "色温+5（暖）", "色温-5（冷）", "饱和度-5（黑白）", "饱和度+5"]


GS_TAB = [0, 48, 104, 152, 208, 256, 288, 320, 352, 384, 416]


def gain26(l):
    return 256 + 26 * (l - 5)


def gain10(l, neg=False):
    d = l - 5
    return 256 + 10 * (-d if neg else d)


def targets(lv_b, lv_k, lv_t, lv_s=5):
    return dict(gb=gain26(lv_b), gk=gain26(lv_k), gr=gain10(lv_t), gbl=gain10(lv_t, True), gs=GS_TAB[lv_s])


def approach(cur, tgt, step):
    if cur + step <= tgt:
        return cur + step
    if cur >= tgt + step:
        return cur - step
    return tgt


class ColorState:
    def __init__(self, step=8):
        self.step = step
        self.g = dict(gb=256, gk=256, gr=256, gbl=256, gs=256)

    def frame_start(self, lv_b, lv_k, lv_t, lv_s=5):
        t = targets(lv_b, lv_k, lv_t, lv_s)
        for k in self.g:
            self.g[k] = approach(self.g[k], t[k], self.step)

    def tables(self):
        return make_tables(self.g["gb"], self.g["gk"], self.g["gr"], self.g["gbl"]) + [self.g["gs"]]


def make_tables(gb, gk, gr, gbl):
    x = np.arange(256, dtype=np.int64)
    out = []
    for gc in (gr, 256, gbl):
        v0 = (x * gb) >> 8
        v1 = (v0 * gc) >> 8
        v2 = (((v1 - 128) * gk) >> 8) + 128
        out.append(np.clip(v2, 0, 255).astype(np.uint8))
    return out


def apply(img, tabs):
    """img: (H,W,3) uint8 -> 查表后的图；tabs 第 4 项（如果有）是饱和度增益 gs"""
    o = np.empty_like(img)
    for c in range(3):
        o[..., c] = tabs[c][img[..., c]]
    gs = tabs[3] if len(tabs) > 3 else 256
    m = gs >> 3
    q = o.astype(np.int64)
    y = (77 * q[..., 0] + 150 * q[..., 1] + 29 * q[..., 2]) >> 8
    out = y[..., None] + (((q - y[..., None]) * m) >> 5)
    return np.clip(out, 0, 255).astype(np.uint8)


def test_card():
    """预览用测试图：天空渐变 + 彩色块 + 肤色 + 底部 0~255 灰阶条（看亮度/对比度最直观）"""
    import osd_gen as og
    img = og.demo_background().copy()
    cols = [(220, 40, 40), (40, 180, 60), (40, 80, 220), (240, 200, 60), (230, 180, 150), (120, 60, 160)]
    for i, c in enumerate(cols):
        img[230:300, 40 + i * 95:120 + i * 95] = c
    ramp = np.repeat(np.linspace(0, 255, 640).astype(np.uint8)[None, :], 60, axis=0)
    img[420:480] = ramp[..., None]
    return img


def main():
    from PIL import Image
    out_dir = os.path.join(HERE, "color_preview")
    os.makedirs(out_dir, exist_ok=True)
    import osd_gen as og
    src = test_card()
    ident = make_tables(256, 256, 256, 256)
    assert all((t == np.arange(256)).all() for t in ident), "0 档必须逐位不变"
    rnd = np.random.default_rng(0).integers(0, 256, (64, 64, 3)).astype(np.uint8)
    assert (apply(rnd, ident + [256]) == rnd).all(), "饱和度 0 档必须逐位不变"
    tiles = []
    for (b, k, t, sa), name in zip(PRESETS, PRESET_NAMES):
        g = targets(b, k, t, sa)
        im = apply(src, make_tables(g["gb"], g["gk"], g["gr"], g["gbl"]) + [g["gs"]])
        tiles.append((name, im))
        Image.fromarray(im).save(os.path.join(out_dir, "preset_%s.png" % name.split("（")[0].replace("+", "p").replace("-", "m")))
    # 拼图：7 张缩到 320x240
    from PIL import ImageDraw, ImageFont
    font = ImageFont.truetype(og.FONT_REG, 18)
    sheet = Image.new("RGB", (4 * 324, 3 * 270), (30, 30, 40))
    for i, (name, im) in enumerate(tiles):
        r, c = divmod(i, 4)
        sheet.paste(Image.fromarray(im).resize((320, 240)), (c * 324 + 2, r * 270 + 28))
        ImageDraw.Draw(sheet).text((c * 324 + 6, r * 270 + 4), "%d：%s" % (i, name), font=font, fill=(255, 230, 120))
    sheet.save(os.path.join(out_dir, "all_presets.png"))
    print("预览已生成：", out_dir)


if __name__ == "__main__":
    main()
