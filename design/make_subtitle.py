# -*- coding: utf-8 -*-
"""滚动字幕生成工具（finish_u_1，2026-10-21）：标语.txt -> SUB.BMP（放 TF 卡根目录）
   用法：python make_subtitle.py 标语.txt [输出目录]
     - 标语.txt 用 UTF-8 保存；多行会连成一句，行与行之间空两个全角空格
     - 输出 SUB.BMP（双击就能看字幕的样子）、SUB_预览.png（640x480 整帧效果）
   字形、描边、投影、渐变色、混合公式全部沿用 osd_gen.py（和现在板上的字幕一样），整句连排、字间距 GAP 像素。

   SUB.BMP 格式（B2 存法，FPGA 只认这个工具生成的文件）：
     - Windows 4 位 BMP，高 SB_H = 40，宽 = 8 的倍数（每行正好 W/2 字节，没有行尾补齐），最宽 MAX_W 像素
     - 16 项调色板 = 固定的 FIXED_PAL（所有句子共用，RTL 里写死同一张表，finish_u_2 用户 10-21 选定）：
       B、G、R = 这个颜色在深色底上的样子（看图用）；第 4 个字节（保留字节）= {字形浓度 g, 描边浓度 o}（各 4 位，核对用）
       FPGA 不读调色板，像素值 0~15 直接按 RTL 里的同一张表查 {g, o}
     - 像素按 BMP 习惯从下往上存，FPGA 读的时候把行号反过来
   python make_subtitle.py --default：生成默认标语"热烈欢迎"的 RAM 初值（osd/sub_default.mif、.hex，第 5 块 BRAM32K）
   FPGA 侧：4 块 BRAM32K = 32768 字节 = 65536 个 4 位像素；40 行 x 816 = 32640 个，放得下；
     地址 = 行首地址（每行加一次宽度）+ 列号，不用 {行, 列} 拼（拼的话 4 块只能放 512 宽）。
"""
import os
import struct
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import osd_gen as G  # noqa: E402

H = G.SB_H                        # 40
NBLK = 4                          # BRAM32K 块数（用户 10-20 定：4 块）
MAX_W = (NBLK * 4096 * 2 // H) // 8 * 8      # 816
GAP = 2                           # 字间距
MARGIN = 8                        # 首尾留白
PREVIEW_BG = (24, 28, 36)         # 调色板里"看图用"的颜色按这个深色底算


def render_strip(text):
    """整句一起画，返回 g、o（0~15），宽度补到 8 的倍数。"""
    font = ImageFont.truetype(G.FONT_BOLD, G.SB_SIZE)
    xs, x = [], MARGIN
    for ch in text:
        xs.append(x)
        x += int(round(font.getlength(ch))) + GAP
    w = (x - GAP + MARGIN + 7) // 8 * 8
    t = Image.new("L", (w, H), 0)
    o = Image.new("L", (w, H), 0)
    dt, do = ImageDraw.Draw(t), ImageDraw.Draw(o)
    for ch, x0 in zip(text, xs):
        dt.text((x0, G.SB_BASE), ch, font=font, fill=255, anchor="ls")
        do.text((x0, G.SB_BASE), ch, font=font, fill=255, anchor="ls",
                stroke_width=G.SB_STROKE, stroke_fill=255)
    sh = Image.new("L", (w, H), 0)
    sh.paste(o, (G.SB_SHADOW_DX, G.SB_SHADOW_DY))
    sh = sh.filter(ImageFilter.GaussianBlur(G.SB_SHADOW_BLUR))
    shadow = (np.array(sh).astype(np.float64) * G.SB_SHADOW_K).astype(np.int32)
    g = G.q4(np.array(t))
    oo = np.maximum(np.maximum(G.q4(np.array(o)), G.q4(shadow)), g)
    return g.astype(np.int32), oo.astype(np.int32)


# 固定调色板（用约 120 个常用字、字母、标点按像素数加权 k-means 得到；和每句自己算相比平均差 1.0~1.45 对 0.85~1.16）
#   改这张表必须同时改 osd_engine.v 的 sub_pal
FIXED_PAL = np.array([(0, 0), (15, 15), (0, 15), (0, 1), (0, 2), (0, 3), (0, 6), (0, 5),
                      (0, 4), (0, 8), (12, 15), (1, 15), (7, 15), (4, 15), (0, 12), (2, 15)], np.int32)
DEFAULT_TEXT = "热烈欢迎"


def palette_fixed(g, o):
    """每个像素取 FIXED_PAL 里最近的一项。返回 pal、序号。"""
    d = ((np.stack([g, o], -1)[..., None, :] - FIXED_PAL[None, None]) ** 2).sum(-1)
    return FIXED_PAL, d.argmin(-1).astype(np.int32)


def pixel_bytes(idx):
    """和 BMP 里一样的像素字节（从下往上，每字节高 4 位 = 偶数列）。"""
    h, w = idx.shape
    out = bytearray()
    for r in range(h - 1, -1, -1):
        line = idx[r]
        out += bytes(((int(line[2 * i]) << 4) | int(line[2 * i + 1])) for i in range(w // 2))
    return out


def write_default(hdl_dir):
    g, o = render_strip(DEFAULT_TEXT)
    pal, idx = palette_fixed(g, o)
    data = pixel_bytes(idx)
    depth = 4096
    assert len(data) <= depth
    with open(os.path.join(hdl_dir, "sub_default.hex"), "w") as f:
        for i in range(depth):
            f.write("%02x\n" % (data[i] if i < len(data) else 0))
    with open(os.path.join(hdl_dir, "sub_default.mif"), "w") as f:
        f.write("WIDTH=8;\nDEPTH=%d;\nADDRESS_RADIX=HEX;\nDATA_RADIX=HEX;\nCONTENT BEGIN\n" % depth)
        for i in range(depth):
            f.write("%x : %02x;\n" % (i, data[i] if i < len(data) else 0))
        f.write("END;\n")
    print("默认标语 %s：宽 %d，%d 字节 -> %s/sub_default.mif、.hex（RTL 默认宽度参数要等于 %d）"
          % (DEFAULT_TEXT, g.shape[1], len(data), hdl_dir, g.shape[1]))


def palette16(g, o, iters=30):
    """16 项调色板（按像素数加权的 k-means；第 0 项 (0,0)、第 1 项 (15,15) 固定）。返回 pal[16][2] 和每个像素的序号。"""
    pairs, cnt = np.unique(np.stack([g.ravel(), o.ravel()], 1), axis=0, return_counts=True)
    pts = pairs.astype(np.float64)
    cent = [np.array([0.0, 0.0]), np.array([15.0, 15.0])]
    for i in np.argsort(-cnt):
        if len(cent) >= 16:
            break
        if min(np.abs(pts[i] - c).sum() for c in cent) > 0:
            cent.append(pts[i].copy())
    while len(cent) < 16:                               # 句子太短、组合不到 16 种：补透明（不会被用到）
        cent.append(np.array([0.0, 0.0]))
    cent = np.array(cent)
    for _ in range(iters):
        lab = ((pts[:, None, :] - cent[None, :, :]) ** 2).sum(2).argmin(1)
        for j in range(2, 16):
            m = lab == j
            if m.any():
                cent[j] = (pts[m] * cnt[m, None]).sum(0) / cnt[m].sum()
    pal = np.clip(np.round(cent), 0, 15).astype(np.int32)
    lab = ((pts[:, None, :] - pal[None, :, :]) ** 2).sum(2).argmin(1)
    idx = np.zeros(g.shape, np.int32)
    for (a, b), k in zip(pairs, lab):
        idx[(g == a) & (o == b)] = k
    return pal, idx


def view_rgb(gv, ov, row=H // 2):
    """调色板里的显示颜色：用字幕带中间一行的渐变色，按板上同一公式混到深色底上。"""
    tc = G.sub_grad(row)
    a, c = G.a9(ov), G.a9(gv)
    return tuple(min(255, ((tc[i] * c) >> 8) + ((PREVIEW_BG[i] * (256 - a)) >> 8)) for i in range(3))


def write_bmp(path, idx, pal):
    h, w = idx.shape
    rowb = w // 2
    img = bytearray()
    for r in range(h - 1, -1, -1):                      # 从下往上
        line = idx[r]
        img += bytes(((int(line[2 * i]) << 4) | int(line[2 * i + 1])) for i in range(rowb))
    ptab = bytearray()
    for gv, ov in pal:
        rr, gg, bb = view_rgb(int(gv), int(ov))
        ptab += bytes((bb, gg, rr, (int(gv) << 4) | int(ov)))
    off = 14 + 40 + len(ptab)
    hdr = b"BM" + struct.pack("<IHHI", off + len(img), 0, 0, off)
    dib = struct.pack("<IiiHHIIiiII", 40, w, h, 1, 4, 0, len(img), 2835, 2835, 16, 16)
    with open(path, "wb") as f:
        f.write(hdr + dib + ptab + img)


def read_back(path):
    """按 FPGA 的读法解析：只看保留字节里的 {g, o}。"""
    d = open(path, "rb").read()
    assert d[:2] == b"BM"
    off, = struct.unpack_from("<I", d, 10)
    w, h = struct.unpack_from("<ii", d, 18)
    bpp, = struct.unpack_from("<H", d, 28)
    assert bpp == 4 and h == H and w % 8 == 0 and w <= MAX_W
    pal = [(d[54 + 4 * k + 3] >> 4, d[54 + 4 * k + 3] & 15) for k in range(16)]
    g = np.zeros((h, w), np.int32)
    o = np.zeros((h, w), np.int32)
    for r in range(h):
        base = off + (h - 1 - r) * (w // 2)
        for x in range(w):
            b = d[base + x // 2]
            k = (b >> 4) if x % 2 == 0 else (b & 15)
            g[r, x], o[r, x] = pal[k]
    return g, o


def blend(bg, g, o, x0, y0):
    """和 osd_gen.render 字幕带同一公式：C = tc*a9(g)>>8，A = a9(o)，out = min(255, C + bg*(256-A)>>8)。"""
    out = bg.astype(np.int64).copy()
    a9 = np.vectorize(G.a9)
    for r in range(H):
        tc = G.sub_grad(r)
        x1, x2 = max(0, x0), min(out.shape[1], x0 + g.shape[1])
        if x1 >= x2:
            continue
        gg = a9(g[r, x1 - x0:x2 - x0])
        oo = a9(o[r, x1 - x0:x2 - x0])
        for ch in range(3):
            c = (tc[ch] * gg) >> 8
            out[y0 + r, x1:x2, ch] = np.minimum(255, c + ((out[y0 + r, x1:x2, ch] * (256 - oo)) >> 8))
    return out.astype(np.uint8)


def read_text(path):
    lines = [ln.strip() for ln in open(path, encoding="utf-8-sig")]
    return "　　".join(ln for ln in lines if ln)


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "--default":
        write_default(sys.argv[2] if len(sys.argv) > 2 else
                      os.path.join(HERE, "..", "src", "user_source", "hdl_source", "osd"))
        return 0
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    text = read_text(sys.argv[1])
    outdir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(sys.argv[1]))
    if not text:
        print("标语.txt 是空的")
        return 1
    g, o = render_strip(text)
    w = g.shape[1]
    font = ImageFont.truetype(G.FONT_BOLD, G.SB_SIZE)
    print("标语：%s" % text)
    print("共 %d 个字符，字幕条宽 %d 像素（上限 %d）" % (len(text), w, MAX_W))
    if w > MAX_W:
        # 找出放得下的最长前缀
        k = len(text)
        while k > 0 and render_strip(text[:k])[0].shape[1] > MAX_W:
            k -= 1
        print("放不下！最多放到第 %d 个字：\"%s\"，请删掉后面 %d 个字" % (k, text[:k], len(text) - k))
        return 2
    room = (MAX_W - w) // int(round(font.getlength("中")) + GAP)
    print("还能再加约 %d 个汉字" % room)
    pal, idx = palette_fixed(g, o)
    bmp = os.path.join(outdir, "SUB.BMP")
    write_bmp(bmp, idx, pal)
    g2, o2 = read_back(bmp)
    gq, oq = pal[idx][..., 0], pal[idx][..., 1]
    assert (g2 == gq).all() and (o2 == oq).all(), "读回不一致"
    # 和 8 位原样（A 存法）比较画质
    bg = np.full((H, w, 3), PREVIEW_BG, np.uint8)
    da = blend(bg, g, o, 0, 0).astype(int)
    db = blend(bg, g2, o2, 0, 0).astype(int)
    diff = np.abs(da - db)
    print("和 8 位原样比：平均差 %.2f，差别 > 8 的像素 %.1f%%" % (diff.mean(), 100.0 * (diff.max(2) > 8).mean()))
    # 预览：640x480 整帧，句子开头在 x = 20
    try:
        fr = G.demo_background()
    except Exception:
        fr = np.full((480, 640, 3), (60, 90, 120), np.uint8)
    fr = blend(np.array(fr, np.uint8), g2, o2, 20, G.SUB_TEXT_Y0)
    pv = os.path.join(outdir, "SUB_预览.png")
    Image.fromarray(fr).save(pv)
    print("已生成 %s（%d 字节）和 %s" % (bmp, os.path.getsize(bmp), pv))
    print("把 SUB.BMP 拷到 TF 卡根目录即可")
    return 0


if __name__ == "__main__":
    sys.exit(main())
