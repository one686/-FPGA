# -*- coding: utf-8 -*-
"""第12课：bmp_scaler.v 的逐位一致 Python 模型。

scale(data, out_w, out_h) -> (结果, 输出流, flip)
  结果：'ok' / 'bad'（头不合格）/ 'short'（文件截断）
  输出流：out_w*out_h 个 0xRRGGBB，按写 SDRAM 的顺序（= 文件行顺序）；flip=1 表示写侧上下翻转
to_screen(stream, flip, out_w, out_h) -> 屏幕上从上到下的像素列表

第2步起 scale() = 高质量缩放（与 RTL 逐位一致）：
  缩小（源图比输出区大）：精确区域平均——按面积重叠的整数权重先横向、再纵向累加，最后乘一次倒数；
  放大 / 一样大：双三次（Catmull-Rom，a=-0.5），像素中心对齐，先纵向再横向（4x4 个源像素）；
  一样大时权重是 (0,256,0,0)，与原图逐位相同。
scale_nearest() = 第1步的最近邻（留作对比）。
详细的定点算法见 bmp_scaler.v 文件头，这里的每一步整数运算都和 RTL 相同。
"""
import struct

MAX_W = 1920
MAX_H = 1920


def parse(data):
    """按 RTL 的逐字节检查规则解析头。返回 dict（含 ok 标志），文件不足 54 字节返回 None。"""
    if len(data) < 54:
        return None
    b = data
    hok = (b[0] == 0x42 and b[1] == 0x4D)
    off = b[10] | (b[11] << 8)
    for p in (12, 13, 20, 21, 27, 29, 31, 32, 33):
        hok &= (b[p] == 0)
    w16 = b[18] | (b[19] << 8)
    h16 = b[22] | (b[23] << 8)
    h_hi0 = b[24] == 0 and b[25] == 0
    h_hi1 = b[24] == 0xFF and b[25] == 0xFF
    hok &= (b[26] == 1)
    hok &= b[28] in (24, 32)
    is32 = b[28] == 32
    hok &= b[30] in (0, 3)
    comp3 = b[30] == 3
    # 头读到 max(54, off)（off<54 时读到 54 停）；掩码在 54~65 且只在读进头部时才检查
    hdr_len = off if off >= 54 else 54
    if len(data) < hdr_len:
        return None
    mask_ok = True
    for p in range(54, min(66, hdr_len)):
        exp = 0xFF if p in (56, 59, 62) else 0x00
        mask_ok &= (b[p] == exp)
    habs = ((~h16) + 1) & 0xFFFF if h_hi1 else h16
    ok = (hok and (h_hi0 or h_hi1) and off >= 54 and 0 < w16 <= MAX_W and 0 < habs <= MAX_H
          and ((not comp3) or (is32 and off >= 66 and mask_ok)))
    return dict(ok=ok, off=off, w=w16, h=habs, is32=is32, flip=0 if h_hi1 else 1, hdr_len=hdr_len)


def geometry(w, h, out_w, out_h):
    if w * out_h >= h * out_w:                     # 宽图：输出宽 = out_w
        q = ((h * out_w + (w >> 1)) // w) & 0x7FF
        ow, oh = out_w, (q or 1)
    else:                                          # 高图：输出高 = out_h
        q = ((w * out_h + (h >> 1)) // h) & 0x7FF
        oh, ow = out_h, (q or 1)
    ml = (out_w - ow) >> 1
    m0 = (out_h - oh) >> 1
    mb = (out_h - oh) - m0
    return ow, oh, ml, m0, mb


def bitlen(v):
    return int(v).bit_length()


def cubic_table():
    """Catmull-Rom 权重表：t = T/256，返回 [(a, b, c, d)]，权重和恰好 256（b 由其余三个算出）。
    RTL（finish_inc_3 L5 起）直接用 cubic_rom() 生成的常数 ROM（bmp_cubic.mif），不再每张图现算。"""
    tab = []
    for T in range(256):
        t2, t3 = T * T, T * T * T
        a = (-t3 + 512 * t2 - 65536 * T + 65536) >> 17
        c = (-3 * t3 + 1024 * t2 + 65536 * T + 65536) >> 17
        d = (t3 - 256 * t2 + 65536) >> 17
        tab.append((a, 256 - a - c - d, c, d))
    return tab


CUBIC = cubic_table()


def cubic_rom():
    """常数 ROM 的 256 个 30 位字：{a[5:0], b[8:0], c[8:0], d[5:0]}（a、d 有符号 -19~0，b、c 无符号 0~256）。
    RTL 读出后 a、d 符号扩展、b、c 补 0 到 10 位，四个权重都直接取，不再算 b = 256 - a - c - d。"""
    words = []
    for a, b, c, d in CUBIC:
        assert -32 <= a <= 0 and -32 <= d <= 0 and 0 <= b <= 511 and 0 <= c <= 511
        words.append(((a & 0x3f) << 24) | (b << 15) | (c << 6) | (d & 0x3f))
    return words


def write_cubic_rom(d):
    """写 bmp_cubic.mif（TD 用）和 bmp_cubic.hex（仿真 $readmemh 用）到目录 d"""
    import os
    words = cubic_rom()
    nl = chr(10)
    with open(os.path.join(d, "bmp_cubic.mif"), "w") as f:
        f.write("WIDTH=30;" + nl + "DEPTH=256;" + nl + "ADDRESS_RADIX=HEX;" + nl + "DATA_RADIX=HEX;" + nl + "CONTENT BEGIN" + nl)
        for a, w in enumerate(words):
            f.write("%X : %08X;" % (a, w) + nl)
        f.write("END;" + nl)
    with open(os.path.join(d, "bmp_cubic.hex"), "w") as f:
        for w in words:
            f.write("%08x" % w + nl)


def params(w, h, ow, oh):
    """每张图算一次的参数（RTL 里用移位加减的乘除法算）"""
    down = (w > ow) or (h > oh)
    q = {'down': down}
    if down:
        bw, bh = bitlen(w), bitlen(h)
        q['ls'] = max(0, 9 - bw)          # 横向重叠权重左移（小宽度时）
        q['rs'] = max(0, bw - 9)          # 横向累加结果右移（大宽度时）
        q['bh'] = bh
        q['R'] = ((1 << (14 + bw + bh)) + ((w * h) >> 1)) // (w * h)
    else:
        q['sx'] = (w * 65536 + (ow >> 1)) // ow
        q['sy'] = (h * 65536 + (oh >> 1)) // oh
    return q


def _overlap_matrix(n_src, n_out):
    """源 k 占 [k*n_out, (k+1)*n_out)，输出 i 占 [i*n_src, (i+1)*n_src)，返回整数重叠 M[i, k]"""
    import numpy as np
    M = np.zeros((n_out, n_src), dtype=np.int64)
    for k in range(n_src):
        a0, a1 = k * n_out, (k + 1) * n_out
        i0, i1 = a0 // n_src, (a1 - 1) // n_src
        for i in range(i0, i1 + 1):
            M[i, k] = min(a1, (i + 1) * n_src) - max(a0, i * n_src)
    return M


def _resample(img, ow, oh, q):
    """img: H x W x 3 int64（文件行顺序），返回 oh x ow x 3"""
    import numpy as np
    h, w = img.shape[:2]
    if q['down']:
        Mx = _overlap_matrix(w, ow) << q['ls']
        My = _overlap_matrix(h, oh)
        A = np.einsum('ix,yxc->yic', Mx, img) >> q['rs']           # 每个源行的横向累加（列完成时右移）
        V = np.einsum('jy,yic->jic', My, A)
        out = ((V >> q['bh']) * q['R'] + (1 << 22)) >> 23
        return np.minimum(out, 255)
    tab = np.array(CUBIC, dtype=np.int64)

    def axis(n_src, n_out, step):
        pos = (step >> 1) - 32768 + step * np.arange(n_out, dtype=np.int64)
        x0 = pos >> 16
        t = (pos >> 8) & 255
        idx = np.clip(x0[:, None] + np.arange(-1, 3)[None, :], 0, n_src - 1)
        return idx, tab[t]

    yi, wy = axis(h, oh, q['sy'])
    xi, wx = axis(w, ow, q['sx'])
    v = np.einsum('jk,jkxc->jxc', wy, img[yi])                     # 纵向：oh x W x 3
    s = np.einsum('ik,jikc->jic', wx, v[:, xi]) + 32768             # 横向
    return np.clip(s >> 16, 0, 255)


def scale(data, out_w=640, out_h=480):
    """高质量缩放（第2步，与 RTL 逐位一致）"""
    import numpy as np
    hd = parse(data)
    if hd is None:
        return 'short', None, 1
    if not hd['ok']:
        return 'bad', None, 1
    w, h, is32 = hd['w'], hd['h'], hd['is32']
    ow, oh, ml, m0, mb = geometry(w, h, out_w, out_h)
    bpp = 4 if is32 else 3
    rowb = w * bpp + (0 if is32 else (w & 3))
    need = hd['hdr_len'] + rowb * h
    if len(data) < need:
        return 'short', None, hd['flip']
    raw = np.frombuffer(data, dtype=np.uint8, count=rowb * h, offset=hd['hdr_len']).reshape(h, rowb)
    pix = raw[:, :w * bpp].reshape(h, w, bpp)[:, :, 2::-1].astype(np.int64)   # BGR(A) -> RGB
    res = _resample(pix, ow, oh, params(w, h, ow, oh))
    frame = np.zeros((out_h, out_w, 3), dtype=np.int64)
    frame[m0:m0 + oh, ml:ml + ow] = res
    flat = (frame[..., 0] << 16) | (frame[..., 1] << 8) | frame[..., 2]
    return 'ok', [int(v) for v in flat.reshape(-1)], hd['flip']


def scale_nearest(data, out_w=640, out_h=480):
    """第1步的最近邻（留作对比）"""
    hd = parse(data)
    if hd is None:
        return 'short', None, 1
    if not hd['ok']:
        return 'bad', None, 1
    w, h, is32 = hd['w'], hd['h'], hd['is32']
    ow, oh, ml, m0, mb = geometry(w, h, out_w, out_h)
    bpp = 4 if is32 else 3
    rowb = w * bpp + (0 if is32 else (w & 3))
    need = hd['hdr_len'] + rowb * h
    if len(data) < need:
        return 'short', None, hd['flip']
    black_row = [0] * out_w
    out = []
    for _ in range(m0):
        out += black_row
    p = hd['hdr_len']
    j = 0
    accy = h
    for y in range(h):
        row = data[p:p + w * bpp]
        p += rowb
        # 列选择：与 RTL 的 S_STORE 相同
        buf = [None] * ow
        i = 0
        accx = w
        for x in range(w):
            pb, pg, pr = row[x * bpp], row[x * bpp + 1], row[x * bpp + 2]
            while i < ow and accx < 2 * ow:
                buf[i] = (pr << 16) | (pg << 8) | pb
                i += 1
                accx += 2 * w
            accx -= 2 * ow
        while j < oh and accy < 2 * oh:
            out += [0] * ml + buf + [0] * (out_w - ml - ow)
            j += 1
            accy += 2 * h
        accy -= 2 * oh
    for _ in range(mb):
        out += black_row
    assert len(out) == out_w * out_h, (len(out), w, h)
    return 'ok', out, hd['flip']


def to_screen(stream, flip, out_w=640, out_h=480):
    if not flip:
        return list(stream)
    scr = []
    for r in range(out_h):
        s = out_h - 1 - r
        scr += stream[s * out_w:(s + 1) * out_w]
    return scr


def make_bmp(img_rgb, bpp=24, top_down=False, bitfields=False, v5=False):
    """把 PIL RGB 图像（或 numpy HxWx3）写成 BMP 字节。bitfields/v5 只用于 32 位。"""
    import numpy as np
    a = np.asarray(img_rgb, dtype=np.uint8)
    h, w = a.shape[:2]
    rows = a if top_down else a[::-1]
    if bpp == 24:
        pix = rows[..., ::-1]                                    # BGR
        pad = (4 - (w * 3) % 4) % 4
        body = b"".join(r.tobytes() + b"\0" * pad for r in pix)
    else:
        alpha = np.full((h, w, 1), 0xFF, np.uint8)
        pix = np.concatenate([rows[..., ::-1], alpha], axis=2)  # BGRA
        body = pix.tobytes()
    comp = 3 if (bpp == 32 and bitfields) else 0
    hdr_size = 124 if v5 else 40
    extra = b""
    if comp == 3 and hdr_size == 40:
        extra = struct.pack("<III", 0x00FF0000, 0x0000FF00, 0x000000FF)
    dib = struct.pack("<IiiHHIIiiII", hdr_size, w, -h if top_down else h, 1, bpp, comp, len(body), 2835, 2835, 0, 0)
    if hdr_size == 124:
        dib += struct.pack("<IIII", 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000)
        dib += b"BGRs" + b"\0" * 36 + struct.pack("<IIIIIII", 0, 0, 0, 4, 0, 0, 0)
        assert len(dib) == 124
    off = 14 + len(dib) + len(extra)
    return b"BM" + struct.pack("<IHHI", off + len(body), 0, 0, off) + dib + extra + body


if __name__ == "__main__":
    # python bmp_scale.py --rom：生成 bmp_scaler 的双三次权重 ROM（src/user_source/hdl_source/SD/bmp_cubic.mif/.hex）
    import os
    import sys
    if "--rom" in sys.argv:
        here = os.path.dirname(os.path.abspath(__file__))
        out = os.path.join(here, "..", "src", "user_source", "hdl_source", "SD")
        write_cubic_rom(out)
        print("写好 %s/bmp_cubic.mif、bmp_cubic.hex（256 x 30）" % os.path.normpath(out))
