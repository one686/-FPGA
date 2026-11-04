# -*- coding: utf-8 -*-
"""第10课：把任意 WAV 转成板子能播的格式（16 位 PCM、48kHz，保留单/双声道）。

板子只播 48kHz / 16 位 / 1~2 声道的 PCM WAV（HDMI 音频固定 48kHz）。常见的 44.1kHz 音乐、
24 位或 8 位的 WAV 在屏幕上会显示"音乐错误 3/4"，用这个脚本转一下再拷进 TF 卡根目录即可。

用法：
  python wav_to_48k.py 输入.wav [输出.wav]          转一个文件（默认输出 = 输入名_48k.wav）
  python wav_to_48k.py 文件夹 输出文件夹             把文件夹里所有 .wav 都转好放进输出文件夹
只用 Python 自带的 wave 模块 + numpy；重采样用带限 sinc 插值（Kaiser 窗），音质比线性插值好得多。
"""
import os
import sys
import wave

import numpy as np

TARGET = 48000


def read_wav(path):
    with wave.open(path, "rb") as w:
        ch, sw, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if sw == 1:
        x = (np.frombuffer(raw, np.uint8).astype(np.float64) - 128) / 128
    elif sw == 2:
        x = np.frombuffer(raw, "<i2").astype(np.float64) / 32768
    elif sw == 3:
        b = np.frombuffer(raw, np.uint8).reshape(-1, 3).astype(np.int32)
        v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        v = np.where(v >= 1 << 23, v - (1 << 24), v)
        x = v.astype(np.float64) / (1 << 23)
    elif sw == 4:
        x = np.frombuffer(raw, "<i4").astype(np.float64) / (1 << 31)
    else:
        raise ValueError("不支持的位深 %d 字节" % sw)
    x = x.reshape(-1, ch)
    if ch > 2:                       # 多声道：取前两个声道
        x = x[:, :2]
    return x, rate


def resample(x, src, dst, taps=32):
    """带限 sinc 插值：每个输出点用附近 2*taps 个输入点，截止频率取两者较低采样率的 0.45 倍"""
    if src == dst:
        return x
    n_out = int(round(len(x) * dst / src))
    t = np.arange(n_out) * (src / dst)
    fc = 0.45 * min(src, dst) / src * 2          # 相对输入奈奎斯特的截止
    beta = 8.6
    out = np.zeros((n_out, x.shape[1]))
    i0 = np.floor(t).astype(np.int64)
    for k in range(-taps + 1, taps + 1):
        idx = i0 + k
        d = t - idx
        win = np.i0(beta * np.sqrt(np.clip(1 - (d / taps) ** 2, 0, 1))) / np.i0(beta)   # Kaiser 窗
        w = fc * np.sinc(fc * d) * win
        ok = (idx >= 0) & (idx < len(x))
        out[ok] += x[idx[ok]] * w[ok, None]
    return out


def write_wav(path, x):
    y = np.clip(np.round(x * 32767), -32768, 32767).astype("<i2")
    with wave.open(path, "wb") as w:
        w.setnchannels(x.shape[1])
        w.setsampwidth(2)
        w.setframerate(TARGET)
        w.writeframes(y.tobytes())


def convert(src, dst):
    x, rate = read_wav(src)
    y = resample(x, rate, TARGET)
    peak = np.max(np.abs(y)) if len(y) else 0
    if peak > 1:                     # 重采样后偶尔超出满幅一点：整体压回去，避免削波
        y = y / peak
    write_wav(dst, y)
    print("%s -> %s   %d Hz %d 声道 -> 48000 Hz 16 位，%.1f 秒" % (os.path.basename(src), dst, rate, y.shape[1], len(y) / TARGET))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    src = sys.argv[1]
    if os.path.isdir(src):
        out = sys.argv[2] if len(sys.argv) > 2 else src + "_48k"
        os.makedirs(out, exist_ok=True)
        for f in sorted(os.listdir(src)):
            if f.lower().endswith(".wav"):
                convert(os.path.join(src, f), os.path.join(out, f))
    else:
        dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + "_48k.wav"
        convert(src, dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())
