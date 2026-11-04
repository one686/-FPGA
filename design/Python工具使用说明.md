# Python 工具使用说明

本目录是作品用到的 Python 工具，共 6 个脚本，分两类：

- **制卡工具（给使用者）：** 把文字做成滚动标语、把音乐转成板子能播的格式；
- **常数表生成工具（给开发者）：** 生成 FPGA 工程里字库、频谱系数、缩放权重等片内常数表（.mif / .hex / .vh）。工程里现成的文件就是用它们生成的，重新运行得到的结果逐字节相同。

FPGA 运行时不需要电脑，也不需要这些脚本。

## 一、运行环境

- Python 3.8 及以上；
- 依赖库：`pip install numpy pillow scipy`；
- 在本目录（`design/`）下运行。本目录要和 `src/` 放在同一级，常数表生成工具会把结果写进 `src/user_source/hdl_source/` 下的对应位置。

```
提交目录/
├── src/user_source/hdl_source/   ← 常数表写到这里
└── design/                       ← 在这里运行脚本
    ├── fonts/                    思源黑体（SIL OFL 开源字体），字库和标语用
    └── tf_card_music/SONG1.WAV   spec_gen.py 做预览和定点检查用的测试音乐
```

## 二、制卡工具

### 1. make_subtitle.py：文字 → 滚动标语 SUB.BMP

把一行文字排成板子能识别的 4 位色字幕条，拷到 TF 卡根目录，屏幕底部就会滚动显示这句标语。

```
python make_subtitle.py 标语.txt [输出目录]
```

- `标语.txt` 用 UTF-8 保存。写多行会连成一句，行与行之间空两个全角空格；
- 输出 `SUB.BMP` 和 `SUB_预览.png`（640×480 整帧效果）；
- 不给输出目录时，输出在 `标语.txt` 所在的文件夹；**给输出目录时，要先建好这个文件夹**；
- 最长约 25 个汉字（字幕条最宽 816 像素）。超长时不生成文件，脚本会告诉你最多能放到第几个字；
- 板子只认这个工具生成的 SUB.BMP（4 位色、高 40 像素）。不合格时自动显示内置标语"热烈欢迎"。

例：
```
python make_subtitle.py 标语.txt
```

### 2. wav_to_48k.py：任意 WAV → 48 kHz / 16 位

板子只播 48 kHz、16 位、单声道或立体声的 PCM WAV（HDMI 音频固定 48 kHz）。常见的 44.1 kHz、24 位 WAV 会显示"音乐错误 3 / 4"，先用这个工具转一下。

```
python wav_to_48k.py 输入.wav [输出.wav]        转一个文件（默认输出：输入名_48k.wav）
python wav_to_48k.py 输入文件夹 输出文件夹       把文件夹里的 .wav 全部转好
```

- 重采样用 Kaiser 窗带限 sinc 插值，音质比线性插值好；保留原来的声道数；
- 只能处理 WAV。MP3 请先用播放器或剪辑软件导出成 WAV；
- 拷卡时按想要的播放顺序逐个复制，歌单顺序就是复制的顺序。

## 三、常数表生成工具

| 脚本 | 命令 | 生成的文件（`src/user_source/hdl_source/` 下） | 对应 RTL |
|---|---|---|---|
| `osd_gen.py` | `python osd_gen.py` | `osd/osd_font.mif/.hex`（抗锯齿字库）、`osd/osd_cell.mif/.hex`（字符格表）、`osd/osd_layout.vh`（版面表） | `osd/osd_font_rom.v`、`osd/osd_cell_rom.v`、`osd/osd_engine.v` |
| `spec_gen.py` | `python spec_gen.py` | `display/spec_coef.mif/.hex`（32 段滤波器系数）、`display/spec_layout.vh`（频谱版面） | `display/audio_spectrum.v` |
| `bmp_scale.py` | `python bmp_scale.py --rom` | `SD/bmp_cubic.mif/.hex`（双三次插值 256 项权重表） | `SD/bmp_scaler.v` |
| `make_subtitle.py` | `python make_subtitle.py --default` | `osd/sub_default.mif/.hex`（内置默认标语"热烈欢迎"） | `osd/sub_strip_ram.v` |

说明：
- `.mif` 给 TD 综合用，`.hex` 给 ModelSim 仿真用，两者是同一份数据；
- `osd_gen.py`、`spec_gen.py` 运行时还会在本目录生成 `preview_*.png`，这是按 RTL 同样算法算出来的画面预览；
- `spec_gen.py --test` 做定点精度检查：用测试音乐和扫频信号，把定点结果与浮点结果比较；
- `osd_gen.py` 会调用同目录的 `color_gen.py`（调色查找表的参考模型），只用来生成调色预览图；
- 改了字库、版面或系数后要重新综合；这几张表都放在块 RAM 里，不占逻辑资源。

## 四、文件清单

| 文件 | 类别 | 作用 |
|---|---|---|
| `make_subtitle.py` | 制卡 / 常数表 | 标语 → SUB.BMP；`--default` 生成内置默认标语 |
| `wav_to_48k.py` | 制卡 | WAV 转 48 kHz / 16 位 |
| `osd_gen.py` | 常数表 | OSD 字库、字符格表、版面表 |
| `spec_gen.py` | 常数表 | 频谱滤波器系数、频谱版面 |
| `bmp_scale.py` | 常数表 | 双三次插值权重表 |
| `color_gen.py` | 被调用 | 调色查找表参考模型（`osd_gen.py` 生成预览时调用） |
| `fonts/` | 资源 | 思源黑体 Regular / Bold |
| `tf_card_music/SONG1.WAV` | 资源 | `spec_gen.py` 的测试音乐 |
