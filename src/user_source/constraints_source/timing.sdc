# ============================================================
# timing.sdc  —  sys_pll_promax_40 单 PLL + 第3课音频方案
#   clk            50MHz 板载时钟，同时驱动 TF/SD 逻辑 (sd_card_clk = clk)
#   sys_pll_m0     clkc[1] 40MHz SDRAM / clkc[2] 40MHz 144° SDRAM 移相
#                  clkc[3] 25MHz 像素  / clkc[4] 125MHz 90° TMDS 串行
#   音频全部在 video_clk 域生成（第3课），已无独立音频时钟
# ============================================================

create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
derive_clocks

rename_clock -name {ext_mem_clk}     -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {ext_mem_clk_sft} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[2]}]
rename_clock -name {video_clk}       -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[3]}]
rename_clock -name {hdmi_5x_clk}     -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[4]}]

# 像素 -> 串行：原来设成 exclusive（不检查），第13课改成"两者之间的路径都不超过 6ns"。
#   原因：exclusive 下装数据用的翻转信号布线没人管，第13课第1步布线后变成 4ns，装数据撞上下一个像素，
#   上板没声音 + 抖动（见 lane_lvds_10_1.v 文件头）。新写法装数据在 26ns 或 34ns，
#   所以翻转信号、像素数据、复位到串行时钟域都只要求 6ns 内到达。
set_max_delay -datapath_only -from [get_clocks {video_clk}] -to [get_clocks {hdmi_5x_clk}] 6
set_max_delay -datapath_only -from [get_clocks {hdmi_5x_clk}] -to [get_clocks {video_clk}] 6

set_false_path -from [get_ports {rst_n}]

# SDRAM 移相时钟：40MHz/144° 下应能自然满足，先不切。
# 如果报告里这两组仍有违例，再打开下面两行（参考工程就是这么处理的）
# set_false_path -from [get_clocks {ext_mem_clk}]     -to [get_clocks {ext_mem_clk_sft}]
# set_false_path -from [get_clocks {ext_mem_clk_sft}] -to [get_clocks {ext_mem_clk}]

# 第11课：帧写 FIFO 的异步清零 fifo_aclr（ext_mem_clk 域）-> write_buf 写侧复位同步器（sd_card_clk 域）
#   复位同步器"异步置位、同步释放"，第一级只用来接异步信号，后面还有一级同步，
#   这条路径本来就不需要满足建立/恢复时间。以前余量为正是布局碰巧，第11课布局变化后报过 -0.125ns。
set_false_path -from [get_regs {u_frame_write_path/frame_fifo_write_m0/fifo_aclr}] -to [get_regs {u_frame_write_path/write_buf/asy_w_rst*}]

# 第13课：40MHz SDRAM 时钟（ext_mem_clk）与 50MHz SD 时钟（clk）、25MHz 像素时钟（video_clk）之间的跨时钟路径。
#   三个时钟都由同一个 PLL 产生，TD 默认按"同步时钟"检查：40MHz 与 50MHz / 25MHz 的时钟沿最近只差 5ns，
#   就把 5ns 当预算，还为了保持时间插缓冲器，这 4 组路径余量只剩 0.1~2ns，布局一变就会红
#   （第13课第3步把菜单计数器加宽 1 位后，Sdr_init_done -> u_sdram_init_sync 报了 -0.231ns）。
#   设计上这些往来全部经过同步器 / 翻转握手 / 异步 FIFO（第5课起），按"异步时钟"约束：
#   只限制数据线延迟不超过 15ns（比 40MHz、50MHz 的周期都短，同步器、握手都满足），不再按时钟沿关系检查。
#   异步 FIFO 格雷码指针有 IP 自带的更严格约束（19.7 / 24.7ns），以 IP 的为准。
#   50MHz <-> 25MHz 是整数倍关系，自然预算 20ns、余量 13ns 以上，当时不动（第12课也改成异步约束，见文件末尾）；
#   SDRAM 移相时钟那一对是真同步，不动。
set_max_delay -datapath_only -from [get_clocks {ext_mem_clk}] -to [get_clocks {clk}]         15
set_max_delay -datapath_only -from [get_clocks {clk}]         -to [get_clocks {ext_mem_clk}] 15
set_max_delay -datapath_only -from [get_clocks {ext_mem_clk}] -to [get_clocks {video_clk}]   15
set_max_delay -datapath_only -from [get_clocks {video_clk}]   -to [get_clocks {ext_mem_clk}] 15

# 第12课：50MHz SD 时钟（clk）<-> 25MHz 像素时钟（video_clk）也按异步约束（用户 2026-10-06 同意）。
#   第12课第2步 LUT 到 90%，布线拥挤，这一对的保持时间报了 -1.4ns（全部是同步器第一级、异步 FIFO 格雷码指针、
#   翻转握手的输入：per_lvl、menu_open、wav_ring 指针等）。设计上这两个时钟之间的往来全部经过同步器/握手/异步 FIFO，
#   按时钟沿关系检查保持时间没有意义；改成只限制数据线延迟 15ns（比 50MHz 周期还短）。
set_max_delay -datapath_only -from [get_clocks {clk}]       -to [get_clocks {video_clk}] 15
set_max_delay -datapath_only -from [get_clocks {video_clk}] -to [get_clocks {clk}]       15
