


`define   DATA_WIDTH                        32
`define   ADDR_WIDTH                        21
`define   DM_WIDTH                          4

`define   ROW_WIDTH                        11
`define   BA_WIDTH                        2

// SDRAM 刷新间隔：
//   SDRAM 时钟 40MHz（周期 25ns）；64ms 内需刷新 2048 行 -> 每 31.25us 刷新一次 = 1250 拍。
//   宏定义必须带括号：否则展开后整数从左往右除，首次相除即得 0，刷新间隔变成 0、实际刷新不足，
//   缓存的图片放久了会丢位出现彩色噪点。
`define	  SDR_CLK_PERIOD				(1000000000/40000000)
`define   SELF_REFRESH_INTERVAL			(64000000/`SDR_CLK_PERIOD/(2**(`ROW_WIDTH)))

