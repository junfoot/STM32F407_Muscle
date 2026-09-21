# STM32F407 Muscle 数据采集工程

本仓库以 **STM32F407VET6** 为主控，实现双 AD7606 共 16 路模拟量同步采集、双通道 DAC8563 输出、4 路 WT9011DCL-RF IMU 接收、USART1 实时上传，以及 SD 卡本地记录。工程由 STM32CubeMX/HAL 生成并使用 Keil MDK-ARM 构建。

> 根目录是当前的双 AD7606 主工程。`STM32F407VET6_ADS1292/` 是另一套双通道 sEMG 采集工程，`STM32F407_9011RF/` 是 IMU/USB Host 功能的参考工程，三者不要混用工程文件或引脚配置。

## 主要功能

- 两片 AD7606 同步采集 16 个通道，量程为 ±5 V，采样率为 2000 Hz。
- TIM3 产生采样节拍，AD7606 #1 的 BUSY 下降沿触发两片 ADC 的串行读数。
- 安富莱 DAC8563 模块提供 A/B 两路输出，电压范围为 0～10 V。
- USB OTG FS 工作于 Host/CDC 模式，通过 CH340 接收 WT9011DCL-RF 的 0～3 号 IMU。
- USART1 以 921600-8-N-1 输出 100 Hz、59 通道 VOFA+ JustFloat 数据，并通过 Receive-to-IDLE DMA 接收文本命令；16 路原始 ADC 后追加 16 路实时 sEMG 滤波值。
- SDIO 以 1-bit 模式写入 FAT16/FAT32 SD 卡；ADC 与原始 IMU 数据记录在同一 `LOGxxxx.BIN` 文件中。
- PA15 按键切换记录状态；PA1 低电平点亮，表示正在记录。
- 串口发送、命令解析和 SD 写入均与 2 kHz 采样解耦，使用 DMA/环形缓冲降低阻塞风险。

## 数据链路

```text
TIM3 2 kHz ──> 两片 AD7606 同步转换 ──> BUSY/EXTI ──> 16 通道原始值
                                                        ├─> SD 环形缓冲 ──> LOGxxxx.BIN（原始值不变）
                                                        ├─> 20 Hz 高通 + 50 Hz 陷波 + 450 Hz 低通 ──> 16 路滤波值
                                                        └─> 20 倍抽取 ──> USART1 100 Hz

9011RF 接收器 ──USB Host/CDC──> IMU 解析器 ──────────────┬─> SD 环形缓冲
                                                        └─> 保持最新值并入串口帧
```

## 串口遥测格式

每帧包含 59 个小端 `float32`，末尾追加 `00 00 80 7F`：

| 索引 | 内容 |
|---|---|
| 0 | 连接状态位：bit0=已检测到 SD 卡，bit1=已连接 USB 设备 |
| 1～2 | DAC A、DAC B 最近一次设定的电压（V） |
| 3～18 | AD7606 的 16 路原始电压（V） |
| 19～34 | 16 路实时滤波 sEMG 电压（V）：20 Hz 二阶 Butterworth 高通、50 Hz 陷波（Q=30）、450 Hz 二阶 Butterworth 低通 |
| 35～58 | IMU 0～3，每个依次为 Roll、Pitch、Yaw（°）和 ax、ay、az（g） |

IMU 两次更新之间保持上一帧数值。完整帧长度为 `59 × 4 + 4 = 240` 字节，可直接使用 VOFA+ 的 JustFloat 协议查看。SD 卡仍只写入原来的 `int16 adc[16]` 原始记录。

状态值为 0～3：`0`=均未连接，`1`=仅 SD，`2`=仅 USB，`3`=SD 与 USB 均已连接。SD 没有独立的卡检测引脚，固件空闲时每秒通过 SDIO 探测一次，因此热插拔状态最多约有 1 秒延迟；该状态与“是否正在记录”无关，记录状态仍由 PA1 LED 和 `REC` 命令显示。

## 串口命令

命令不区分大小写，以 CR 或 LF 结束。

```text
DAC A 3.3        # A 通道输出 +3.3 V
DAC B 2.5        # B 通道输出 +2.5 V
DAC AB 0         # 两路同时输出 0 V
DACR A 49152     # 按 0～65535 原始码设置输出
IMU              # 显示 4 个 IMU 的在线状态与最新数据
REC              # 查询 SD 记录状态
REC START        # 新建文件并开始记录
REC STOP         # 排空缓冲、同步并关闭文件
HELP             # 显示帮助
```

USART1 RX 使用 DMA2 Stream2、256 字节 Receive-to-IDLE 缓冲和 512 字节软件环形队列，避免 2 kHz ADC 高优先级中断导致逐字节接收溢出。`HELP` 末尾会显示 `RX errors`、`RX dropped` 和 `TX dropped` 诊断计数。文本回复与 JustFloat 遥测共用 USART1 TX，PC 程序应能从连续二进制流中识别 ASCII 回复，或以遥测值变化确认命令执行结果。

DAC 原始码映射为 `0x0000 = 0 V`、`0x8000 ≈ 5 V`、`0xFFFF = 10 V`。该映射要求安富莱模块的 J1、J2 均短接 **1–2（单极性 0～10 V）**。固件启动时会将两路输出初始化为 0 V；负电压命令会被限幅为 0 V，超过 10 V 的命令会被限幅为 10 V。

设备启动时只检测 SD 卡，不创建日志文件，也不会自动开始记录。按下 PA15/K3 或发送 `REC START` 后才会创建新文件并开始记录；再次按下按键或发送 `REC STOP` 会排空缓冲并关闭文件。

## SD 文件格式

文件名从 `LOG0000.BIN` 开始选择未占用编号。所有多字节数据均为小端序。

文件头固定为 32 字节：

| 偏移 | 类型 | 内容 |
|---:|---|---|
| 0 | `char[4]` | 标识 `EMGL` |
| 4 | `uint16` | 格式版本，当前为 1 |
| 6 | `uint16` | ADC 通道数，当前为 16 |
| 8 | `uint32` | ADC 采样率，当前为 2000 Hz |
| 12 | `uint8` | IMU 数量，当前为 4 |
| 13～31 | - | 保留，填 0 |

文件头之后交错保存两类记录：

- ADC 记录（37 字节）：`0xA1 + uint32 sample_seq + int16 adc[16]`
- IMU 记录（32 字节）：`0xB1 + uint32 sample_seq + uint8 device_id + uint8 payload[26]`

`sample_seq` 是 2000 Hz 公共时间基准，可用于 ADC/IMU 对齐和检查丢样。记录器使用约 48 KiB RAM 环形缓冲、按扇区批量写入，并每 2 秒执行一次 `f_sync()`；检测到缓冲溢出时会增加丢记录计数。

## 关键引脚

| 功能 | 引脚 |
|---|---|
| USART1 | PA9/TX、PA10/RX |
| USB OTG FS Host | PA11/DM、PA12/DP |
| SDIO 1-bit | PC8/D0、PC12/CK、PD2/CMD |
| 记录按键 / LED | PA15/K3（低有效）、PA1/LED（低有效） |
| DAC8563 | PA2/SYNC、PA4/DIN、PA5/LDAC、PA6/CLR、PA8/SCLK |
| AD7606 #1 | PB0/BUSY、PB8/CS、PB9/DOUT、PC0～PC7/控制与时钟 |
| AD7606 #2 | PD0/DOUT、PD1/OS0、PD7/BUSY、PD8～PD15/控制与时钟 |
| SWD | PA13/SWDIO、PA14/SWCLK |

详细的原始硬件核查记录见 [`烧录固件引脚清单.md`](烧录固件引脚清单.md)。该文件描述的是早期实机固件状态；当前配置应以 `STM32F407_Muscle.ioc` 和 `Core/` 源码为准。

## 工程结构

```text
Core/                         当前主工程应用、驱动和中断代码
Drivers/                      STM32F4 HAL 与 CMSIS
Middlewares/                  FatFs、USB Host Core/CDC
USB_HOST/                     USB Host 初始化与底层配置
MDK-ARM/                      Keil 工程与启动文件
STM32F407_Muscle.ioc          当前主工程 CubeMX 配置
STM32F407VET6_ADS1292/        独立的 ADS1292 双通道 sEMG 工程及数据工具
STM32F407_9011RF/             9011RF 通信参考工程/资料
原理图V2.8--.pdf              当前硬件原理图
Requirements.md               功能需求记录
```

主工程应用文件职责：

- `main.c`：初始化、主循环、采样中断、USB CDC 状态机和遥测组帧。
- `ad7606.c`：双 AD7606 GPIO 模拟串行驱动。
- `dac8563.c`：DAC8563 GPIO 模拟 SPI 驱动及电压映射。
- `imu_parser.c`：9011RF 字节流解析和物理量换算。
- `recorder.c`：FatFs 文件管理、混合记录格式及环形缓冲。
- `serial.c`：USART1 DMA 发送环形缓冲及 `printf` 重定向。
- `cmd.c`：非阻塞文本命令接收与执行。

## 编译与使用

1. 使用 STM32CubeMX 打开 `STM32F407_Muscle.ioc`。当前配置基于 STM32Cube FW_F4 V1.28.3；重新生成代码时保留 User Code 区域。
2. 使用 Keil MDK-ARM 5 打开 `MDK-ARM/STM32F407_Muscle.uvprojx`，选择 `STM32F407VETx` 目标并编译。
3. 通过 SWD 下载固件；串口工具设置为 921600-8-N-1。
4. 如需记录，插入 FAT16/FAT32 SD 卡并按下 PA15/K3，或发送 `REC START`；LED 点亮后表示记录已经开始。再次按下按键或发送 `REC STOP` 可停止记录。上电本身不会创建文件或开始记录。
5. 如需 IMU，连接 9011RF USB 接收器，并确保 USB Host 端口能够提供稳定的 5 V VBUS。

## 注意事项

- SD 停止记录时应等待 LED 熄灭及文件关闭信息输出后再拔卡或断电。
- USB Host 固件只负责 USB 外设与 CDC 通信，外部 VBUS 供电能力取决于实际硬件。
- AD7606 数据读取在 EXTI 中完成，修改 GPIO 模拟时序或中断优先级时需重新验证 2 kHz 下的完整性。
- `STM32F407VET6_ADS1292/SD_DATA/parse_sd_data.py` 面向 ADS1292 工程的 `EMGxxxx.BIN`/`IMUxxxx.BIN`，不能直接解析根工程的 `LOGxxxx.BIN`。
- 修改外设、引脚、时钟或中断配置时，应同步更新 `STM32F407_Muscle.ioc`。
