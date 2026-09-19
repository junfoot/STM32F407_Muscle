# 设计目标
设计STM32F407的嵌入式程序，可以与ADS1292连接并采集两路sEMG。通过按键控制SD卡记录并用led灯显示状态。另外也可以通过串口与上位机LK-M1292R-2EMG-Python连接控制采集。

- 主要参考[STM32F407](../STM32F407)文件夹，这是已经在另一个开发板实现了采集功能。
- 现在用的开发板信息参考文件[text](STM32_F4VX_M原理图.PDF)。
- 引脚的选择已在ioc中配置
- 若有相关修改，cubemx的ioc文件也要同时更新
- ADS1292 两路同步采集；SD 卡记录优先，并且要保证正确完整
- SD卡记录与上位机记录完全解耦。
- SD 卡使用 SDIO；采集期间的 SD 写入不依赖上位机连接状态。
- 默认采集频率为2000Hz，并保证这个频率下能工作
- 串口波特率用460800

## 已实现功能

- ADS1292 双通道同步采集，默认 `2000 SPS`、PGA=12、RLD 开启。
- PA8/EXTI8 在 DRDY 下降沿读取一个完整的 9 字节 ADS1292 数据帧。
- SD 和串口使用互相独立的静态环形队列。串口未连接、停止传输或发生拥塞时，不会停止 SD 记录。
- SD 使用 SDIO 1-bit 和 FatFs。每 32 个样本（512 字节）批量写入，约每秒 `f_sync` 一次，停止记录时先排空队列再同步并关闭文件。
- PA0 按键消抖后切换 SD 记录；PA1 LED 常亮表示正在记录，熄灭表示空闲，快速闪烁表示 ADS1292 或 SD 错误。
- USART1 使用 460800-8-N-1、DMA 发送和 Receive-to-IDLE DMA 接收，兼容 `LK-M1292R-2EMG-Python` 的 A5/5A 请求帧和 AA/55 响应帧。
- 上位机开始/停止命令只控制串口数据流；不会改变按键启动的 SD 记录状态。

## SD 文件格式

文件名从 `EMG0001.BIN` 自动递增。文件由 512 字节文件头和连续的 16 字节小端记录组成：

| 偏移 | 类型 | 内容 |
|---:|---|---|
| 0 | `char[8]` | 标识 `EMG2K01`（末尾补 0） |
| 8 | `uint16` | 文件头长度，固定 512 |
| 10 | `uint16` | 单条记录长度，固定 16 |
| 12 | `uint32` | 采样率 |
| 16 | `uint8` | PGA |
| 17 | `uint8` | 通道使能掩码 |
| 18 | `uint8` | RLD 是否开启 |
| 20 | `uint32` | 开始记录时的 HAL 毫秒 tick |

每条记录依次为 `uint32 sequence`、`uint32 tick_ms`、`float32 ch1_uV`、`float32 ch2_uV`。`sequence` 可用于检测缓冲溢出或丢样。

## 使用方法

1. 插入 FAT16/FAT32 格式的 SD 卡并上电。
2. 按一次 PA0 开始记录；LED 常亮。再次按下停止，等待 LED 熄灭后再拔卡或断电。
3. 上位机串口选择 460800 波特率。仓库中的 Python 示例已同步改为 460800。
4. SD 记录期间不允许上位机更改采样参数，以保证单个文件的文件头与全部样本一致。

## 工程说明

- Keil 工程：`MDK-ARM/STM32F407VET6_ADS1292.uvprojx`
- CubeMX 配置：`STM32F407VET6_ADS1292.ioc`（USART1 波特率和中断优先级已同步）
- FatFs R0.12c 位于 `Middlewares/FatFs`，底层通过 `HAL_SD_*` 访问当前工程的 SDIO 句柄。
- DRDY 的抢占优先级为 1，USART1/DMA 为 2，避免串口传输打断 2 kHz 采样入口。

## SD 维护诊断命令

串口命令 `0x07` 用于自动化测试 SD 记录路径，执行的初始化、启动、排空和关闭流程与 PA0 按键相同。无负载表示查询，负载 `01` 表示开始记录，`00` 表示停止记录。

响应负载固定为 20 字节：`state[1] + dropped[4] + records_written[4] + filename[11]`，多字节整数均为小端。`state` 取值为 0（空闲）、1（记录中）、2（正在排空关闭）或 3（错误）。

## 9011RF IMU 接收与记录

- PA11/PA12 配置为 USB OTG FS Host（DM/DP），通过 USB CDC 以 460800-8-N-1 接收 9011RF；USB 中断优先级为 2，低于 ADS1292 DRDY 的优先级 1。
- 每收到一条完整的 `[device_id][0x55][0x61][26-byte payload]` 就立即生成一条 IMU 记录，不按 ADS1292 采样率重采样，因此保留 IMU 原始输出频率和设备 ID。
- 开始 SD 记录后会创建同编号的 `EMGxxxx.BIN` 和 `IMUxxxx.BIN`。停止记录时两个队列都会先排空，再同步并关闭文件。
- `IMUxxxx.BIN` 包含 512 字节文件头，标识为 `IMU9011`，其后为连续的 40 字节小端记录：`uint32 sequence`、`uint32 tick_ms`、`uint8 device_id`、3 字节保留、`int16 accel[3]`、`int16 gyro[3]`、`int16 mag[3]`、`int16 angle[3]`、`int16 battery_raw`、`uint16 reserved`。
- 换算关系：加速度=`raw*16/32768 g`，角速度=`raw*2000/32768 dps`，角度=`raw*180/32768 deg`，电池电压=`battery_raw/100 V`。
- 串口流开启时，IMU 使用现有 AA/55 响应帧格式、命令码 `0x08`，负载由一至四条上述 40 字节记录组成；EMG 的 `0x06` 格式保持不变。发送器在 EMG 与 IMU 队列之间公平轮转，避免四路 IMU 持续到达时饿死 EMG。
- 只读诊断命令 `0x09` 返回 32 字节：`usb_state[1] + cdc_state[1] + connected[1] + reserved[1] + imu_frames[4] + online_mask[4] + uart_imu_dropped[4] + sd_imu_dropped[4] + usb_rx_bytes[4] + sd_error_detail[4] + uart_emg_dropped[4]`，用于区分 USB 未枚举、CDC 无数据、IMU 解析和 SD/串口写入问题。`sd_error_detail` 的高字节为记录器阶段、低字节为 FatFs `FRESULT`。
- 配套 Python 上位机已能解析 `0x08`。导出 EMG CSV 时，如收到过 IMU 数据，会同时生成同路径的 `*_imu.csv`。

注意：目标板必须给 USB Host 端口/9011RF 接收器提供 5 V VBUS；当前固件只配置 USB 外设，不控制外部 VBUS 电源开关。
