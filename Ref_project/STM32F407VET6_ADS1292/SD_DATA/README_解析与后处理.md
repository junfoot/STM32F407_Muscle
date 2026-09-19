# SD 数据解析与上位机后处理

`parse_sd_data.py` 按 STM32 工程 `sd_recorder.c` 的实际存储格式解析
`EMG2K01` 与 `IMU9011` 文件，并根据文件头自动识别类型。EMG BIN 内两个
通道已经是 `float32` 微伏值，不需要再次按 ADS1292 满量程换算。IMU BIN
会同时输出原始整数和按文件头量程换算后的加速度、角速度、角度与电池电压。

## 2000 Hz 后处理

默认对 CH1、CH2 分别建立独立状态，并严格按照配套 Python 上位机的顺序执行：

1. 18 Hz、六阶 IIR 高通；
2. 50 Hz、40 阶梳状陷波；
3. Direct Form I，零初始状态，因果、逐样本处理。

因此滤波文件开头会保留与上位机相同的启动瞬态，不会静默删除采样点。
当前滤波系数对应本项目的 2000 Hz 采样率；其他采样率文件可用
`--no-filter` 仅解析原始值。

## IMU 解析

IMU CSV 包含设备 ID、时间戳、加速度（raw/g）、角速度（raw/deg/s）、
磁场原始值、角度（raw/deg）和电池（raw/V）。JSON 记录文件头、设备列表、
各设备记录数、连续性和各物理量统计；PNG 显示加速度、角速度、磁场、角度
和电池五联图。`--no-filter` 与 `--trim-seconds` 只影响 EMG。

## sEMG/IMU 对齐

同一文件夹中编号相同的 EMGxxxx.BIN 和 IMUxxxx.BIN 会自动配对；指定其中
任意一个单文件时也会寻找另一个文件。脚本检查两者 start_tick_ms 一致，
然后按每个 device_id 将 IMU 物理量线性插值到 2000 Hz sEMG 时间轴。角度
采用跨正负 180 度边界的最短路径插值，仅保留所有设备共同覆盖的时间范围。

对齐结果为 ALIGNEDxxxx.csv、ALIGNEDxxxx.json 和 ALIGNEDxxxx.png。CSV
中的 imuN_source_offset_ms 表示插值时距最近 IMU 原始样本的时间；JSON
记录对齐方法、重叠区间、插值距离和数据质量警告。对齐 PNG 仅显示滤波后的两个 sEMG 通道，以及每个 IMU 设备的 X/Y/Z 三轴角度。绘图时会对三轴角度分别执行 unwrap，消除跨越正负 180 度边界造成的假跳变；CSV 仍保留标准角度范围。

## 裁剪头尾数据

`--trim-start-seconds` 和 `--trim-end-seconds` 分别控制滤波完成后从
开头、结尾裁掉的时长。例如开头裁掉 1 秒、结尾裁掉 0.5 秒：

```powershell
python STM32F407VET6_ADS1292\SD_DATA\parse_sd_data.py `
  STM32F407VET6_ADS1292\SD_DATA\EMG0001.BIN `
  --trim-start-seconds 1 --trim-end-seconds 0.5
```

程序先对完整数据滤波，再裁剪输出，所以可去掉开头的滤波启动瞬态。
CSV 中的 `record_index`、`sequence` 和 `time_s` 仍对应原始 BIN 的位置，
不会在裁剪后重新编号。JSON 分别记录头部和尾部请求秒数、实际秒数及样本数。
两个参数均不传时默认不裁剪。旧参数 `--trim-seconds` 暂时保留为兼容入口，
表示头尾裁剪相同时长，但不能与两个新参数混用。

## 使用

在项目根目录执行：

```powershell
python STM32F407VET6_ADS1292\SD_DATA\parse_sd_data.py `
  STM32F407VET6_ADS1292\SD_DATA\EMG0001.BIN
```

输入既可以是单个 BIN，也可以是文件夹；`--recursive` 可递归扫描。
结果默认生成在每个源 BIN 所在的文件夹，也可用 `-o OUTPUT_DIR` 指定目录：

- `EMG0001.csv`：时间、序号、原始微伏值、滤波后微伏值和丢样标记；
- `EMG0001.json`：文件头、连续性检查、滤波链和统计量；
- `EMG0001.png`：CH1/CH2 原始与滤波后四联图；
- `IMU0001.csv`：各传感器原始值与换算后的物理量；
- `IMU0001.json`：文件头、设备分布、连续性检查和统计量；
- `IMU0001.png`：加速度、角速度、磁场、角度和电池五联图。

仅解包而不滤波：

```powershell
python STM32F407VET6_ADS1292\SD_DATA\parse_sd_data.py `
  STM32F407VET6_ADS1292\SD_DATA\EMG0001.BIN --no-filter
```

CSV 中的 `ch1_raw_uV`、`ch2_raw_uV` 始终来自 BIN 原始记录；后处理只写入
`ch1_filtered_uV`、`ch2_filtered_uV`，不会修改原始数据。
