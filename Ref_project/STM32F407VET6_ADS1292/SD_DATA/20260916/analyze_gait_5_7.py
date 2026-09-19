#!/usr/bin/env python3
"""Compare trials 5--7 (level walking, stair descent, stair ascent).

The analysis deliberately avoids comparing individual IMU axes across sensors.
It uses rotation-invariant vector norms and PCA-derived principal motion axes,
which remain useful when the sensor coordinate frames are not well aligned.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import binary_closing, binary_opening, uniform_filter1d
from scipy.signal import butter, sosfiltfilt, welch


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "analysis_results"
TRIALS = {5: "平地行走", 6: "下楼梯", 7: "上楼梯"}
FS = 2000.0
DS = 10
FS_MOTION = FS / DS


def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    padded = np.r_[False, mask, False].astype(np.int8)
    changes = np.diff(padded)
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))


def rms_envelope(x: np.ndarray, samples: int) -> np.ndarray:
    return np.sqrt(np.maximum(uniform_filter1d(x * x, samples, mode="nearest"), 0.0))


def principal_signal(xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centered = xyz - np.mean(xyz, axis=0, keepdims=True)
    cov = np.cov(centered, rowvar=False)
    values, vectors = np.linalg.eigh(cov)
    order = np.argsort(values)[::-1]
    values = np.maximum(values[order], 0.0)
    pc = centered @ vectors[:, order[0]]
    return pc, values


def dominant_stride_frequency(signal: np.ndarray, fs: float) -> float:
    if len(signal) < int(2 * fs):
        return float("nan")
    sos = butter(4, [0.45, 2.2], btype="bandpass", fs=fs, output="sos")
    filtered = sosfiltfilt(sos, signal)
    nperseg = min(len(filtered), int(8 * fs))
    freq, power = welch(filtered, fs=fs, nperseg=nperseg)
    band = (freq >= 0.55) & (freq <= 1.8)
    if not np.any(band):
        return float("nan")
    fb, pb = freq[band], power[band]
    peak = int(np.argmax(pb))
    # Quadratic interpolation reduces FFT-bin quantisation when possible.
    if 0 < peak < len(pb) - 1:
        y0, y1, y2 = np.log(pb[peak - 1 : peak + 2] + np.finfo(float).tiny)
        delta = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2)
        return float(fb[peak] + delta * (fb[1] - fb[0]))
    return float(fb[peak])


def load_trial(number: int) -> dict[str, np.ndarray]:
    # time, filtered EMG, then offset/acceleration/gyroscope for each IMU.
    usecols = [0, 6, 7]
    for base in (8, 22, 36):
        usecols += [base, base + 1, base + 2, base + 3, base + 4, base + 5, base + 6]
    a = np.loadtxt(
        ROOT / f"ALIGNED{number:04d}.csv",
        delimiter=",",
        skiprows=1,
        usecols=usecols,
        dtype=np.float64,
    )
    result: dict[str, np.ndarray] = {"time": a[:, 0], "emg": a[:, 1:3]}
    pos = 3
    for device in range(3):
        result[f"imu{device}_offset"] = a[:, pos]
        result[f"imu{device}_acc"] = a[:, pos + 1 : pos + 4]
        result[f"imu{device}_gyro"] = a[:, pos + 4 : pos + 7]
        pos += 7
    return result


def active_window(data: dict[str, np.ndarray]) -> tuple[int, int, np.ndarray, float]:
    # Downsample after calculating norms. IMU values were linearly interpolated to
    # 2 kHz, so 200 Hz is ample for human motion while avoiding duplicate weighting.
    gyro = []
    dynamic_acc = []
    for device in range(3):
        g = np.linalg.norm(data[f"imu{device}_gyro"], axis=1)[::DS]
        acc = np.linalg.norm(data[f"imu{device}_acc"], axis=1)[::DS]
        gyro.append(g)
        dynamic_acc.append(np.abs(acc - uniform_filter1d(acc, int(FS_MOTION), mode="nearest")))
    gyro_term = np.median(np.vstack(gyro), axis=0)
    acc_term = np.median(np.vstack(dynamic_acc), axis=0)
    gyro_term = uniform_filter1d(gyro_term, max(1, int(0.20 * FS_MOTION)))
    acc_term = uniform_filter1d(acc_term, max(1, int(0.20 * FS_MOTION)))
    # Robustly scale and combine two orientation-invariant motion measures.
    def scale(x: np.ndarray) -> np.ndarray:
        lo, hi = np.percentile(x, [15, 90])
        return np.clip((x - lo) / max(hi - lo, np.finfo(float).eps), 0, 2)

    score = 0.7 * scale(gyro_term) + 0.3 * scale(acc_term)
    threshold = 0.24
    mask = score > threshold
    mask = binary_closing(mask, structure=np.ones(max(1, int(0.65 * FS_MOTION)), bool))
    mask = binary_opening(mask, structure=np.ones(max(1, int(0.20 * FS_MOTION)), bool))
    candidates = [(s, e) for s, e in runs(mask) if e - s >= 1.5 * FS_MOTION]
    if not candidates:
        candidates = [(0, len(score))]
    # Prefer duration, with a small reward for high activity; this rejects brief
    # handling impulses at the head/tail.
    start_ds, end_ds = max(
        candidates,
        key=lambda se: (se[1] - se[0]) * (1 + 0.15 * float(np.mean(score[se[0] : se[1]]))),
    )
    return start_ds * DS, min(end_ds * DS, len(data["time"])), score, threshold


def analyze_trial(number: int) -> tuple[dict[str, float | int | str], dict[str, np.ndarray]]:
    data = load_trial(number)
    start, end, score, threshold = active_window(data)
    t = data["time"]
    active_t = t[start:end]
    duration = float(active_t[-1] - active_t[0]) if len(active_t) > 1 else 0.0

    env = np.column_stack(
        [rms_envelope(data["emg"][:, ch], int(0.10 * FS)) for ch in range(2)]
    )
    emg_active = data["emg"][start:end]
    env_active = env[start:end]
    row: dict[str, float | int | str] = {
        "trial": number,
        "condition": TRIALS[number],
        "recording_duration_s": round(float(t[-1] - t[0]), 4),
        "active_start_s": round(float(t[start]), 4),
        "active_end_s": round(float(t[end - 1]), 4),
        "active_duration_s": round(duration, 4),
    }

    for ch, side in enumerate(("left", "right")):
        signal = emg_active[:, ch]
        envelope = env_active[:, ch]
        row[f"emg_{side}_rms_uV"] = float(np.sqrt(np.mean(signal * signal)))
        row[f"emg_{side}_median_env_uV"] = float(np.median(envelope))
        row[f"emg_{side}_p95_env_uV"] = float(np.percentile(envelope, 95))
        row[f"emg_{side}_iav_uV_s_per_s"] = float(np.trapezoid(np.abs(signal), dx=1 / FS) / duration)

    l_rms = float(row["emg_left_rms_uV"])
    r_rms = float(row["emg_right_rms_uV"])
    row["emg_bilateral_asymmetry_pct"] = 200 * abs(l_rms - r_rms) / max(l_rms + r_rms, 1e-12)
    row["emg_envelope_correlation"] = float(np.corrcoef(env_active[:, 0], env_active[:, 1])[0, 1])

    stride_freqs = []
    principal_series = []
    for device, label in enumerate(("left_thigh", "right_thigh", "sacrum")):
        xyz = data[f"imu{device}_gyro"][start:end:DS]
        pc, eig = principal_signal(xyz)
        principal_series.append(pc)
        total = float(np.sum(eig))
        row[f"imu_{label}_gyro_rms_dps"] = float(np.sqrt(np.mean(np.sum(xyz * xyz, axis=1))))
        row[f"imu_{label}_principal_variance_pct"] = 100 * float(eig[0]) / max(total, 1e-12)
        if device < 2:
            stride_freqs.append(dominant_stride_frequency(pc, FS_MOTION))

        acc = data[f"imu{device}_acc"][start:end:DS]
        acc_norm = np.linalg.norm(acc, axis=1)
        row[f"imu_{label}_dynamic_acc_rms_g"] = float(
            np.sqrt(np.mean((acc_norm - uniform_filter1d(acc_norm, int(FS_MOTION))) ** 2))
        )

    # A non-sinusoidal thigh waveform can have a stronger second harmonic than
    # its gait-cycle fundamental. If the legs disagree by almost exactly 2x,
    # fold the higher estimate down before deriving cadence.
    raw_stride_freqs = stride_freqs.copy()
    if all(np.isfinite(stride_freqs)):
        low_i, high_i = np.argsort(stride_freqs)
        ratio = stride_freqs[high_i] / max(stride_freqs[low_i], 1e-12)
        if 1.75 <= ratio <= 2.25:
            stride_freqs[high_i] /= 2.0
    valid_stride = np.asarray([f for f in stride_freqs if np.isfinite(f)])
    row["left_stride_frequency_raw_hz"] = float(raw_stride_freqs[0])
    row["right_stride_frequency_raw_hz"] = float(raw_stride_freqs[1])
    row["left_stride_frequency_hz"] = float(stride_freqs[0])
    row["right_stride_frequency_hz"] = float(stride_freqs[1])
    row["cadence_steps_per_min"] = float(120 * np.mean(valid_stride)) if len(valid_stride) else float("nan")
    row["stride_frequency_asymmetry_pct"] = (
        200 * abs(stride_freqs[0] - stride_freqs[1]) / max(sum(stride_freqs), 1e-12)
    )

    data["env"] = env
    data["active_bounds"] = np.array([start, end])
    data["motion_score"] = score
    data["motion_threshold"] = np.array([threshold])
    return row, data


def write_csv(rows: list[dict[str, float | int | str]]) -> None:
    fields = list(rows[0])
    with (OUT / "metrics_5_7.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def make_overview(rows: list[dict[str, float | int | str]], datasets: dict[int, dict[str, np.ndarray]]) -> None:
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(3, 3, figsize=(15, 10), constrained_layout=True)
    for column, number in enumerate(TRIALS):
        data = datasets[number]
        row = rows[column]
        t = data["time"]
        start, end = data["active_bounds"]
        for ax in axes[:, column]:
            ax.axvspan(t[start], t[end - 1], color="#dff2df", alpha=0.65, zorder=0)
        axes[0, column].plot(t, data["env"][:, 0], lw=0.7, label="左竖脊肌")
        axes[0, column].plot(t, data["env"][:, 1], lw=0.7, label="右竖脊肌")
        axes[0, column].set_ylabel("100 ms RMS (µV)")
        axes[0, column].set_title(
            f"{number}: {TRIALS[number]}\n活动窗 {row['active_start_s']:.1f}–{row['active_end_s']:.1f} s"
        )
        if column == 0:
            axes[0, column].legend(fontsize=8)

        for device, label in enumerate(("左大腿", "右大腿", "尾椎")):
            gyro_norm = np.linalg.norm(data[f"imu{device}_gyro"], axis=1)
            axes[1, column].plot(t[::DS], gyro_norm[::DS], lw=0.65, label=label)
        axes[1, column].set_ylabel("角速度模长 (°/s)")
        if column == 0:
            axes[1, column].legend(fontsize=8)

        tm = t[::DS][: len(data["motion_score"])]
        axes[2, column].plot(tm, data["motion_score"], color="black", lw=0.8)
        axes[2, column].axhline(data["motion_threshold"][0], color="red", ls="--", lw=0.8)
        axes[2, column].set_ylabel("无轴向活动分数")
        axes[2, column].set_xlabel("时间 (s)")
    fig.savefig(OUT / "overview_5_7.png", dpi=180)
    plt.close(fig)


def fmt(value: float, digits: int = 1) -> str:
    return f"{value:.{digits}f}"


def make_report(rows: list[dict[str, float | int | str]]) -> None:
    by_trial = {int(r["trial"]): r for r in rows}
    flat = by_trial[5]
    lines = [
        "# 第 5–7 组步态数据初步分析",
        "",
        "## 数据与方法",
        "",
        "- 第5组：平地行走；第6组：下楼梯；第7组：上楼梯。",
        "- sEMG1/2 分别为左/右竖脊肌；IMU0/1/2 分别位于左大腿、右大腿、尾椎骨。",
        "- 用三枚 IMU 的角速度模长和动态加速度模长联合识别持续运动区间，排除头尾静止与短促拿放冲击。绿色区域见总览图。",
        "- 肌电采用已有 18 Hz 高通 + 50 Hz 梳状陷波结果，再计算 100 ms RMS 包络；只在自动识别的活动窗内统计。原转换流程已先删除滤波后的前 1 s，进一步减弱了零初值瞬态。",
        "- 为降低安装轴不一致的影响，IMU 比较采用三轴向量模长、三轴协方差特征值以及各传感器自己的 PCA 主运动轴；没有把不同传感器的同名单轴直接相减。",
        "",
        "## 自动分段与主要指标",
        "",
        "|组别|工况|活动区间 (s)|时长 (s)|左EMG RMS (µV)|右EMG RMS (µV)|双侧差异 (%)|包络相关|步频 (步/min)|尾椎动态加速度 RMS (g)|",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for r in rows:
        lines.append(
            f"|{r['trial']}|{r['condition']}|{fmt(float(r['active_start_s']))}–{fmt(float(r['active_end_s']))}|"
            f"{fmt(float(r['active_duration_s']))}|{fmt(float(r['emg_left_rms_uV']))}|"
            f"{fmt(float(r['emg_right_rms_uV']))}|{fmt(float(r['emg_bilateral_asymmetry_pct']))}|"
            f"{fmt(float(r['emg_envelope_correlation']), 2)}|{fmt(float(r['cadence_steps_per_min']))}|"
            f"{fmt(float(r['imu_sacrum_dynamic_acc_rms_g']), 3)}|"
        )

    lines += ["", "## 相对平地的变化", ""]
    for n in (6, 7):
        r = by_trial[n]
        l_change = 100 * float(r["emg_left_rms_uV"]) / float(flat["emg_left_rms_uV"]) - 100
        r_change = 100 * float(r["emg_right_rms_uV"]) / float(flat["emg_right_rms_uV"]) - 100
        trunk_change = 100 * float(r["imu_sacrum_dynamic_acc_rms_g"]) / float(flat["imu_sacrum_dynamic_acc_rms_g"]) - 100
        lines.append(
            f"- {r['condition']}：左/右竖脊肌 RMS 相对平地分别为 {l_change:+.1f}% / {r_change:+.1f}%；"
            f"尾椎动态加速度 RMS 为 {trunk_change:+.1f}%。"
        )

    lines += [
        "",
        "## 可以支持的初步结论",
        "",
        "1. 三种任务的竖脊肌负荷、左右协同性、步频及躯干动态冲击可以进行定量比较；具体数值以上表和 CSV 为准。",
        "2. 单轴波形不能跨 IMU 直接比较，因为安装姿态差异会把同一身体运动投影到不同轴；本分析的模长/PCA 结果更适合当前数据。",
        "3. 大腿角速度的第一主成分方差占比可衡量运动是否近似集中在一个主旋转轴；占比高时，PCA 主轴信号可作为近似屈伸波形，但仍不是解剖学关节角。",
        "4. 当前只有每种工况一次、同一受试者的一段记录，没有 MVC 归一化、足接触事件或安装标定，因此只能作为描述性个案结果，不能做组间统计推断，也不宜把原始 µV 直接解释为普遍的肌肉负荷大小。",
        "",
        "## 质量控制与后续建议",
        "",
        "- 对齐文件显示 EMG 序列连续、无缺样；但 IMU 插值最近源样本距离的最大值在部分通道达到约 0.1–0.36 s。应查看原始时间戳，把源样本距离过大的区间标为低可信，而不是把插值曲线当作真实高频采样。",
        "- 正式采集前做 5–10 s 静止站立标定，记录每个 IMU 的安装方向；若要算髋/躯干角，建议保存四元数并用传感器到解剖坐标系的标定旋转矩阵变换。",
        "- 增加同步脚踏开关或足部 IMU，明确脚跟着地/离地事件；每工况至少重复多次，并采集 MVC，之后可做步态周期归一化和统计检验。",
        "",
        "## 输出文件",
        "",
        "- `metrics_5_7.csv`：完整数值指标。",
        "- `overview_5_7.png`：EMG 包络、三枚 IMU 角速度模长与自动活动窗。",
        "- `analysis_parameters.json`：关键参数，便于复现。",
        "",
    ]
    (OUT / "report_5_7.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUT.mkdir(exist_ok=True)
    rows = []
    datasets = {}
    for number in TRIALS:
        row, data = analyze_trial(number)
        rows.append(row)
        datasets[number] = data
    write_csv(rows)
    make_overview(rows, datasets)
    make_report(rows)
    parameters = {
        "emg_sample_rate_hz": FS,
        "motion_analysis_rate_hz": FS_MOTION,
        "emg_envelope": "100 ms moving RMS",
        "motion_segmentation": "median of rotation-invariant gyro/dynamic-acc norms; robust scaling; threshold 0.24",
        "stride_frequency": "PCA gyro principal component, 0.45-2.2 Hz bandpass, Welch peak 0.55-1.8 Hz",
        "caveat": "Descriptive single-recording analysis; no MVC or anatomical-frame calibration.",
    }
    (OUT / "analysis_parameters.json").write_text(
        json.dumps(parameters, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(rows, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
