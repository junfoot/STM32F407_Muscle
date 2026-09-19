#!/usr/bin/env python3
"""Parse EMG2K01/IMU9011 SD data next to the source BIN files."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterator, Sequence

MAGIC = b"EMG2K01"
HEADER_PREFIX = struct.Struct("<8sHHIBBBxI")
RECORD = struct.Struct("<IIff")
IMU_MAGIC = b"IMU9011"
IMU_HEADER_PREFIX = struct.Struct("<8sHHIIIII")
IMU_RECORD = struct.Struct("<IIB3x3h3h3h3hhH")
DEFAULT_INPUT = Path(__file__).resolve().parent

# Exact 2000 Hz coefficients from LK-M1292R-2EMG-Python/emg_filter.py.
HP_B = [0.896495417452848, -5.37897250471709, 13.4474312617927,
        -17.9299083490570, 13.4474312617927, -5.37897250471709,
        0.896495417452848]
HP_A = [1, -5.78151843076305, 13.9313314341308,
        -17.9085161935195, 12.9528179055271, -4.99781871952794,
        0.803704033513957]
NOTCH_B = [0.897874972040025] + [0.0] * 39 + [-0.897874972040025]
NOTCH_A = [1.0] + [0.0] * 39 + [-0.795749944080050]


@dataclass(frozen=True)
class FileHeader:
    magic: str
    header_size: int
    record_size: int
    sample_rate_hz: int
    pga: int
    channel_mask: int
    rld_enabled: int
    start_tick_ms: int


@dataclass(frozen=True)
class Record:
    record_index: int
    sequence: int
    time_s: float
    tick_ms: int
    tick_elapsed_ms: int
    ch1_raw_uV: float
    ch2_raw_uV: float
    sequence_gap_before: int
    byte_offset_adjustment: int = 0
    imputed_fields: str = ""


@dataclass(frozen=True)
class ImuFileHeader:
    magic: str
    header_size: int
    record_size: int
    start_tick_ms: int
    accel_full_scale_g: int
    gyro_full_scale_dps: int
    angle_full_scale_deg: int
    battery_divisor: int


@dataclass(frozen=True)
class ImuRecord:
    record_index: int
    sequence: int
    time_s: float
    tick_ms: int
    tick_elapsed_ms: int
    device_id: int
    accel_x_raw: int
    accel_y_raw: int
    accel_z_raw: int
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    gyro_x_raw: int
    gyro_y_raw: int
    gyro_z_raw: int
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    mag_x_raw: int
    mag_y_raw: int
    mag_z_raw: int
    angle_x_raw: int
    angle_y_raw: int
    angle_z_raw: int
    angle_x_deg: float
    angle_y_deg: float
    angle_z_deg: float
    battery_raw: int
    battery_voltage: float
    sequence_gap_before: int


@dataclass(frozen=True)
class ProcessedRecord:
    raw: Record
    ch1_filtered_uV: float
    ch2_filtered_uV: float


class IirFilter:
    """Direct Form I with zero state, matching the vendor implementation."""

    def __init__(self, numerator: Sequence[float], denominator: Sequence[float]):
        self.b = [float(value) for value in numerator]
        self.a = [float(value) for value in denominator]
        self.x_history = [0.0] * len(self.b)
        self.y_history = [0.0] * (len(self.a) - 1)

    def process_sample(self, value: float) -> float:
        for index in range(len(self.x_history) - 1, 0, -1):
            self.x_history[index] = self.x_history[index - 1]
        self.x_history[0] = value
        output = 0.0
        for index in range(len(self.b)):
            output += self.b[index] * self.x_history[index]
        for index in range(len(self.y_history)):
            output -= self.a[index + 1] * self.y_history[index]
        output /= self.a[0]
        for index in range(len(self.y_history) - 1, 0, -1):
            self.y_history[index] = self.y_history[index - 1]
        if self.y_history:
            self.y_history[0] = output
        return output


class Emg2000Filter:
    """18 Hz sixth-order high-pass followed by a 50 Hz comb notch."""

    def __init__(self):
        self.high_pass = IirFilter(HP_B, HP_A)
        self.notch = IirFilter(NOTCH_B, NOTCH_A)

    def process_sample(self, value: float) -> float:
        return self.notch.process_sample(self.high_pass.process_sample(value))


@dataclass(frozen=True)
class ParseResult:
    source: str
    header: FileHeader
    source_record_count: int
    record_count: int
    first_sequence: int
    last_sequence: int
    missing_samples: int
    discontinuities: int
    trailing_bytes: int
    source_duration_s: float
    duration_s: float
    trimmed_head_samples: int
    trimmed_tail_samples: int
    processing: dict[str, object]
    statistics: dict[str, dict[str, float]]


def uint32_delta(value: int, origin: int) -> int:
    return (value - origin) & 0xFFFFFFFF


def read_header(data: bytes, source: Path) -> FileHeader:
    if len(data) < HEADER_PREFIX.size:
        raise ValueError(f"{source.name}: file is too short for an EMG2K01 header")
    magic_raw, header_size, record_size, rate, pga, mask, rld, start_tick = \
        HEADER_PREFIX.unpack_from(data)
    if magic_raw.rstrip(b"\0") != MAGIC:
        raise ValueError(f"{source.name}: invalid magic; expected EMG2K01")
    if header_size < HEADER_PREFIX.size or header_size > len(data):
        raise ValueError(f"{source.name}: invalid header_size {header_size}")
    if record_size != RECORD.size:
        raise ValueError(f"{source.name}: record_size is {record_size}, expected 16")
    if rate == 0:
        raise ValueError(f"{source.name}: sample_rate is zero")
    return FileHeader("EMG2K01", header_size, record_size, rate, pga, mask,
                      rld, start_tick)


def parse_file(path: Path) -> tuple[FileHeader, list[Record], int]:
    data = path.read_bytes()
    header = read_header(data, path)
    position = header.header_size
    samples: list[list[object]] = []
    while position + RECORD.size <= len(data):
        values = list(RECORD.unpack_from(data, position))
        adjustment = 0
        if samples:
            expected = (int(samples[-1][0]) + 1) & 0xFFFFFFFF
            if int(values[0]) != expected:
                for candidate_adjustment in (-4, 4, -8, 8, -12, 12):
                    candidate = position + candidate_adjustment
                    if candidate < header.header_size or candidate + RECORD.size > len(data):
                        continue
                    candidate_sequence = struct.unpack_from("<I", data, candidate)[0]
                    if candidate_sequence != expected:
                        continue
                    following = candidate + RECORD.size
                    next_valid = (
                        following + 4 > len(data)
                        or struct.unpack_from("<I", data, following)[0]
                        == ((expected + 1) & 0xFFFFFFFF))
                    candidate_tick = struct.unpack_from("<I", data, candidate + 4)[0]
                    tick_plausible = uint32_delta(
                        candidate_tick, int(samples[-1][1])) < 1000
                    if next_valid and tick_plausible:
                        adjustment = candidate_adjustment
                        position = candidate
                        values = list(RECORD.unpack_from(data, position))
                        break
        if adjustment < 0 and samples:
            overlap = -adjustment
            if overlap >= 4:
                samples[-1][3] = None
                samples[-1][5] = "ch2_raw_uV"
            if overlap >= 8:
                samples[-1][2] = None
                samples[-1][5] = "ch1_raw_uV,ch2_raw_uV"
        samples.append([*values, adjustment, ""])
        position += RECORD.size
    if not samples:
        raise ValueError(f"{path.name}: no complete sample records")
    for sample_index, sample in enumerate(samples):
        for value_index in (2, 3):
            if sample[value_index] is not None:
                continue
            left = sample_index - 1
            while left >= 0 and samples[left][value_index] is None:
                left -= 1
            right = sample_index + 1
            while right < len(samples) and samples[right][value_index] is None:
                right += 1
            if left >= 0 and right < len(samples):
                ratio = (sample_index - left) / (right - left)
                sample[value_index] = (
                    float(samples[left][value_index])
                    + ratio * (float(samples[right][value_index])
                               - float(samples[left][value_index])))
            elif left >= 0:
                sample[value_index] = float(samples[left][value_index])
            elif right < len(samples):
                sample[value_index] = float(samples[right][value_index])
            else:
                raise ValueError(f"{path.name}: cannot impute damaged EMG value")
    first_sequence, first_tick = int(samples[0][0]), int(samples[0][1])
    records: list[Record] = []
    for index, sample in enumerate(samples):
        sequence, tick_ms = int(sample[0]), int(sample[1])
        gap = 0
        if index:
            step = uint32_delta(sequence, int(samples[index - 1][0]))
            if 1 < step < 0x80000000:
                gap = step - 1
        records.append(Record(
            index, sequence,
            uint32_delta(sequence, first_sequence) / header.sample_rate_hz,
            tick_ms, uint32_delta(tick_ms, first_tick),
            float(sample[2]), float(sample[3]), gap,
            int(sample[4]), str(sample[5])))
    return header, records, len(data) - position


def read_imu_header(data: bytes, source: Path) -> ImuFileHeader:
    if len(data) < IMU_HEADER_PREFIX.size:
        raise ValueError(f"{source.name}: file is too short for an IMU9011 header")
    values = IMU_HEADER_PREFIX.unpack_from(data)
    magic_raw, header_size, record_size = values[:3]
    if magic_raw.rstrip(b"\0") != IMU_MAGIC:
        raise ValueError(f"{source.name}: invalid magic; expected IMU9011")
    if header_size < IMU_HEADER_PREFIX.size or header_size > len(data):
        raise ValueError(f"{source.name}: invalid header_size {header_size}")
    if record_size != IMU_RECORD.size:
        raise ValueError(
            f"{source.name}: record_size is {record_size}, expected {IMU_RECORD.size}")
    if values[7] == 0:
        raise ValueError(f"{source.name}: battery_divisor is zero")
    return ImuFileHeader("IMU9011", header_size, record_size, *values[3:])


def parse_imu_file(path: Path) -> tuple[ImuFileHeader, list[ImuRecord], int]:
    data = path.read_bytes()
    header = read_imu_header(data, path)
    count, trailing = divmod(len(data) - header.header_size, header.record_size)
    if count == 0:
        raise ValueError(f"{path.name}: no complete IMU records")
    unpacked = [IMU_RECORD.unpack_from(
        data, header.header_size + index * header.record_size)
        for index in range(count)]
    first_tick = unpacked[0][1]
    records: list[ImuRecord] = []
    for index, values in enumerate(unpacked):
        sequence, tick_ms, device_id = values[:3]
        accel = values[3:6]
        gyro = values[6:9]
        mag = values[9:12]
        angle = values[12:15]
        battery_raw = values[15]
        gap = 0
        if index:
            step = uint32_delta(sequence, unpacked[index - 1][0])
            if 1 < step < 0x80000000:
                gap = step - 1
        accel_g = [value * header.accel_full_scale_g / 32768.0
                   for value in accel]
        gyro_dps = [value * header.gyro_full_scale_dps / 32768.0
                    for value in gyro]
        angle_deg = [value * header.angle_full_scale_deg / 32768.0
                     for value in angle]
        elapsed_ms = uint32_delta(tick_ms, first_tick)
        records.append(ImuRecord(
            index, sequence, elapsed_ms / 1000.0, tick_ms, elapsed_ms, device_id,
            *accel, *accel_g, *gyro, *gyro_dps, *mag, *angle, *angle_deg,
            battery_raw, battery_raw / header.battery_divisor, gap))
    return header, records, trailing


def process_records(records: Sequence[Record], sample_rate_hz: int,
                    enabled: bool) -> list[ProcessedRecord]:
    if not enabled:
        return [ProcessedRecord(record, record.ch1_raw_uV, record.ch2_raw_uV)
                for record in records]
    if sample_rate_hz != 2000:
        raise ValueError("upper-PC-compatible post-processing currently requires "
                         "a 2000 Hz file (or use --no-filter)")
    ch1_filter, ch2_filter = Emg2000Filter(), Emg2000Filter()
    return [ProcessedRecord(record,
                            ch1_filter.process_sample(record.ch1_raw_uV),
                            ch2_filter.process_sample(record.ch2_raw_uV))
            for record in records]


def trim_records(records: Sequence[ProcessedRecord], sample_rate_hz: int,
                 start_seconds: float, end_seconds: float
                 ) -> tuple[list[ProcessedRecord], int, int]:
    """Crop independent durations from the start and end after filtering."""
    for option, value in (("--trim-start-seconds", start_seconds),
                          ("--trim-end-seconds", end_seconds)):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{option} must be a finite value >= 0")
    start_samples = int(round(start_seconds * sample_rate_hz))
    end_samples = int(round(end_seconds * sample_rate_hz))
    if start_samples + end_samples >= len(records):
        available_s = len(records) / sample_rate_hz
        raise ValueError(
            f"cannot trim {start_seconds:g} s from the start and "
            f"{end_seconds:g} s from the end of a {available_s:.6f} s file")
    end_index = len(records) - end_samples if end_samples else len(records)
    return list(records[start_samples:end_index]), start_samples, end_samples


def signal_stats(values: Sequence[float]) -> dict[str, float]:
    finite = [value for value in values if math.isfinite(value)]
    mean = sum(finite) / len(finite)
    std = math.sqrt(sum((value - mean) ** 2 for value in finite) / len(finite))
    return {"min_uV": min(finite), "max_uV": max(finite),
            "mean_uV": mean, "std_uV": std}


def build_result(path: Path, header: FileHeader,
                 source_records: Sequence[Record],
                 processed: Sequence[ProcessedRecord], trailing: int,
                 filter_enabled: bool, trim_start_seconds: float,
                 trim_end_seconds: float, trim_start_samples: int,
                 trim_end_samples: int) -> ParseResult:
    records = [item.raw for item in processed]
    discontinuities = sum(
        uint32_delta(source_records[index].sequence,
                     source_records[index - 1].sequence) != 1
        for index in range(1, len(source_records)))
    statistics = {
        "ch1_raw": signal_stats([record.ch1_raw_uV for record in records]),
        "ch2_raw": signal_stats([record.ch2_raw_uV for record in records]),
        "ch1_filtered": signal_stats([record.ch1_filtered_uV for record in processed]),
        "ch2_filtered": signal_stats([record.ch2_filtered_uV for record in processed]),
    }
    processing: dict[str, object] = {
        "enabled": filter_enabled,
        "implementation": "vendor upper-PC compatible; causal Direct Form I",
        "initial_conditions": "zero; startup transient retained",
        "chain": (["18 Hz sixth-order high-pass", "50 Hz order-40 comb notch"]
                  if filter_enabled else ["bypass"]),
        "trim": {
            "requested_start_seconds": trim_start_seconds,
            "requested_end_seconds": trim_end_seconds,
            "start_samples": trim_start_samples,
            "end_samples": trim_end_samples,
            "actual_start_seconds": trim_start_samples / header.sample_rate_hz,
            "actual_end_seconds": trim_end_samples / header.sample_rate_hz,
            "applied_after_filtering": True,
        },
        "binary_recovery": {
            "alignment_corrections": sum(
                record.byte_offset_adjustment != 0 for record in source_records),
            "imputed_records": sum(
                bool(record.imputed_fields) for record in source_records),
            "events": [
                {
                    "record_index": record.record_index,
                    "byte_offset_adjustment": record.byte_offset_adjustment,
                    "imputed_fields": record.imputed_fields,
                }
                for record in source_records
                if record.byte_offset_adjustment or record.imputed_fields
            ],
        },
    }
    source_duration = source_records[-1].time_s
    output_duration = records[-1].time_s - records[0].time_s
    return ParseResult(
        path.name, header, len(source_records), len(records),
        records[0].sequence, records[-1].sequence,
        sum(record.sequence_gap_before for record in source_records),
        discontinuities, trailing, source_duration, output_duration,
        trim_start_samples, trim_end_samples, processing, statistics)


def write_csv(path: Path, records: Sequence[ProcessedRecord]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["record_index", "sequence", "time_s", "tick_ms",
                         "tick_elapsed_ms", "ch1_raw_uV", "ch2_raw_uV",
                         "ch1_filtered_uV", "ch2_filtered_uV",
                         "sequence_gap_before", "byte_offset_adjustment",
                         "imputed_fields"])
        for item in records:
            raw = item.raw
            writer.writerow([raw.record_index, raw.sequence, f"{raw.time_s:.7f}",
                             raw.tick_ms, raw.tick_elapsed_ms,
                             f"{raw.ch1_raw_uV:.9g}", f"{raw.ch2_raw_uV:.9g}",
                             f"{item.ch1_filtered_uV:.12g}",
                             f"{item.ch2_filtered_uV:.12g}",
                             raw.sequence_gap_before,
                             raw.byte_offset_adjustment, raw.imputed_fields])


def write_imu_csv(path: Path, records: Sequence[ImuRecord]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(asdict(records[0]))
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(asdict(record) for record in records)


def basic_stats(values: Sequence[float]) -> dict[str, float]:
    finite = [value for value in values if math.isfinite(value)]
    mean = sum(finite) / len(finite)
    std = math.sqrt(sum((value - mean) ** 2 for value in finite) / len(finite))
    return {"min": min(finite), "max": max(finite), "mean": mean, "std": std}


def build_imu_result(path: Path, header: ImuFileHeader,
                     records: Sequence[ImuRecord], trailing: int) -> dict[str, object]:
    device_counts: dict[str, int] = {}
    for record in records:
        key = str(record.device_id)
        device_counts[key] = device_counts.get(key, 0) + 1
    stat_fields = (
        "accel_x_g", "accel_y_g", "accel_z_g",
        "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
        "mag_x_raw", "mag_y_raw", "mag_z_raw",
        "angle_x_deg", "angle_y_deg", "angle_z_deg", "battery_voltage")
    discontinuities = sum(
        uint32_delta(records[index].sequence, records[index - 1].sequence) != 1
        for index in range(1, len(records)))
    return {
        "source": path.name,
        "data_type": "IMU",
        "header": asdict(header),
        "record_count": len(records),
        "first_sequence": records[0].sequence,
        "last_sequence": records[-1].sequence,
        "missing_samples": sum(record.sequence_gap_before for record in records),
        "discontinuities": discontinuities,
        "trailing_bytes": trailing,
        "duration_s": records[-1].time_s,
        "device_ids": sorted(int(key) for key in device_counts),
        "device_record_counts": device_counts,
        "statistics": {
            name: basic_stats([float(getattr(record, name)) for record in records])
            for name in stat_fields},
    }


def write_metadata(path: Path, result: object) -> None:
    payload = result if isinstance(result, dict) else asdict(result)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                    encoding="utf-8")


def finite_range(values: Sequence[float]) -> tuple[float, float]:
    finite = [value for value in values if math.isfinite(value)]
    low, high = min(finite), max(finite)
    margin = max(1.0, abs(low) * 0.02) if low == high else (high - low) * 0.05
    return low - margin, high + margin


def write_plot(path: Path, records: Sequence[ProcessedRecord],
               result: ParseResult, dpi: int) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("PNG output requires matplotlib") from exc
    step = max(1, math.ceil(len(records) / 250_000))
    plotted = records[::step]
    time_s = [item.raw.time_s for item in plotted]
    series = ([item.raw.ch1_raw_uV for item in plotted],
              [item.ch1_filtered_uV for item in plotted],
              [item.raw.ch2_raw_uV for item in plotted],
              [item.ch2_filtered_uV for item in plotted])
    labels = ("CH1 raw (uV)", "CH1 filtered (uV)",
              "CH2 raw (uV)", "CH2 filtered (uV)")
    colors = ("#78909c", "#1565c0", "#a1887f", "#c62828")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True,
                             constrained_layout=True)
    for axis, values, label, color in zip(axes, series, labels, colors):
        axis.plot(time_s, values, color=color, linewidth=0.65)
        axis.set_ylabel(label)
        axis.set_ylim(*finite_range(values))
        axis.grid(True, color="#c7c7c7", alpha=0.45, linewidth=0.5)
    axes[0].set_title(f"{result.source} | {result.header.sample_rate_hz} Hz | "
                      f"PGA {result.header.pga} | {result.record_count} samples | "
                      "18 Hz HP + 50 Hz notch")
    axes[-1].set_xlabel("Time from first stored sample (s)")
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def write_imu_plot(path: Path, records: Sequence[ImuRecord],
                   source_name: str, dpi: int) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("PNG output requires matplotlib") from exc
    step = max(1, math.ceil(len(records) / 250_000))
    plotted = records[::step]
    devices = sorted({record.device_id for record in plotted})
    groups = (
        (("accel_x_g", "accel_y_g", "accel_z_g"), "Acceleration (g)"),
        (("gyro_x_dps", "gyro_y_dps", "gyro_z_dps"), "Angular rate (deg/s)"),
        (("mag_x_raw", "mag_y_raw", "mag_z_raw"), "Magnetic field (raw)"),
        (("angle_x_deg", "angle_y_deg", "angle_z_deg"), "Angle (deg)"),
        (("battery_voltage",), "Battery (V)"),
    )
    colors = ("#1565c0", "#c62828", "#2e7d32")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True,
                             constrained_layout=True)
    for axis, (fields, label) in zip(axes, groups):
        all_values: list[float] = []
        for device_id in devices:
            device_records = [record for record in plotted
                              if record.device_id == device_id]
            time_s = [record.time_s for record in device_records]
            for field_index, field in enumerate(fields):
                values = [float(getattr(record, field))
                          for record in device_records]
                all_values.extend(values)
                suffix = ("xyz"[field_index] if len(fields) == 3 else "")
                axis.plot(time_s, values, color=colors[field_index],
                          linewidth=0.65, alpha=0.8,
                          label=f"ID {device_id} {suffix}".rstrip())
        axis.set_ylabel(label)
        axis.set_ylim(*finite_range(all_values))
        axis.grid(True, color="#c7c7c7", alpha=0.45, linewidth=0.5)
        axis.legend(loc="upper right", ncol=min(6, max(1, len(devices) * len(fields))),
                    fontsize=7)
    axes[0].set_title(f"{source_name} | {len(records)} IMU records")
    axes[-1].set_xlabel("Time from first stored IMU record (s)")
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


ALIGN_IMU_FIELDS = (
    "accel_x_g", "accel_y_g", "accel_z_g",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "mag_x_raw", "mag_y_raw", "mag_z_raw",
    "angle_x_deg", "angle_y_deg", "angle_z_deg", "battery_voltage")


def pair_key(source: Path) -> tuple[Path, str] | None:
    stem = source.stem.upper()
    if not (stem.startswith("EMG") or stem.startswith("IMU")):
        return None
    return source.parent.resolve(), stem[3:]


def add_single_file_partner(files: list[Path], source: Path) -> None:
    key = pair_key(source)
    if key is None:
        return
    wanted_prefix = "IMU" if source.stem.upper().startswith("EMG") else "EMG"
    wanted_stem = wanted_prefix + key[1]
    for candidate in source.parent.iterdir():
        if (candidate.is_file() and candidate.suffix.lower() == ".bin"
                and candidate.stem.upper() == wanted_stem):
            if candidate not in files:
                files.append(candidate)
            return


def emg_common_times(header: FileHeader, source_records: Sequence[Record],
                     records: Sequence[ProcessedRecord]) -> tuple[list[float], str]:
    continuous = all(
        uint32_delta(source_records[index].sequence,
                     source_records[index - 1].sequence) == 1
        for index in range(1, len(source_records)))
    origin_s = uint32_delta(
        source_records[0].tick_ms, header.start_tick_ms) / 1000.0
    if continuous:
        first_sequence = source_records[0].sequence
        return [
            origin_s + uint32_delta(item.raw.sequence, first_sequence)
            / header.sample_rate_hz
            for item in records
        ], "sequence/sample_rate"
    return [
        origin_s + item.raw.record_index / header.sample_rate_hz
        for item in records
    ], "record_index/sample_rate (sequence discontinuities detected)"


def prepare_imu_groups(header: ImuFileHeader,
                       records: Sequence[ImuRecord]
                       ) -> dict[int, tuple[list[float], list[ImuRecord]]]:
    grouped: dict[int, list[tuple[float, ImuRecord]]] = {}
    for record in records:
        time_s = uint32_delta(record.tick_ms, header.start_tick_ms) / 1000.0
        grouped.setdefault(record.device_id, []).append((time_s, record))
    result: dict[int, tuple[list[float], list[ImuRecord]]] = {}
    for device_id, values in grouped.items():
        values.sort(key=lambda item: item[0])
        times: list[float] = []
        samples: list[ImuRecord] = []
        for time_s, record in values:
            if times and time_s == times[-1]:
                samples[-1] = record
            else:
                times.append(time_s)
                samples.append(record)
        result[device_id] = times, samples
    return result


def interpolate_imu(times: Sequence[float], records: Sequence[ImuRecord],
                    query_s: float) -> tuple[dict[str, float], float]:
    right = bisect.bisect_left(times, query_s)
    if right < len(times) and times[right] == query_s:
        record = records[right]
        return ({field: float(getattr(record, field))
                 for field in ALIGN_IMU_FIELDS}, 0.0)
    if right == 0 or right == len(times):
        raise ValueError("alignment query is outside an IMU device time range")
    left = right - 1
    span = times[right] - times[left]
    ratio = 0.0 if span == 0.0 else (query_s - times[left]) / span
    values: dict[str, float] = {}
    for field in ALIGN_IMU_FIELDS:
        first = float(getattr(records[left], field))
        second = float(getattr(records[right], field))
        if field.startswith("angle_"):
            delta = (second - first + 180.0) % 360.0 - 180.0
            value = (first + ratio * delta + 180.0) % 360.0 - 180.0
        else:
            value = first + ratio * (second - first)
        values[field] = value
    nearest_ms = min(query_s - times[left], times[right] - query_s) * 1000.0
    return values, nearest_ms


def align_datasets(emg_source: Path, emg_header: FileHeader,
                   source_records: Sequence[Record],
                   emg_records: Sequence[ProcessedRecord],
                   imu_source: Path, imu_header: ImuFileHeader,
                   imu_records: Sequence[ImuRecord]
                   ) -> tuple[list[dict[str, object]], dict[str, object]]:
    if emg_header.start_tick_ms != imu_header.start_tick_ms:
        raise ValueError(
            f"{emg_source.name}/{imu_source.name}: start_tick_ms differs "
            f"({emg_header.start_tick_ms} vs {imu_header.start_tick_ms})")
    emg_times, time_basis = emg_common_times(
        emg_header, source_records, emg_records)
    groups = prepare_imu_groups(imu_header, imu_records)
    if not groups:
        raise ValueError(f"{imu_source.name}: no IMU devices to align")
    overlap_start = max([emg_times[0]]
                        + [times[0] for times, _ in groups.values()])
    overlap_end = min([emg_times[-1]]
                      + [times[-1] for times, _ in groups.values()])
    if overlap_end < overlap_start:
        raise ValueError(
            f"{emg_source.name}/{imu_source.name}: no common time range")
    device_ids = sorted(groups)
    offsets: dict[int, list[float]] = {device_id: [] for device_id in device_ids}
    rows: list[dict[str, object]] = []
    for time_s, item in zip(emg_times, emg_records):
        if not overlap_start <= time_s <= overlap_end:
            continue
        raw = item.raw
        row: dict[str, object] = {
            "time_s": time_s,
            "emg_record_index": raw.record_index,
            "emg_sequence": raw.sequence,
            "emg_tick_ms": raw.tick_ms,
            "ch1_raw_uV": raw.ch1_raw_uV,
            "ch2_raw_uV": raw.ch2_raw_uV,
            "ch1_filtered_uV": item.ch1_filtered_uV,
            "ch2_filtered_uV": item.ch2_filtered_uV,
        }
        for device_id in device_ids:
            values, offset_ms = interpolate_imu(
                groups[device_id][0], groups[device_id][1], time_s)
            prefix = f"imu{device_id}_"
            row[prefix + "source_offset_ms"] = offset_ms
            offsets[device_id].append(offset_ms)
            for field, value in values.items():
                row[prefix + field] = value
        rows.append(row)
    if not rows:
        raise ValueError("common time range contains no EMG samples")
    metadata: dict[str, object] = {
        "emg_source": emg_source.name,
        "imu_source": imu_source.name,
        "start_tick_ms": emg_header.start_tick_ms,
        "emg_sample_rate_hz": emg_header.sample_rate_hz,
        "device_ids": device_ids,
        "record_count": len(rows),
        "overlap_start_s": rows[0]["time_s"],
        "overlap_end_s": rows[-1]["time_s"],
        "overlap_duration_s": float(rows[-1]["time_s"])
                              - float(rows[0]["time_s"]),
        "emg_time_basis": time_basis,
        "emg_sequence_continuous": "discontinuities" not in time_basis,
        "warnings": ([] if "discontinuities" not in time_basis else [
            "EMG sequence discontinuities were detected; record_index was used "
            "for a monotonic alignment timeline, but affected signal values "
            "should be treated as potentially corrupted."]),
        "imu_alignment": "per-device linear interpolation onto the EMG timeline",
        "angle_interpolation": "shortest path across the +/-180 degree boundary",
        "outside_common_range": "omitted",
        "nearest_imu_source_offset_ms": {
            str(device_id): {
                "max": max(offsets[device_id]),
                "mean": sum(offsets[device_id]) / len(offsets[device_id]),
            }
            for device_id in device_ids
        },
    }
    return rows, metadata


def write_aligned_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def unwrap_degrees(values: Sequence[float]) -> list[float]:
    """Remove +/-180 degree wrap jumps while preserving true increments."""
    if not values:
        return []
    unwrapped = [float(values[0])]
    offset = 0.0
    previous = float(values[0])
    for value_raw in values[1:]:
        value = float(value_raw)
        delta = value - previous
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        unwrapped.append(value + offset)
        previous = value
    return unwrapped


def write_aligned_plot(path: Path, rows: Sequence[dict[str, object]],
                       device_ids: Sequence[int], title: str, dpi: int) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("PNG output requires matplotlib") from exc
    step = max(1, math.ceil(len(rows) / 250_000))
    plotted = rows[::step]
    time_s = [float(row["time_s"]) for row in plotted]
    path.parent.mkdir(parents=True, exist_ok=True)
    panel_count = 2 + len(device_ids)
    fig, axes = plt.subplots(
        panel_count, 1, figsize=(14, max(9, panel_count * 2.2)), sharex=True,
        constrained_layout=True)
    emg_series = (
        [float(row["ch1_filtered_uV"]) for row in plotted],
        [float(row["ch2_filtered_uV"]) for row in plotted])
    for axis, values, label, color in zip(
            axes[:2], emg_series,
            ("sEMG CH1 filtered (uV)", "sEMG CH2 filtered (uV)"),
            ("#1565c0", "#c62828")):
        axis.plot(time_s, values, color=color, linewidth=0.65)
        axis.set_ylabel(label)
        axis.set_ylim(*finite_range(values))
    angle_colors = ("#1565c0", "#c62828", "#2e7d32")
    for panel, device_id in zip(axes[2:], device_ids):
        prefix = f"imu{device_id}_"
        combined: list[float] = []
        for axis_name, color in zip("xyz", angle_colors):
            values = unwrap_degrees(
                [float(row[prefix + f"angle_{axis_name}_deg"])
                 for row in plotted])
            combined.extend(values)
            panel.plot(time_s, values, color=color, linewidth=0.7,
                       label=axis_name.upper())
        panel.set_ylabel(f"IMU {device_id}\nAngle (deg)")
        panel.set_ylim(*finite_range(combined))
        panel.legend(loc="upper right", ncol=3, fontsize=7)
    for axis in axes:
        axis.grid(True, color="#c7c7c7", alpha=0.45, linewidth=0.5)
    axes[0].set_title(title)
    axes[-1].set_xlabel("Common time from recording start (s)")
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def discover_files(source: Path, recursive: bool) -> Iterator[Path]:
    if source.is_file():
        yield source
    elif source.is_dir():
        candidates = source.rglob("*") if recursive else source.iterdir()
        yield from sorted(path for path in candidates
                          if path.is_file() and path.suffix.lower() == ".bin")
    else:
        raise FileNotFoundError(f"input path does not exist: {source}")


def detect_file_type(source: Path) -> str:
    with source.open("rb") as stream:
        magic = stream.read(8).rstrip(b"\0")
    if magic == MAGIC:
        return "EMG"
    if magic == IMU_MAGIC:
        return "IMU"
    raise ValueError(
        f"{source.name}: unsupported magic {magic!r}; expected EMG2K01 or IMU9011")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Parse EMG2K01 and IMU9011 BIN files next to their source")
    parser.add_argument("input", type=Path, nargs="?", default=DEFAULT_INPUT)
    parser.add_argument(
        "-o", "--output", type=Path,
        help="output directory (default: each BIN file's own directory)")
    parser.add_argument("--recursive", action="store_true")
    parser.add_argument("--no-filter", action="store_true")
    parser.add_argument(
        "--trim-start-seconds", type=float, metavar="SECONDS",
        help="crop this duration from the start after filtering")
    parser.add_argument(
        "--trim-end-seconds", type=float, metavar="SECONDS",
        help="crop this duration from the end after filtering")
    parser.add_argument(
        "--trim-seconds", type=float, metavar="SECONDS",
        help="deprecated compatibility option: crop both ends equally; "
             "cannot be combined with the two independent trim options")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--dpi", type=int, default=160)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if (args.trim_seconds is not None
            and (args.trim_start_seconds is not None
                 or args.trim_end_seconds is not None)):
        parser.error("--trim-seconds cannot be combined with "
                     "--trim-start-seconds or --trim-end-seconds")
    if args.trim_seconds is not None:
        trim_start_seconds = trim_end_seconds = args.trim_seconds
    else:
        trim_start_seconds = args.trim_start_seconds or 0.0
        trim_end_seconds = args.trim_end_seconds or 0.0
    try:
        files = list(discover_files(args.input, args.recursive))
        if args.input.is_file():
            add_single_file_partner(files, args.input)
            files.sort()
    except OSError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    if not files:
        print(f"Error: no BIN files found under {args.input}", file=sys.stderr)
        return 1
    failures = 0
    emg_sets: dict[
        tuple[Path, str],
        tuple[Path, FileHeader, list[Record], list[ProcessedRecord]]] = {}
    imu_sets: dict[
        tuple[Path, str],
        tuple[Path, ImuFileHeader, list[ImuRecord]]] = {}
    for source in files:
        try:
            output = args.output or source.parent
            file_type = detect_file_type(source)
            if file_type == "EMG":
                header, raw_records, trailing = parse_file(source)
                all_processed = process_records(
                    raw_records, header.sample_rate_hz, not args.no_filter)
                processed, trim_start_samples, trim_end_samples = trim_records(
                    all_processed, header.sample_rate_hz,
                    trim_start_seconds, trim_end_seconds)
                result = build_result(
                    source, header, raw_records, processed, trailing,
                    not args.no_filter, trim_start_seconds, trim_end_seconds,
                    trim_start_samples, trim_end_samples)
                write_csv(output / f"{source.stem}.csv", processed)
                write_metadata(output / f"{source.stem}.json", result)
                if not args.no_plot:
                    write_plot(
                        output / f"{source.stem}.png", processed, result, args.dpi)
                print(
                    f"OK {source.name} [EMG]: {result.source_record_count} source "
                    f"samples, {result.record_count} output samples, "
                    f"{header.sample_rate_hz} Hz, {result.duration_s:.6f} s, "
                    f"trim={trim_start_samples}/{trim_end_samples} samples, "
                    f"gaps={result.discontinuities}, "
                    f"missing={result.missing_samples}")
                key = pair_key(source)
                if key is not None:
                    emg_sets[key] = (source, header, raw_records, processed)
            else:
                imu_header, imu_records, trailing = parse_imu_file(source)
                imu_result = build_imu_result(
                    source, imu_header, imu_records, trailing)
                write_imu_csv(output / f"{source.stem}.csv", imu_records)
                write_metadata(output / f"{source.stem}.json", imu_result)
                if not args.no_plot:
                    write_imu_plot(output / f"{source.stem}.png", imu_records,
                                   source.name, args.dpi)
                print(
                    f"OK {source.name} [IMU]: {len(imu_records)} records, "
                    f"{imu_result['duration_s']:.3f} s, "
                    f"devices={imu_result['device_ids']}, "
                    f"gaps={imu_result['discontinuities']}, "
                    f"missing={imu_result['missing_samples']}")
                key = pair_key(source)
                if key is not None:
                    imu_sets[key] = (source, imu_header, imu_records)
        except (OSError, ValueError, RuntimeError, struct.error) as exc:
            failures += 1
            print(f"FAILED {source}: {exc}", file=sys.stderr)
    for key in sorted(set(emg_sets) & set(imu_sets),
                      key=lambda item: (str(item[0]), item[1])):
        try:
            emg_source, emg_header, raw_records, emg_records = emg_sets[key]
            imu_source, imu_header, imu_records = imu_sets[key]
            rows, alignment = align_datasets(
                emg_source, emg_header, raw_records, emg_records,
                imu_source, imu_header, imu_records)
            output = args.output or emg_source.parent
            aligned_stem = "ALIGNED" + key[1]
            write_aligned_csv(output / f"{aligned_stem}.csv", rows)
            write_metadata(output / f"{aligned_stem}.json", alignment)
            if not args.no_plot:
                write_aligned_plot(
                    output / f"{aligned_stem}.png", rows,
                    alignment["device_ids"],
                    f"{emg_source.name} + {imu_source.name}", args.dpi)
            print(
                f"OK {aligned_stem} [ALIGNED]: {len(rows)} EMG-rate rows, "
                f"{alignment['overlap_duration_s']:.6f} s, "
                f"devices={alignment['device_ids']}")
        except (OSError, ValueError, RuntimeError) as exc:
            failures += 1
            print(f"FAILED alignment {key[1]}: {exc}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
