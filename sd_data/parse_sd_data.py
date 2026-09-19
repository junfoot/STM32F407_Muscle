#!/usr/bin/env python3
"""Parse STM32F407_Muscle LOGxxxx.BIN files.

The firmware stores a 32-byte header followed by an interleaved stream:

    ADC: 0xA1 + uint32 sample_seq + 16 x int16, 37 bytes
    IMU: 0xB1 + uint32 sample_seq + uint8 device_id + 13 x int16, 32 bytes

All fields are little-endian.  The script exports separate ADC and IMU CSV
files, one summary per BIN file, and overview PNG plots.
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

MAGIC = b"EMGL"
HEADER_SIZE = 32
ADC_TAG = 0xA1
IMU_TAG = 0xB1
ADC_RECORD = struct.Struct("<BI16h")
IMU_RECORD = struct.Struct("<BIB13h")
ADC_RATE_DEFAULT = 2000


@dataclass(frozen=True)
class AdcRecord:
    record_index: int
    byte_offset: int
    sample_seq: int
    raw: tuple[int, ...]


@dataclass(frozen=True)
class ImuRecord:
    record_index: int
    byte_offset: int
    sample_seq: int
    device_id: int
    raw: tuple[int, ...]


@dataclass
class ParsedLog:
    source: Path
    header: dict[str, int | str]
    adc: list[AdcRecord]
    imu: list[ImuRecord]
    record_count: int
    trailing_bytes: int
    unknown_tags: int
    warnings: list[str]
    first_sequence: int
    file_bytes: int


def u32_delta(value: int, origin: int) -> int:
    """Unsigned uint32 difference, including sequence-counter wraparound."""

    return (value - origin) & 0xFFFFFFFF


def discover_files(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    if source.is_dir():
        return sorted(
            path for path in source.iterdir()
            if path.is_file() and path.suffix.lower() == ".bin"
        )
    raise FileNotFoundError(f"input path does not exist: {source}")


def find_next_record_marker(data: bytes, start: int) -> int | None:
    """Find a plausible later record marker after a corrupted byte."""

    for offset in range(start, len(data)):
        tag = data[offset]
        if tag == ADC_TAG and offset + ADC_RECORD.size <= len(data):
            return offset
        if tag == IMU_TAG and offset + IMU_RECORD.size <= len(data):
            return offset
    return None


def parse_file(path: Path) -> ParsedLog:
    data = path.read_bytes()
    if len(data) < HEADER_SIZE:
        raise ValueError(f"{path.name}: file is shorter than the 32-byte header")

    magic, format_version, adc_channels, adc_rate_hz, imu_count = struct.unpack_from(
        "<4sHHIB", data, 0
    )
    if magic != MAGIC:
        raise ValueError(f"{path.name}: invalid magic {magic!r}, expected {MAGIC!r}")
    if format_version != 1:
        raise ValueError(f"{path.name}: unsupported format version {format_version}")
    if adc_channels != 16:
        raise ValueError(f"{path.name}: expected 16 ADC channels, got {adc_channels}")
    if adc_rate_hz == 0:
        raise ValueError(f"{path.name}: ADC sample rate is zero")

    header: dict[str, int | str] = {
        "magic": magic.decode("ascii"),
        "format_version": format_version,
        "adc_channels": adc_channels,
        "adc_rate_hz": adc_rate_hz,
        "imu_count": imu_count,
        "header_size": HEADER_SIZE,
    }
    adc_records: list[AdcRecord] = []
    imu_records: list[ImuRecord] = []
    warnings: list[str] = []
    position = HEADER_SIZE
    record_index = 0
    first_sequence: int | None = None
    trailing_bytes = 0
    unknown_tags = 0

    while position < len(data):
        tag = data[position]
        if tag == ADC_TAG:
            if position + ADC_RECORD.size > len(data):
                trailing_bytes = len(data) - position
                warnings.append(
                    f"truncated ADC record at byte {position} ({trailing_bytes} bytes remain)"
                )
                break
            _, sequence, *raw = ADC_RECORD.unpack_from(data, position)
            if first_sequence is None:
                first_sequence = sequence
            adc_records.append(
                AdcRecord(record_index, position, sequence, tuple(raw))
            )
            position += ADC_RECORD.size
            record_index += 1
            continue

        if tag == IMU_TAG:
            if position + IMU_RECORD.size > len(data):
                trailing_bytes = len(data) - position
                warnings.append(
                    f"truncated IMU record at byte {position} ({trailing_bytes} bytes remain)"
                )
                break
            _, sequence, device_id, *raw = IMU_RECORD.unpack_from(data, position)
            if first_sequence is None:
                first_sequence = sequence
            imu_records.append(
                ImuRecord(record_index, position, sequence, device_id, tuple(raw))
            )
            position += IMU_RECORD.size
            record_index += 1
            continue

        unknown_tags += 1
        next_position = find_next_record_marker(data, position + 1)
        if next_position is None:
            trailing_bytes = len(data) - position
            warnings.append(
                f"unknown record tag 0x{tag:02X} at byte {position}; no later marker"
            )
            break
        skipped = next_position - position
        warnings.append(
            f"unknown record tag 0x{tag:02X} at byte {position}; "
            f"resynchronized after {skipped} bytes"
        )
        position = next_position

    if not adc_records and not imu_records:
        raise ValueError(f"{path.name}: no complete ADC or IMU records found")
    assert first_sequence is not None

    return ParsedLog(
        source=path,
        header=header,
        adc=adc_records,
        imu=imu_records,
        record_count=record_index,
        trailing_bytes=trailing_bytes,
        unknown_tags=unknown_tags,
        warnings=warnings,
        first_sequence=first_sequence,
        file_bytes=len(data),
    )


def sequence_metrics(records: Sequence[AdcRecord | ImuRecord]) -> dict[str, int | None]:
    discontinuities = 0
    missing_forward = 0
    backward = 0
    for previous, current in zip(records, records[1:]):
        step = u32_delta(current.sample_seq, previous.sample_seq)
        if step != 1:
            discontinuities += 1
            if step < 0x80000000:
                missing_forward += max(step - 1, 0)
            else:
                backward += 1
    return {
        "record_count": len(records),
        "first_sequence": records[0].sample_seq if records else None,
        "last_sequence": records[-1].sample_seq if records else None,
        "discontinuities": discontinuities,
        "missing_forward_samples": missing_forward,
        "backward_steps": backward,
    }


def time_s(sequence: int, first_sequence: int, sample_rate_hz: int) -> float:
    return u32_delta(sequence, first_sequence) / float(sample_rate_hz)


def adc_rows(parsed: ParsedLog) -> list[dict[str, object]]:
    rate = int(parsed.header["adc_rate_hz"])
    rows: list[dict[str, object]] = []
    for record in parsed.adc:
        row: dict[str, object] = {
            "record_index": record.record_index,
            "byte_offset": record.byte_offset,
            "sample_seq": record.sample_seq,
            "time_s": time_s(record.sample_seq, parsed.first_sequence, rate),
        }
        for channel, raw in enumerate(record.raw, start=1):
            row[f"adc_raw_ch{channel:02d}"] = raw
        for channel, raw in enumerate(record.raw, start=1):
            # Same conversion used by main.c: +/-5 V, 5 / 32768 V per code.
            row[f"adc_volts_ch{channel:02d}"] = raw * 5.0 / 32768.0
        rows.append(row)
    return rows


def imu_rows(parsed: ParsedLog) -> list[dict[str, object]]:
    rate = int(parsed.header["adc_rate_hz"])
    rows: list[dict[str, object]] = []
    for record in parsed.imu:
        accel = record.raw[0:3]
        gyro = record.raw[3:6]
        mag = record.raw[6:9]
        angle = record.raw[9:12]
        battery = record.raw[12]
        row: dict[str, object] = {
            "record_index": record.record_index,
            "byte_offset": record.byte_offset,
            "sample_seq": record.sample_seq,
            "time_s": time_s(record.sample_seq, parsed.first_sequence, rate),
            "device_id": record.device_id,
        }
        for axis, raw in zip("xyz", accel):
            row[f"accel_{axis}_raw"] = raw
            row[f"accel_{axis}_g"] = raw / 32768.0 * 16.0
        for axis, raw in zip("xyz", gyro):
            row[f"gyro_{axis}_raw"] = raw
            row[f"gyro_{axis}_dps"] = raw / 32768.0 * 2000.0
        for axis, raw in zip("xyz", mag):
            row[f"mag_{axis}_raw"] = raw
        for axis, raw in zip("xyz", angle):
            row[f"angle_{axis}_raw"] = raw
            row[f"angle_{axis}_deg"] = raw / 32768.0 * 180.0
        row["battery_raw"] = battery
        row["battery_voltage"] = battery / 100.0
        rows.append(row)
    return rows


def per_device_backward_steps(records: Sequence[ImuRecord]) -> int:
    by_device: dict[int, list[ImuRecord]] = defaultdict(list)
    for record in records:
        by_device[record.device_id].append(record)
    count = 0
    for device_records in by_device.values():
        for previous, current in zip(device_records, device_records[1:]):
            if u32_delta(current.sample_seq, previous.sample_seq) >= 0x80000000:
                count += 1
    return count


def summary(parsed: ParsedLog) -> dict[str, object]:
    adc_metric = sequence_metrics(parsed.adc)
    imu_metric = sequence_metrics(parsed.imu)
    device_counts = Counter(record.device_id for record in parsed.imu)
    all_records = parsed.adc + parsed.imu
    last_record = max(all_records, key=lambda record: record.record_index)
    last_sequence = last_record.sample_seq
    status = "PASS" if not parsed.warnings else "PASS_WITH_WARNINGS"
    return {
        "source_file": parsed.source.name,
        "file_bytes": parsed.file_bytes,
        "header_magic": parsed.header["magic"],
        "format_version": parsed.header["format_version"],
        "adc_channels": parsed.header["adc_channels"],
        "adc_rate_hz": parsed.header["adc_rate_hz"],
        "imu_count_in_header": parsed.header["imu_count"],
        "adc_record_count": len(parsed.adc),
        "imu_record_count": len(parsed.imu),
        "total_record_count": parsed.record_count,
        "first_sample_seq": parsed.first_sequence,
        "last_sample_seq_in_file_order": last_sequence,
        "duration_s_from_sequence": time_s(
            last_sequence, parsed.first_sequence, int(parsed.header["adc_rate_hz"])
        ),
        "adc_sequence_discontinuities": adc_metric["discontinuities"],
        "adc_missing_forward_samples": adc_metric["missing_forward_samples"],
        "adc_backward_steps": adc_metric["backward_steps"],
        "imu_file_order_discontinuities": imu_metric["discontinuities"],
        "imu_backward_steps_per_device": per_device_backward_steps(parsed.imu),
        "imu_device_ids": ";".join(str(device_id) for device_id in sorted(device_counts)),
        "imu_records_by_device": ";".join(
            f"{device_id}:{device_counts[device_id]}"
            for device_id in sorted(device_counts)
        ),
        "trailing_bytes": parsed.trailing_bytes,
        "unknown_record_tags": parsed.unknown_tags,
        "validation_status": status,
        "warnings": " | ".join(parsed.warnings),
    }


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, values: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(values))
        writer.writeheader()
        writer.writerow(values)


def downsample(values: Sequence[object], max_points: int) -> list[object]:
    step = max(1, (len(values) + max_points - 1) // max_points)
    return list(values[::step])


def write_adc_plot(path: Path, rows: Sequence[dict[str, object]], title: str, max_points: int) -> None:
    if not rows:
        return
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plotted = downsample(rows, max_points)
    times = [float(row["time_s"]) for row in plotted]
    figure, axes = plt.subplots(4, 4, figsize=(18, 12), sharex=True)
    axes_flat = list(axes.flat)
    for channel, axis in enumerate(axes_flat, start=1):
        values = [float(row[f"adc_volts_ch{channel:02d}"]) for row in plotted]
        axis.plot(times, values, linewidth=0.65, color="#1565c0")
        axis.set_title(f"ADC CH{channel:02d}", fontsize=9)
        axis.set_ylabel("V", fontsize=8)
        axis.grid(True, alpha=0.3, linewidth=0.5)
        axis.tick_params(labelsize=7)
    for axis in axes_flat[-4:]:
        axis.set_xlabel("time (s)", fontsize=8)
    figure.suptitle(title, fontsize=14)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def write_imu_plot(path: Path, rows: Sequence[dict[str, object]], title: str, max_points: int) -> None:
    if not rows:
        return
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    devices = sorted({int(row["device_id"]) for row in rows})
    colors = {device: plt.cm.tab10(index % 10) for index, device in enumerate(devices)}
    groups = [
        ("Angle (deg)", ("angle_x_deg", "angle_y_deg", "angle_z_deg")),
        ("Acceleration (g)", ("accel_x_g", "accel_y_g", "accel_z_g")),
        ("Angular rate (deg/s)", ("gyro_x_dps", "gyro_y_dps", "gyro_z_dps")),
        ("Battery (V)", ("battery_voltage",)),
    ]
    axis_labels = ("x", "y", "z")
    for axis, (label, fields) in zip(axes, groups):
        for device in devices:
            device_rows = [row for row in rows if int(row["device_id"]) == device]
            device_rows = downsample(device_rows, max_points)
            times = [float(row["time_s"]) for row in device_rows]
            for field_index, field in enumerate(fields):
                suffix = axis_labels[field_index] if len(fields) == 3 else ""
                axis.plot(
                    times,
                    [float(row[field]) for row in device_rows],
                    linewidth=0.7,
                    color=colors[device],
                    linestyle=("-", "--", ":")[field_index] if len(fields) == 3 else "-",
                    label=f"ID {device} {suffix}".rstrip(),
                )
        axis.set_ylabel(label, fontsize=9)
        axis.grid(True, alpha=0.3, linewidth=0.5)
        axis.legend(loc="upper right", fontsize=7, ncol=max(1, min(6, len(devices) * len(fields))))
    axes[-1].set_xlabel("time from first stored record (s)")
    figure.suptitle(title, fontsize=14)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def process_file(path: Path, output_dir: Path, no_plot: bool, max_plot_points: int) -> dict[str, object]:
    parsed = parse_file(path)
    adc = adc_rows(parsed)
    imu = imu_rows(parsed)
    file_summary = summary(parsed)
    stem = path.stem

    write_csv(output_dir / f"{stem}_adc.csv", adc)
    write_csv(output_dir / f"{stem}_imu.csv", imu)
    write_summary(output_dir / f"{stem}_summary.csv", file_summary)
    with (output_dir / f"{stem}_summary.json").open("w", encoding="utf-8") as stream:
        json.dump(file_summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    if not no_plot:
        write_adc_plot(
            output_dir / f"{stem}_adc_overview.png",
            adc,
            f"{path.name} - ADC overview",
            max_plot_points,
        )
        write_imu_plot(
            output_dir / f"{stem}_imu_overview.png",
            imu,
            f"{path.name} - IMU overview",
            max_plot_points,
        )
    return file_summary


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Parse STM32F407_Muscle EMGL LOGxxxx.BIN files into CSV and PNG"
    )
    parser.add_argument(
        "input",
        type=Path,
        nargs="?",
        default=Path(__file__).resolve().parent,
        help="BIN file or directory containing BIN files (default: sd_data)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="output directory (default: <input directory>/parsed)",
    )
    parser.add_argument("--no-plot", action="store_true", help="skip PNG generation")
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=1600,
        help="maximum plotted points per series (default: 1600)",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.max_plot_points < 100:
        parser.error("--max-plot-points must be at least 100")

    try:
        files = discover_files(args.input)
    except (FileNotFoundError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    if not files:
        print(f"ERROR: no .BIN files found under {args.input}", file=sys.stderr)
        return 1

    if args.output is not None:
        output_dir = args.output
    elif args.input.is_file():
        output_dir = args.input.parent / "parsed"
    else:
        output_dir = args.input / "parsed"
    output_dir.mkdir(parents=True, exist_ok=True)

    summaries: list[dict[str, object]] = []
    failures = 0
    for path in files:
        try:
            result = process_file(path, output_dir, args.no_plot, args.max_plot_points)
            summaries.append(result)
            print(
                f"OK {path.name}: ADC={result['adc_record_count']}, "
                f"IMU={result['imu_record_count']}, bytes={result['file_bytes']}, "
                f"status={result['validation_status']}"
            )
        except (OSError, ValueError, struct.error, RuntimeError) as exc:
            failures += 1
            print(f"FAILED {path.name}: {exc}", file=sys.stderr)

    if summaries:
        write_csv(output_dir / "all_logs_summary.csv", summaries)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
