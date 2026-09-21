#!/usr/bin/env python3
"""Parse the STM32F407_Muscle USART1 telemetry stream.

The firmware sends a mixed stream containing ASCII log/command responses and
VOFA+ JustFloat telemetry frames.  A telemetry frame is:

    59 little-endian float32 values + b"\\x00\\x00\\x80\\x7f"  (240 bytes)

Channel layout (from Core/Src/main.c):

    0       connection status: bit0=SD present, bit1=USB connected
    1..2    DAC A/B output voltage, V
    3..18   16 raw ADC channels, V
    19..34  16 filtered sEMG channels, V
    35..58  IMU 0..3, each roll/pitch/yaw (deg), ax/ay/az (g)

Examples:

    python parse_serial.py --port COM5
    python parse_serial.py --port COM5 --output data\\telemetry.csv --text-log data\\uart.log
    python parse_serial.py --input capture.bin --output telemetry.csv

The --input mode is useful when a serial terminal has saved a raw binary
capture.  pyserial is only required for live serial-port mode.
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, TextIO


BAUDRATE = 921600
CHANNEL_COUNT = 59
FRAME_SIZE = CHANNEL_COUNT * 4 + 4
TAIL = b"\x00\x00\x80\x7f"
ADC_COUNT = 16
EMG_COUNT = 16
IMU_COUNT = 4
IMU_VALUES = ("roll_deg", "pitch_deg", "yaw_deg", "ax_g", "ay_g", "az_g")


@dataclass(frozen=True)
class TelemetryFrame:
    """One decoded firmware telemetry frame."""

    values: tuple[float, ...]

    @property
    def status(self) -> int:
        return int(round(self.values[0]))

    @property
    def sd_present(self) -> bool:
        return bool(self.status & 0x01)

    @property
    def usb_connected(self) -> bool:
        return bool(self.status & 0x02)


def decode_frame(raw: bytes) -> TelemetryFrame:
    """Decode one complete 240-byte JustFloat frame."""

    if len(raw) != FRAME_SIZE:
        raise ValueError(f"expected {FRAME_SIZE} bytes, got {len(raw)}")
    if raw[-4:] != TAIL:
        raise ValueError(f"invalid JustFloat tail: {raw[-4:].hex(' ')}")

    values = struct.unpack(f"<{CHANNEL_COUNT}f", raw[: CHANNEL_COUNT * 4])
    if not math.isfinite(values[0]):
        raise ValueError("status channel is not finite")

    status = round(values[0])
    if abs(values[0] - status) > 1e-4 or not 0 <= status <= 3:
        raise ValueError(f"invalid status channel: {values[0]!r}")
    return TelemetryFrame(values)


class JustFloatStreamParser:
    """Recover JustFloat frames from a stream that also contains ASCII text.

    The firmware's TX ring queues each telemetry frame as one contiguous
    240-byte block, so the fixed distance from the frame start to the tail is
    enough to recover alignment.  Candidate tails are checked using the
    status channel to reject accidental byte patterns in log text or float
    payloads.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.frames_seen = 0
        self.invalid_candidates = 0
        self.text_bytes = 0

    def feed(self, data: bytes) -> tuple[list[TelemetryFrame], bytes]:
        """Feed bytes and return ``(decoded_frames, non-frame_text_bytes)``."""

        if data:
            self._buffer.extend(data)

        frames: list[TelemetryFrame] = []
        text = bytearray()

        while True:
            # Search all currently available tails.  Looking for a valid
            # candidate instead of blindly taking the first tail allows us to
            # skip a rare accidental tail in an ASCII/binary prefix.
            search_from = 0
            valid: tuple[int, int, TelemetryFrame] | None = None
            first_short_tail_end: int | None = None

            while True:
                tail_pos = self._buffer.find(TAIL, search_from)
                if tail_pos < 0:
                    break
                frame_end = tail_pos + len(TAIL)
                if frame_end < FRAME_SIZE:
                    if first_short_tail_end is None:
                        first_short_tail_end = frame_end
                    search_from = tail_pos + 1
                    continue

                frame_start = frame_end - FRAME_SIZE
                try:
                    candidate = decode_frame(self._buffer[frame_start:frame_end])
                except ValueError:
                    self.invalid_candidates += 1
                    search_from = tail_pos + 1
                    continue

                valid = (frame_start, frame_end, candidate)
                break

            if valid is not None:
                frame_start, frame_end, frame = valid
                if frame_start:
                    text.extend(self._buffer[:frame_start])
                del self._buffer[:frame_end]
                frames.append(frame)
                self.frames_seen += 1
                continue

            # A short tail cannot belong to a complete telemetry frame.  It
            # is therefore safe to classify it as text and discard it.
            if first_short_tail_end is not None:
                text.extend(self._buffer[:first_short_tail_end])
                del self._buffer[:first_short_tail_end]
                continue

            # No tail yet: retain at most one frame minus one byte, because a
            # future tail may complete a frame spanning two read() calls.
            # Everything older than that is necessarily non-frame text.
            keep = FRAME_SIZE - 1
            if len(self._buffer) > keep:
                discard = len(self._buffer) - keep
                text.extend(self._buffer[:discard])
                del self._buffer[:discard]
            break

        self.text_bytes += len(text)
        return frames, bytes(text)

    def flush_text(self) -> bytes:
        """Return buffered bytes at end-of-input (normally partial text)."""

        text = bytes(self._buffer)
        self._buffer.clear()
        self.text_bytes += len(text)
        return text


def csv_fieldnames() -> list[str]:
    fields = [
        "frame_index",
        "host_time_s",
        "status",
        "sd_present",
        "usb_connected",
        "dac_a_volts",
        "dac_b_volts",
    ]
    fields.extend(f"adc_ch{i:02d}_volts" for i in range(1, ADC_COUNT + 1))
    fields.extend(f"emg_ch{i:02d}_volts" for i in range(1, EMG_COUNT + 1))
    for imu in range(IMU_COUNT):
        fields.extend(f"imu{imu}_{name}" for name in IMU_VALUES)
    return fields


def frame_to_row(frame: TelemetryFrame, frame_index: int, host_time_s: float) -> dict[str, object]:
    values = frame.values
    row: dict[str, object] = {
        "frame_index": frame_index,
        "host_time_s": f"{host_time_s:.6f}",
        "status": frame.status,
        "sd_present": int(frame.sd_present),
        "usb_connected": int(frame.usb_connected),
        "dac_a_volts": f"{values[1]:.9g}",
        "dac_b_volts": f"{values[2]:.9g}",
    }

    for i in range(ADC_COUNT):
        row[f"adc_ch{i + 1:02d}_volts"] = f"{values[3 + i]:.9g}"
    for i in range(EMG_COUNT):
        row[f"emg_ch{i + 1:02d}_volts"] = f"{values[19 + i]:.9g}"
    for imu in range(IMU_COUNT):
        base = 35 + imu * len(IMU_VALUES)
        for offset, name in enumerate(IMU_VALUES):
            row[f"imu{imu}_{name}"] = f"{values[base + offset]:.9g}"
    return row


def format_text(data: bytes) -> str:
    """Decode firmware ASCII logs without allowing bad bytes to stop parsing."""

    return data.decode("utf-8", errors="replace")


def write_text(text_log: TextIO, data: bytes, quiet: bool) -> None:
    if not data:
        return
    decoded = format_text(data)
    text_log.write(decoded)
    text_log.flush()
    if not quiet:
        # Keep the firmware's line endings when possible; avoid an extra blank
        # line for CR/LF by printing the raw decoded chunk without a separator.
        print(decoded, end="", flush=True)


def run_stream(
    chunks: Iterable[bytes],
    csv_file: TextIO,
    text_log: TextIO,
    quiet: bool,
    max_frames: int | None,
    progress_interval_s: float,
) -> int:
    parser = JustFloatStreamParser()
    writer = csv.DictWriter(csv_file, fieldnames=csv_fieldnames())
    writer.writeheader()

    frame_count = 0
    last_progress = time.monotonic()
    started = last_progress

    for chunk in chunks:
        frames, text = parser.feed(chunk)
        write_text(text_log, text, quiet)
        for frame in frames:
            frame_count += 1
            writer.writerow(frame_to_row(frame, frame_count, time.monotonic() - started))
            csv_file.flush()

            if not quiet and time.monotonic() - last_progress >= progress_interval_s:
                print(
                    f"[telemetry] frames={frame_count} status={frame.status} "
                    f"SD={'on' if frame.sd_present else 'off'} "
                    f"USB={'on' if frame.usb_connected else 'off'}",
                    flush=True,
                )
                last_progress = time.monotonic()

            if max_frames is not None and frame_count >= max_frames:
                return frame_count

    write_text(text_log, parser.flush_text(), quiet)
    if not quiet:
        print(
            f"[done] frames={frame_count}, invalid_candidates={parser.invalid_candidates}, "
            f"text_bytes={parser.text_bytes}",
            flush=True,
        )
    return frame_count


def serial_chunks(port: str, baudrate: int, timeout_s: float, duration_s: float | None) -> Iterable[bytes]:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "live serial mode requires pyserial; install it with: python -m pip install pyserial"
        ) from exc

    deadline = None if duration_s is None else time.monotonic() + duration_s
    with serial.Serial(port=port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=timeout_s) as ser:
        while deadline is None or time.monotonic() < deadline:
            data = ser.read(4096)
            if data:
                yield data


def file_chunks(path: Path, chunk_size: int = 4096) -> Iterable[bytes]:
    with path.open("rb") as source:
        while True:
            chunk = source.read(chunk_size)
            if not chunk:
                return
            yield chunk


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="live serial port, for example COM5 or /dev/ttyUSB0")
    source.add_argument("--input", type=Path, help="raw binary serial capture file")
    parser.add_argument("--baud", type=int, default=BAUDRATE, help=f"baud rate (default: {BAUDRATE})")
    parser.add_argument("--duration", type=float, help="stop live capture after this many seconds")
    parser.add_argument("--max-frames", type=int, help="stop after writing this many telemetry frames")
    parser.add_argument("--output", type=Path, default=Path("serial_telemetry.csv"), help="telemetry CSV path")
    parser.add_argument("--text-log", type=Path, default=Path("serial_text.log"), help="ASCII log/reply path")
    parser.add_argument("--quiet", action="store_true", help="do not echo firmware text and progress to the console")
    parser.add_argument("--progress-interval", type=float, default=1.0, help="progress print interval in seconds")
    return parser


def open_text_output(path: Path) -> TextIO:
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open("w", encoding="utf-8", newline="")


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.duration is not None and args.duration <= 0:
        raise SystemExit("--duration must be greater than zero")
    if args.max_frames is not None and args.max_frames <= 0:
        raise SystemExit("--max-frames must be greater than zero")
    if args.progress_interval <= 0:
        raise SystemExit("--progress-interval must be greater than zero")
    if args.input is not None and not args.input.is_file():
        raise SystemExit(f"input file does not exist: {args.input}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as csv_file, open_text_output(args.text_log) as text_log:
        try:
            if args.input is not None:
                chunks = file_chunks(args.input)
            else:
                chunks = serial_chunks(args.port, args.baud, 0.5, args.duration)
            run_stream(
                chunks,
                csv_file,
                text_log,
                args.quiet,
                args.max_frames,
                args.progress_interval,
            )
        except KeyboardInterrupt:
            if not args.quiet:
                print("\n[stopped]", flush=True)
        except (OSError, RuntimeError) as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
