#!/usr/bin/env python3
"""
pipeline/scripts/collect_data.py
==================================
Real-time data collector: reads CSV rows from the ESP32-S3 Serial port
and saves them to labeled recording files.

Usage:
    python scripts/collect_data.py --port COM3 --label Healthy   --duration 120
    python scripts/collect_data.py --port COM3 --label Imbalance --duration 120

The firmware must be running with LOG_CSV_ENABLED=1 (default).

Output:
    data/raw/<Label>/<Label>_<timestamp>.csv
"""

import argparse
import datetime
import os
import sys
import time
from pathlib import Path

VALID_LABELS = ["Healthy", "Imbalance", "Misalignment", "Overload", "Friction"]
CSV_HEADER = (
    "ts_ms,vib_rms_m,vib_mean_m,vib_std_m,vib_rms_b,vib_mean_b,vib_std_b,"
    "vib_kurtosis_m,vib_skew_m,vib_crest_m,"
    "fft_peak_freq_hz,fft_peak_mag,"
    "cur_rms_mA,cur_mean_mA,cur_std_mA,"
    "temp_motor_C,temp_bearing_C,"
    "rpm_motor,rpm_base,"
    "as_state,ml_label,ml_conf,fault"
)


def main():
    parser = argparse.ArgumentParser(description="Collect labeled motor data from Serial")
    parser.add_argument("--port",     required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud",     default=115200, type=int)
    parser.add_argument("--label",    required=True, choices=VALID_LABELS,
                        help="Ground-truth fault class being recorded")
    parser.add_argument("--duration", default=120, type=int,
                        help="Recording duration in seconds (default 120)")
    parser.add_argument("--out-dir",  default="data/raw", help="Output directory")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("pyserial not found. Run: pip install pyserial")

    ts = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
    out_dir = Path(args.out_dir) / args.label
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{args.label}_{ts}.csv"

    print(f"[INFO] Opening {args.port} @ {args.baud} baud")
    print(f"[INFO] Recording label: {args.label}")
    print(f"[INFO] Duration: {args.duration}s")
    print(f"[INFO] Output: {out_path}")
    print("[INFO] Ctrl+C to stop early")

    header_written = False
    row_count = 0
    t_end = time.time() + args.duration

    try:
        ser = serial.Serial(args.port, args.baud, timeout=2.0)
        time.sleep(2.0)   # allow ESP32 to reset

        with open(out_path, "w") as f:
            while time.time() < t_end:
                line = ser.readline().decode("utf-8", errors="replace").strip()

                # Skip debug/info/warn/error log lines
                if not line or line.startswith("["):
                    continue

                # Detect CSV header from firmware
                if line.startswith("ts_ms"):
                    if not header_written:
                        # Override with known-good header
                        f.write(CSV_HEADER + "\n")
                        header_written = True
                    continue

                # Write CSV row
                if header_written and "," in line:
                    # Validate column count
                    if len(line.split(",")) >= 19:
                        f.write(line + "\n")
                        f.flush()
                        row_count += 1
                        if row_count % 50 == 0:
                            elapsed = args.duration - int(t_end - time.time())
                            print(f"  {elapsed}s / {args.duration}s — {row_count} rows", end="\r")

    except KeyboardInterrupt:
        print(f"\n[INFO] Stopped by user")
    except serial.SerialException as e:
        print(f"[ERROR] Serial error: {e}")
        sys.exit(1)

    print(f"\n[DONE] Saved {row_count} rows → {out_path}")
    if row_count < 50:
        print("[WARN] Very few rows collected — check baud rate and LOG_CSV_ENABLED=1")


if __name__ == "__main__":
    main()
