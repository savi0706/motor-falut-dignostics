#!/usr/bin/env python3
"""
pipeline/scripts/preprocess.py
================================
Data preprocessing for motor fault classification.

Reads raw CSV files from data/raw/<class>/<file>.csv
Outputs a processed dataset to data/processed/dataset.csv

Expected raw CSV format (from firmware CSV logger):
  ts_ms, vib_rms_m, vib_mean_m, vib_std_m, vib_rms_b, vib_mean_b, vib_std_b,
  vib_kurtosis_m, vib_skew_m, vib_crest_m,
  fft_peak_freq_hz, fft_peak_mag,
  cur_rms_mA, cur_mean_mA, cur_std_mA,
  temp_motor_C, temp_bearing_C,
  rpm_motor, rpm_base,
  as_state, ml_label, ml_conf, fault

OR can also read the MotorMind-format CSV with columns:
  time_ms, vib_rms, vib_mean, vib_std, current_rms, current_mean,
  current_std, rpm, temp_C, status

Bugs fixed vs original MotorMind train_motor_model.py:
  1. Duplicate rows removed (time_ms was non-monotonic due to copy-paste)
  2. Row-level train/test split replaced by recording-level split
     (prevents temporal leakage)
  3. Label strings mapped to int indices consistent with firmware
  4. Missing-value imputation documented
  5. NaN-guarding added for INA219/DS18B20 dropouts
"""

import argparse
import os
import sys
import pandas as pd
import numpy as np
from pathlib import Path

# ── Configuration ────────────────────────────────────────────────────────────
LABEL_MAP = {
    # MotorMind legacy labels -> our 5-class scheme
    "NORMAL":       "Healthy",
    "HEALTHY":      "Healthy",
    "STOPPED":      None,          # Exclude: motor not running
    "UNBALANCE":    "Imbalance",
    "IMBALANCE":    "Imbalance",
    "MISALIGNMENT": "Misalignment",# Missing in provided repos
    "OVERCURRENT":  None,          # Exclude: Electrical overcurrent is NOT mechanical Overload
    "OVERLOAD":     "Overload",    # Missing in provided repos
    "FRICTION":     "Friction",    # Missing in provided repos
    "OVERHEAT":     None,          # Exclude: thermal condition
}

CLASS_TO_INT = {
    "Healthy":      0,
    "Imbalance":    1,
    "Misalignment": 2,
    "Overload":     3,
    "Friction":     4,
}

FEATURE_COLS_NEW = [
    "vib_rms_m", "vib_mean_m", "vib_std_m",
    "vib_rms_b", "vib_std_b",
    "vib_kurtosis_m", "vib_skew_m", "vib_crest_m",
    "fft_peak_freq_hz", "fft_peak_mag",
    "cur_rms_mA", "cur_mean_mA", "cur_std_mA",
    "temp_motor_C", "rpm_motor", "rpm_delta",
]

FEATURE_COLS_LEGACY = [
    "vib_rms", "vib_mean", "vib_std",
    "current_rms", "current_mean", "current_std",
    "rpm", "temp_C",
]

# ── Helpers ────────────────────────────────────────────────────────────────
def detect_format(df: pd.DataFrame) -> str:
    """Detect whether this is new firmware CSV or MotorMind legacy format."""
    if "vib_rms_m" in df.columns:
        return "new"
    if "vib_rms" in df.columns:
        return "legacy"
    raise ValueError(f"Unknown CSV format: {list(df.columns)}")


def load_and_map(filepath: Path) -> pd.DataFrame:
    """Load a single CSV file and normalise to unified schema."""
    df = pd.read_csv(filepath)
    fmt = detect_format(df)

    if fmt == "legacy":
        # Map MotorMind column names
        df = df.rename(columns={
            "time_ms":      "ts_ms",
            "vib_rms":      "vib_rms_m",
            "vib_mean":     "vib_mean_m",
            "vib_std":      "vib_std_m",
            "current_rms":  "cur_rms_mA",  # note: MotorMind uses A, we use mA
            "current_mean": "cur_mean_mA",
            "current_std":  "cur_std_mA",
            "rpm":          "rpm_motor",
            "temp_C":       "temp_motor_C",
            "status":       "fault",
        })
        # Legacy current is in Amps; convert to mA for consistency
        for col in ["cur_rms_mA", "cur_mean_mA", "cur_std_mA"]:
            if col in df.columns:
                df[col] = df[col] * 1000.0

        # Fill missing new-format columns with defaults
        for col in set(FEATURE_COLS_NEW) - set(df.columns):
            df[col] = 0.0
        df["vib_rms_b"] = df["vib_rms_m"]   # no base IMU in legacy
        df["vib_std_b"] = df["vib_std_m"]
        df["vib_kurtosis_m"] = 0.0
        df["vib_skew_m"]     = 0.0
        df["vib_crest_m"]    = df["vib_rms_m"]  # placeholder
        df["fft_peak_freq_hz"] = 0.0
        df["fft_peak_mag"]     = 0.0
        df["rpm_base"]  = df["rpm_motor"]
        df["rpm_delta"] = 0.0

    # Source file tracking
    df["source_file"] = str(filepath)

    # Map label strings
    df["fault_raw"] = df["fault"].astype(str).str.upper().str.strip()
    df["class_name"] = df["fault_raw"].map(lambda x: LABEL_MAP.get(x, None))
    df["label"] = df["class_name"].map(lambda x: CLASS_TO_INT.get(x, -1))

    return df


def load_all(data_dir: Path) -> pd.DataFrame:
    """
    Load from directory structure:
        data/raw/<class_name>/<file>.csv   ← structured (one class per folder)
     OR data/raw/<file>.csv               ← flat (label col present)
    """
    frames = []
    csv_files = list(data_dir.glob("**/*.csv"))
    if not csv_files:
        print(f"[WARN] No CSV files found in {data_dir}")
        return pd.DataFrame()

    for f in csv_files:
        try:
            df = load_and_map(f)
            # For structured folders, override fault label from folder name
            parent = f.parent.name.upper()
            if parent in LABEL_MAP and LABEL_MAP[parent] is not None:
                df["class_name"] = LABEL_MAP[parent]
                df["label"] = CLASS_TO_INT[LABEL_MAP[parent]]
            frames.append(df)
        except Exception as e:
            print(f"[WARN] Skipping {f}: {e}")

    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def clean(df: pd.DataFrame) -> pd.DataFrame:
    """Remove exclusions, duplicates, and invalid rows."""
    # Remove STOPPED, OVERHEAT (no target class)
    df = df[df["label"] >= 0].copy()

    # Sort by source_file + ts_ms to enable recording-level split
    df = df.sort_values(["source_file", "ts_ms"]).reset_index(drop=True)

    # ── BUG FIX: remove duplicate rows (MotorMind dataset had copy-paste duplicates)
    before = len(df)
    df = df.drop_duplicates(subset=["source_file", "ts_ms", "class_name"])
    after = len(df)
    if before != after:
        print(f"[INFO] Removed {before - after} duplicate rows")

    # NaN imputation
    for col in FEATURE_COLS_NEW:
        if col in df.columns:
            n_nan = df[col].isna().sum()
            if n_nan:
                print(f"[INFO] Imputing {n_nan} NaN in {col} with column median")
                df[col] = df[col].fillna(df[col].median())

    return df


def recording_level_split(df: pd.DataFrame, test_fraction: float = 0.2, seed: int = 42):
    """
    Split by unique (source_file) recording IDs to prevent temporal leakage.
    ── BUG FIX vs original: original used row-level random split which
       allows adjacent time steps from the same recording to appear in
       both train and test sets (leakage).
    """
    rng = np.random.default_rng(seed)
    recordings = df["source_file"].unique()

    if len(recordings) == 1:
        print("[WARN] Only one source file -- using time-ordered row split.")
        df_sorted = df.sort_values("ts_ms").reset_index(drop=True)
        n_test = max(1, int(len(df_sorted) * test_fraction))
        train = df_sorted.iloc[:-n_test].copy()
        test  = df_sorted.iloc[-n_test:].copy()
        print(f"[INFO] Rows:  train={len(train)}, test={len(test)}")
        return train, test

    rng.shuffle(recordings)
    n_test = max(1, int(len(recordings) * test_fraction))
    test_recordings  = set(recordings[:n_test])
    train_recordings = set(recordings[n_test:])

    train = df[df["source_file"].isin(train_recordings)].copy()
    test  = df[df["source_file"].isin(test_recordings)].copy()

    print(f"[INFO] Split: {len(train_recordings)} recordings in train, "
          f"{len(test_recordings)} in test")
    print(f"[INFO] Rows:  train={len(train)}, test={len(test)}")
    return train, test


def save(df: pd.DataFrame, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False)
    print(f"[INFO] Saved {len(df)} rows -> {path}")


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Preprocess motor fault data")
    parser.add_argument("--data-dir",  default="data/raw",       help="Raw data directory")
    parser.add_argument("--out-dir",   default="data/processed",  help="Processed output directory")
    parser.add_argument("--test-frac", default=0.2,  type=float,  help="Test set fraction")
    parser.add_argument("--seed",      default=42,   type=int)
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    out_dir  = Path(args.out_dir)

    print(f"[INFO] Loading data from: {data_dir}")
    df = load_all(data_dir)

    if df.empty:
        print("[ERROR] No data loaded. Ensure CSV files exist in data/raw/")
        print()
        print("  Expected structures:")
        print("    data/raw/<FaultClass>/<recording>.csv")
        print("  OR")
        print("    data/raw/<recording>.csv  (with 'fault' column)")
        print()
        print("  Place the MotorMind dataset.txt as:")
        print("    data/raw/mixed/dataset.csv")
        sys.exit(1)

    df = clean(df)

    # Class distribution
    print("\n[INFO] Class distribution:")
    for cls, idx in CLASS_TO_INT.items():
        n = (df["label"] == idx).sum()
        print(f"  {idx} {cls:15s}: {n:5d} rows")

    # Save full cleaned dataset
    save(df[FEATURE_COLS_NEW + ["class_name", "label", "source_file", "ts_ms"]],
         out_dir / "dataset_full.csv")

    # Recording-level split
    train, test = recording_level_split(df, args.test_frac, args.seed)
    save(train[FEATURE_COLS_NEW + ["class_name", "label"]], out_dir / "train.csv")
    save(test [FEATURE_COLS_NEW + ["class_name", "label"]], out_dir / "test.csv")

    print("\n[DONE] Preprocessing complete")


if __name__ == "__main__":
    main()
