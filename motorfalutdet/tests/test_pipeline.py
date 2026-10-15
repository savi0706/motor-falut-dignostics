#!/usr/bin/env python3
"""
tests/test_pipeline.py
========================
Automated tests for the preprocessing and feature consistency.
Run with: python -m pytest tests/ -v

Tests:
  1. Preprocessing with MotorMind legacy CSV data
  2. Recording-level split produces no leakage
  3. Feature vector dimensionality matches FEATURE_COUNT=16
  4. Label mapping correctness
  5. Duplicate row removal
  6. NaN handling
"""

import sys
import os
import numpy as np
import pandas as pd
import pytest
from pathlib import Path
from io import StringIO

# Add pipeline scripts to path
sys.path.insert(0, str(Path(__file__).parent.parent / "pipeline" / "scripts"))

from preprocess import (
    load_and_map, clean, recording_level_split,
    CLASS_TO_INT, LABEL_MAP, FEATURE_COLS_NEW
)

# ── Fixtures ─────────────────────────────────────────────────────────────────

LEGACY_CSV = """time_ms,vib_rms,vib_mean,vib_std,current_rms,current_mean,current_std,rpm,temp_C,status
1000,9.17,9.17,0.07,0.04,0.02,0.04,0.00,30.87,STOPPED
2000,9.17,9.17,0.06,0.04,0.02,0.04,0.00,30.87,NORMAL
3000,9.42,9.41,0.24,0.22,0.20,0.10,953.34,30.94,NORMAL
4000,8.91,8.90,0.16,0.26,0.21,0.14,1295.10,37.00,OVERCURRENT
5000,8.84,8.83,0.33,0.24,0.19,0.13,1339.53,38.75,UNBALANCE
6000,8.84,8.83,0.33,0.24,0.19,0.13,1339.53,38.75,UNBALANCE
"""

# Duplicate row (same time_ms and values as row at 5000)
LEGACY_DUP_CSV = LEGACY_CSV + "5000,8.84,8.83,0.33,0.24,0.19,0.13,1339.53,38.75,UNBALANCE\n"

# ── Tests ─────────────────────────────────────────────────────────────────────

class TestLabelMapping:
    def test_normal_maps_healthy(self):
        assert LABEL_MAP["NORMAL"] == "Healthy"

    def test_unbalance_maps_imbalance(self):
        assert LABEL_MAP["UNBALANCE"] == "Imbalance"

    def test_overcurrent_excluded_not_overload(self):
        # Electrical overcurrent must NOT be equated with mechanical overload
        assert LABEL_MAP["OVERCURRENT"] is None

    def test_stopped_excluded(self):
        assert LABEL_MAP["STOPPED"] is None

    def test_class_indices_contiguous(self):
        indices = sorted(CLASS_TO_INT.values())
        assert indices == list(range(len(CLASS_TO_INT))), \
            "Class indices must be 0..N-1 with no gaps"

    def test_healthy_is_index_0(self):
        assert CLASS_TO_INT["Healthy"] == 0, "Healthy must be index 0 (matches firmware LABEL_HEALTHY)"


class TestPreprocessing:
    def _make_df(self, csv_text: str, filepath: str = "data/raw/test_file.csv") -> pd.DataFrame:
        buf = StringIO(csv_text)
        # Simulate file on disk
        import tempfile, os
        with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
            f.write(csv_text)
            name = f.name
        try:
            df = load_and_map(Path(name))
        finally:
            os.unlink(name)
        return df

    def test_legacy_loads_all_columns(self):
        df = self._make_df(LEGACY_CSV)
        for col in FEATURE_COLS_NEW:
            assert col in df.columns, f"Missing column: {col}"

    def test_stopped_excluded_by_clean(self):
        df = self._make_df(LEGACY_CSV)
        df = clean(df)
        assert (df["label"] < 0).sum() == 0, "Excluded rows must have label < 0"
        # STOPPED rows should be gone
        assert "STOPPED" not in df.get("fault_raw", pd.Series()).values

    def test_duplicate_removal(self):
        df = self._make_df(LEGACY_DUP_CSV)
        n_before = len(df)
        df = clean(df)
        # After clean, no duplicate (ts_ms, class_name) pairs
        dupes = df.duplicated(subset=["ts_ms", "class_name"])
        assert dupes.sum() == 0, "Duplicates were not removed"

    def test_feature_vector_length(self):
        assert len(FEATURE_COLS_NEW) == 16, \
            f"Feature vector must have 16 elements, got {len(FEATURE_COLS_NEW)}"

    def test_nan_imputation(self):
        csv_with_nan = LEGACY_CSV.replace("0.33", "")
        df = self._make_df(csv_with_nan)
        df = clean(df)
        for col in FEATURE_COLS_NEW:
            if col in df.columns:
                assert df[col].isna().sum() == 0, f"NaN not imputed in {col}"


class TestSplitIntegrity:
    def test_recording_level_split_no_leakage(self):
        """
        Train and test sets must share no source_file recordings.
        This is the key fix over the original MotorMind code.
        """
        # Build a multi-recording DataFrame
        frames = []
        for i in range(10):
            df_i = pd.DataFrame({
                "ts_ms":       range(100),
                "source_file": [f"file_{i}.csv"] * 100,
                "label":       [i % 5] * 100,
                "class_name":  ["Healthy"] * 100,
            })
            for col in FEATURE_COLS_NEW:
                df_i[col] = np.random.randn(100)
            frames.append(df_i)

        df = pd.concat(frames, ignore_index=True)
        train, test = recording_level_split(df, test_fraction=0.2, seed=42)

        train_files = set(train["source_file"].unique())
        test_files  = set(test["source_file"].unique())

        assert len(train_files & test_files) == 0, \
            "DATA LEAKAGE: same recording appears in both train and test sets!"

    def test_split_covers_all_recordings(self):
        """All recordings appear in exactly one split."""
        frames = []
        for i in range(8):
            df_i = pd.DataFrame({
                "ts_ms":       range(50),
                "source_file": [f"rec_{i}.csv"] * 50,
                "label":       [0] * 50,
                "class_name":  ["Healthy"] * 50,
            })
            for col in FEATURE_COLS_NEW:
                df_i[col] = np.random.randn(50)
            frames.append(df_i)

        df = pd.concat(frames, ignore_index=True)
        train, test = recording_level_split(df)
        all_in = set(train["source_file"].unique()) | set(test["source_file"].unique())
        all_df = set(df["source_file"].unique())
        assert all_in == all_df


class TestFeatureConsistency:
    def test_feature_names_match_count(self):
        """FEATURE_COLS_NEW length must equal firmware FEATURE_COUNT=16."""
        assert len(FEATURE_COLS_NEW) == 16

    def test_first_feature_is_vib_rms(self):
        """Feature index 0 must be vib_rms_m (matches firmware fv.data[0])."""
        assert FEATURE_COLS_NEW[0] == "vib_rms_m"

    def test_label_indices_match_firmware(self):
        """
        Class index mapping must match firmware constants in config.h:
          LABEL_HEALTHY=0, LABEL_IMBALANCE=1, LABEL_MISALIGNMENT=2,
          LABEL_OVERLOAD=3, LABEL_FRICTION=4
        """
        assert CLASS_TO_INT["Healthy"]      == 0
        assert CLASS_TO_INT["Imbalance"]    == 1
        assert CLASS_TO_INT["Misalignment"] == 2
        assert CLASS_TO_INT["Overload"]     == 3
        assert CLASS_TO_INT["Friction"]     == 4


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
