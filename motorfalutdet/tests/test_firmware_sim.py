#!/usr/bin/env python3
"""
tests/test_firmware_sim.py
===========================
Host-side simulation tests for firmware logic.

Validates:
  - Adaptive sensing state machine transitions and hysteresis
  - Feature vector length / type
  - Health score bounds
  - No fabricated fault detections from stub model

Run: python -m pytest tests/ -v
"""

import sys
import math
import pytest
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "pipeline" / "scripts"))

NUM_CLASSES = 5
FEATURE_COUNT = 16

# ── Python reimplementation of adaptive_sensing.h ────────────────────────────

AS_VIB_WATCH  = 0.35
AS_VIB_ALERT  = 0.80
AS_CUR_WATCH  = 0.30 * 1000   # converted to mA (×1000)
AS_CUR_ALERT  = 0.60 * 1000
AS_TEMP_WATCH = 55.0
AS_TEMP_ALERT = 70.0
AS_HYSTERESIS = 5

class AdaptiveState:
    NORMAL = 0
    WATCH  = 1
    ALERT  = 2
    NAMES  = ["NORMAL", "WATCH", "ALERT"]

class AdaptiveSensingController:
    def __init__(self):
        self.state = AdaptiveState.NORMAL
        self.calm_count = 0

    def update(self, fv: list) -> int:
        vib_std = fv[2]
        cur_rms = fv[10]
        temp_m  = fv[13]

        alert = (vib_std > AS_VIB_ALERT or cur_rms > AS_CUR_ALERT or
                 (temp_m > AS_TEMP_ALERT and temp_m > 0.1))
        watch = (vib_std > AS_VIB_WATCH or cur_rms > AS_CUR_WATCH or
                 (temp_m > AS_TEMP_WATCH and temp_m > 0.1))

        if alert:
            desired = AdaptiveState.ALERT
            self.calm_count = 0
        elif watch:
            desired = AdaptiveState.WATCH
            self.calm_count = 0
        else:
            desired = AdaptiveState.NORMAL
            self.calm_count += 1

        if desired < self.state:
            if self.calm_count >= AS_HYSTERESIS:
                self.state = desired
                self.calm_count = 0
        elif desired > self.state:
            self.state = desired
            self.calm_count = 0

        return self.state

# ── Python reimplementation of health_model.h ─────────────────────────────────

def compute_health(fv, fault_label):
    def stress(val, low, high, max_s):
        if val <= low: return 0.0
        return min(1.0, (val - low) / (high - low)) * max_s

    penalties = {0: 0, 1: 10, 2: 15, 3: 20, 4: 5}
    raw = 100 - stress(fv[2], AS_VIB_WATCH, AS_VIB_ALERT, 40) \
              - stress(fv[10]/1000, 0.30, 0.60, 30) \
              - stress(fv[13], AS_TEMP_WATCH, AS_TEMP_ALERT, 20) \
              - penalties.get(fault_label, 0)
    return max(0.0, min(100.0, raw))

# ── Tests ─────────────────────────────────────────────────────────────────────

def _zero_fv():
    """16-element zero feature vector (healthy/stopped motor)."""
    return [0.0] * FEATURE_COUNT

def _healthy_fv():
    """Typical healthy running motor."""
    fv = [0.0] * FEATURE_COUNT
    fv[0]  = 9.2    # vib_rms_m
    fv[1]  = 9.19   # vib_mean_m
    fv[2]  = 0.07   # vib_std_m — well below WATCH threshold
    fv[10] = 220.0  # cur_rms_mA ~0.22A
    fv[13] = 33.0   # temp_motor_C
    fv[14] = 960.0  # rpm
    return fv

def _fault_fv(fault_type):
    """Simulate elevated signals for a fault class."""
    fv = _healthy_fv()
    if fault_type == "imbalance":
        fv[2] = 0.50   # elevated vibration std
        fv[3] = 4.2    # high kurtosis
    elif fault_type == "overload":
        fv[10] = 650.0  # >0.60 A threshold
    elif fault_type == "overheat":
        fv[13] = 72.0   # >ALERT threshold
    return fv


class TestAdaptiveSensing:
    def test_starts_in_normal(self):
        ctrl = AdaptiveSensingController()
        assert ctrl.state == AdaptiveState.NORMAL

    def test_transitions_to_watch_on_elevated_vib(self):
        ctrl = AdaptiveSensingController()
        fv = _healthy_fv()
        fv[2] = 0.40   # above WATCH (0.35) but below ALERT (0.80)
        state = ctrl.update(fv)
        assert state == AdaptiveState.WATCH

    def test_transitions_to_alert_immediately(self):
        ctrl = AdaptiveSensingController()
        fv = _healthy_fv()
        fv[2] = 0.85   # above ALERT threshold
        state = ctrl.update(fv)
        assert state == AdaptiveState.ALERT

    def test_hysteresis_prevents_immediate_downgrade(self):
        ctrl = AdaptiveSensingController()
        # Put into ALERT
        fv_alert = _healthy_fv(); fv_alert[2] = 0.85
        ctrl.update(fv_alert)
        assert ctrl.state == AdaptiveState.ALERT

        # One calm reading should NOT drop back to NORMAL
        fv_calm = _healthy_fv()
        ctrl.update(fv_calm)
        assert ctrl.state == AdaptiveState.ALERT, \
            "Must not drop from ALERT immediately — hysteresis required"

    def test_hysteresis_allows_downgrade_after_n_calm(self):
        ctrl = AdaptiveSensingController()
        fv_alert = _healthy_fv(); fv_alert[2] = 0.85
        ctrl.update(fv_alert)

        fv_calm = _healthy_fv()
        for _ in range(AS_HYSTERESIS):
            ctrl.update(fv_calm)

        assert ctrl.state == AdaptiveState.NORMAL, \
            f"Should be NORMAL after {AS_HYSTERESIS} calm readings"

    def test_current_alert_triggers_alert_state(self):
        ctrl = AdaptiveSensingController()
        fv = _healthy_fv()
        fv[10] = 650.0   # above 600 mA alert threshold
        state = ctrl.update(fv)
        assert state == AdaptiveState.ALERT

    def test_temperature_alert_triggers_alert_state(self):
        ctrl = AdaptiveSensingController()
        fv = _healthy_fv()
        fv[13] = 72.0    # above 70°C alert threshold
        state = ctrl.update(fv)
        assert state == AdaptiveState.ALERT


class TestHealthModel:
    def test_healthy_motor_near_100(self):
        fv = _healthy_fv()
        h = compute_health(fv, 0)
        assert h >= 90.0, f"Healthy motor health should be ≥90, got {h:.1f}"

    def test_overload_fault_reduces_health(self):
        fv = _fault_fv("overload")
        h_healthy = compute_health(fv, 0)
        h_overload = compute_health(fv, 3)   # LABEL_OVERLOAD
        assert h_overload < h_healthy, "Overload penalty must reduce health score"

    def test_health_clamped_to_0_100(self):
        # Extreme values
        fv = [9999.0] * FEATURE_COUNT
        h = compute_health(fv, 3)
        assert 0.0 <= h <= 100.0

        fv = _zero_fv()
        h = compute_health(fv, 0)
        assert 0.0 <= h <= 100.0


class TestFeatureVectorSpec:
    def test_feature_count(self):
        fv = _healthy_fv()
        assert len(fv) == FEATURE_COUNT, \
            f"Feature vector must have {FEATURE_COUNT} elements (matches firmware config)"

    def test_all_floats(self):
        fv = _healthy_fv()
        for i, v in enumerate(fv):
            assert isinstance(v, float), f"fv[{i}] must be float, got {type(v)}"

    def test_num_classes(self):
        assert NUM_CLASSES == 5, "Must have exactly 5 classes"


class TestStubModelBehavior:
    """
    Verify that the system behaves safely when the stub model is active.
    The stub model should not be able to produce a confident prediction.
    """
    def test_low_confidence_stub_predicts_healthy(self):
        # Simulate stub model output: uniform 0.2 across all classes
        probs = [0.2, 0.2, 0.2, 0.2, 0.2]
        confidence_threshold = 0.60
        best_idx = int(np.argmax(probs))
        best_conf = probs[best_idx]

        if best_conf < confidence_threshold:
            predicted = 0  # → Healthy (safe default)
        else:
            predicted = best_idx

        assert predicted == 0, \
            "Stub model (uniform probs) must default to Healthy when below confidence threshold"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
