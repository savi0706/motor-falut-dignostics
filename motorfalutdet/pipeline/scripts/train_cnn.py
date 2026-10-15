#!/usr/bin/env python3
"""
pipeline/scripts/train_cnn.py
==============================
Train a compact 1D-CNN for 5-class motor fault classification.

Uses the 16-element feature vector from the preprocessing pipeline.
Compares CNN with a Random Forest baseline.
Exports the best model as a TFLite FlatBuffer.

Run after preprocess.py:
    cd pipeline
    python scripts/train_cnn.py

Outputs:
    models/motor_fault_cnn.tflite       — deployment model
    models/motor_fault_rf.pkl           — baseline RF model
    models/training_report.json         — metrics + provenance
    models/confusion_matrix.png         — test set confusion matrix

NOTE: Training requires labeled data in data/processed/train.csv
      and data/processed/test.csv (produced by preprocess.py).
      If these files are absent, this script exits with instructions.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

# ── Optional imports with helpful error messages ──────────────────────────────
try:
    import sklearn
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.metrics import (accuracy_score, classification_report,
                                  confusion_matrix, f1_score)
    from sklearn.preprocessing import StandardScaler
    import joblib
except ImportError:
    sys.exit("scikit-learn not found. Run: pip install scikit-learn joblib")

try:
    import tensorflow as tf
except ImportError:
    sys.exit("TensorFlow not found. Run: pip install tensorflow")

# ── Config ────────────────────────────────────────────────────────────────────
FEATURE_COLS = [
    "vib_rms_m", "vib_mean_m", "vib_std_m",
    "vib_rms_b", "vib_std_b",
    "vib_kurtosis_m", "vib_skew_m", "vib_crest_m",
    "fft_peak_freq_hz", "fft_peak_mag",
    "cur_rms_mA", "cur_mean_mA", "cur_std_mA",
    "temp_motor_C", "rpm_motor", "rpm_delta",
]
NUM_CLASSES = 5
CLASS_NAMES = ["Healthy", "Imbalance", "Misalignment", "Overload", "Friction"]

# ── Helpers ────────────────────────────────────────────────────────────────────
def load_split(data_dir: Path):
    train_path = data_dir / "train.csv"
    test_path  = data_dir / "test.csv"

    if not train_path.exists() or not test_path.exists():
        print("[ERROR] Missing train.csv or test.csv in", data_dir)
        print("        Run preprocessing first:")
        print("          python scripts/preprocess.py")
        sys.exit(1)

    train = pd.read_csv(train_path)
    test  = pd.read_csv(test_path)

    # Validate columns
    missing = [c for c in FEATURE_COLS if c not in train.columns]
    if missing:
        sys.exit(f"[ERROR] Missing columns in train.csv: {missing}")

    X_train = train[FEATURE_COLS].values.astype(np.float32)
    y_train = train["label"].values.astype(np.int64)
    X_test  = test[FEATURE_COLS].values.astype(np.float32)
    y_test  = test["label"].values.astype(np.int64)

    print(f"[INFO] Train: {X_train.shape}, Test: {X_test.shape}")
    return X_train, y_train, X_test, y_test


def normalize(X_train, X_test):
    scaler = StandardScaler()
    X_train_n = scaler.fit_transform(X_train)
    X_test_n  = scaler.transform(X_test)
    return X_train_n, X_test_n, scaler


# ── Baseline: Random Forest ────────────────────────────────────────────────────
def train_random_forest(X_train, y_train, X_test, y_test):
    print("\n[RF] Training Random Forest baseline...")
    t0 = time.time()
    rf = RandomForestClassifier(
        n_estimators=100, max_depth=None,
        n_jobs=-1, random_state=42,
        class_weight="balanced"
    )
    rf.fit(X_train, y_train)
    t1 = time.time()

    y_pred = rf.predict(X_test)
    acc  = accuracy_score(y_test, y_pred)
    f1   = f1_score(y_test, y_pred, average="macro", zero_division=0)
    report = classification_report(y_test, y_pred,
                                   labels=list(range(NUM_CLASSES)),
                                   target_names=CLASS_NAMES, zero_division=0)

    print(f"[RF] Accuracy: {acc:.4f}  Macro-F1: {f1:.4f}  Time: {t1-t0:.1f}s")
    print(report)

    return rf, {"rf_accuracy": float(acc), "rf_macro_f1": float(f1),
                "rf_report": report}


# ── 1D-CNN ────────────────────────────────────────────────────────────────────
def build_cnn(n_features: int, n_classes: int):
    """
    Compact 1D-CNN treating the feature vector as a 1-step sequence.
    Architecture chosen for minimal flash/RAM on ESP32-S3:
      Dense → BN → Dropout → Dense → BN → Dropout → Dense (softmax)
    This is a fully-connected DNN (no Conv1D) because our feature vector
    is already computed (not raw waveform). Raw-waveform CNN would require
    streaming WINDOW_SIZE samples to the model, which is feasible but not
    needed here given the pre-extracted features.
    """
    inp = tf.keras.Input(shape=(n_features,), name="features")
    x = tf.keras.layers.Dense(64, activation="relu")(inp)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.Dropout(0.3)(x)
    x = tf.keras.layers.Dense(32, activation="relu")(x)
    x = tf.keras.layers.BatchNormalization()(x)
    x = tf.keras.layers.Dropout(0.2)(x)
    out = tf.keras.layers.Dense(n_classes, activation="softmax", name="predictions")(x)
    model = tf.keras.Model(inputs=inp, outputs=out, name="motor_fault_dnn")
    return model


def train_cnn(X_train, y_train, X_test, y_test,
              epochs: int = 60, batch_size: int = 32):
    print("\n[CNN] Training compact DNN...")
    n_features = X_train.shape[1]

    model = build_cnn(n_features, NUM_CLASSES)
    model.summary()

    # One-hot encode labels
    y_train_oh = tf.keras.utils.to_categorical(y_train, NUM_CLASSES)
    y_test_oh  = tf.keras.utils.to_categorical(y_test,  NUM_CLASSES)

    # Class weights for imbalanced data (only compute over classes present in y_train)
    from sklearn.utils.class_weight import compute_class_weight
    classes_present = np.unique(y_train)
    weights = compute_class_weight("balanced", classes=classes_present, y=y_train)
    class_weights = {i: 1.0 for i in range(NUM_CLASSES)}
    for c, w in zip(classes_present, weights):
        class_weights[int(c)] = float(w)

    model.compile(
        optimizer=tf.keras.optimizers.Adam(1e-3),
        loss="categorical_crossentropy",
        metrics=["accuracy"]
    )

    callbacks = [
        tf.keras.callbacks.EarlyStopping(
            monitor="val_accuracy", patience=10,
            restore_best_weights=True),
        tf.keras.callbacks.ReduceLROnPlateau(
            monitor="val_loss", factor=0.5, patience=5, min_lr=1e-5),
    ]

    t0 = time.time()
    history = model.fit(
        X_train, y_train_oh,
        validation_data=(X_test, y_test_oh),
        epochs=epochs, batch_size=batch_size,
        class_weight=class_weights,
        callbacks=callbacks,
        verbose=1
    )
    t1 = time.time()

    # Evaluate
    y_pred = np.argmax(model.predict(X_test), axis=1)
    acc  = accuracy_score(y_test, y_pred)
    f1   = f1_score(y_test, y_pred, average="macro", zero_division=0)
    report = classification_report(y_test, y_pred,
                                   labels=list(range(NUM_CLASSES)),
                                   target_names=CLASS_NAMES, zero_division=0)

    print(f"[CNN] Accuracy: {acc:.4f}  Macro-F1: {f1:.4f}  Time: {t1-t0:.1f}s")
    print(report)

    return model, history, {
        "cnn_accuracy": float(acc),
        "cnn_macro_f1": float(f1),
        "cnn_report": report
    }


# ── TFLite Export ─────────────────────────────────────────────────────────────
def export_tflite(model, scaler, out_dir: Path):
    """Convert to TFLite FlatBuffer + generate firmware header."""
    out_dir.mkdir(parents=True, exist_ok=True)

    # Full-precision TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    tflite_path = out_dir / "motor_fault_cnn.tflite"
    tflite_path.write_bytes(tflite_model)
    print(f"[TFLite] Saved {len(tflite_model)} bytes -> {tflite_path}")

    # INT8 quantized (smaller, faster — optional)
    try:
        conv_q = tf.lite.TFLiteConverter.from_keras_model(model)
        conv_q.optimizations = [tf.lite.Optimize.DEFAULT]
        tflite_q = conv_q.convert()
        q_path = out_dir / "motor_fault_cnn_int8.tflite"
        q_path.write_bytes(tflite_q)
        print(f"[TFLite-INT8] Saved {len(tflite_q)} bytes -> {q_path}")
    except Exception as e:
        print(f"[WARN] INT8 quantization failed: {e}")

    # Generate C header for firmware
    header_path = out_dir / "model_data.h"
    _write_c_header(tflite_model, header_path, scaler)
    print(f"[Header] Written -> {header_path}")
    print(f"         Copy to firmware/include/model_data.h")

    return str(tflite_path)


def _write_c_header(data: bytes, path: Path, scaler):
    """Write model_data.h with scaler constants embedded."""
    lines = [
        "/* Auto-generated by pipeline/scripts/train_cnn.py",
        " * DO NOT EDIT — regenerate by running the training pipeline.",
        " * Feature normalization constants are embedded below.",
        " */",
        "#pragma once",
        "#ifndef MODEL_DATA_H",
        "#define MODEL_DATA_H",
        "",
        "// Scaler mean and std for inference-time normalization",
        "// Apply: normalized[i] = (raw[i] - SCALER_MEAN[i]) / SCALER_STD[i]",
        f"const float SCALER_MEAN[{len(scaler.mean_)}] = {{",
        "  " + ", ".join(f"{v:.6f}f" for v in scaler.mean_) + "",
        "};",
        f"const float SCALER_STD[{len(scaler.scale_)}] = {{",
        "  " + ", ".join(f"{v:.6f}f" for v in scaler.scale_) + "",
        "};",
        "",
        f"// Model size: {len(data)} bytes",
        "alignas(8) const unsigned char g_model_data[] = {",
    ]

    # Hex dump in rows of 16
    hex_rows = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_rows.append("  " + ", ".join(f"0x{b:02X}" for b in chunk) + ",")
    lines += hex_rows
    lines += [
        "};",
        f"const unsigned int g_model_data_len = {len(data)};",
        "",
        "#endif // MODEL_DATA_H",
    ]
    path.write_text("\n".join(lines))


# ── Confusion Matrix Plot ─────────────────────────────────────────────────────
def save_confusion_matrix(y_true, y_pred, out_dir: Path):
    try:
        import matplotlib.pyplot as plt
        import matplotlib
        matplotlib.use("Agg")
        from sklearn.metrics import ConfusionMatrixDisplay

        cm = confusion_matrix(y_true, y_pred, labels=list(range(NUM_CLASSES)))
        disp = ConfusionMatrixDisplay(cm, display_labels=CLASS_NAMES)
        fig, ax = plt.subplots(figsize=(8, 7))
        disp.plot(ax=ax, cmap="Blues", colorbar=True)
        ax.set_title("Test Set Confusion Matrix - Motor Fault CNN")
        plt.tight_layout()
        path = out_dir / "confusion_matrix.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        print(f"[Plot] Confusion matrix -> {path}")
    except ImportError:
        print("[WARN] matplotlib not installed — skipping confusion matrix plot")


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Train motor fault classifier")
    parser.add_argument("--data-dir",   default="data/processed")
    parser.add_argument("--model-dir",  default="models")
    parser.add_argument("--epochs",     default=60,  type=int)
    parser.add_argument("--batch-size", default=32,  type=int)
    parser.add_argument("--seed",       default=42,  type=int)
    args = parser.parse_args()

    np.random.seed(args.seed)
    tf.random.set_seed(args.seed)

    data_dir  = Path(args.data_dir)
    model_dir = Path(args.model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)

    # Load data
    X_train, y_train, X_test, y_test = load_split(data_dir)

    # Normalize
    X_train_n, X_test_n, scaler = normalize(X_train, X_test)
    joblib.dump(scaler, model_dir / "scaler.pkl")

    # Baseline RF (raw features — RF is scale-invariant)
    rf, rf_metrics = train_random_forest(X_train, y_train, X_test, y_test)
    joblib.dump(rf, model_dir / "motor_fault_rf.pkl")

    # CNN (normalized features)
    cnn, history, cnn_metrics = train_cnn(
        X_train_n, y_train, X_test_n, y_test,
        epochs=args.epochs, batch_size=args.batch_size)

    # Select best model for deployment
    best = "CNN" if cnn_metrics["cnn_accuracy"] >= rf_metrics["rf_accuracy"] else "RF"
    print(f"\n[DECISION] Selected model for deployment: {best}")

    # Export TFLite if CNN wins
    tflite_path = None
    if best == "CNN":
        tflite_path = export_tflite(cnn, scaler, model_dir)

    # Confusion matrix
    y_pred_cnn = np.argmax(cnn.predict(X_test_n), axis=1)
    save_confusion_matrix(y_test, y_pred_cnn, model_dir)

    # Training report
    report = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "n_train": int(len(X_train)),
        "n_test":  int(len(X_test)),
        "feature_cols": FEATURE_COLS,
        "class_names": CLASS_NAMES,
        "selected_model": best,
        "tflite_path": str(tflite_path) if tflite_path else None,
        **rf_metrics,
        **cnn_metrics,
    }
    report_path = model_dir / "training_report.json"
    report_path.write_text(json.dumps(report, indent=2))
    print(f"\n[Report] -> {report_path}")
    print(f"\n=== Training Complete ===")
    print(f"  RF  Accuracy: {rf_metrics['rf_accuracy']:.4f}")
    print(f"  CNN Accuracy: {cnn_metrics['cnn_accuracy']:.4f}")
    print(f"  Deployed:     {best}")


if __name__ == "__main__":
    main()
