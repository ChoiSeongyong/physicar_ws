"""Optional, confidence-gated field-camera steering adviser.

This never owns the vehicle: it provides a bounded blend toward a diagnostic
field-data prediction only when the current image looks statistically like the
training footage. Any unavailable/broken model, unexpected model contract, or
out-of-distribution image returns zero blend, preserving v4 exactly.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


class FieldAdviser:
    def __init__(self, model_path: str, max_blend: float = .25, max_delta_deg: float = 3.0):
        self.session = None
        self.mean = self.std = None
        self.max_blend, self.max_delta_deg = max_blend, max_delta_deg
        try:
            path = Path(model_path)
            meta = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
            self.mean = np.asarray(meta["image_mean_bgr"], dtype=np.float32)
            self.std = np.maximum(np.asarray(meta["image_std_bgr"], dtype=np.float32), .02)
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads, options.inter_op_num_threads = 1, 1
            self.session = ort.InferenceSession(str(path), sess_options=options,
                                                providers=["CPUExecutionProvider"])
            shape = self.session.get_inputs()[0].shape
            if shape[-3:] != [3, 72, 96]:
                self.session = None
        except Exception:
            self.session = None

    @property
    def ready(self):
        return self.session is not None

    def advise(self, image, baseline_deg: float) -> tuple[float, float]:
        """Return (safe steering, blend), defaulting exactly to baseline."""
        if self.session is None or image is None:
            return baseline_deg, 0.0
        try:
            small = cv2.resize(image, (96, 72), interpolation=cv2.INTER_AREA)
            values = small.astype(np.float32) / 255.0
            current_mean = values.mean(axis=(0, 1))
            # Continuous OOD gate: ≥3σ colour shift is no advisory influence.
            z = float(np.mean(np.abs(current_mean - self.mean) / self.std))
            blend = self.max_blend * max(0.0, min(1.0, 1.0 - z / 3.0))
            if blend <= 0.0:
                return baseline_deg, 0.0
            data = values.transpose(2, 0, 1)[None]
            proposed = float(self.session.run(None, {"image": data})[0][0][0]) * 20.0
            # It is an adviser trained on old controller commands, never a full
            # replacement. Bound both the correction and its authority.
            delta = max(-self.max_delta_deg, min(self.max_delta_deg, proposed - baseline_deg))
            return baseline_deg + blend * delta, blend
        except Exception:
            return baseline_deg, 0.0
