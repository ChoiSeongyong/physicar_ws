#!/usr/bin/env python3
"""Train a compact field-camera steering adviser from dataset/diag.

The diagnostic captures are recorded every fourth controller tick.  For a debug
frame fNNNNN, its label is telemetry row NNNNN*4 from the same session.  This
program deliberately trains an *adviser*, not a replacement driving policy:
its output is consumed only by autodrive_trained.py behind an OOD/confidence
safety gate, and the original v4 controller remains the fallback.
"""
from __future__ import annotations

import argparse
import csv
import json
import random
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

ROOT = Path(__file__).resolve().parents[1]


class AdviserNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 5, stride=2, padding=2), nn.ReLU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(32, 48, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.head = nn.Sequential(nn.Flatten(), nn.Linear(48, 32), nn.ReLU(),
                                  nn.Linear(32, 1), nn.Tanh())

    def forward(self, image):
        return self.head(self.features(image))


def image_tensor(path: Path) -> np.ndarray | None:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        return None
    image = cv2.resize(image, (96, 72), interpolation=cv2.INTER_AREA)
    return image.transpose(2, 0, 1).astype(np.float32) / 255.0


def load_sessions(root: Path):
    samples, skipped = [], 0
    for session in sorted(root.glob("*/telemetry.csv")):
        rows = list(csv.DictReader(session.open(encoding="utf-8")))
        folder = session.parent
        # Raw frames are unannotated originals; debug f*.jpg can include masks.
        # Use raw where present so deployment sees the same camera distribution.
        for frame in sorted(folder.glob("f*.jpg")):
            try:
                index = int(frame.stem[1:])
                row = rows[index * 4]
                steer = float(row["steer"])
            except (ValueError, IndexError, KeyError):
                skipped += 1
                continue
            raw = folder / "raw" / f"r{index:05d}.png"
            image = image_tensor(raw if raw.exists() else frame)
            if image is None or not np.isfinite(steer) or abs(steer) > 20.0:
                skipped += 1
                continue
            samples.append((image, steer / 20.0, str(folder.name)))
    return samples, skipped


class Samples(Dataset):
    def __init__(self, samples): self.samples = samples
    def __len__(self): return len(self.samples)
    def __getitem__(self, i):
        image, target, _ = self.samples[i]
        return torch.from_numpy(image), torch.tensor([target], dtype=torch.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=ROOT / "dataset" / "diag")
    parser.add_argument("--out", type=Path, default=ROOT / "models" / "v4_field_adviser.onnx")
    parser.add_argument("--epochs", type=int, default=120)
    args = parser.parse_args()

    random.seed(20260826); np.random.seed(20260826); torch.manual_seed(20260826)
    samples, skipped = load_sessions(args.data)
    if len(samples) < 80:
        raise SystemExit(f"학습 샘플 부족: {len(samples)}개")
    # Preserve the latest field-HSV session as the held-out session whenever it
    # exists. This prevents adjacent near-identical video frames leaking into
    # validation.
    names = sorted({s[2] for s in samples})
    valid_name = next((n for n in names if "010952" in n), names[-1])
    train = [s for s in samples if s[2] != valid_name]
    valid = [s for s in samples if s[2] == valid_name]
    if len(train) < 40 or len(valid) < 20:
        random.shuffle(samples); cut = int(len(samples) * .8); train, valid = samples[:cut], samples[cut:]
        valid_name = "random-fallback"
    model = AdviserNet()
    opt = torch.optim.AdamW(model.parameters(), lr=2e-3, weight_decay=2e-4)
    loss_fn = nn.SmoothL1Loss(beta=.12)
    loader = DataLoader(Samples(train), batch_size=32, shuffle=True)
    best, best_state = float("inf"), None
    for epoch in range(args.epochs):
        model.train()
        for x, y in loader:
            # modest photometric augmentation for exposure variation, not flips:
            # a horizontal flip would require changing the steering sign.
            x = torch.clamp(x * (0.85 + .30 * torch.rand(x.shape[0], 1, 1, 1)), 0, 1)
            opt.zero_grad(); loss = loss_fn(model(x), y); loss.backward(); opt.step()
        model.eval()
        with torch.no_grad():
            errs = [(model(x.unsqueeze(0)).item() - y.item()) ** 2 for x, y in Samples(valid)]
        mse = float(np.mean(errs))
        if mse < best:
            best, best_state = mse, {k: v.detach().clone() for k, v in model.state_dict().items()}
        if epoch in (0, args.epochs - 1) or epoch % 30 == 29:
            print(f"epoch {epoch + 1:3d}: validation RMSE={best ** .5 * 20:.3f} deg")
    model.load_state_dict(best_state); model.eval()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    example = torch.zeros((1, 3, 72, 96), dtype=torch.float32)
    # Torch 2.6 defaults to the new exporter, which requires optional
    # `onnxscript`. The installed offline image intentionally omits it; the
    # mature legacy exporter needs only the already-installed `onnx` package.
    torch.onnx.export(model, example, args.out, input_names=["image"], output_names=["steer"],
                      dynamic_axes={"image": {0: "batch"}, "steer": {0: "batch"}},
                      opset_version=17, dynamo=False)
    # Image colour envelope is a deterministic OOD gate. It makes the adviser
    # inert on SIM or any visually unlike surface rather than extrapolating.
    all_images = np.stack([s[0] for s in train])
    means = all_images.mean(axis=(0, 2, 3)); stds = all_images.std(axis=(0, 2, 3))
    meta = {"samples": len(samples), "train": len(train), "valid": len(valid),
            "validation_session": valid_name, "skipped": skipped,
            "valid_rmse_deg": round(best ** .5 * 20, 4),
            "image_mean_bgr": means.round(6).tolist(), "image_std_bgr": stds.round(6).tolist(),
            "model_input": [1, 3, 72, 96], "label": "diagnostic controller steer, degrees"}
    args.out.with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(meta, ensure_ascii=False))


if __name__ == "__main__": main()
