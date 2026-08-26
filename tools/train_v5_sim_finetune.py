#!/usr/bin/env python3
"""Fine-tune the v4 steering-adviser architecture on SIM diversity captures.

The v4 ONNX artifact is inference-only, so this intentionally uses the same
compact architecture and v4's safety-gated deployment contract rather than
claiming to recover unavailable optimizer state. It validates by whole driving
profile, preventing adjacent video frames from leaking into validation.
"""
from __future__ import annotations
import argparse, csv, json, random
from pathlib import Path
import cv2, numpy as np, torch
import onnx
from torch import nn
from torch.utils.data import Dataset, DataLoader
ROOT = Path(__file__).resolve().parents[1]

class AdviserNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.features=nn.Sequential(nn.Conv2d(3,16,5,2,2),nn.ReLU(),nn.Conv2d(16,32,3,2,1),nn.ReLU(),nn.Conv2d(32,48,3,2,1),nn.ReLU(),nn.AdaptiveAvgPool2d((1,1)))
        self.head=nn.Sequential(nn.Flatten(),nn.Linear(48,32),nn.ReLU(),nn.Linear(32,1),nn.Tanh())
    def forward(self,x): return self.head(self.features(x))

def load(root):
    rows=[]
    for label in sorted(root.glob('*/labels.csv')):
        for r in csv.DictReader(label.open(encoding='utf-8')):
            path=label.parent/'frames'/r['frame']; image=cv2.imread(str(path))
            if image is None: continue
            image=cv2.resize(image,(96,72),interpolation=cv2.INTER_AREA).transpose(2,0,1).astype(np.float32)/255.
            rows.append((image,float(r['steer_deg'])/20.,r['profile']))
    return rows
class Data(Dataset):
    def __init__(self,rows): self.rows=rows
    def __len__(self): return len(self.rows)
    def __getitem__(self,i):
        x,y,_=self.rows[i]; return torch.from_numpy(x),torch.tensor([y],dtype=torch.float32)

def main():
    p=argparse.ArgumentParser(); p.add_argument('--data',type=Path,default=ROOT/'dataset'/'v5_sim'); p.add_argument('--out',type=Path,default=ROOT/'models'/'v5_sim_adviser.onnx'); p.add_argument('--epochs',type=int,default=100); a=p.parse_args()
    random.seed(20260827); np.random.seed(20260827); torch.manual_seed(20260827)
    rows=load(a.data)
    if len(rows)<200: raise SystemExit(f'not enough samples: {len(rows)}')
    holdout='right_to_left' if any(r[2]=='right_to_left' for r in rows) else sorted({r[2] for r in rows})[-1]
    train=[r for r in rows if r[2]!=holdout]; valid=[r for r in rows if r[2]==holdout]
    model=AdviserNet(); opt=torch.optim.AdamW(model.parameters(),lr=1e-3,weight_decay=2e-4); loss_fn=nn.SmoothL1Loss(beta=.10); best=float('inf'); state=None
    for epoch in range(a.epochs):
        model.train()
        for x,y in DataLoader(Data(train),batch_size=32,shuffle=True):
            x=torch.clamp(x*(.88+.24*torch.rand(x.shape[0],1,1,1)),0,1); opt.zero_grad(); loss_fn(model(x),y).backward(); opt.step()
        model.eval()
        with torch.no_grad():
            squared = []
            for x, y in DataLoader(Data(valid), batch_size=64):
                squared.append(((model(x) - y) ** 2).flatten())
            mse = float(torch.cat(squared).mean())
        if mse < best:
            best = mse
            state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        if epoch in (0,a.epochs-1) or epoch%25==24: print(f'epoch {epoch+1}: validation RMSE={best**.5*20:.3f} deg')
    model.load_state_dict(state); model.eval(); a.out.parent.mkdir(parents=True,exist_ok=True)
    example = torch.zeros((1, 3, 72, 96), dtype=torch.float32)
    torch.onnx.export(model, example, a.out, input_names=['image'], output_names=['steer'],
                      dynamic_axes={'image': {0: 'batch'}, 'steer': {0: 'batch'}},
                      opset_version=17, dynamo=False)
    arr = np.stack([r[0] for r in train])
    meta = {
        'base': 'v4 adviser architecture; SIM diversity fine-tune',
        'samples': len(rows), 'train': len(train), 'valid': len(valid),
        'validation_profile': holdout, 'valid_rmse_deg': round(best ** .5 * 20, 4),
        'image_mean_bgr': arr.mean(axis=(0, 2, 3)).round(6).tolist(),
        'image_std_bgr': np.clip(arr.std(axis=(0, 2, 3)), .02, None).round(6).tolist(),
        'model_input': [1, 3, 72, 96], 'label': 'SIM route teacher steering, degrees',
    }
    a.out.with_suffix('.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(json.dumps(meta))
if __name__=='__main__': main()
