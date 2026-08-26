#!/usr/bin/env python3
"""Summarise straight-versus-corner steering behaviour from an autodrive CSV."""
import csv
import sys

path = sys.argv[1]
rows = []
with open(path, newline="") as f:
    for row in csv.DictReader(f):
        try:
            rows.append({k: float(row[k]) if k not in ("source",) and row[k] else 0.0
                         for k in row})
        except ValueError:
            continue

for name, pred in (
    ("straight", lambda r: r["view"] >= .70 and abs(r["slope"]) <= .035 and abs(r["offset"]) <= .16),
    ("corner", lambda r: r["view"] < .55 or abs(r["slope"]) >= .10),
):
    x = [r for r in rows if pred(r)]
    s = [r["steer_cmd_deg"] for r in x]
    flips = sum(a*b < 0 and abs(a) >= 1 and abs(b) >= 1 for a,b in zip(s, s[1:]))
    deltas = [abs(b-a) for a,b in zip(s, s[1:])]
    line = (name, "n", len(x), "duration_s", round(len(x)/15, 1),
            "mean_abs_steer", round(sum(map(abs,s))/len(s), 2),
            "max_abs_steer", round(max(map(abs,s)), 2),
            "sign_flips", flips, "flip_rate_hz", round(flips/(len(s)/15), 2),
            "mean_step_deg", round(sum(deltas)/len(deltas), 2),
            "p95_step_deg", round(sorted(deltas)[int(.95*(len(deltas)-1))], 2))
    if "raw_steer_cmd_deg" in x[0]:
        raw = [r["raw_steer_cmd_deg"] for r in x]
        changed = [abs(a-b) for a,b in zip(s, raw)]
        line += ("filter_changed_pct", round(100 * sum(d > .01 for d in changed)/len(s), 1),
                 "raw_mean_step_deg", round(sum(abs(b-a) for a,b in zip(raw, raw[1:])) / max(1, len(raw)-1), 2))
    print(*line)

# Stable straight runs make the noise/alternation visible without joining gaps.
runs, run = [], []
for r in rows:
    ok = r["view"] >= .70 and abs(r["slope"]) <= .035 and abs(r["offset"]) <= .16
    if ok:
        run.append(r)
    elif run:
        if len(run) >= 20: runs.append(run)
        run=[]
if len(run) >= 20: runs.append(run)
print("stable_straight_runs", len(runs))
for i, run in enumerate(sorted(runs, key=len, reverse=True)[:5], 1):
    s = [r["steer_cmd_deg"] for r in run]
    print("run", i, "t", f'{run[0]["elapsed_s"]:.2f}-{run[-1]["elapsed_s"]:.2f}',
          "n", len(run), "steer", " ".join(f"{v:+.1f}" for v in s[:30]))
