"""Grid-search conservative traditional-gait pose parameters in MuJoCo."""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM = HERE / "traditional_gait_sim.py"


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--shift", default="0,1,2,3,4,5,6")
    p.add_argument("--length", default="0,0.002,0.004,0.006")
    p.add_argument("--height", default="0,0.001,0.002,0.003,0.004")
    p.add_argument("--move", type=float, default=0.8)
    p.add_argument("--fall-deg", type=float, default=45)
    p.add_argument("--max-tilt-deg", type=float, default=12)
    p.add_argument("--min-clearance", type=float, default=0.001)
    p.add_argument("--limit", type=int, default=0, help="最多测试组合数，0=全部")
    a = p.parse_args()
    shifts = [float(x) for x in a.shift.split(",")]
    lengths = [float(x) for x in a.length.split(",")]
    heights = [float(x) for x in a.height.split(",")]
    candidates = []
    combos = [(s, x, z) for s in shifts for x in lengths for z in heights]
    if a.limit:
        combos = combos[:a.limit]
    with tempfile.TemporaryDirectory(prefix="jduck-gait-sweep-") as td:
        for n, (shift, length, height) in enumerate(combos, 1):
            out = Path(td) / f"{n}.csv"
            cmd = [sys.executable, str(SIM), "--action", "cycle", "--move", str(a.move), "--hold", "0",
                   "--shift-deg", str(shift), "--step-length", str(length), "--step-height", str(height),
                   "--fall-deg", str(a.fall_deg), "--csv", str(out)]
            env = dict(os.environ, PYTHONUTF8="1")
            run = subprocess.run(cmd, cwd=HERE.parent, env=env, text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if run.returncode or "摔倒阈值触发" in run.stdout or not out.exists():
                print(f"{n:03d}/{len(combos)} FALL shift={shift:g} length={length:g} height={height:g}")
                continue
            rows = list(csv.DictReader(out.open(encoding="utf-8")))
            tilt = max(float(r["tilt_deg"]) for r in rows)
            # Foot sites are near zero while planted; only count height beyond 1 mm.
            clearance = max(max(float(r["left_foot_z_m"]), float(r["right_foot_z_m"])) for r in rows)
            stable = tilt <= a.max_tilt_deg and (height == 0 or clearance >= a.min_clearance)
            print(f"{n:03d}/{len(combos)} {'PASS' if stable else 'MISS'} shift={shift:g} length={length:g} "
                  f"height={height:g} max_tilt={tilt:.1f} clearance={clearance:.4f}")
            if stable:
                candidates.append((tilt, shift, length, height, clearance))
    print("\nCandidates (lowest tilt first):")
    for tilt, shift, length, height, clearance in sorted(candidates)[:10]:
        print(f"shift={shift:g} deg, step_length={length:g} m, step_height={height:g} m, "
              f"max_tilt={tilt:.2f} deg, clearance={clearance:.4f} m")
    if not candidates:
        print("No stable parameter set found; revise the weight-shift/controller model before tuning gait amplitude.")


if __name__ == "__main__":
    main()
