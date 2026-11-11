"""Policies in the robot-calibrated sim (real_calib.py's replica of the board's jduck_run.py loop
+ the fitted servo params): zero-command stand next to the real log, plus (--walk) every walking
direction the board allows (--max-vx 0.08 --max-vy 0.05 --max-wz 0.25, and the full training range).

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_tools/calib_stand_eval.py A.onnx [B.onnx ...] [--log ~/jduck_ops/stand_O1.npz]
"""
import argparse, math, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import real_calib as R, jduck_run, sim_robot as S  # noqa: E402
from calibration import Calibration, load_servo_map  # noqa: E402

# best fit to stand_O1.npz (2026-11-10): friction 0, stall 1.4 N·m, +10 ms latency, damping 0.05
FIT = dict(friction=0.0, stall=1.4, backlash=0.0, extra_delay_ms=10.0, damping=0.05)
# the board's key limits (jduck_run.py --max-vx 0.08 --max-vy 0.05 --max-wz 0.25) and the training range
WALK_CASES = [("前进 0.08", (0.08, 0, 0)), ("后退 0.08", (-0.08, 0, 0)), ("左移 0.05", (0, 0.05, 0)),
              ("右移 0.05", (0, -0.05, 0)), ("左转 0.25", (0, 0, 0.25)), ("右转 0.25", (0, 0, -0.25)),
              ("前进 0.2", (0.2, 0, 0)), ("后退 0.2", (-0.2, 0, 0))]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("onnx", nargs="+", type=Path)
    p.add_argument("--log", type=Path, default=Path.home() / "jduck_ops" / "stand_O1.npz")
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed0", type=int, default=0, help="first seed (re-check with seeds the gate did not use)")
    p.add_argument("--walk", action="store_true", help="also run every walking direction (15 s each)")
    p.add_argument("--stall", type=float, default=None,
                   help="servo stall torque N·m (default = the fit, 1.4). 2026-11-11: O8d passed at 1.4 but "
                        "leaned back / fell at ≤ 0.96 on the robot — gate at both ends of the plausible range")
    a = p.parse_args()
    if a.stall is not None:
        FIT["stall"] = a.stall
    cal = Calibration.load(HERE / "calibration.yaml"); smap = load_servo_map(HERE / "servo_map.yaml")
    real = dict(np.load(a.log, allow_pickle=True))
    wire = float(np.median((real["el"] - real["t_pol"]) / real["active"].sum(1)))
    print(f"{'':28s} " + "  ".join(f"{n:>10s}" for n in R.NAMES))
    print(f"{'真机 ' + a.log.name:28s} " + "  ".join(f"{v:10.3f}" for v in R.feats(real)))
    for path in a.onnx:
        pol = jduck_run.Policy(path)
        fs, fell = [], 0
        for s in range(a.seed0, a.seed0 + a.seeds):
            d = R.run_sim(pol, cal, smap, dict(S.SimRobot.FW_ORIG), FIT["friction"], FIT["stall"], FIT["backlash"],
                          FIT["extra_delay_ms"], wire, seed=s, damping=FIT["damping"])
            if d is None:
                fell += 1
            else:
                fs.append(R.feats(d))
        line = "  ".join(f"{v:10.3f}" for v in np.mean(fs, 0)) if fs else "全摔"
        print(f"{'校准仿真 ' + path.name:28s} {line}" + (f"   摔 {fell}/{a.seeds}" if fell else ""))
        if not a.walk:
            continue
        T = 15.0
        for name, c in WALK_CASES:
            rs = [R.run_sim(pol, cal, smap, dict(S.SimRobot.FW_ORIG), FIT["friction"], FIT["stall"], FIT["backlash"],
                            FIT["extra_delay_ms"], wire, seed=s, damping=FIT["damping"], cmd3=c, T=T) for s in range(a.seed0, a.seed0 + a.seeds)]
            ok = [r for r in rs if r is not None]
            if not ok:
                print(f"      {name:16s} 全摔 {len(rs)}/{len(rs)}"); continue
            dx = np.mean([r["dxy"] for r in ok], 0); yaw = np.mean([r["yaw"] for r in ok])
            ideal = (c[0] * T, c[1] * T, math.degrees(c[2] * T))
            print(f"      {name:16s} dx {dx[0]:+.2f} dy {dx[1]:+.2f} m（理想 {ideal[0]:+.1f},{ideal[1]:+.1f}）"
                  f" 偏航 {yaw:+5.0f}°（理想 {ideal[2]:+.0f}°）" + (f"  摔 {len(rs) - len(ok)}/{len(rs)}" if len(ok) < len(rs) else ""))


if __name__ == "__main__":
    main()
