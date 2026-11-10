"""Zero-command stand of one or more policies in the robot-calibrated sim (real_calib.py's
replica of the board's jduck_run.py loop + the fitted servo params), next to the real log.

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


def main():
    p = argparse.ArgumentParser()
    p.add_argument("onnx", nargs="+", type=Path)
    p.add_argument("--log", type=Path, default=Path.home() / "jduck_ops" / "stand_O1.npz")
    p.add_argument("--seeds", type=int, default=3)
    a = p.parse_args()
    cal = Calibration.load(HERE / "calibration.yaml"); smap = load_servo_map(HERE / "servo_map.yaml")
    real = dict(np.load(a.log, allow_pickle=True))
    wire = float(np.median((real["el"] - real["t_pol"]) / real["active"].sum(1)))
    print(f"{'':28s} " + "  ".join(f"{n:>10s}" for n in R.NAMES))
    print(f"{'真机 ' + a.log.name:28s} " + "  ".join(f"{v:10.3f}" for v in R.feats(real)))
    for path in a.onnx:
        pol = jduck_run.Policy(path)
        fs, fell = [], 0
        for s in range(a.seeds):
            d = R.run_sim(pol, cal, smap, dict(S.SimRobot.FW_ORIG), FIT["friction"], FIT["stall"], FIT["backlash"],
                          FIT["extra_delay_ms"], wire, seed=s, damping=FIT["damping"])
            if d is None:
                fell += 1
            else:
                fs.append(R.feats(d))
        line = "  ".join(f"{v:10.3f}" for v in np.mean(fs, 0)) if fs else "全摔"
        print(f"{'校准仿真 ' + path.name:28s} {line}" + (f"   摔 {fell}/{a.seeds}" if fell else ""))


if __name__ == "__main__":
    main()
