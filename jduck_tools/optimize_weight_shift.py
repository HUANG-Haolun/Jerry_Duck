"""Random-search contact-preserving left weight-shift poses in MuJoCo."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sim_robot as S  # noqa: E402
from calibration import JOINTS, Calibration, load_servo_map  # noqa: E402
from servo_bus import CMD_MOVE  # noqa: E402

DEG = math.pi / 180


def main() -> None:
    ap = argparse.ArgumentParser(description="搜索左侧重心偏移且双脚保持接触的关节姿态")
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--seconds", type=float, default=0.6)
    ap.add_argument("--fw-k", type=float, default=16)
    ap.add_argument("--support", choices=("left", "right", "double"), default="left")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    cal = Calibration.load(HERE / "calibration.yaml")
    smap = load_servo_map(HERE / "servo_map.yaml")
    best = []
    names = ("left_hip_roll", "right_hip_roll", "left_hip_pitch", "right_hip_pitch",
             "left_knee", "right_knee", "left_ankle", "right_ankle")
    idx = [JOINTS.index(n) for n in names]
    ranges = np.array([[4, 22], [4, 22], [-12, 12], [-12, 12],
                       [-12, 20], [-12, 20], [-20, 20], [-20, 20]], dtype=float)
    dt = 0.01
    for trial in range(a.trials):
        # Leftward pelvis shift uses positive bilateral hip roll in the MJCF convention.
        params = rng.uniform(ranges[:, 0], ranges[:, 1])
        params[1] = np.clip(params[0] + rng.normal(0, 4), *ranges[1])
        q = np.zeros(len(JOINTS))
        q[idx] = params * DEG
        fw = dict(S.SimRobot.FW_PD, K=a.fw_k)
        robot = S.SimRobot(cal, smap, np.zeros(len(JOINTS)), fw=fw)
        nsteps = round(a.seconds / dt)
        tilt_max = 0.0
        for _ in range(nsteps):
            for i, name in enumerate(JOINTS):
                adc = int(round(cal.joints[name].to_adc(float(q[i]), cal.k_rad_per_count)))
                robot.command(smap[name], CMD_MOVE, adc)
            robot.step(dt)
            tilt_max = max(tilt_max, robot.tilt_deg())
        d = robot.d
        lz, rz = float(d.site("left_foot").xpos[2]), float(d.site("right_foot").xpos[2])
        com_y = float(d.subtree_com[1][1])
        ly, ry = float(d.site("left_foot").xpos[1]), float(d.site("right_foot").xpos[1])
        if a.support == "left":
            # Left sole must stay on the floor, right foot should clear it, COM over left sole.
            score = (abs(lz) * 220 + abs(rz - 0.008) * 140
                     + max(0.0, abs(com_y - ly) - 0.018) * 150
                     + max(0.0, tilt_max - 12) * 0.04)
            contact = abs(lz) <= 0.003 and rz >= 0.005 and abs(com_y - ly) <= 0.021 and tilt_max <= 12
        elif a.support == "right":
            score = (abs(rz) * 220 + abs(lz - 0.008) * 140
                     + max(0.0, abs(com_y - ry) - 0.018) * 150
                     + max(0.0, tilt_max - 12) * 0.04)
            contact = abs(rz) <= 0.003 and lz >= 0.005 and abs(com_y - ry) <= 0.021 and tilt_max <= 12
        else:
            score = (max(0.0, 0.03 - com_y) * 100 + abs(lz) * 180 + abs(rz) * 180
                     + max(0.0, tilt_max - 8) * 0.02)
            contact = abs(lz) <= 0.003 and abs(rz) <= 0.003
        candidate = (score, trial, params.copy(), com_y, ly, ry, lz, rz, tilt_max, contact)
        best.append(candidate)
        if trial % 10 == 0 or contact:
            print(f"{trial:03d}: score={score:.3f} COMy={com_y:+.3f}m feet=({lz:+.3f},{rz:+.3f})m "
                  f"tilt_max={tilt_max:.1f} contact={contact}", flush=True)
    print("\nTop candidates (left hip roll, right hip roll, hip pitch L/R, knee L/R, ankle L/R; degrees):")
    for score, trial, params, com_y, ly, ry, lz, rz, tilt, contact in sorted(best, key=lambda x: x[0])[:8]:
        print(f"score={score:.3f} contact={contact} COMy={com_y:+.4f} feet=({lz:+.4f},{rz:+.4f}) "
              f"tilt_max={tilt:.1f} q={np.round(params, 1).tolist()}")


if __name__ == "__main__":
    main()
