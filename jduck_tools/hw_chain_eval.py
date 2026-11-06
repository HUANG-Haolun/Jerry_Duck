"""Judge a policy the way the robot runs it: jduck_run.py's control loop on sim_robot.py.

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_tools/hw_chain_eval.py jduck_xxx.onnx [--fw 16,0,0]

jduck_eval.py feeds the policy training-style observations (true q, true qd). That
hid the real problem: on the robot q is ADC-quantized and qd is a 20 ms finite
difference, and policies that stood perfectly still in jduck_eval jittered here.
Each case runs 15 s, 3 seeds (different ADC phase via the sim's start pose noise). Jitter is
measured over 5–15 s: r2 looked quiet for the first few seconds and only then started to jitter.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import jduck_run  # noqa: E402
import sim_robot as S  # noqa: E402
from calibration import Calibration, load_servo_map  # noqa: E402

LEGS = [*range(0, 5), *range(9, 14)]
CASES = [("零速", 0, 0, 0), ("前进 0.2", 0.2, 0, 0), ("后退 0.2", -0.2, 0, 0),
         ("侧走 0.12", 0, 0.12, 0), ("原地转 0.8", 0, 0, 0.8), ("边走边转", 0.15, 0, 0.5)]


def run(policy, cal, smap, fw, cmd3, seed, T=15.0):
    robot = S.SimRobot(cal, smap, policy.home, fw=fw, seed=seed)
    rng = np.random.default_rng(seed)
    robot.d.qpos[robot.qa] += rng.uniform(-0.01, 0.01, len(robot.qa))   # vary ADC phase / start
    bus = S.SimBus(robot)
    joints = jduck_run.Joints(bus, cal, smap, math.radians(5))
    imu = S.SimImu(robot)
    home = policy.home
    cmd = np.zeros(13, np.float32)
    cmd[:3] = cmd3
    q = joints.move(home)
    q_old = q.copy()
    qd_f = np.zeros(14, np.float32)
    last = np.zeros(14, np.float32)
    dt = jduck_run.CONTROL_DT
    x0 = robot.d.qpos[:2].copy()
    yaw, yp = 0.0, robot.yaw_deg()
    tp, dts, qds = None, [], []
    for tick in range(int(T / dt)):
        t_used = robot.d.time
        w, g = imu.read()
        qd_f += policy.qd_ema * ((q - q_old) / dt - qd_f)    # same filter as jduck_run.py
        obs = np.concatenate([w, g, q - home, qd_f, last, cmd]).astype(np.float32)
        act = policy(obs).astype(np.float32)
        last = act
        tgt = home + act * policy.action_scale
        q_new = joints.move(tgt)
        q_old, q = q, q_new
        robot.step(max(0.0, dt - (robot.d.time - t_used)))
        y = robot.yaw_deg()
        yaw += (y - yp + 180.0) % 360.0 - 180.0
        yp = y
        if tick * dt > 5.0:
            if tp is not None:
                dts.append(np.degrees(np.abs(tgt - tp)[LEGS]).mean())
            qds.append(np.abs(robot.d.qvel[robot.va][LEGS]).mean())
        tp = tgt
        if robot.tilt_deg() > 50:
            return None
    dx = robot.d.qpos[:2] - x0
    return dict(dx=dx, yaw=yaw, dtgt=float(np.mean(dts)), qd=float(np.mean(qds)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("onnx", nargs="+", type=Path)
    p.add_argument("--fw", default="16,0,0")
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--k-sweep", default="12,20", help="extra firmware K values for the zero-command check ('' = off)")
    a = p.parse_args()
    k, ff, db = (float(x) for x in a.fw.split(","))
    fw = dict(S.SimRobot.FW_PD, K=k, FF=ff, DB=db)
    cal = Calibration.load(HERE / "calibration.yaml")
    smap = load_servo_map(HERE / "servo_map.yaml")
    for path in a.onnx:
        policy = jduck_run.Policy(path)
        print(f"\n=== {path.name}  [真机控制链路, fw {a.fw}, {a.seeds} 次 × 15 s，抖动看 5–15 s]")
        for name, vx, vy, wz in CASES:
            rs = [run(policy, cal, smap, fw, (vx, vy, wz), s) for s in range(a.seeds)]
            ok = [r for r in rs if r]
            fell = len(rs) - len(ok)
            if not ok:
                print(f"  {name:10s} 全摔")
                continue
            dx = np.mean([r["dx"] for r in ok], axis=0)
            yaw = np.mean([r["yaw"] for r in ok])
            line = (f"  {name:10s} dx={dx[0]:+.2f} dy={dx[1]:+.2f} m (理想 {vx * 15:+.1f},{vy * 15:+.1f})  "
                    f"yaw {yaw:+5.0f}° (理想 {math.degrees(wz * 15):+.0f}°)")
            if name == "零速":
                line += (f"  | 目标每步变化 中位 {np.median([r['dtgt'] for r in ok]):.3f}° 最大 {max(r['dtgt'] for r in ok):.3f}°"
                         f"  腿|qd| 中位 {np.median([r['qd'] for r in ok]):.4f}")
            if fell:
                line += f"  摔 {fell}/{len(rs)}"
            print(line)
        for kk in [float(x) for x in a.k_sweep.split(",") if x]:
            fwk = dict(fw, K=kk)
            rs = [run(policy, cal, smap, fwk, (0, 0, 0), s) for s in range(a.seeds)]
            ok = [r for r in rs if r]
            if not ok:
                print(f"  零速 K={kk:g}   全摔")
                continue
            print(f"  零速 K={kk:g}   目标每步变化 中位 {np.median([r['dtgt'] for r in ok]):.3f}° 最大 {max(r['dtgt'] for r in ok):.3f}°"
                  f"  腿|qd| 中位 {np.median([r['qd'] for r in ok]):.4f}" + (f"  摔 {len(rs) - len(ok)}/{len(rs)}" if len(ok) < len(rs) else ""))


if __name__ == "__main__":
    main()
