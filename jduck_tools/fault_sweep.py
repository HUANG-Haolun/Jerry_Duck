"""Which robot-side fault reproduces a symptom? Runs a policy through jduck_run's loop on sim_robot
with one fault at a time, and reports straight-walk drift and whether a turn ends in a fall.

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_tools/fault_sweep.py ../../models/jduck_velstand_d1.onnx [--fw 8,0,0]

Faults (each alone):
  IMU mounted tilted (pitch / roll ±5°), IMU gyro-z sign flipped, IMU x/y axes swapped,
  hip_yaw zero off ±5°, all zeros random ±3°, low torque (battery sag, 75 %),
  firmware K twice the trained value, control loop at 25 / 30 ms, one extra tick of obs delay.
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


def rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])}[axis]


class FaultImu(S.SimImu):
    def __init__(self, robot, R=None, flip_gz=False, swap_xy=False, flip=None):
        super().__init__(robot)
        self.R, self.flip_gz, self.swap_xy, self.flip = R, flip_gz, swap_xy, flip

    def read(self):
        w, g = super().read()
        if self.R is not None:
            w, g = (self.R @ w).astype(np.float32), (self.R @ g).astype(np.float32)
        if self.flip_gz:
            w = w * np.array([1, 1, -1], np.float32)
        if self.flip is not None:          # one IMU axis mapped with the wrong sign (gyro + accel)
            w, g = w * self.flip, g * self.flip
        if self.swap_xy:
            w, g = w[[1, 0, 2]], g[[1, 0, 2]]
        return w, g


def run(policy, cal, smap, fw, cmd3, f, seed=0, T=10.0):
    zo = None
    if f.get("hip_yaw_deg"):
        zo = np.zeros(14); zo[0] = zo[9] = math.radians(f["hip_yaw_deg"])
    if f.get("zero_rand_deg"):
        zo = np.radians(np.random.default_rng(seed + 7).uniform(-f["zero_rand_deg"], f["zero_rand_deg"], 14))
    import copy
    cal_r = cal
    if f.get("flip_sign"):            # the ROBOT's joint turns the other way than calibration.yaml says
        cal_r = copy.deepcopy(cal)
        for j in f["flip_sign"]:
            cal_r.joints[j].sign *= -1
    smap_r = smap
    if f.get("swap_ids"):             # two servos wired to each other's joint
        a_, b_ = f["swap_ids"]; smap_r = dict(smap); smap_r[a_], smap_r[b_] = smap[b_], smap[a_]
    robot = S.SimRobot(cal_r, smap_r, policy.home, fw=dict(fw, K=fw["K"] * f.get("k_mult", 1.0)), seed=seed,
                       stall=0.96 * f.get("torque", 1.0), zero_offset=zo, friction=f.get("friction"))
    if f.get("com_dx") or f.get("com_dy") or f.get("head_mass"):
        import mujoco
        tb = mujoco.mj_name2id(robot.m, mujoco.mjtObj.mjOBJ_BODY, "trunk_base")
        robot.m.body_ipos[tb, 0] += f.get("com_dx", 0.0)
        robot.m.body_ipos[tb, 1] += f.get("com_dy", 0.0)
        if f.get("head_mass"):
            for bname in ("head_pitch_link", "head_yaw_link", "head_roll_link"):
                bid = mujoco.mj_name2id(robot.m, mujoco.mjtObj.mjOBJ_BODY, bname)
                if bid >= 0:
                    robot.m.body_mass[bid] *= f["head_mass"]
                    robot.m.body_inertia[bid] *= f["head_mass"]
        mujoco.mj_setConst(robot.m, robot.d)
    bus = S.SimBus(robot)
    joints = jduck_run.Joints(bus, cal, smap, math.radians(5))
    imu = FaultImu(robot, f.get("R"), f.get("flip_gz", False), f.get("swap_xy", False), f.get("flip"))
    home = policy.home
    cmd = np.zeros(13, np.float32); cmd[:3] = cmd3
    q = joints.move(home); q_old = q.copy()
    qd_f = np.zeros(14, np.float32); last = np.zeros(14, np.float32)
    dt = f.get("dt", jduck_run.CONTROL_DT)
    hist = []
    x0 = robot.d.qpos[:2].copy(); yaw, yp = 0.0, robot.yaw_deg()
    for tick in range(int(T / dt)):
        t_used = robot.d.time
        w, g = imu.read()
        qd_f += policy.qd_ema * ((q - q_old) / dt - qd_f)
        obs = np.concatenate([w, g, q - home, qd_f, last, cmd]).astype(np.float32)
        hist.append(obs)
        if f.get("extra_lag"):
            obs = hist[-2] if len(hist) > 1 else obs
        act = policy(obs).astype(np.float32); last = act
        q_new = joints.move(home + act * policy.action_scale)
        q_old, q = q, q_new
        robot.step(max(0.0, dt - (robot.d.time - t_used)))
        y = robot.yaw_deg(); yaw += (y - yp + 180) % 360 - 180; yp = y
        if robot.tilt_deg() > 50:
            R = robot.d.xmat[1].reshape(3, 3)
            fwd = R[:, 0]                                  # body x (forward) in world
            way = "向后" if fwd[2] > 0.3 else "向前" if fwd[2] < -0.3 else "向侧面"
            return dict(fell=tick * dt, way=way, yaw=yaw)
    dx = robot.d.qpos[:2] - x0
    return dict(fell=None, dx=dx, yaw=yaw)


FAULTS = [
    ("无故障", {}),
    ("IMU 前后装歪 +5°", {"R": rot("y", 5)}),
    ("IMU 前后装歪 −5°", {"R": rot("y", -5)}),
    ("IMU 左右装歪 +5°", {"R": rot("x", 5)}),
    ("IMU 陀螺 z 轴反号", {"flip_gz": True}),
    ("IMU x/y 轴对调", {"swap_xy": True}),
    ("两侧髋 yaw 零位 +5°", {"hip_yaw_deg": 5}),
    ("两侧髋 yaw 零位 −5°", {"hip_yaw_deg": -5}),
    ("所有零位随机 ±3°", {"zero_rand_deg": 3}),
    ("力矩只有 75%（电压低）", {"torque": 0.75}),
    ("固件 K 是训练的 2 倍", {"k_mult": 2.0}),
    ("控制周期 25 ms", {"dt": 0.025}),
    ("控制周期 30 ms", {"dt": 0.030}),
    ("观测多晚一拍", {"extra_lag": True}),
    ("IMU x 轴反号（前后）", {"flip": np.array([-1, 1, 1], np.float32)}),
    ("IMU y 轴反号（左右）", {"flip": np.array([1, -1, 1], np.float32)}),
    ("左髋 yaw 方向反了", {"flip_sign": ["left_hip_yaw"]}),
    ("两侧髋 yaw 方向都反了", {"flip_sign": ["left_hip_yaw", "right_hip_yaw"]}),
    ("左髋 roll 方向反了", {"flip_sign": ["left_hip_roll"]}),
    ("左踝方向反了", {"flip_sign": ["left_ankle"]}),
    ("左右髋 yaw 舵机接反", {"swap_ids": ("left_hip_yaw", "right_hip_yaw")}),
    ("左髋 yaw/roll 舵机接反", {"swap_ids": ("left_hip_yaw", "left_hip_roll")}),
]
COM = [
    ("躯干重心后移 1 cm", {"com_dx": -0.01}),
    ("躯干重心后移 2 cm", {"com_dx": -0.02}),
    ("躯干重心前移 2 cm", {"com_dx": 0.02}),
    ("躯干重心左移 1.5 cm", {"com_dy": 0.015}),
    ("头重 1.3 倍", {"head_mass": 1.3}),
    ("头重 1.3 倍 + 重心后移 1 cm", {"head_mass": 1.3, "com_dx": -0.01}),
    ("重心后移 2 cm + 力矩 85%", {"com_dx": -0.02, "torque": 0.85}),
]
COMBOS = [
    ("关节摩擦 0.05", {"friction": 0.05}),
    ("关节摩擦 0.10", {"friction": 0.10}),
    ("摩擦 0.05 + 力矩 85%", {"friction": 0.05, "torque": 0.85}),
    ("摩擦 0.05 + 零位随机 ±3°", {"friction": 0.05, "zero_rand_deg": 3}),
    ("力矩 85% + 零位随机 ±3°", {"torque": 0.85, "zero_rand_deg": 3}),
    ("力矩 85% + 周期 25 ms + 零位 ±3°", {"torque": 0.85, "dt": 0.025, "zero_rand_deg": 3}),
    ("IMU 前后装歪 5° + 零位 ±3°", {"R": rot("y", 5), "zero_rand_deg": 3}),
]
import os
if os.environ.get("COM"):
    FAULTS = FAULTS[:1] + COM
if os.environ.get("COMBO"):
    FAULTS = FAULTS[:1] + COMBOS
if os.environ.get("ONLY_NEW"):
    FAULTS = FAULTS[:1] + FAULTS[-8:]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("onnx", type=Path)
    p.add_argument("--fw", default="8,0,0")
    p.add_argument("--seeds", type=int, default=2)
    a = p.parse_args()
    if a.fw == "orig":                       # stock firmware: pwm = clamp(|err|, 48, 255), deadband 2
        fw = dict(S.SimRobot.FW_ORIG)
    else:
        v = [float(x) for x in a.fw.split(",")]
        fw = dict(S.SimRobot.FW_PD, K=v[0], FF=v[1], DB=v[2], floor=v[3] if len(v) > 3 else 0.0)
    cal = Calibration.load(HERE / "calibration.yaml")
    smap = load_servo_map(HERE / "servo_map.yaml")
    pol = jduck_run.Policy(a.onnx)
    print(f"=== {a.onnx.name}  fw {a.fw}  每项 {a.seeds} 次 × 10 s")
    print(f"{'故障':22s} | 前进 0.2 m/s：前进 / 侧偏 / 偏航         | 按 a 左转 0.8 rad/s")
    for name, f in FAULTS:
        fr = [run(pol, cal, smap, fw, (0.2, 0, 0), f, s) for s in range(a.seeds)]
        tr = [run(pol, cal, smap, fw, (0, 0, 0.8), f, s) for s in range(a.seeds)]
        ok = [r for r in fr if r["fell"] is None]
        fwd = (f"{np.mean([r['dx'][0] for r in ok]):+.2f} / {np.mean([r['dx'][1] for r in ok]):+.2f} m / "
               f"{np.mean([r['yaw'] for r in ok]):+4.0f}°" if ok else "")
        if len(ok) < len(fr):
            fwd += f"  摔 {len(fr) - len(ok)}/{len(fr)}"
        fell = [r for r in tr if r["fell"] is not None]
        if fell:
            turn = "摔 " + ", ".join(f"{r['fell']:.1f}s {r['way']}（已转 {r['yaw']:+.0f}°）" for r in fell)
        else:
            turn = f"没摔，转了 {np.mean([r['yaw'] for r in tr]):+.0f}°（理想 {math.degrees(0.8 * 10):+.0f}°）"
        print(f"{name:22s} | {fwd:38s} | {turn}", flush=True)


if __name__ == "__main__":
    main()
