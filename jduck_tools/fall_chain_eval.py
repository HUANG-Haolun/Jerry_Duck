"""Fall + get-up through the robot's control loop (jduck_run.py on sim_robot.py, groundcontact model).

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_tools/fall_chain_eval.py WALK.onnx [--getup GETUP.onnx] [--fw 8,0,0]

Two batteries, each over several seeds:
  1. start lying (face down / face up / on the side), zero command, 15 s: does it end upright?
  2. walk forward 0.15 m/s for 30 s with a hard shove every 6 s: how many shoves knock it over,
     how many of those does it get back up from, and does it end upright?
With --getup, jduck_run.PolicySwitch picks the policy each tick exactly as on the robot;
without it the walk policy runs alone (a VelStand policy should do both).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import jduck_run  # noqa: E402
import sim_robot as S  # noqa: E402
from calibration import Calibration, load_servo_map  # noqa: E402

UP_TILT, UP_Z = 25.0, 0.115


def lie(robot, kind, rng):
    """Put the robot on the floor face down / face up / on its side, joints at HOME, and let it settle."""
    yaw = rng.uniform(-math.pi, math.pi)
    qy = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    base = {"face_down": (0.7071, 0, 0.7071, 0), "face_up": (0.7071, 0, -0.7071, 0),
            "side": (0.7071, 0.7071, 0, 0)}[kind]
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, qy, np.array(base))
    robot.d.qpos[:3] = [0, 0, 0.15]
    robot.d.qpos[3:7] = q
    robot.d.qvel[:] = 0
    mujoco.mj_forward(robot.m, robot.d)


class Loop:
    """jduck_run.py's control loop, one tick at a time."""

    def __init__(self, walk, getup, cal, smap, fw, seed):
        self.robot = S.SimRobot(cal, smap, walk.home, fw=fw, seed=seed, scene=S.SCENE_GC)
        self.bus = S.SimBus(self.robot)
        self.joints = jduck_run.Joints(self.bus, cal, smap, math.radians(5))
        self.imu = S.SimImu(self.robot)
        self.walk, self.home = walk, walk.home
        self.switch = jduck_run.PolicySwitch(walk, getup) if getup else None
        self.cmd = np.zeros(13, np.float32)
        self.last = np.zeros(14, np.float32)
        self.qd_f = np.zeros(14, np.float32)
        self.t = 0.0
        self.q = self.joints.move(self.home)
        self.q_old = self.q.copy()

    def tick(self):
        dt = jduck_run.CONTROL_DT
        t_used = self.robot.d.time
        w, g = self.imu.read()
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(g[2])))))
        self.qd_f += self.walk.qd_ema * ((self.q - self.q_old) / dt - self.qd_f)
        obs = np.concatenate([w, g, self.q - self.home, self.qd_f, self.last, self.cmd]).astype(np.float32)
        pol = self.walk
        if self.switch is not None:
            pol = self.switch.select(tilt, self.q - self.home, self.t)
            if pol is self.switch.getup:
                obs[48:] = 0.0
        act = pol(obs).astype(np.float32)
        self.last = act
        q_new = self.joints.move(self.home + act * pol.action_scale)
        self.q_old, self.q = self.q, q_new
        self.robot.step(max(0.0, dt - (self.robot.d.time - t_used)))
        self.t += dt

    def upright(self):
        return self.robot.tilt_deg() < UP_TILT and self.robot.d.qpos[2] > UP_Z


def main():
    p = argparse.ArgumentParser()
    p.add_argument("walk", type=Path)
    p.add_argument("--getup", type=Path, default=None)
    p.add_argument("--fw", default="8,0,0")
    p.add_argument("--seeds", type=int, default=4)
    p.add_argument("--shove", type=float, default=0.9, help="shove: trunk velocity kick, m/s")
    a = p.parse_args()
    v = [float(x) for x in a.fw.split(",")]
    fw = dict(S.SimRobot.FW_PD, K=v[0], FF=v[1], DB=v[2], floor=v[3] if len(v) > 3 else 0.0)
    cal = Calibration.load(HERE / "calibration.yaml")
    smap = load_servo_map(HERE / "servo_map.yaml")
    walk = jduck_run.Policy(a.walk)
    getup = jduck_run.Policy(a.getup) if a.getup else None
    print(f"=== 走路 {a.walk.name}" + (f" + 起身 {a.getup.name}（摔倒后切换）" if getup else "（单策略）")
          + f"  [真机控制链路 + 贴地碰撞模型, fw {a.fw}]")
    dt = jduck_run.CONTROL_DT

    for kind in ("face_down", "face_up", "side"):
        ok, times = 0, []
        for s in range(a.seeds):
            L = Loop(walk, getup, cal, smap, fw, s)
            lie(L.robot, kind, np.random.default_rng(100 + s))
            L.robot.step(0.3)
            L.q = L.joints.move(L.home)
            L.q_old = L.q.copy()
            t_up = None
            for i in range(int(15 / dt)):
                L.tick()
                if t_up is None and L.upright():
                    t_up = L.t
            if L.upright():
                ok += 1
                times.append(t_up)
        tt = f"  中位用时 {np.median(times):.1f} s" if times else ""
        print(f"  躺着起身 {kind:9s} 最后站着 {ok}/{a.seeds}{tt}")

    tot_falls = tot_recov = end_up = 0
    dist = []
    for s in range(a.seeds):
        rng = np.random.default_rng(200 + s)
        L = Loop(walk, getup, cal, smap, fw, s)
        L.cmd[0] = 0.15
        down_since, falls, recov = None, 0, 0
        x0 = L.robot.d.qpos[0]
        for i in range(int(30 / dt)):
            if i > 0 and i % int(6 / dt) == 0:
                ang = rng.uniform(-math.pi, math.pi)
                L.robot.d.qvel[0] += a.shove * math.cos(ang)
                L.robot.d.qvel[1] += a.shove * math.sin(ang)
            L.tick()
            fallen = L.robot.tilt_deg() > 60
            if fallen and down_since is None:
                down_since, falls = L.t, falls + 1
            if down_since is not None and L.upright() and L.t - down_since > 0.3:
                recov, down_since = recov + 1, None
        tot_falls += falls
        tot_recov += recov
        end_up += L.upright()
        dist.append(L.robot.d.qpos[0] - x0)
    print(f"  走路 0.15 m/s + 每 6 s 一推（{a.shove} m/s）×30 s：摔倒 {tot_falls} 次，自己爬起 {tot_recov} 次，"
          f"结束时站着 {end_up}/{a.seeds}，平均前进 {np.mean(dist):+.2f} m")


if __name__ == "__main__":
    main()
