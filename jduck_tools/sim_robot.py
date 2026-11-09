"""MuJoCo stand-in for the real jduck, behind the same interfaces as the hardware.

`SimBus` answers UBT-12HC frames the way the firmware does. It reports the ADC
position (through calibration.yaml) and runs the firmware law at 1 kHz with a DC
motor. Default = what the current policies are trained on: firmware/ubt12hc_pd at
K=16 FF=0 DB=0, 0.96 N·m, 4.8 rad/s (official 7.4 V spec).
`SimImu` returns the trunk gyro and gravity. Use them to test jduck_run.py
without hardware:

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_tools/sim_robot.py --policy jduck_r7_old_quiet4s1_seed2.onnx
    uv run --frozen python ../../jduck_tools/sim_robot.py --policy ... --vx 0.2 --fw 16,0,1

Uses mujoco from the microduck_rl venv.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibration import JOINTS, Calibration, load_servo_map  # noqa: E402
from servo_bus import CMD_HOLD, CMD_MOVE, CMD_RELAX, Reply  # noqa: E402

SCENE = HERE.parent / "jduck_mjcf" / "scene_walk.xml"
SCENE_GC = HERE.parent / "jduck_mjcf" / "scene_groundcontact.xml"   # body also collides with the floor


class SimRobot:
    # Firmware laws: pwm = clamp(K*|err| + FF, floor, 255), 0 if |err| <= DB.
    FW_ORIG = dict(K=1, FF=0, floor=48, DB=2, timeout_ms=0)        # send_rev_230400_ult.ino
    FW_PD = dict(K=16, FF=0, floor=0, DB=0, timeout_ms=200, params=True)   # firmware/ubt12hc_pd defaults

    def __init__(self, cal: Calibration, servo_map: dict[str, int], home_q: np.ndarray,
                 stall=0.96, no_load=4.8, seed=0, fw=None, friction=None, scene=None):
        self.m = mujoco.MjModel.from_xml_path(str(scene or SCENE))
        self.m.opt.timestep = 0.001                     # = firmware rate
        self.d = mujoco.MjData(self.m)
        self.qa = np.array([self.m.jnt_qposadr[self.m.joint(j).id] for j in JOINTS])
        self.va = np.array([self.m.jnt_dofadr[self.m.joint(j).id] for j in JOINTS])
        if friction is not None:                        # gearbox Coulomb friction, N·m (JDUCK_FRICTION)
            self.m.dof_frictionloss[self.va] = friction
        self.k = cal.k_rad_per_count
        self.cal = [cal.joints[j] for j in JOINTS]
        self.by_id = {servo_map[j]: i for i, j in enumerate(JOINTS)}
        self.stall, self.no_load = stall, no_load
        self.fw = dict(fw or self.FW_PD)
        n = len(JOINTS)
        self.last_cmd_t = np.zeros(n)
        self.enabled = np.zeros(n, bool)
        self.target_adc = np.zeros(n)
        self.prev_cmd = np.zeros(n, int)
        # place at HOME, soles on the floor
        d = self.d
        d.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
        d.qpos[self.qa] = home_q
        mujoco.mj_forward(self.m, d)
        sole = min(d.site("left_foot").xpos[2], d.site("right_foot").xpos[2])
        d.qpos[2] = 0.5 - sole + 0.001
        mujoco.mj_forward(self.m, d)
        self.rng = np.random.default_rng(seed)

    def adc(self, i: int) -> int:
        return int(round(self.cal[i].to_adc(float(self.d.qpos[self.qa[i]]), self.k)))

    def step(self, seconds: float) -> None:
        for _ in range(max(1, int(round(seconds / self.m.opt.timestep)))):
            tau = np.zeros(len(JOINTS))
            fw = self.fw
            for i in range(len(JOINTS)):
                if fw["timeout_ms"] and self.enabled[i] and \
                        (self.d.time - self.last_cmd_t[i]) * 1000 > fw["timeout_ms"]:
                    self.enabled[i] = False                # firmware comm timeout → relax
                if not self.enabled[i]:
                    continue
                err = self.target_adc[i] - self.adc(i)     # firmware: target - current (ADC)
                if abs(err) <= fw["DB"]:
                    continue
                pwm = min(255, max(fw["floor"], fw["K"] * abs(err) + fw["FF"]))
                # ADC→rad sign: q = sign*(adc-zero)*k, so +ADC error is sign*+q
                duty = math.copysign(pwm / 255, err) * self.cal[i].sign
                qd = self.d.qvel[self.va[i]]
                tau[i] = np.clip(self.stall * (duty - qd / self.no_load), -self.stall, self.stall)
            self.d.qfrc_applied[:] = 0
            self.d.qfrc_applied[self.va] = tau
            mujoco.mj_step(self.m, self.d)

    def command(self, sid: int, cmd: int, pos: int) -> Reply | None:
        i = self.by_id.get(sid)
        if i is None:
            return None
        cur = self.adc(i)
        self.last_cmd_t[i] = self.d.time
        if cmd in (0x10, 0x11, 0x12):                 # ubt12hc_pd parameter commands
            if not self.fw.get("params"):
                return None                           # stock firmware: no such command
            idx, val = pos >> 8, pos & 0xFF
            keys = ("K", "FF", "DB", "timeout_10ms")
            reply = -1
            if cmd == 0x10 and idx < 4:
                self.fw[keys[idx]] = val
                if keys[idx] == "timeout_10ms":
                    self.fw["timeout_ms"] = val * 10
                reply = val
            elif cmd == 0x11 and idx < 4:
                reply = int(self.fw[keys[idx]]) if keys[idx] != "timeout_10ms" else self.fw["timeout_ms"] // 10
            elif cmd == 0x12 and idx == 0xA5 and val == 0x5A:
                reply = 0
            return Reply(sid, cur, reply)
        if cmd == CMD_MOVE:
            self.target_adc[i] = min(950, max(50, pos))
            self.enabled[i] = True
        elif cmd == CMD_HOLD:
            if self.prev_cmd[i] != CMD_HOLD:
                self.target_adc[i] = cur
            self.enabled[i] = True
        elif cmd == CMD_RELAX:
            self.enabled[i] = False
        self.prev_cmd[i] = cmd
        return Reply(sid, cur, 0)

    def yaw_deg(self) -> float:
        q = self.d.qpos[3:7]
        return math.degrees(math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))

    def tilt_deg(self) -> float:
        up = self.d.xmat[1].reshape(3, 3)[:, 2]
        return math.degrees(math.acos(min(1.0, up[2])))


class SimBus:
    """Drop-in for servo_bus.ServoBus. Each transaction also advances sim time
    by the wire time of one request + reply (~0.74 ms at 230400 baud)."""

    WIRE_S = 17 * 10 / 230400

    def __init__(self, robot: SimRobot):
        self.robot = robot

    def transact(self, servo_id, cmd, pos=0, timeout_s=None):
        r = self.robot.command(servo_id, cmd, pos)
        self.robot.step(self.WIRE_S)
        return r

    def relax(self, servo_id):
        return self.transact(servo_id, CMD_RELAX)

    def close(self):
        pass


class SimImu:
    def __init__(self, robot: SimRobot):
        self.r = robot
        self.gyro = mujoco.mj_name2id(robot.m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")

    def read(self):
        d, m = self.r.d, self.r.m
        a = m.sensor_adr[self.gyro]
        w = d.sensordata[a:a + 3].astype(np.float32)
        R = d.xmat[1].reshape(3, 3)
        g = (R.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)
        return w, g


def main():
    import argparse

    import jduck_run

    p = argparse.ArgumentParser()
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--vx", type=float, default=0.0)
    p.add_argument("--vy", type=float, default=0.0)
    p.add_argument("--wz", type=float, default=0.0)
    p.add_argument("--fw", default="16,0,0", help="K,FF,DB of firmware/ubt12hc_pd; 'orig' = stock firmware")
    a = p.parse_args()

    cal = Calibration.load(HERE / "calibration.yaml")
    servo_map = load_servo_map(HERE / "servo_map.yaml")
    policy = jduck_run.Policy(a.policy)
    if a.fw == "orig":
        fw = SimRobot.FW_ORIG
    else:
        k, ff, db = (float(x) for x in a.fw.split(","))
        fw = dict(SimRobot.FW_PD, K=k, FF=ff, DB=db)
    robot = SimRobot(cal, servo_map, policy.home, fw=fw)
    bus = SimBus(robot)
    joints = jduck_run.Joints(bus, cal, servo_map, math.radians(5))
    imu = SimImu(robot)
    home = policy.home
    cmd = np.zeros(13, np.float32)
    cmd[0], cmd[1], cmd[2] = a.vx, a.vy, a.wz

    q = joints.move(home)            # power on at HOME
    q_old = q.copy()
    last = np.zeros(14, np.float32)
    x0 = robot.d.qpos[:2].copy()
    dt = jduck_run.CONTROL_DT
    yaw0 = yaw_prev = robot.yaw_deg()
    yaw_acc, tgt_prev, dtgt, legs_qd = 0.0, None, [], []
    legs = [*range(0, 5), *range(9, 14)]
    for tick in range(int(a.seconds / dt)):
        t_used = robot.d.time
        w, g = imu.read()
        qd = (q - q_old) / dt
        obs = np.concatenate([w, g, q - home, qd, last, cmd]).astype(np.float32)
        act = policy(obs).astype(np.float32)
        last = act
        tgt = home + act * policy.action_scale
        q_new = joints.move(tgt)
        q_old, q = q, q_new
        robot.step(max(0.0, dt - (robot.d.time - t_used)))   # rest of the 20 ms tick
        y = robot.yaw_deg()
        yaw_acc += (y - yaw_prev + 180.0) % 360.0 - 180.0
        yaw_prev = y
        if tick * dt > 3.0:                                  # settled: measure jitter
            if tgt_prev is not None:
                dtgt.append(np.degrees(np.abs(tgt - tgt_prev)[legs]).mean())
            legs_qd.append(np.abs(robot.d.qvel[robot.va][legs]).mean())
        tgt_prev = tgt
        if robot.tilt_deg() > 50:
            print(f"FELL at t={tick * dt:.2f}s")
            return
    dx = robot.d.qpos[:2] - x0
    print(f"OK {a.seconds:.0f}s fw={a.fw}: tilt {robot.tilt_deg():.1f}°  moved dx={dx[0]:+.2f} dy={dx[1]:+.2f} m  "
          f"yaw {yaw_acc:+.0f}°  | after 3 s: target change {np.mean(dtgt):.3f}°/step, leg |qd| {np.mean(legs_qd):.4f} rad/s")


if __name__ == "__main__":
    main()
