"""Calibrate sim_robot against a real zero-command stand log (jduck_run_log.py --log).

The sim loop replays jduck_run.py exactly as it ran on the robot (board copy at
/userdata/jduck-runtime/a8df940): per-servo bus time measured from the log (1.29 ms, sim
default 0.74), neck/head IDs refreshed every other tick (--head-rate-div 2), qd per joint from
its own sample times, policy blend-in. Then a grid over the unknowns (joint friction, servo
strength, extra servo latency, gear backlash) and a distance to the real log on:
leg target change per tick, leg/head q std, gyro std, dominant leg frequency.

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_tools/real_calib.py ~/jduck_ops/stand_O1.npz --fw orig
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import jduck_run  # noqa: E402
import sim_robot as S  # noqa: E402
from calibration import JOINTS, Calibration, load_servo_map  # noqa: E402

LEGS = [*range(0, 5), *range(9, 14)]
HEADS = [5, 6, 7, 8]
SLOW_HEAD = [5, 7, 8]        # neck_pitch, head_yaw, head_roll: every other tick on the robot
DT = 0.02


class BacklashRobot(S.SimRobot):
    """SimRobot + optional gear backlash and extra command latency.

    Backlash: the motor side (angle `mot`, in output units) drives the output joint through a
    dead zone of ±bl/2. The firmware reads the OUTPUT angle (the ADC pot sits on the output
    shaft), so inside the gap the servo spins its motor freely and the joint gets no torque.
    Latency: a CMD 0x01 takes effect extra_delay_ms after it is sent (the reply is immediate)."""

    J_ROTOR = 0.005          # reflected rotor inertia, kg·m² (= the MJCF joint armature)

    def __init__(self, cal, smap, home, fw, seed, friction, stall, backlash, extra_delay_ms, damping=None, no_load=4.8):
        super().__init__(cal, smap, home, fw=fw, seed=seed, friction=friction, stall=stall, no_load=no_load)
        if damping is not None:
            self.m.dof_damping[self.va] = damping
        self.bl = backlash
        self.mot = self.d.qpos[self.qa].copy()
        self.mv = np.zeros(len(JOINTS))
        self.extra_delay = extra_delay_ms / 1000
        self.queue = []

    def command(self, sid, cmd, pos):
        if self.extra_delay <= 0 or cmd != S.CMD_MOVE:
            return super().command(sid, cmd, pos)
        i = self.by_id.get(sid)
        if i is None:
            return None
        self.last_cmd_t[i] = self.d.time
        self.queue.append((self.d.time + self.extra_delay, sid, cmd, pos))
        return S.Reply(sid, self.adc(i), 0)

    def step(self, seconds):
        dt = self.m.opt.timestep
        for _ in range(max(1, int(round(seconds / dt)))):
            if self.queue:
                due = [x for x in self.queue if x[0] <= self.d.time]
                self.queue = [x for x in self.queue if x[0] > self.d.time]
                for _, sid, cmd, pos in due:
                    S.SimRobot.command(self, sid, cmd, pos)
            if self.bl <= 0:
                S.SimRobot.step(self, dt)
                continue
            fw = self.fw
            tau = np.zeros(len(JOINTS))
            for i in range(len(JOINTS)):
                if not self.enabled[i]:
                    continue
                err = self.target_adc[i] - self.adc(i)
                if abs(err) <= fw["DB"]:
                    continue
                pwm = min(255, max(fw["floor"], fw["K"] * abs(err) + fw["FF"]))
                duty = math.copysign(pwm / 255, err) * self.cal[i].sign
                tau[i] = np.clip(self.stall * (duty - self.mv[i] / self.no_load), -self.stall, self.stall)
            q = self.d.qpos[self.qa].copy()
            qv = self.d.qvel[self.va]
            h = self.bl / 2
            gap = self.mot - q
            # engaged: motor at an edge of the gap and pushing (or moving) into the contact
            at_edge = np.abs(gap) >= h - 1e-7
            engaged = at_edge & ((np.sign(tau) == np.sign(gap)) & (tau != 0) | (np.sign(self.mv - qv) == np.sign(gap)))
            # free motor: rotor inertia J_ROTOR accelerated by the motor torque
            mv_free = self.mv + tau / self.J_ROTOR * dt
            mot_free = self.mot + mv_free * dt
            hit = (~engaged) & (np.abs(mot_free - q) >= h)
            self.mot = np.where(engaged, q + np.sign(gap) * h, np.clip(mot_free, q - h, q + h))
            self.mv = np.where(engaged | hit, qv, mv_free)
            self.d.qfrc_applied[:] = 0
            self.d.qfrc_applied[self.va] = np.where(engaged, tau, 0.0)
            mujoco.mj_step(self.m, self.d)
            # an engaged motor stays in contact: it moves with the output it is pushing
            qn = self.d.qpos[self.qa]
            self.mot = np.where(engaged, qn + np.sign(gap) * h, self.mot)
            self.mv = np.where(engaged, self.d.qvel[self.va], self.mv)


def make_robot(cal, smap, home, fw, seed, friction, stall, backlash, extra_delay_ms, damping=None, no_load=4.8):
    return BacklashRobot(cal, smap, home, fw, seed, friction, stall, backlash, extra_delay_ms, damping, no_load)


class Bus(S.SimBus):
    def __init__(self, robot, wire_s):
        super().__init__(robot)
        self.wire = wire_s

    def transact(self, servo_id, cmd, pos=0, timeout_s=None):
        r = self.robot.command(servo_id, cmd, pos)
        self.robot.step(self.wire)
        return r


def run_sim(policy, cal, smap, fw, friction, stall, backlash, extra_delay_ms, wire_s, seed=0, T=25.0, blend_s=0.0,
            damping=None, no_load=4.8, gyro_noise=0.0, cmd3=(0.0, 0.0, 0.0), fall_deg=45.0):
    robot = make_robot(cal, smap, policy.home, fw, seed, friction, stall, backlash, extra_delay_ms, damping, no_load)
    rng = np.random.default_rng(seed)
    robot.d.qpos[robot.qa] += rng.uniform(-0.01, 0.01, len(robot.qa))
    robot.mot = robot.d.qpos[robot.qa].copy()
    bus = Bus(robot, wire_s)
    joints = jduck_run.Joints(bus, cal, smap, math.radians(5))
    imu = S.SimImu(robot)
    home = policy.home
    q = joints.move(home)
    sample_q, sample_t = q.copy(), np.full(14, -DT)
    qd_f = np.zeros(14, np.float32); last = np.zeros(14, np.float32); cmd = np.zeros(13, np.float32)
    cmd[:3] = cmd3
    x0 = robot.d.qpos[:2].copy(); yaw_acc, yp = 0.0, robot.yaw_deg()
    full = [i for i in range(14) if i not in SLOW_HEAD]
    L = {k: [] for k in ("q", "target", "gyro", "grav", "tilt")}
    for tick in range(int(T / DT)):
        t0 = robot.d.time
        w, g = imu.read()
        if gyro_noise > 0:                       # IMU noise / board vibration the policy sees
            w = w + rng.normal(0, gyro_noise, 3).astype(np.float32)
        obs = np.concatenate([w, g, q - home, qd_f, last, cmd]).astype(np.float32)
        raw = policy(obs).astype(np.float32)
        a = raw * (min(1.0, (tick + 1) * DT / blend_s) if blend_s > 0 else 1.0)
        last = a
        tgt = home + a * policy.action_scale
        active = full + SLOW_HEAD[tick % 2::2]
        # = the board's Joints.move(target, q_prev, indices): only `active` servos get CMD 0x01,
        #   the others keep their last reading (and keep their last target in the servo)
        tgt_c = np.clip(tgt, joints.q_lo, joints.q_hi)
        q_new = q.copy()
        for i in active:
            adc = int(round(joints.cal[i].to_adc(float(tgt_c[i]), joints.k)))
            r = bus.transact(joints.ids[i], S.CMD_MOVE, max(50, min(950, adc)))
            q_new[i] = joints._to_q(i, r.pos)
        now = robot.d.time
        for i in active:
            qd_f[i] += policy.qd_ema * ((q_new[i] - sample_q[i]) / max(1e-3, now - sample_t[i]) - qd_f[i])
            sample_q[i], sample_t[i] = q_new[i], now
        q = q_new
        for k, v in (("q", q), ("target", tgt), ("gyro", w), ("grav", g)):
            L[k].append(np.array(v))
        robot.step(max(0.0, DT - (robot.d.time - t0)))
        L.setdefault("tilt", []).append(robot.tilt_deg())
        y = robot.yaw_deg(); yaw_acc += (y - yp + 180.0) % 360.0 - 180.0; yp = y
        if robot.tilt_deg() > fall_deg:
            return None
    out = {k: np.stack(v) for k, v in L.items()}
    out["dxy"] = robot.d.qpos[:2] - x0
    out["yaw"] = yaw_acc
    return out


_W = None


def _one(args):
    combo, seed = args
    pol_path, fw, wire, keys = _W
    c = dict(zip(keys, combo))
    cal = Calibration.load(HERE / "calibration.yaml"); smap = load_servo_map(HERE / "servo_map.yaml")
    pol = jduck_run.Policy(Path(pol_path))
    fw = dict(fw)
    for k_, fk in (("floor", "floor"), ("deadband", "DB"), ("kfw", "K")):
        if k_ in c:
            fw[fk] = c[k_]
    d = run_sim(pol, cal, smap, fw, c.get("friction", 0.02), c.get("stall", 0.96), math.radians(c.get("backlash_deg", 0.0)),
                c.get("delay_ms", 0.0), wire, seed=seed, damping=c.get("damping"), no_load=c.get("no_load", 4.8),
                gyro_noise=c.get("gyro_noise", 0.0))
    return None if d is None else feats(d)


def feats(d, skip=300):
    sl = slice(skip, None)
    tg, q, g = d["target"][sl], d["q"][sl], d["gyro"][sl]
    jit = np.degrees(np.abs(np.diff(tg, axis=0)))[:, LEGS].mean()
    qs_leg = np.degrees(q[:, LEGS].std(0)).mean()
    qs_head = np.degrees(q[:, HEADS].std(0)).mean()
    x = tg[:, LEGS] - tg[:, LEGS].mean(0)
    P = (np.abs(np.fft.rfft(x, axis=0)) ** 2).sum(1); P[0] = 0
    f = np.fft.rfftfreq(len(x), DT)
    fc = float((f * P).sum() / P.sum())                   # spectral centroid, Hz
    return np.array([jit, qs_leg, qs_head, *g.std(0), fc])


NAMES = ["腿目标变化°", "腿q std°", "头q std°", "陀螺x", "陀螺y", "陀螺z", "腿频谱重心Hz"]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("log", type=Path)
    p.add_argument("--policy", type=Path, default=HERE.parent / "models" / "jduck_O1_orig_full.onnx")
    p.add_argument("--fw", default="orig")
    p.add_argument("--quick", action="store_true", help="smaller grid")
    p.add_argument("--grid", default=None, help="python dict literal overriding the grid")
    p.add_argument("--seeds", type=int, default=1)
    a = p.parse_args()
    fw = dict(S.SimRobot.FW_ORIG) if a.fw == "orig" else dict(S.SimRobot.FW_PD, **dict(zip(("K", "FF", "DB"), map(float, a.fw.split(",")))))
    cal = Calibration.load(HERE / "calibration.yaml"); smap = load_servo_map(HERE / "servo_map.yaml")
    pol = jduck_run.Policy(a.policy)
    real = dict(np.load(a.log, allow_pickle=True))
    el = (real["el"] - real["t_pol"]); n = real["active"].sum(1)
    wire = float(np.median(el / n))
    fr = feats(real)
    print(f"真机：每个舵机通信 {wire * 1000:.2f} ms")
    print("  " + "  ".join(f"{k} {v:.3f}" for k, v in zip(NAMES, fr)))
    scale = np.maximum(np.abs(fr), 1e-3)
    grid = dict(friction=[0.0, 0.02, 0.05], stall=[0.96, 0.8, 1.15],
                backlash_deg=[0.0, 1.0, 2.0, 3.0], delay_ms=[0.0, 10.0, 20.0])
    if a.quick:
        grid = dict(friction=[0.02], stall=[0.96], backlash_deg=[0.0, 2.0], delay_ms=[0.0, 10.0])
    if a.grid:
        grid = eval(a.grid)
    rows = []
    base = None
    combos = list(itertools.product(*grid.values()))
    from multiprocessing import Pool
    global _W
    _W = (str(a.policy), fw, wire, list(grid.keys()))
    with Pool(min(22, len(combos) * a.seeds)) as pool:
        flat = pool.map(_one, [(c, s) for c in combos for s in range(a.seeds)])
    res = {}
    for (c, s), r in zip([(c, s) for c in combos for s in range(a.seeds)], flat):
        res.setdefault(c, []).append(r)
    keys = list(grid.keys())
    def lab(c):
        return "  ".join(f"{k} {v:g}" for k, v in zip(keys, c))
    for combo in combos:
        rs = res[combo]
        ok = [r for r in rs if r is not None]
        if not ok:
            rows.append((1e9, combo, None, len(rs)))
            continue
        fs = np.mean(ok, axis=0)
        dist = float(np.sqrt((((fs - fr) / scale) ** 2).mean()))
        rows.append((dist, combo, fs, len(rs) - len(ok)))
        pass
    rows.sort(key=lambda r: r[0])
    print("\n=== 最接近真机的 5 组（距离越小越像）")
    print("  真机          " + "  ".join(f"{v:.3f}" for v in fr))

    for dist, combo, fs, nf in rows[:10]:
        if fs is None:
            continue
        print(f"  {lab(combo):60s} " + "  ".join(f"{v:.3f}" for v in fs) + f"   距离 {dist:.2f}" + (f"（摔 {nf}）" if nf else ""))
    print(f"  （全摔的组合：{sum(1 for r in rows if r[2] is None)}/{len(rows)}）")


if __name__ == "__main__":
    main()
