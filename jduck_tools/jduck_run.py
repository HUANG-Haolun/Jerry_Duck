#!/usr/bin/env python3
"""jduck on-board 50 Hz policy runner (Taishanpi RK3566 or any Linux box).

    python3 jduck_tools/jduck_run.py --port /dev/ttyS3 --policy jduck_walk.onnx --dry-run
    python3 jduck_tools/jduck_run.py --port /dev/ttyS3 --policy jduck_walk.onnx

One control tick (20 ms), with the same observation as training (61D):

    [ base_ang_vel(3) | projected_gravity(3) | q - HOME (14) | qd (14) | last_action (14) | command (13) ]

  - q comes from the servo ADC via calibration.yaml. qd is a finite difference of
    q, because the servo's own 40 ms speed is too coarse.
  - Angular velocity and gravity come from the IMU in the trunk_base frame. The
    IMU axis mapping is set in --imu-config (see imu_lsm6dsv16x.py).
  - target = HOME + action, converted to ADC with calibration.yaml and clamped to
    the calibrated range (minus a margin), then sent as CMD 0x01.

Safety:
  - --dry-run reads sensors and runs the policy but never powers the servos.
  - Any read failure, a NaN, or a tilt beyond --max-tilt makes every servo relax.
  - Ctrl+C makes every servo relax.
  - Start with the robot HUNG UP (feet off the ground).

Commands come from the keyboard in this terminal (same keys as infer_policy.py),
or from an Xbox pad via /dev/input/js* with --gamepad.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibration import JOINTS, Calibration, load_servo_map  # noqa: E402
from servo_bus import CMD_MOVE, ServoBus  # noqa: E402

CONTROL_DT = 0.02
CMD_DIM = 13


# ---------------------------------------------------------------- policy
class Policy:
    """ONNX policy plus the HOME pose stored in its metadata at export."""

    def __init__(self, path: Path):
        import onnxruntime as ort

        self.sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.out = self.sess.get_outputs()[0].name
        meta = self.sess.get_modelmeta().custom_metadata_map
        names = meta.get("joint_names", "").split(",")
        if names != JOINTS:
            raise SystemExit(f"ONNX joint order {names} != expected {JOINTS}")
        self.home = np.array([float(x) for x in meta["default_joint_pos"].split(",")], np.float32)
        self.action_scale = float(meta.get("action_scale", "1.0"))
        # qd low-pass used in training (JDUCK_QD_EMA); 1 = raw finite difference
        self.qd_ema = float(meta.get("jduck_qd_ema", "1"))
        n_in = self.sess.get_inputs()[0].shape[1]
        if n_in != 61:
            raise SystemExit(f"policy expects {n_in}D obs, this runner builds 61D")

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return self.sess.run([self.out], {self.inp: obs[None].astype(np.float32)})[0][0]


class PolicySwitch:
    """Walk policy while upright, stand-up policy after a fall (until VelStand works on jduck).
    Fall: tilt > fall_deg → get-up. Back to walking: tilt < up_deg AND every leg joint within
    leg_tol of HOME, held for hold_s. Both policies share the 61-D obs and HOME."""

    LEGS = [*range(0, 5), *range(9, 14)]

    def __init__(self, walk: Policy, getup: Policy, fall_deg=45.0, up_deg=12.0, leg_tol=0.35, hold_s=0.6):
        self.walk, self.getup = walk, getup
        self.fall_deg, self.up_deg, self.leg_tol, self.hold_s = fall_deg, up_deg, leg_tol, hold_s
        self.active, self.ok_since, self.switches = walk, None, 0

    def select(self, tilt_deg: float, dq: np.ndarray, t: float) -> Policy:
        if self.active is self.walk:
            if tilt_deg > self.fall_deg:
                self.active, self.ok_since = self.getup, None
                self.switches += 1
        else:
            ok = tilt_deg < self.up_deg and np.abs(dq[self.LEGS]).max() < self.leg_tol
            if not ok:
                self.ok_since = None
            elif self.ok_since is None:
                self.ok_since = t
            elif t - self.ok_since >= self.hold_s:
                self.active = self.walk
                self.switches += 1
        return self.active


# ---------------------------------------------------------------- joints
class Joints:
    """14 servos over the bus: ADC <-> URDF rad via calibration.yaml."""

    def __init__(self, bus: ServoBus, cal: Calibration, servo_map: dict[str, int],
                 margin_rad: float):
        missing = [j for j in JOINTS if j not in servo_map or not cal.joints.get(j) or not cal.joints[j].complete]
        if missing:
            raise SystemExit(f"not calibrated / no servo ID: {missing}")
        self.bus, self.k = bus, cal.k_rad_per_count
        self.cal = [cal.joints[j] for j in JOINTS]
        self.ids = [servo_map[j] for j in JOINTS]
        lo_hi = [c.urdf_range(self.k, margin_rad) for c in self.cal]
        self.q_lo = np.array([a for a, _ in lo_hi], np.float32)
        self.q_hi = np.array([b for _, b in lo_hi], np.float32)

    def _to_q(self, i: int, adc: int) -> float:
        return self.cal[i].to_rad(adc, self.k)

    def read_relaxed(self) -> np.ndarray | None:
        """Positions in rad, sending CMD 0x03 (motor OFF). Use only while unpowered:
        before the ramp, and in --dry-run."""
        q = np.empty(len(JOINTS), np.float32)
        for i, sid in enumerate(self.ids):
            r = self.bus.relax(sid)
            if r is None:
                return None
            q[i] = self._to_q(i, r.pos)
        return q

    def move(self, q_target: np.ndarray) -> np.ndarray | None:
        """Send targets (rad) and return the positions read back from the replies."""
        q_target = np.clip(q_target, self.q_lo, self.q_hi)
        q = np.empty(len(JOINTS), np.float32)
        for i, sid in enumerate(self.ids):
            adc = int(round(self.cal[i].to_adc(float(q_target[i]), self.k)))
            r = self.bus.transact(sid, CMD_MOVE, max(50, min(950, adc)))
            if r is None:
                return None
            q[i] = self._to_q(i, r.pos)
        return q

    def relax_all(self) -> None:
        for _ in range(2):
            for sid in self.ids:
                self.bus.relax(sid)


# ---------------------------------------------------------------- IMU
class NoImu:
    """Upright, still. Only for --dry-run without an IMU."""

    def read(self):
        return np.zeros(3, np.float32), np.array([0.0, 0.0, -1.0], np.float32)


def quat_to_gravity(qw, qx, qy, qz) -> np.ndarray:
    """Body-frame unit gravity from a body->world quaternion (same as training)."""
    v = np.array([0.0, 0.0, -1.0])
    q_vec = np.array([qx, qy, qz])
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(-q_vec, v) * qw * 2.0   # inverse rotation
    c = q_vec * np.dot(q_vec, v) * 2.0
    return (a + b + c).astype(np.float32)


# ---------------------------------------------------------------- commands
@contextmanager
def raw_keys():
    if not sys.stdin.isatty():
        yield
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class Commands:
    """13D command block: [vx, vy, wz, head(4), body(6)]."""

    def __init__(self, vmax=(0.25, 0.15, 0.8)):
        self.vmax = vmax
        self.cmd = np.zeros(CMD_DIM, np.float32)
        self.quit = False
        self.js = None

    def open_gamepad(self, dev: str) -> None:
        self.js = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = [0.0] * 8

    def poll(self) -> None:
        vx, vy, wz = self.vmax
        # keyboard
        while sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":                       # arrow keys: ESC [ A/B/C/D
                seq = sys.stdin.read(2)
                ch = {"[A": "up", "[B": "down", "[C": "right", "[D": "left"}.get(seq, "")
            c = self.cmd
            if ch == "up":    c[0] = min(vx, c[0] + 0.05)
            elif ch == "down": c[0] = max(-vx, c[0] - 0.05)
            elif ch == "left": c[1] = min(vy, c[1] + 0.05)
            elif ch == "right": c[1] = max(-vy, c[1] - 0.05)
            elif ch in "aA":  c[2] = min(wz, c[2] + 0.2)
            elif ch in "eE":  c[2] = max(-wz, c[2] - 0.2)
            elif ch == " ":   c[:3] = 0
            elif ch in "qQ":  self.quit = True
        # gamepad (Linux joystick API: 8-byte events)
        if self.js is not None:
            try:
                while True:
                    ev = os.read(self.js, 8)
                    if len(ev) < 8:
                        break
                    _, value, etype, number = np.frombuffer(ev, dtype=np.dtype("<u4,<i2,u1,u1"))[0]
                    if etype & 0x02 and number < len(self.axes):
                        self.axes[number] = value / 32767.0
            except BlockingIOError:
                pass
            dz = lambda a: 0.0 if abs(a) < 0.12 else a  # noqa: E731
            # Xbox on xpad: 0 LX, 1 LY, 3 RX (2 is LT on most mappings)
            self.cmd[0] = -dz(self.axes[1]) * vx
            self.cmd[1] = -dz(self.axes[0]) * vy
            self.cmd[2] = -dz(self.axes[3]) * wz


# ---------------------------------------------------------------- main loop
def run(args) -> None:
    cal = Calibration.load(args.cal)
    servo_map = load_servo_map(args.map)
    policy = Policy(args.policy)
    switch = PolicySwitch(policy, Policy(args.getup_policy)) if args.getup_policy else None
    if switch is not None and np.abs(switch.getup.home - policy.home).max() > 1e-4:
        raise SystemExit("两个策略的 HOME 不一样")
    if args.imu_config:
        from imu_lsm6dsv16x import Imu
        imu = Imu.from_config(args.imu_config)
    elif args.dry_run:
        print("⚠ 没有 --imu-config：dry-run 用假的 IMU（水平静止）")
        imu = NoImu()
    else:
        raise SystemExit("真机运行需要 --imu-config")
    cmds = Commands()
    if args.gamepad:
        cmds.open_gamepad(args.gamepad)

    bus = ServoBus(args.port, args.baud, args.timeout, args.direction)
    joints = Joints(bus, cal, servo_map, math.radians(args.limit_margin_deg))
    home = policy.home
    last_action = np.zeros(len(JOINTS), np.float32)
    q = joints.read_relaxed()
    if q is None:
        raise SystemExit("读不到全部 14 个舵机")
    print("当前姿态与 HOME 的差（度）：", np.round(np.degrees(q - home), 1))

    if not args.dry_run:
        print(f"\n⚠ 即将上电，先用 {args.ramp_s:.0f} s 慢慢移到 HOME。机器人要吊起来！")
        if input("继续？[y/N] ").strip().lower() != "y":
            joints.relax_all()
            return
        # ramp to HOME (each reply also gives us the position)
        q_start = q.copy()
        n = max(1, int(args.ramp_s / CONTROL_DT))
        for i in range(1, n + 1):
            t0 = time.monotonic()
            q_new = joints.move(q_start + (home - q_start) * (i / n))
            if q_new is None:
                joints.relax_all()
                raise SystemExit("ramp 时舵机无应答，已卸力")
            q = q_new
            time.sleep(max(0.0, CONTROL_DT - (time.monotonic() - t0)))
        print("已到 HOME。按方向键/A/E 给速度，空格停，q 退出。")

    # State: `q` is the latest measured position. On hardware it comes from the
    # replies to last tick's CMD 0x01 (the firmware answers with the ADC read just
    # before applying the command). `q_old` is the one before it, for qd.
    q_old, t_old = q.copy(), time.monotonic() - CONTROL_DT
    qd_f = np.zeros(len(JOINTS), np.float32)
    worst_dt = 0.0
    tick = 0
    try:
        with raw_keys():
            while not cmds.quit:
                t0 = time.monotonic()
                cmds.poll()
                ang_vel, grav = imu.read()
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(grav[2])))))
                if switch is None and tilt > args.max_tilt:
                    print(f"\n倾斜 {tilt:.0f}° > {args.max_tilt}°，卸力停止")
                    break
                qd_f += policy.qd_ema * ((q - q_old) / max(1e-3, t0 - t_old) - qd_f)
                qd = qd_f
                obs = np.concatenate([ang_vel, grav, q - home, qd, last_action, cmds.cmd]).astype(np.float32)
                if not np.isfinite(obs).all():
                    print("\nobs 有 NaN，卸力停止")
                    break
                if switch is not None:
                    pol = switch.select(tilt, q - home, tick * CONTROL_DT)
                    if pol is switch.getup:
                        obs[48:] = 0.0                   # the get-up policy is driven at zero command
                    action = pol(obs).astype(np.float32)
                else:
                    action = policy(obs).astype(np.float32)
                last_action = action
                target = home + action * policy.action_scale
                q_new = joints.read_relaxed() if args.dry_run else joints.move(target)
                if q_new is None:
                    print("\n舵机无应答，卸力停止")
                    break
                q_old, t_old = q, t0
                q = q_new
                tick += 1
                el = time.monotonic() - t0
                worst_dt = max(worst_dt, el)
                if tick % 50 == 0:
                    sys.stdout.write(
                        f"\rcmd vx={cmds.cmd[0]:+.2f} vy={cmds.cmd[1]:+.2f} wz={cmds.cmd[2]:+.2f} | "
                        f"tilt {tilt:4.1f}° | loop {el * 1000:4.1f} ms (worst {worst_dt * 1000:4.1f}) "
                        f"| max|a| {np.abs(action).max():.2f}"
                        + (f" | {'起身' if switch.active is switch.getup else '走路'}" if switch else "") + "   ")
                    sys.stdout.flush()
                time.sleep(max(0.0, CONTROL_DT - (time.monotonic() - t0)))
    finally:
        joints.relax_all()
        bus.close()
        print("\n已卸力。最慢一拍 %.1f ms（要 < 20 ms）" % (worst_dt * 1000))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True)
    p.add_argument("--policy", type=Path, required=True)
    p.add_argument("--baud", type=int, default=230400)
    p.add_argument("--direction", default="none", choices=["none", "rts", "rts-inv"])
    p.add_argument("--timeout", type=float, default=0.004, help="每个舵机应答超时（秒）；14 个要在 20 ms 内跑完")
    p.add_argument("--cal", type=Path, default=HERE / "calibration.yaml")
    p.add_argument("--map", type=Path, default=HERE / "servo_map.yaml")
    p.add_argument("--imu-config", type=Path, default=None)
    p.add_argument("--gamepad", default=None, help="例如 /dev/input/js0")
    p.add_argument("--dry-run", action="store_true", help="只读传感器 + 跑策略，不给舵机上电")
    p.add_argument("--ramp-s", type=float, default=3.0)
    p.add_argument("--max-tilt", type=float, default=50.0, help="超过就卸力（给了 --getup-policy 时不生效）")
    p.add_argument("--getup-policy", type=Path, default=None,
                   help="摔倒（倾斜 > 45°）后切到这个起身策略，站稳后切回 --policy")
    p.add_argument("--limit-margin-deg", type=float, default=5.0)
    run(p.parse_args())


if __name__ == "__main__":
    main()
