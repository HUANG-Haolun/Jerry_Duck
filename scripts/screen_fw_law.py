"""Cheap CPU screen of candidate servo firmware laws before spending a PPO run.

    cd third_party/microduck_rl && uv run --frozen python ../../scripts/screen_fw_law.py

For each law pwm = clamp(K*|err| + FF, floor, 255) (0 inside the deadband), at
1 kHz like the firmware, on the jduck MuJoCo model:
  stand   hold HOME 4 s on the floor: tilt, joint jitter (rad/s), falls
  swing   knee in the air tracking ±15° at 2 Hz: amplitude ratio, no overdrive
  sag     steady-state error at 0.15 N·m external load (deg)
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "jduck_tools"))
from calibration import JOINTS  # noqa: E402

SCENE = HERE.parent / "jduck_mjcf" / "scene_walk.xml"
K_RAD = 2 * math.pi / 1024
HOME = np.array([0, 0, -0.25, 0.25, 0, 0, 0, 0, 0, 0, 0, 0.25, -0.25, 0], float)
STALL, W0 = 0.8, 3.0


def law_torque(q, qd, qt, phase, K, FF, floor, db):
    err = np.round(qt / K_RAD + phase) - np.floor(q / K_RAD + phase)
    mag = np.clip(K * np.abs(err) + FF, floor, 255)
    pwm = np.where(np.abs(err) <= db, 0.0, mag)
    tau = STALL * (np.sign(err) * pwm / 255 - qd / W0)
    tau = np.where(pwm == 0, 0.0, tau)
    return np.clip(tau, -STALL, STALL)


def setup(m):
    qa = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
    va = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
    return qa, va


def stand(m, law, seed=0, T=4.0):
    qa, va = setup(m)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    phase = rng.random(14)
    d.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
    d.qpos[qa] = HOME
    mujoco.mj_forward(m, d)
    d.qpos[2] = 0.5 - min(d.site("left_foot").xpos[2], d.site("right_foot").xpos[2]) + 0.001
    d.qpos[qa] = HOME + rng.uniform(-0.03, 0.03, 14)
    jit = []
    for s in range(int(T / m.opt.timestep)):
        d.qfrc_applied[:] = 0
        d.qfrc_applied[va] = law_torque(d.qpos[qa], d.qvel[va], HOME, phase, *law)
        mujoco.mj_step(m, d)
        up = d.xmat[1].reshape(3, 3)[:, 2]
        if up[2] < math.cos(math.radians(45)):
            return None
        if s * m.opt.timestep > T - 1:
            jit.append(np.abs(d.qvel[va]).mean())
    up = d.xmat[1].reshape(3, 3)[:, 2]
    return math.degrees(math.acos(min(1, up[2]))), float(np.mean(jit))


def swing(m, law, f=2.0, amp_deg=15.0):
    qa, va = setup(m)
    d = mujoco.MjData(m)
    d.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
    d.qpos[qa] = HOME
    i = JOINTS.index("left_knee")
    phase = np.full(14, 0.5)
    amp = math.radians(amp_deg)
    qs, ts = [], []
    n = int(3.0 / m.opt.timestep)
    for s in range(n):
        t = s * m.opt.timestep
        tgt = HOME.copy()
        tgt[i] += amp * math.sin(2 * math.pi * f * t)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[va] = law_torque(d.qpos[qa], d.qvel[va], tgt, phase, *law)
        d.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]   # pin trunk in the air
        d.qvel[:6] = 0
        mujoco.mj_step(m, d)
        if t > 1.0:
            qs.append(d.qpos[qa[i]] - HOME[i])
            ts.append(tgt[i] - HOME[i])
    return float(np.std(qs) / np.std(ts))


def sag(law, load=0.15):
    K, FF, floor, db = law
    # steady state: STALL*pwm/255 = load
    pwm = load / STALL * 255
    if FF >= pwm or floor >= pwm:
        return (db + 1) * math.degrees(K_RAD)
    return max(db + 1, (pwm - FF) / K) * math.degrees(K_RAD)


def main():
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    m.opt.timestep = 0.001
    laws = [("current fw", (1, 0, 48, 2))]
    for K in (2, 3, 5, 8, 12):
        for FF in (0, 15, 30):
            laws.append((f"K={K:<2} FF={FF:<2}", (K, FF, 0, 1)))
    print(f"{'law':14s} {'stand (tilt°, jitter rad/s)':>30s}  {'swing 2Hz±15°':>14s}  {'sag@0.15Nm':>10s}")
    for name, law in laws:
        st = [stand(m, law, s) for s in range(2)]
        st_s = "FELL" if any(x is None for x in st) else f"{np.mean([x[0] for x in st]):5.1f}°, {np.mean([x[1] for x in st]):.3f}"
        print(f"{name:14s} {st_s:>30s}  {swing(m, law):14.2f}  {sag(law):9.1f}°")


if __name__ == "__main__":
    main()
