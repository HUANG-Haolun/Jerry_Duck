"""Does the servo law jitter on its own while holding a pose? (no policy involved)

    cd third_party/microduck_rl && uv run --frozen python ../../scripts/screen_hold_jitter.py

For each law pwm = clamp(K*|err| + FF, 0, 255), 0 inside the deadband, at 1 kHz with ADC
quantization, 0.96 N·m / 4.8 rad/s (official 7.4 V):
  hold    stand at HOME 4 s: mean leg |qd| in the last second (rad/s) and tilt
  settle  knee in the air, step 10° and wait 1 s: final error in ADC counts
          (big error = the law can't overcome static friction and stops short)

PD kp=8 kd=0.2 (what v0 stood still with) is the reference: jitter 0.000.
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "jduck_tools"))
import screen_fw_law as S  # noqa: E402
from calibration import JOINTS  # noqa: E402

S.STALL, S.W0 = 0.96, 4.8
HOME = S.HOME
LEGS = [i for i, j in enumerate(JOINTS) if "hip" in j or "knee" in j or "ankle" in j]


def hold(m, law, seed):
    qa, va = S.setup(m)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    ph = rng.random(14)
    d.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
    d.qpos[qa] = HOME
    mujoco.mj_forward(m, d)
    d.qpos[2] = 0.5 - min(d.site("left_foot").xpos[2], d.site("right_foot").xpos[2]) + 0.001
    d.qpos[qa] = HOME + rng.uniform(-0.02, 0.02, 14)
    jit = []
    for s in range(4000):
        d.qfrc_applied[:] = 0
        d.qfrc_applied[va] = law(d.qpos[qa], d.qvel[va], HOME, ph)
        mujoco.mj_step(m, d)
        if d.xmat[1].reshape(3, 3)[2, 2] < math.cos(math.radians(45)):
            return None
        if s > 3000:
            jit.append(np.abs(d.qvel[va[LEGS]]).mean())
    up = d.xmat[1].reshape(3, 3)[:, 2]
    return float(np.mean(jit)), math.degrees(math.acos(min(1, up[2])))


def settle(m, law):
    qa, va = S.setup(m)
    d = mujoco.MjData(m)
    d.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
    d.qpos[qa] = HOME
    i = JOINTS.index("left_knee")
    tgt = HOME.copy()
    tgt[i] += math.radians(10)
    ph = np.full(14, 0.5)
    for _ in range(1000):
        d.qfrc_applied[:] = 0
        d.qfrc_applied[va] = law(d.qpos[qa], d.qvel[va], tgt, ph)
        d.qpos[:7] = [0, 0, 0.5, 1, 0, 0, 0]
        d.qvel[:6] = 0
        mujoco.mj_step(m, d)
    return abs(tgt[i] - d.qpos[qa[i]]) / S.K_RAD


def main():
    m = mujoco.MjModel.from_xml_path(str(S.SCENE))
    m.opt.timestep = 0.001
    laws = [("PD kp8 kd0.2 (v0)", lambda q, qd, qt, ph: np.clip(8 * (qt - q) - 0.2 * qd, -S.STALL, S.STALL))]
    for K in (16, 10):
        for FF in (15, 8, 0):
            for DB in (2, 1, 0):
                laws.append((f"K={K:<2} FF={FF:<2} DB={DB}",
                             lambda q, qd, qt, ph, K=K, FF=FF, DB=DB: S.law_torque(q, qd, qt, ph, K, FF, 0, DB)))
    print(f"{'law':20s} {'hold jitter rad/s':>18s} {'tilt':>6s} {'settle err (counts)':>20s}")
    for name, law in laws:
        h = [hold(m, law, s) for s in range(2)]
        if any(x is None for x in h):
            hs = f"{'FELL':>18s} {'':>6s}"
        else:
            hs = f"{np.mean([x[0] for x in h]):18.4f} {np.mean([x[1] for x in h]):5.1f}°"
        print(f"{name:20s} {hs} {settle(m, law):20.1f}")


if __name__ == "__main__":
    main()
