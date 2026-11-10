"""Compare zero-command standing on the robot with the same policy in sim, joint by joint.

    # 1. on the robot: stand at zero command for ≥ 15 s, recording every tick
    python3 jduck_tools/jduck_run.py --port /dev/ttyS3 --policy models/X.onnx --imu-config jduck_tools/imu.yaml --log stand.npz
    # 2. on the PC (needs mujoco + onnxruntime: run from third_party/microduck_rl with uv)
    uv run --frozen python ../../jduck_tools/jitter_compare.py stand.npz [--fw 8,0,0]
    uv run --frozen python ../../jduck_tools/jitter_compare.py --sim-only models/X.onnx   # no robot log: sim side only

NOT a zero-offset detector: with a wrong adc_zero the robot's own readings and actions are
self-consistent (sim test: left knee +3°, ankle −2° wrong → table indistinguishable). Check zeros
against an external reference (calibrate.py monitor at a known pose / an angle gauge at HOME).

What it tells you
  - mean q − HOME per joint, robot vs sim: where the robot actually rests.
  - mean action per joint (the policy's steady push). A joint whose zero is off makes the
    policy push it the same way all the time: |Δ mean action| ≳ 1.5° with a steady sign.
  - trunk pitch/roll from the IMU gravity: a lean that sim does not have = IMU mounting
    angle or HOME (the policy balances the trunk, so a wrong zero shows up as a lean).
  - jitter (target change per tick) and q std per joint, and the dominant frequency.
    ~25 Hz (alternating every tick) = a loop oscillation through quantized q / Δq qd;
    1–5 Hz = slow balance sway.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibration import JOINTS  # noqa: E402

DT = 0.02
SHORT = ["L髋yaw", "L髋roll", "L髋pitch", "L膝", "L踝", "脖子", "头pitch", "头yaw", "头roll",
         "R髋yaw", "R髋roll", "R髋pitch", "R膝", "R踝"]


def zero_cmd_segment(d, skip_s=3.0):
    cmd = d["cmd"]
    still = np.all(np.abs(cmd) < 1e-6, axis=1)
    # longest run of zero command
    best, cur, start, bs = 0, 0, 0, 0
    for i, z in enumerate(still):
        if z:
            if cur == 0:
                start = i
            cur += 1
            if cur > best:
                best, bs = cur, start
        else:
            cur = 0
    sk = int(skip_s / DT)
    if best - sk < int(5 / DT):
        raise SystemExit(f"零速段只有 {best * DT:.1f} s，至少要 {skip_s + 5:.0f} s")
    return slice(bs + sk, bs + best)


def stats(q, target, action, grav, home):
    dq = np.degrees(q - home)
    dtgt = np.degrees(np.abs(np.diff(target, axis=0)))
    g = grav.mean(0)
    pitch = math.degrees(math.atan2(-g[0], -g[2]))
    roll = math.degrees(math.atan2(g[1], -g[2]))
    a = np.degrees(action)
    # dominant frequency of the target per joint
    x = target - target.mean(0)
    spec = np.abs(np.fft.rfft(x, axis=0))
    f = np.fft.rfftfreq(len(x), DT)
    spec[0] = 0
    fdom = f[spec.argmax(0)]
    return dict(dq=dq.mean(0), qstd=dq.std(0), act=a.mean(0), jit=dtgt.mean(0), fdom=fdom,
                pitch=pitch, roll=roll, jit_all=float(dtgt[:, LEGS].mean()))


LEGS = [*range(0, 5), *range(9, 14)]


def sim_log(policy_path, fw, T=20.0, seed=0, zero_offset=None):
    import jduck_run
    import sim_robot as S
    from calibration import Calibration, load_servo_map
    cal = Calibration.load(HERE / "calibration.yaml")
    smap = load_servo_map(HERE / "servo_map.yaml")
    pol = jduck_run.Policy(Path(policy_path))
    robot = S.SimRobot(cal, smap, pol.home, fw=fw, seed=seed, zero_offset=zero_offset)
    bus = S.SimBus(robot)
    joints = jduck_run.Joints(bus, cal, smap, math.radians(5))
    imu = S.SimImu(robot)
    home = pol.home
    q = joints.move(home); q_old = q.copy()
    qd_f = np.zeros(14, np.float32); last = np.zeros(14, np.float32); cmd = np.zeros(13, np.float32)
    L = {k: [] for k in ("q", "target", "action", "grav", "cmd")}
    for tick in range(int(T / DT)):
        t_used = robot.d.time
        w, g = imu.read()
        qd_f += pol.qd_ema * ((q - q_old) / DT - qd_f)
        obs = np.concatenate([w, g, q - home, qd_f, last, cmd]).astype(np.float32)
        act = pol(obs).astype(np.float32); last = act
        tgt = home + act * pol.action_scale
        for k, v in (("q", q), ("target", tgt), ("action", act), ("grav", g), ("cmd", cmd[:3])):
            L[k].append(np.array(v, np.float32))
        q_new = joints.move(tgt)
        q_old, q = q, q_new
        robot.step(max(0.0, DT - (robot.d.time - t_used)))
    return {k: np.stack(v) for k, v in L.items()} | {"home": home}


def table(rs, names):
    print(f"\n{'关节':8s}" + "".join(f" | {n:^33s}" for n in names))
    print(f"{'':8s}" + " | q−HOME 均值  q抖动std  平均动作  每步目标变化 主频" * 0
          + "".join(" | q−HOME  qstd  平均动作  抖动  主频 " for _ in names))
    for j in range(14):
        row = f"{SHORT[j]:8s}"
        for r in rs:
            row += (f" | {r['dq'][j]:+6.2f}° {r['qstd'][j]:4.2f}° {r['act'][j]:+6.2f}° "
                    f"{r['jit'][j]:5.2f}° {r['fdom'][j]:4.1f}Hz")
        print(row)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("log", nargs="?", type=Path, help="jduck_run.py --log 存的 .npz")
    p.add_argument("--policy", type=Path, default=None, help="仿真用的策略（默认取日志里记录的路径）")
    p.add_argument("--fw", default="8,0,0")
    p.add_argument("--sim-only", type=Path, default=None, help="没有真机日志时：只跑仿真这一侧")
    a = p.parse_args()
    import sim_robot as S
    v = [float(x) for x in a.fw.split(",")]
    fw = dict(S.SimRobot.FW_PD, K=v[0], FF=v[1], DB=v[2], floor=v[3] if len(v) > 3 else 0.0)

    if a.sim_only:
        s = sim_log(a.sim_only, fw)
        sl = slice(int(3 / DT), None)
        rs = stats(s["q"][sl], s["target"][sl], s["action"][sl], s["grav"][sl], s["home"])
        table([rs], ["仿真"])
        print(f"\n仿真躯干倾斜：pitch {rs['pitch']:+.1f}°  roll {rs['roll']:+.1f}°；腿每步目标变化平均 {rs['jit_all']:.3f}°")
        return

    d = dict(np.load(a.log, allow_pickle=True))
    seg = zero_cmd_segment(d)
    home = d["home"]
    real = stats(d["q"][seg], d["target"][seg], d["action"][seg], d["grav"][seg], home)
    pol = a.policy or Path(str(d["policy"]))
    s = sim_log(pol, fw)
    sl = slice(int(3 / DT), None)
    sim = stats(s["q"][sl], s["target"][sl], s["action"][sl], s["grav"][sl], s["home"])
    dts = d["dt"][seg]
    print(f"真机日志 {a.log}：零速段 {(seg.stop - seg.start) * DT:.1f} s；控制周期 中位 {np.median(dts) * 1000:.1f} ms，"
          f"最大 {dts.max() * 1000:.1f} ms，超过 25 ms 的拍数 {(dts > 0.025).sum()}")
    table([real, sim], ["真机", "仿真"])
    print(f"\n躯干倾斜（IMU 重力）：真机 pitch {real['pitch']:+.1f}° roll {real['roll']:+.1f}°   "
          f"仿真 pitch {sim['pitch']:+.1f}° roll {sim['roll']:+.1f}°")
    print(f"腿每步目标变化平均：真机 {real['jit_all']:.3f}°   仿真 {sim['jit_all']:.3f}°")

    print("\n诊断：")
    dp, dr = real["pitch"] - sim["pitch"], real["roll"] - sim["roll"]
    if abs(dp) > 3 or abs(dr) > 3:
        print(f"  ⚠ 躯干比仿真多倾 pitch {dp:+.1f}° / roll {dr:+.1f}°：先查 IMU 安装角（imu.yaml），再查髋/踝零位")
    dact = real["act"] - sim["act"]
    sus = [j for j in range(14) if abs(dact[j]) > 1.5]
    for j in sus:
        print(f"  ⚠ {SHORT[j]}：平均动作比仿真多 {dact[j]:+.1f}°（策略一直往这个方向使劲）→ 这个关节的零位可能偏了约 {-dact[j]:+.1f}°")
    hot = [j for j in LEGS if real["jit"][j] > 2 * max(sim["jit"][j], 0.1)]
    for j in hot:
        kind = "逐拍来回（控制环振荡，看 q 量化 / 延迟 / 固件参数）" if real["fdom"][j] > 15 else "慢晃（平衡）"
        print(f"  ⚠ {SHORT[j]} 抖得比仿真厉害：{real['jit'][j]:.2f}° vs {sim['jit'][j]:.2f}°，主频 {real['fdom'][j]:.1f} Hz → {kind}")
    if (dts > 0.025).sum() > 0.02 * len(dts):
        print("  ⚠ 控制周期经常超过 25 ms：观测会比训练时更旧，这本身就会引起抖动")
    if not (sus or hot) and abs(dp) <= 3 and abs(dr) <= 3:
        print("  真机和仿真看不出明显差别。")
    print("  （零位标错在这里看不出来：机器人读数和动作是自洽的，零位要用量角器 / 已知姿态对照 calibrate.py monitor 来查）")


if __name__ == "__main__":
    main()
