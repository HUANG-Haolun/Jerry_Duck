"""Policies in the robot-calibrated sim (real_calib.py's replica of the board's jduck_run.py loop
+ the fitted servo params): zero-command stand next to the real log, plus (--walk) every walking
direction the board allows (--max-vx 0.08 --max-vy 0.05 --max-wz 0.25, and the full training range).

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_tools/calib_stand_eval.py A.onnx [B.onnx ...] [--log ~/jduck_ops/stand_O1.npz]
"""
import argparse, math, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import real_calib as R, jduck_run, sim_robot as S  # noqa: E402
from calibration import Calibration, load_servo_map  # noqa: E402

# best fit to stand_O1.npz (2026-11-10): friction 0, stall 1.4 N·m, +10 ms latency, damping 0.05
FIT = dict(friction=0.0, stall=1.4, backlash=0.0, extra_delay_ms=5.0, damping=0.05, no_load=5.5)
# 2026-11-12 step-test (left leg hung, 5–40° steps, 7.2 V): stall 1.25–1.4, no-load 5.5 rad/s, +5 ms latency
# reproduce 10/50/90 % rise times and peak speed within ~10 % (was 4.8 rad/s / 10 ms).
# the board's key limits (jduck_run.py --max-vx 0.08 --max-vy 0.05 --max-wz 0.25) and the training range
WALK_CASES = [("前进 0.08", (0.08, 0, 0)), ("后退 0.08", (-0.08, 0, 0)), ("左移 0.05", (0, 0.05, 0)),
              ("右移 0.05", (0, -0.05, 0)), ("左转 0.25", (0, 0, 0.25)), ("右转 0.25", (0, 0, -0.25)),
              ("前进 0.2", (0.2, 0, 0)), ("后退 0.2", (-0.2, 0, 0))]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("onnx", nargs="+", type=Path)
    p.add_argument("--log", type=Path, default=Path.home() / "jduck_ops" / "stand_O1.npz")
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed0", type=int, default=0, help="first seed (re-check with seeds the gate did not use)")
    p.add_argument("--walk", action="store_true", help="also run every walking direction (15 s each)")
    p.add_argument("--fw", default="orig", help="'orig' = stock firmware, or K,FF,DB of firmware/ubt12hc_pd (e.g. 8,0,0)")
    p.add_argument("--wz-scale", type=float, default=1.0, help="= jduck_run --wz-scale: yaw sent to the policy = key × this")
    p.add_argument("--stand-hold", action="store_true",
                   help="walk runs use jduck_run.StandHold and add a walk→stop→stand case per direction")
    p.add_argument("--stand-hip-pitch-deg", type=float, default=0.0,
                   help="with --stand-hold, offset the standing target: left hip pitch +deg, right -deg")
    p.add_argument("--stand-ankle-deg", type=float, default=0.0,
                   help="with --stand-hold, offset the standing target: left ankle +deg, right -deg")
    p.add_argument("--stall", type=float, default=None,
                   help="servo stall torque N·m (default = the fit, 1.4). 2026-11-11: O8d passed at 1.4 but "
                        "leaned back / fell at ≤ 0.96 on the robot — gate at both ends of the plausible range")
    a = p.parse_args()
    if not math.isfinite(a.stand_hip_pitch_deg) or abs(a.stand_hip_pitch_deg) > 10:
        p.error("--stand-hip-pitch-deg 必须在 -10 到 10 度之间")
    if a.stand_hip_pitch_deg and not a.stand_hold:
        p.error("--stand-hip-pitch-deg 需要 --stand-hold")
    if not math.isfinite(a.stand_ankle_deg) or abs(a.stand_ankle_deg) > 10:
        p.error("--stand-ankle-deg 必须在 -10 到 10 度之间")
    if a.stand_ankle_deg and not a.stand_hold:
        p.error("--stand-ankle-deg 需要 --stand-hold")
    global FW
    FW = (dict(S.SimRobot.FW_ORIG) if a.fw == "orig" else
          dict(K=float(a.fw.split(",")[0]), FF=float(a.fw.split(",")[1]), DB=float(a.fw.split(",")[2]),
               floor=0, timeout_ms=200, params=True))
    if a.stall is not None:
        FIT["stall"] = a.stall
    cal = Calibration.load(HERE / "calibration.yaml"); smap = load_servo_map(HERE / "servo_map.yaml")
    stand_home_offset = np.zeros(len(jduck_run.JOINTS), np.float32)
    stand_home_offset[jduck_run.JOINTS.index("left_hip_pitch")] = np.radians(a.stand_hip_pitch_deg)
    stand_home_offset[jduck_run.JOINTS.index("right_hip_pitch")] = -np.radians(a.stand_hip_pitch_deg)
    stand_home_offset[jduck_run.JOINTS.index("left_ankle")] = np.radians(a.stand_ankle_deg)
    stand_home_offset[jduck_run.JOINTS.index("right_ankle")] = -np.radians(a.stand_ankle_deg)
    real = dict(np.load(a.log, allow_pickle=True))
    wire = float(np.median((real["el"] - real["t_pol"]) / real["active"].sum(1)))
    print(f"{'':28s} " + "  ".join(f"{n:>10s}" for n in R.NAMES))
    print(f"{'真机 ' + a.log.name:28s} " + "  ".join(f"{v:10.3f}" for v in R.feats(real)))
    for path in a.onnx:
        pol = jduck_run.Policy(path)
        fs, fell, poses, pitches = [], 0, [], []
        for s in range(a.seed0, a.seed0 + a.seeds):
            d = R.run_sim(pol, cal, smap, dict(FW), FIT["friction"], FIT["stall"], FIT["backlash"],
                          FIT["extra_delay_ms"], wire, seed=s, damping=FIT["damping"], no_load=FIT["no_load"],
                          stand_hold=jduck_run.StandHold() if a.stand_hold else None,
                          stand_home_offset=stand_home_offset if a.stand_hold else None)
            if d is None:
                fell += 1
            else:
                fs.append(R.feats(d))
                dq = np.degrees(d["q"][300:] - pol.home - (stand_home_offset if a.stand_hold else 0)).mean(0)
                poses.append([dq[2], dq[3], dq[11], dq[12], *dq[[0, 1, 4, 9, 10, 13]]])
                pitches.append(float(np.degrees(np.arctan2(d["grav"][300:, 0], -d["grav"][300:, 2])).mean()))
        line = "  ".join(f"{v:10.3f}" for v in np.mean(fs, 0)) if fs else "全摔"
        print(f"{'校准仿真 ' + path.name:28s} {line}" + (f"   摔 {fell}/{a.seeds}" if fell else ""))
        if poses:
            # standing posture: mean |q − HOME| of hip_pitch / knee (both legs) after 6 s. 2026-11-11: O8d
            # stood with knees bent ±4–7° and asymmetric hip pitch, tipped forward on the robot; O1 stands
            # within ±0.3° of HOME there. Gate: ≤ 3°.
            P = np.mean(poses, 0)
            # 2026-11-12: O15b passed hip pitch/knee at 4.5° but stood with left hip roll −4~−10° and ankle +7°
            # (right foot forward, fell on the robot) → the max now covers all 10 leg joints.
            print(f"      站姿 髋pitch 左 {P[0]:+5.1f}° 右 {P[2]:+5.1f}°  膝 左 {P[1]:+5.1f}° 右 {P[3]:+5.1f}°"
                  f"  髋yaw {P[4]:+5.1f}/{P[7]:+5.1f}  髋roll {P[5]:+5.1f}/{P[8]:+5.1f}  踝 {P[6]:+5.1f}/{P[9]:+5.1f}"
                  f"  最大偏离 {np.abs(P).max():.1f}°")
            # trunk pitch at zero command (+ forward). 2026-11-12: O18a −2.9° here, −7.4° on the robot, fell
            # backwards; O1 −0.2° here.
            print(f"      躯干俯仰 {np.mean(pitches):+.1f}°")
        if not a.walk:
            continue
        T = 15.0
        for name, c in WALK_CASES:
            c = (c[0], c[1], c[2] * a.wz_scale)
            rs = [R.run_sim(pol, cal, smap, dict(FW), FIT["friction"], FIT["stall"], FIT["backlash"],
                            FIT["extra_delay_ms"], wire, seed=s, damping=FIT["damping"], cmd3=c, T=T, no_load=FIT["no_load"],
                            stand_hold=jduck_run.StandHold() if a.stand_hold else None,
                            stand_home_offset=stand_home_offset if a.stand_hold else None) for s in range(a.seed0, a.seed0 + a.seeds)]
            ok = [r for r in rs if r is not None]
            if not ok:
                print(f"      {name:16s} 全摔 {len(rs)}/{len(rs)}"); continue
            dx = np.mean([r["dxy"] for r in ok], 0); yaw = np.mean([r["yaw"] for r in ok])
            # walking shake: leg-target content above 8 Hz (2026-11-16: N1 at the step-test servo fit shook
            # 3.1° here, 0.5° at the 0.96 N·m it was trained on — the robot shook as soon as it walked)
            def _hf(r):
                x = r["target"][150:][:, R.LEGS]; x = x - x.mean(0)
                X = np.fft.rfft(x, axis=0); X[np.fft.rfftfreq(len(x), R.DT) < 8] = 0
                return float(np.degrees(np.fft.irfft(X, n=len(x), axis=0)).std(0).mean())
            hf = np.mean([_hf(r) for r in ok])
            ideal = (c[0] * T, c[1] * T, math.degrees(c[2] / a.wz_scale * T))
            print(f"      {name:16s} dx {dx[0]:+.2f} dy {dx[1]:+.2f} m（理想 {ideal[0]:+.1f},{ideal[1]:+.1f}）"
                  f" 偏航 {yaw:+5.0f}°（理想 {ideal[2]:+.0f}°） 高频抖 {hf:.2f}°" + (f"  摔 {len(rs) - len(ok)}/{len(rs)}" if len(ok) < len(rs) else ""))


        if a.stand_hold:
            # walk 4 s then stop: StandHold fades to HOME; must still be upright 6 s later
            for name, c in WALK_CASES[:6]:
                rs = [R.run_sim(pol, cal, smap, dict(FW), FIT["friction"], FIT["stall"], FIT["backlash"],
                                FIT["extra_delay_ms"], wire, seed=s, damping=FIT["damping"], T=11.0, no_load=FIT["no_load"],
                                stand_hold=jduck_run.StandHold(), cmd_sched=[(1.0, (c[0], c[1], c[2] * a.wz_scale)), (5.0, (0.0, 0.0, 0.0))],
                                stand_home_offset=stand_home_offset)
                      for s in range(a.seed0, a.seed0 + a.seeds)]
                fell = sum(r is None for r in rs)
                print(f"      停 {name:14s} " + (f"摔 {fell}/{len(rs)}" if fell else "站住"))


if __name__ == "__main__":
    main()
