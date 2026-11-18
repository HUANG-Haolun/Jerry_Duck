"""Run the current traditional-gait joint-pose prototype in MuJoCo, no hardware."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import sim_robot as S  # noqa: E402
from calibration import JOINTS, Calibration, load_servo_map  # noqa: E402
from servo_bus import CMD_MOVE  # noqa: E402


def main() -> None:
    p = argparse.ArgumentParser(description="在 MuJoCo 中仿真传统步态关节姿态（无真机串口）")
    p.add_argument("--action", choices=("stand", "shift_left", "shift_right", "lift_left", "lift_right", "cycle"), default="cycle")
    p.add_argument("--move", type=float, default=1.0, help="每个姿态的插值时间，秒")
    p.add_argument("--hold", type=float, default=1.0, help="姿态保持时间，秒")
    p.add_argument("--shift-deg", type=float, default=3.0)
    p.add_argument("--left-roll-deg", type=float, help="左髋Roll偏移；用于重心转移参数试验")
    p.add_argument("--right-roll-deg", type=float, help="右髋Roll偏移；用于重心转移参数试验")
    p.add_argument("--knee-bend-deg", type=float, default=0.0, help="重心转移时双膝弯曲量")
    p.add_argument("--ankle-comp-deg", type=float, default=0.0, help="重心转移时双踝Pitch补偿量")
    p.add_argument("--hip-pitch-deg", type=float, default=0.0, help="重心转移时双髋Pitch偏移量")
    p.add_argument("--lift-deg", type=float, default=3.0)
    p.add_argument("--step-length", type=float, default=0.02, help="摆动脚前移距离，米")
    p.add_argument("--step-height", type=float, default=0.01, help="摆动脚抬高距离，米")
    p.add_argument("--fall-deg", type=float, default=45.0)
    p.add_argument("--scene", choices=("walk", "groundcontact"), default="walk")
    p.add_argument("--csv", type=Path, help="可选：保存每帧姿态/脚底高度 CSV")
    a = p.parse_args()
    if not (0 < a.move <= 10 and 0 <= a.hold <= 30 and 0 < a.fall_deg < 90
            and 0 <= a.step_length <= 0.06 and 0 <= a.step_height <= 0.03):
        p.error("move/hold/fall 参数超出范围")

    cal = Calibration.load(HERE / "calibration.yaml")
    smap = load_servo_map(HERE / "servo_map.yaml")
    home = np.zeros(len(JOINTS), dtype=float)
    scene = S.SCENE_GC if a.scene == "groundcontact" else S.SCENE
    robot = S.SimRobot(cal, smap, home, scene=scene)
    dt = 0.02
    samples: list[tuple[float, float, float, float, float]] = []

    def pose(name: str) -> np.ndarray:
        q = home.copy()
        def put(joint: str, deg: float) -> None:
            if joint in JOINTS:
                q[JOINTS.index(joint)] = math.radians(deg)
        if name in ("shift_left", "shift_right"):
            side = 1 if name == "shift_left" else -1
            left_roll = a.left_roll_deg if a.left_roll_deg is not None else a.shift_deg
            right_roll = a.right_roll_deg if a.right_roll_deg is not None else a.shift_deg
            put("left_hip_roll", side * left_roll)
            put("right_hip_roll", side * right_roll)
            for leg in ("left", "right"):
                put(f"{leg}_knee", a.knee_bend_deg)
                put(f"{leg}_ankle", a.ankle_comp_deg)
                put(f"{leg}_hip_pitch", a.hip_pitch_deg)
        elif name in ("lift_left", "lift_right"):
            side = 1 if name == "lift_left" else -1
            leg = "left" if side > 0 else "right"
            put(f"{leg}_hip_pitch", side * a.lift_deg / 2)
            put(f"{leg}_knee", a.lift_deg)
            put(f"{leg}_ankle", -side * a.lift_deg / 2)
        return q

    def record(stage: str, target: np.ndarray, duration: float) -> bool:
        start = robot.d.qpos[robot.qa].copy()
        count = max(1, round(duration / dt))
        swing_leg = "right" if stage == "lift_right" else "left" if stage == "lift_left" else None
        site_id = mujoco.mj_name2id(robot.m, mujoco.mjtObj.mjOBJ_SITE,
                                    f"{swing_leg}_foot") if swing_leg else -1
        swing_joints = ([JOINTS.index(f"{swing_leg}_{suffix}")
                         for suffix in ("hip_pitch", "knee", "ankle")] if swing_leg else [])
        foot_start = robot.d.site(site_id).xpos.copy() if swing_leg else None

        def foot_ik(goal: np.ndarray) -> np.ndarray:
            """Damped least-squares MuJoCo site IK for the swing leg's sagittal joints."""
            qsave = robot.d.qpos[robot.qa].copy()
            dofs = robot.va[swing_joints]
            qadr = robot.qa[swing_joints]
            jacp = np.zeros((3, robot.m.nv))
            jacr = np.zeros((3, robot.m.nv))
            for _ in range(35):
                mujoco.mj_forward(robot.m, robot.d)
                err = goal - robot.d.site(site_id).xpos
                if np.linalg.norm(err) < 2e-4:
                    break
                mujoco.mj_jacSite(robot.m, robot.d, jacp, jacr, site_id)
                J = jacp[:, dofs]
                dq = J.T @ np.linalg.solve(J @ J.T + 2e-4 * np.eye(3), err)
                robot.d.qpos[qadr] += np.clip(dq, -0.12, 0.12)
            result = robot.d.qpos[robot.qa].copy()
            robot.d.qpos[robot.qa] = qsave
            mujoco.mj_forward(robot.m, robot.d)
            return result

        for n in range(count + 1):
            u = n / count
            blend = u * u * (3 - 2 * u)
            qcmd = start + blend * (target - start)
            if swing_leg:
                # Smooth fore-aft swing and a half-sine clearance arc.
                goal = foot_start.copy()
                goal[0] += a.step_length * blend
                goal[2] += a.step_height * math.sin(math.pi * u)
                qcmd = foot_ik(goal)
            for i, joint in enumerate(JOINTS):
                adc = int(round(cal.joints[joint].to_adc(float(qcmd[i]), cal.k_rad_per_count)))
                reply = robot.command(smap[joint], CMD_MOVE, adc)
                if reply is None:
                    raise RuntimeError(f"模拟器未找到关节 {joint}")
            robot.step(dt)
            left = robot.d.site("left_foot").xpos[2]
            right = robot.d.site("right_foot").xpos[2]
            tilt = robot.tilt_deg()
            samples.append((robot.d.time, tilt, left, right, float(robot.d.qpos[2])))
            if tilt >= a.fall_deg:
                print(f"摔倒阈值触发: stage={stage}, t={robot.d.time:.2f}s, tilt={tilt:.1f}°")
                return False
        com = robot.d.subtree_com[1]
        left_y = robot.d.site("left_foot").xpos[1]
        right_y = robot.d.site("right_foot").xpos[1]
        support = "right" if stage == "shift_right" else "left" if stage in ("shift_left", "lift_right") else "both"
        print(f"{stage}: tilt={robot.tilt_deg():.2f}°, COM_y={com[1]:.4f}m, "
              f"feet_y=({left_y:.4f},{right_y:.4f})m, support={support}, "
              f"foot_z=({left:.4f},{right:.4f})m")
        return True

    sequence = [a.action] if a.action != "cycle" else [
        "stand", "shift_left", "lift_right", "stand", "shift_right", "lift_left", "stand"
    ]
    for stage in sequence:
        if not record(stage, pose(stage), a.move):
            break
        if a.hold:
            if not record("hold", pose(stage), a.hold):
                break
    if a.csv:
        a.csv.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(a.csv, np.asarray(samples), delimiter=",",
                   header="time_s,tilt_deg,left_foot_z_m,right_foot_z_m,trunk_z_m", comments="")
        print(f"CSV: {a.csv}")


if __name__ == "__main__":
    main()
