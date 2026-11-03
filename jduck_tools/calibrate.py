#!/usr/bin/env python3
"""jduck joint calibration tool for UBT-12HC bus servos.

Run it with the system python (pyserial + pyyaml), e.g. from rl_control/:

    python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 scan
    python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 monitor          # all relaxed, live ADC
    python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 measure-k --id 3 # counts per radian
    python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 joint            # calibrate joints one by one
    python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 joint left_knee  # just one joint
    python3 jduck_tools/calibrate.py report                               # summary + URDF ranges (no port)
    python3 jduck_tools/calibrate.py apply                                # write ranges into the URDF
    python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 verify left_knee # powered: small +/- move

Everything except `verify` keeps the motors OFF (CMD 0x03), so you can move the
joints by hand. `verify` turns ONE servo on and nudges it ±5° around where it
is. Hang the robot up (feet off the ground) before running it.

See the docstring of each subcommand below, and jduck_tools/README.md.
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibration import (  # noqa: E402
    JOINTS, Calibration, JointCal, apply_to_urdf, load_servo_map, read_urdf_limits,
)
from servo_bus import CMD_HOLD, CMD_MOVE, ServoBus  # noqa: E402

DEFAULT_CAL = HERE / "calibration.yaml"
DEFAULT_MAP = HERE / "servo_map.yaml"
DEFAULT_URDF = HERE.parent / "jduck_description" / "urdf" / "jduck_description.urdf"
JUMP_COUNTS = 60   # a jump this large between two ~20 ms reads is not hand motion


# ---------------------------------------------------------------- terminal helpers
@contextmanager
def raw_keys():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def key_pressed() -> str | None:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def status_line(text: str) -> None:
    sys.stdout.write("\r\033[K" + text)
    sys.stdout.flush()


# ---------------------------------------------------------------- subcommands
def cmd_scan(args, bus: ServoBus) -> None:
    """Probe IDs 1..N (relax) and list who answers."""
    found = bus.scan(range(1, args.max_id + 1))
    if not found:
        print("没有舵机应答。检查：供电、接线、波特率(230400)、--direction、串口名。")
        return
    servo_map = load_servo_map(args.map) if args.map.exists() else {}
    by_id = {v: k for k, v in servo_map.items()}
    print(f"找到 {len(found)} 个舵机：")
    for sid, r in sorted(found.items()):
        print(f"  ID {sid:3d}  ADC={r.pos:4d}  speed={r.speed_dps:+5d}°/s   map: {by_id.get(sid, '（servo_map 里没有）')}")
    missing = sorted(set(servo_map.values()) - set(found))
    if missing:
        print(f"servo_map 里有但没应答的 ID: {missing}")


def cmd_monitor(args, bus: ServoBus) -> None:
    """All mapped servos relaxed; live ADC table. Move a joint by hand to find its ID."""
    servo_map = load_servo_map(args.map)
    cal = Calibration.load(args.cal)
    print("全部卸力。用手掰关节，看哪一行在变。按 q 退出。\n")
    names = [n for n in JOINTS if n in servo_map]
    first = {}
    with raw_keys():
        while key_pressed() != "q":
            lines = []
            for n in names:
                r = bus.relax(servo_map[n])
                if r is None:
                    lines.append(f"  {n:16s} ID{servo_map[n]:3d}   ---- 无应答")
                    continue
                first.setdefault(n, r.pos)
                c = cal.joints.get(n)
                ang = (f"{math.degrees(c.to_rad(r.pos, cal.k_rad_per_count)):+7.1f}°"
                       if c and c.adc_zero is not None and c.sign else "     -  ")
                moved = r.pos - first[n]
                bar = "#" * min(20, abs(moved) // 5)
                lines.append(f"  {n:16s} ID{servo_map[n]:3d}  ADC={r.pos:4d}  Δ={moved:+5d} {ang} {bar}")
            sys.stdout.write("\033[H\033[J" + "\n".join(lines) + "\n\n按 q 退出\n")
            sys.stdout.flush()
            time.sleep(0.05)


def cmd_measure_k(args, bus: ServoBus) -> None:
    """Measure rad/count: record two poses a known angle apart (default 90°)."""
    print(f"舵机 ID {args.id} 卸力。量 k 需要两个已知角度差的姿态（默认 {args.angle_deg}°）。")
    print("建议：输出轴上装指针或贴量角器，或用手机水平仪 App 贴在舵盘上。")
    readings = []
    with raw_keys():
        for label in ("姿态 A", f"姿态 B（从 A 转过 {args.angle_deg}°）"):
            print(f"\n摆到{label}，按空格记录：")
            while True:
                r = bus.relax(args.id)
                status_line(f"  ADC = {r.pos if r else '----'}")
                if key_pressed() == " " and r is not None:
                    readings.append(r.pos)
                    print(f"  记录 {r.pos}")
                    break
                time.sleep(0.03)
    delta = abs(readings[1] - readings[0])
    if delta == 0:
        print("两次读数相同，没测到。")
        return
    k = math.radians(args.angle_deg) / delta
    print(f"\nΔADC = {delta}  →  k = {k:.6f} rad/计数 = {math.degrees(k):.4f}°/计数"
          f"（规格书标称 0.3516°/计数）")
    print(f"相当于满量程 1024 计数 ≈ {math.degrees(k) * 1024:.0f}°")
    cal = Calibration.load(args.cal)
    if input("写入 calibration.yaml？[y/N] ").strip().lower() == "y":
        cal.k_rad_per_count = k
        cal.save(args.cal)
        print(f"已保存 {args.cal}")


def _calibrate_one(name: str, sid: int, bus: ServoBus, cal: Calibration,
                   urdf_limits: dict[str, tuple[float, float]]) -> JointCal | None:
    c = JointCal(id=sid)
    lo, hi = urdf_limits.get(name, (float("nan"), float("nan")))
    print(f"\n=== {name}（ID {sid}）  当前 URDF 范围 [{lo:+.3f}, {hi:+.3f}] rad ===")
    print("先在 view_urdf.py / RViz 里看好：这个关节 0 位长什么样、往正方向转是哪边。")
    steps = [
        ("zero", "1) 摆到 URDF 的 0 位（和模型截图一致），按 [空格] 记录零位"),
        ("plus", "2) 从零位往 URDF **正方向** 转 20° 以上，按 [空格]（用来判断方向）"),
        ("sweep", "3) 慢慢扫过两端（碰到机械限位或别的零件就停），回到中间，按 [空格] 结束"),
    ]
    sweep_min, sweep_max, last = None, None, None
    with raw_keys():
        for key, prompt in steps:
            print(prompt + "   ([s] 跳过这个关节, [q] 退出)")
            while True:
                r = bus.relax(sid)
                k = key_pressed()
                if k in ("s", "q"):
                    print()
                    return None if k == "s" else "quit"   # type: ignore[return-value]
                if r is None:
                    status_line("  ---- 无应答")
                    time.sleep(0.05)
                    continue
                if key == "sweep":
                    if last is not None and abs(r.pos - last) > JUMP_COUNTS:
                        c.jumps += 1
                    last = r.pos
                    sweep_min = r.pos if sweep_min is None else min(sweep_min, r.pos)
                    sweep_max = r.pos if sweep_max is None else max(sweep_max, r.pos)
                    status_line(f"  ADC={r.pos:4d}   已扫到 [{sweep_min}, {sweep_max}]   跳变 {c.jumps}")
                else:
                    extra = f"  (Δ相对零位 {r.pos - c.adc_zero:+d})" if c.adc_zero is not None else ""
                    status_line(f"  ADC={r.pos:4d}{extra}")
                if k == " ":
                    if key == "zero":
                        c.adc_zero = r.pos
                    elif key == "plus":
                        d = r.pos - c.adc_zero
                        if abs(d) < 20:
                            print(f"\n  只动了 {d} 个计数，转大一点再按空格")
                            continue
                        c.sign = 1 if d > 0 else -1
                    else:
                        c.adc_min, c.adc_max = sweep_min, sweep_max
                    print(f"  ✓")
                    break
                time.sleep(0.02)
    lo_q, hi_q = c.urdf_range(cal.k_rad_per_count, cal.margin_rad)
    print(f"  零位 ADC={c.adc_zero}  方向 sign={c.sign:+d}  ADC 范围 [{c.adc_min}, {c.adc_max}]")
    print(f"  → URDF 范围（含 {math.degrees(cal.margin_rad):.0f}° 余量）[{lo_q:+.3f}, {hi_q:+.3f}] rad"
          f" = [{math.degrees(lo_q):+.1f}°, {math.degrees(hi_q):+.1f}°]")
    return c


def cmd_joint(args, bus: ServoBus) -> None:
    """Calibrate joints (zero, direction, range) by hand, motors off."""
    servo_map = load_servo_map(args.map)
    cal = Calibration.load(args.cal)
    urdf_limits = read_urdf_limits(args.urdf) if args.urdf.exists() else {}
    names = args.names or [n for n in JOINTS if n in servo_map]
    for n in names:
        if n not in servo_map:
            print(f"{n} 不在 servo_map.yaml 里，跳过")
            continue
        res = _calibrate_one(n, servo_map[n], bus, cal, urdf_limits)
        if res == "quit":
            break
        if res is not None:
            cal.joints[n] = res
            cal.save(args.cal)   # save after every joint so nothing is lost
            print(f"  已保存到 {args.cal}")
    for w in cal.warnings():
        print("⚠", w)


def cmd_report(args) -> None:
    """Print calibration status and the URDF ranges it implies."""
    cal = Calibration.load(args.cal)
    urdf_limits = read_urdf_limits(args.urdf) if args.urdf.exists() else {}
    print(f"k = {cal.k_rad_per_count:.6f} rad/计数 ({math.degrees(cal.k_rad_per_count):.4f}°)  "
          f"余量 = {math.degrees(cal.margin_rad):.1f}°\n")
    print(f"{'关节':16s} {'ID':>3s} {'零位':>5s} {'方向':>4s} {'ADC范围':>12s}   {'新 URDF 范围 (rad)':>20s}   {'旧 URDF 范围':>18s}")
    for n in JOINTS:
        c = cal.joints.get(n)
        old = urdf_limits.get(n)
        old_s = f"[{old[0]:+.2f}, {old[1]:+.2f}]" if old else "-"
        if c is None or not c.complete:
            print(f"{n:16s} {'-':>3s} {'未标定':>5s}{'':38s}   {old_s:>18s}")
            continue
        lo, hi = c.urdf_range(cal.k_rad_per_count, cal.margin_rad)
        print(f"{n:16s} {c.id:3d} {c.adc_zero:5d} {c.sign:+4d} [{c.adc_min:4d},{c.adc_max:4d}]   "
              f"[{lo:+.3f}, {hi:+.3f}]       {old_s:>18s}")
    ws = cal.warnings()
    if ws:
        print()
        for w in ws:
            print("⚠", w)


def cmd_apply(args) -> None:
    """Write calibrated ranges into the URDF <limit> tags (backs up the URDF first)."""
    cal = Calibration.load(args.cal)
    done = [n for n in JOINTS if n in cal.joints and cal.joints[n].complete]
    if not done:
        print("还没有标定好的关节。")
        return
    cmd_report(args)
    if input(f"\n把上面 {len(done)} 个关节的新范围写进 {args.urdf}？[y/N] ").strip().lower() != "y":
        return
    changed = apply_to_urdf(cal, args.urdf)
    print(f"已更新 {len(changed)} 个关节（原文件备份为 {args.urdf.with_suffix('.urdf.precal.bak').name}）。")
    print("接下来：重新生成 MJCF 并重找 HOME：\n"
          "  cd third_party/microduck_rl && uv run --frozen python ../../jduck_mjcf/build_mjcf.py")


def cmd_verify(args, bus: ServoBus) -> None:
    """POWERED check of one joint: nudge +step / -step around its current pose.

    The joint should move towards URDF + and then URDF -. If it moves the other
    way, the sign is wrong. Hang the robot up first.
    """
    servo_map = load_servo_map(args.map)
    cal = Calibration.load(args.cal)
    n = args.name
    c = cal.joints.get(n)
    if c is None or not c.complete:
        print(f"{n} 还没标定完。")
        return
    sid = servo_map[n]
    r = bus.relax(sid)
    if r is None:
        print("无应答。")
        return
    start = r.pos
    step = round(math.radians(args.step_deg) / cal.k_rad_per_count)
    lo_adc, hi_adc = sorted((c.adc_min, c.adc_max))
    print(f"⚠ 即将给 {n}（ID {sid}）上电。机器人要吊起来、手离开关节。当前 ADC={start}，"
          f"每次动 ±{args.step_deg}°（{step} 计数）。")
    if input("继续？[y/N] ").strip().lower() != "y":
        return
    try:
        bus.transact(sid, CMD_HOLD)
        for label, dq in (("URDF 正方向", +1), ("回到起点", 0), ("URDF 负方向", -1), ("回到起点", 0)):
            target = round(start + c.sign * dq * step)
            target = max(lo_adc, min(hi_adc, target))
            bus.transact(sid, CMD_MOVE, target)
            time.sleep(args.dwell)
            r = bus.transact(sid, CMD_HOLD)
            got = r.pos if r else None
            print(f"  {label:10s} 目标 ADC={target:4d}  实际 {got}  "
                  f"({math.degrees(c.to_rad(got, cal.k_rad_per_count)):+.1f}° URDF)" if got is not None
                  else f"  {label:10s} 无应答")
            if input("  方向对吗？[Y/n] ").strip().lower() == "n":
                print("  → 方向反了：重新标定这个关节（joint 子命令），注意第 2 步要往 URDF 正方向转。")
                break
    finally:
        bus.relax(sid)
        print("已卸力。")


def cmd_speed_test(args, bus: ServoBus) -> None:
    """No-load speed: big step targets (firmware saturates at PWM 255 when
    |err| >= 255 counts), poll the position, and fit the slope
    of the constant-speed middle part. The joint must be free to swing (robot hung
    up, or the servo on the bench without load)."""
    k = math.radians(360.0 / 1024)
    sid = args.id
    r = bus.relax(sid)
    if r is None:
        print("无应答。")
        return
    lo, hi = args.lo, args.hi
    print(f"⚠ 舵机 ID {sid} 将在 ADC {lo} ↔ {hi}（≈{(hi - lo) * 0.3516:.0f}°）之间全速来回 {args.reps} 次。"
          "这段行程里不能有东西挡住！")
    if input("继续？[y/N] ").strip().lower() != "y":
        return
    speeds = []
    try:
        bus.transact(sid, CMD_MOVE, lo)
        time.sleep(1.5)
        for rep in range(args.reps):
            for tgt in (hi, lo):
                bus.transact(sid, CMD_MOVE, tgt)
                t0 = time.monotonic()
                log = []
                while time.monotonic() - t0 < args.window:
                    # Re-send the MOVE to poll. CMD 0x02 would replace the target
                    # with the current position; a MOVE reply carries the position too.
                    rr = bus.transact(sid, CMD_MOVE, tgt)
                    if rr is not None:
                        log.append((time.monotonic() - t0, rr.pos))
                ps = [p for t, p in log]
                start = hi if tgt == lo else lo
                # The firmware only gives PWM 255 while |target - pos| >= 255 counts
                # (pwm = clamp(|err|, 48, 255)), so the joint runs at no-load speed
                # only in that part. Skip the first ~40 counts (acceleration) and
                # fit while the remaining error is still >= PWM_SAT counts. A fit
                # over the whole travel underestimates the speed about 2x.
                sel = [(t, p) for t, p in log
                       if abs(p - start) >= 40 and abs(tgt - p) >= args.pwm_sat]
                if len(sel) < 4:
                    print(f"  第 {rep + 1} 次 → {tgt}: 样本太少（{len(log)} 个），"
                          f"终点读数 {ps[-1] if ps else None}，加大 --window 或检查是否卡住")
                    continue
                tt = [t for t, _ in sel]
                pp = [p for _, p in sel]
                n = len(tt)
                mt, mp = sum(tt) / n, sum(pp) / n
                slope = sum((a - mt) * (b - mp) for a, b in zip(tt, pp)) / sum((a - mt) ** 2 for a in tt)
                w = abs(slope) * k
                speeds.append(w)
                print(f"  第 {rep + 1} 次 → {tgt}: {math.degrees(w):6.1f} °/s = {w:4.2f} rad/s  ({n} 个点)")
    finally:
        bus.relax(sid)
    if speeds:
        w = sum(speeds) / len(speeds)
        print(f"\n实测全速 ≈ {w:.2f} rad/s = {math.degrees(w):.0f} °/s = {math.radians(60) / w:.3f} s/60°")
        # The reading is below the motor's no-load speed (joint friction/damping,
        # inertia over a finite stroke). Map it back with the same test run on
        # the sim model (sim_robot.py, current firmware, left knee, ADC 234↔848):
        #   model no_load 2/3/4/6/8/10 rad/s  →  this test reads 1.73/2.46/3.11/4.21/5.09/5.78
        table = [(1.73, 2.0), (2.46, 3.0), (3.11, 4.0), (4.21, 6.0), (5.09, 8.0), (5.78, 10.0)]
        if w <= table[0][0]:
            w_model = w * table[0][1] / table[0][0]
        elif w >= table[-1][0]:
            w_model = table[-1][1] + (w - table[-1][0]) * 2.9   # last segment slope, rough
            print("  （超出标定表，仿真参数是外推的）")
        else:
            for (a, wa), (b, wb) in zip(table, table[1:]):
                if a <= w <= b:
                    w_model = wa + (w - a) / (b - a) * (wb - wa)
                    break
        print(f"→ 仿真模型参数 no_load_speed ≈ {w_model:.1f} rad/s"
              f"（训练时 JDUCK_NOLOAD={w_model:.1f}；URDF 现在是 3.0）")
        print("  表是用左膝、当前固件、ADC 234↔848 标的；其它关节/行程会有偏差。")


# ---------------------------------------------------------------- main
def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="串口，例如 /dev/ttyUSB0（report/apply 不需要）")
    p.add_argument("--baud", type=int, default=230400)
    p.add_argument("--direction", default="none", choices=["none", "rts", "rts-inv"],
                   help="半双工方向切换：none=模块自动；rts / rts-inv=用 RTS 脚控制")
    p.add_argument("--timeout", type=float, default=0.02, help="等待应答的超时（秒）")
    p.add_argument("--cal", type=Path, default=DEFAULT_CAL)
    p.add_argument("--map", type=Path, default=DEFAULT_MAP)
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="扫描总线上应答的 ID")
    s.add_argument("--max-id", type=int, default=20)
    sub.add_parser("monitor", help="全部卸力，实时显示各关节 ADC")
    s = sub.add_parser("measure-k", help="测 rad/计数")
    s.add_argument("--id", type=int, required=True)
    s.add_argument("--angle-deg", type=float, default=90.0)
    s = sub.add_parser("joint", help="逐个关节标定零位/方向/范围（卸力）")
    s.add_argument("names", nargs="*", choices=JOINTS + [], metavar="joint")
    sub.add_parser("report", help="打印标定结果和对应的 URDF 范围")
    sub.add_parser("apply", help="把标定范围写进 URDF")
    s = sub.add_parser("speed-test", help="测空载转速（大幅全速往返，关节必须能自由转）")
    s.add_argument("--id", type=int, required=True)
    s.add_argument("--lo", type=int, default=200, help="ADC 起点（要在该舵机的标定范围内）")
    s.add_argument("--hi", type=int, default=800, help="ADC 终点；行程要 > 300 计数才有全速段")
    s.add_argument("--pwm-sat", type=int, default=255,
                   help="固件 PWM 饱和所需误差（计数）。当前固件 pwm=|err| → 255；改成 pwm=K|err|+FF 后填 (255-FF)/K")
    s.add_argument("--reps", type=int, default=3)
    s.add_argument("--window", type=float, default=2.0, help="每次运动记录多少秒")
    s = sub.add_parser("verify", help="上电小幅动一下，检查方向（先吊起来）")
    s.add_argument("name", choices=JOINTS)
    s.add_argument("--step-deg", type=float, default=5.0)
    s.add_argument("--dwell", type=float, default=0.6)
    args = p.parse_args()

    if args.cmd == "report":
        return cmd_report(args)
    if args.cmd == "apply":
        return cmd_apply(args)
    if not args.port:
        p.error(f"{args.cmd} 需要 --port")
    try:
        bus = ServoBus(args.port, args.baud, args.timeout, args.direction)
    except Exception as e:  # serial.SerialException, PermissionError, ...
        sys.exit(f"打不开串口 {args.port}: {e}\n"
                 "检查：USB 是否插好（ls /dev/ttyUSB* /dev/ttyACM*）、用户是否在 dialout 组。")
    with bus:
        {"scan": cmd_scan, "monitor": cmd_monitor, "measure-k": cmd_measure_k,
         "joint": cmd_joint, "verify": cmd_verify, "speed-test": cmd_speed_test}[args.cmd](args, bus)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n中断。")
