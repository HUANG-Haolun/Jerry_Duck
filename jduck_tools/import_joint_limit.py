#!/usr/bin/env python3
"""Import the friend's `joint_limit.txt` (servo_ID / init_pose / max / min / link)
into calibration.yaml + servo_map.yaml.

Assumptions (check with `calibrate.py verify` on the robot):
  - init_pose is the ADC reading at the URDF 0 pose
  - 1024 counts / 360° (the spec)
  - sign is the one whose range best reproduces the URDF <limit>s, which were
    evidently entered from this same table. For 4 joints the two signs fit about
    equally badly; those are listed as UNSURE.

    python3 jduck_tools/import_joint_limit.py [jduck_tools/joint_limit.txt]
"""

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibration import JOINTS, Calibration, JointCal, read_urdf_limits  # noqa: E402

SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "joint_limit.txt"
URDF = HERE.parent / "jduck_description" / "urdf" / "jduck_description.urdf"
CAL = HERE / "calibration.yaml"
MAP = HERE / "servo_map.yaml"
SURE_RATIO = 2.0   # the better sign must fit at least this much better


def main() -> None:
    rows = [ln.split() for ln in SRC.read_text().splitlines()[1:] if ln.strip()]
    lim = read_urdf_limits(URDF)
    cal = Calibration.load(CAL)
    k = cal.k_rad_per_count
    servo_ids, unsure, extra = {}, [], {}
    for sid, init, mx, mn, link in rows:
        j = link.removesuffix("_link")
        sid, init, mx, mn = int(sid), int(init), int(mx), int(mn)
        if j not in JOINTS:
            extra[j] = sid           # e.g. mouth (passive in sim)
            continue
        servo_ids[j] = sid
        lo, hi = lim[j]
        err = {}
        for s in (1, -1):
            a, b = sorted((s * (mn - init) * k, s * (mx - init) * k))
            err[s] = abs(a - lo) + abs(b - hi)
        sign = min(err, key=err.get)
        if err[-sign] < SURE_RATIO * err[sign]:
            unsure.append(f"{j} (err +1={err[1]:.2f}, -1={err[-1]:.2f})")
        cal.joints[j] = JointCal(id=sid, adc_zero=init, sign=sign, adc_min=mn, adc_max=mx)
    cal.save(CAL)

    lines = [
        "# 关节名 -> 舵机 ID。来自 joint_limit.txt（朋友提供）。",
        "# 用 `python3 calibrate.py monitor` 卸力后逐个掰关节确认。",
    ]
    if extra:
        lines.append("# 不在 14 个策略关节里的舵机: " + ", ".join(f"{j}={i}" for j, i in extra.items()))
    lines.append("servo_ids:")
    lines += [f"  {j}: {servo_ids[j]}" for j in JOINTS if j in servo_ids]
    MAP.write_text("\n".join(lines) + "\n")

    print(f"wrote {CAL} and {MAP} from {SRC}")
    print(f"extra servos (not in the 14 policy joints): {extra}")
    if unsure:
        print("SIGN UNSURE — verify on the robot:\n  " + "\n  ".join(unsure))


if __name__ == "__main__":
    main()
