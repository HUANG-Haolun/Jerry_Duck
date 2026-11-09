#!/usr/bin/env python3
"""Read / set / save the runtime control parameters of servos running the
`firmware/ubt12hc_pd` firmware (CMD 0x10 / 0x11 / 0x12). The stock firmware
ignores these commands and gives no reply.

    python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 get --id 9
    python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 get --all
    python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 set --id 9 --db 1
    python3 jduck_tools/servo_params.py --port /dev/ttyUSB0 set --all --kp 16 --ff 0 --db 0 --save   # = training (r7/r8)

`set` only changes RAM (lost at power-off) unless --save is given. After tuning
on the bench, tell Claude the final values so the simulator uses the same law
(JDUCK_FW=K,FF,DB).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibration import load_servo_map  # noqa: E402
from servo_bus import ServoBus  # noqa: E402

CMD_SET_PARAM, CMD_GET_PARAM, CMD_SAVE_PARAMS = 0x10, 0x11, 0x12
PARAMS = {"kp": 0, "ff": 1, "db": 2, "timeout": 3}   # timeout in units of 10 ms
LIMITS = {"kp": (1, 64), "ff": (0, 120), "db": (0, 10), "timeout": (0, 255)}  # = firmware's load check


def get_all(bus: ServoBus, sid: int) -> dict[str, int] | None:
    out = {}
    for name, idx in PARAMS.items():
        r = bus.transact(sid, CMD_GET_PARAM, idx << 8)
        if r is None or r.speed_dps < 0:
            return None
        out[name] = r.speed_dps
    return out


def set_param(bus: ServoBus, sid: int, name: str, value: int) -> bool:
    """Set one parameter in RAM (not saved). True if the servo echoed the new value."""
    lo, hi = LIMITS[name]
    v = max(lo, min(hi, int(value)))
    r = bus.transact(sid, CMD_SET_PARAM, (PARAMS[name] << 8) | v)
    return r is not None and r.speed_dps == v


def fmt(p: dict[str, int]) -> str:
    t = p["timeout"]
    return (f"KP={p['kp']:3d}  FF={p['ff']:3d}  DEADBAND={p['db']:2d}  "
            f"TIMEOUT={'off' if t == 0 else f'{t * 10} ms'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--direction", default="none", choices=["none", "rts", "rts-inv"])
    ap.add_argument("--map", type=Path, default=HERE / "servo_map.yaml")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("get", "set"):
        s = sub.add_parser(name)
        g = s.add_mutually_exclusive_group(required=True)
        g.add_argument("--id", type=int)
        g.add_argument("--all", action="store_true", help="servo_map.yaml 里的全部舵机 + 嘴（ID 4）")
        if name == "set":
            for p in PARAMS:
                s.add_argument(f"--{p}", type=int)
            s.add_argument("--save", action="store_true", help="同时写入 EEPROM（断电保留）")
    a = ap.parse_args()

    if a.all:
        ids = sorted(set(load_servo_map(a.map).values()) | {4})
    else:
        ids = [a.id]

    changes = {}
    if a.cmd == "set":
        changes = {p: getattr(a, p) for p in PARAMS if getattr(a, p) is not None}
        for p, v in changes.items():
            lo, hi = LIMITS[p]
            if not lo <= v <= hi:
                sys.exit(f"--{p} {v} 超出范围 [{lo}, {hi}]（超出的值上电时固件也会拒绝加载）")
        if not changes and not a.save:
            sys.exit("set 至少要给一个参数（--kp/--ff/--db/--timeout）或 --save")

    with ServoBus(a.port, a.baud, 0.02, a.direction) as bus:
        for sid in ids:
            for p, v in changes.items():
                r = bus.transact(sid, CMD_SET_PARAM, (PARAMS[p] << 8) | v)
                if r is None:
                    print(f"ID {sid:2d}: 无应答（是不是还是原版固件？）")
                    break
                if r.speed_dps != v:
                    print(f"ID {sid:2d}: 设 {p}={v} 失败（回复 {r.speed_dps}）")
            if a.cmd == "set" and a.save:
                r = bus.transact(sid, CMD_SAVE_PARAMS, (0xA5 << 8) | 0x5A)
                if r is None or r.speed_dps != 0:
                    print(f"ID {sid:2d}: 存 EEPROM 失败")
            p = get_all(bus, sid)
            if p is None:
                print(f"ID {sid:2d}: 读参数失败（原版固件不支持这些命令）")
            else:
                print(f"ID {sid:2d}: {fmt(p)}{'  （已存 EEPROM）' if a.cmd == 'set' and a.save else ''}")


if __name__ == "__main__":
    main()
