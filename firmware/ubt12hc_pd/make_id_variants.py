#!/usr/bin/env python3
"""Generate one sketch per servo ID (SERVO_ID is a compile-time #define).

    python3 firmware/ubt12hc_pd/make_id_variants.py

writes firmware/ubt12hc_pd/build/id_XX/id_XX.ino for every ID in
jduck_tools/servo_map.yaml plus the mouth (ID 4). Open each one in the Arduino
IDE (MiniCore → ATmega8, External 14.7456 MHz) and upload with "Upload Using
Programmer". Arduino needs the folder and the .ino to share a name, hence id_XX/id_XX.ino.
"""

import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "ubt12hc_pd.ino"
OUT = HERE / "build"
# joint_limit.txt (the friend's table): IDs 1-5 and 7-16, there is no 6.
IDS = [1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]


def main() -> None:
    text = SRC.read_bytes().decode("utf-8")
    pat = re.compile(r"^(#define SERVO_ID[ \t]+)\d+", re.M)
    if len(pat.findall(text)) != 1:
        raise SystemExit("expected exactly one '#define SERVO_ID <n>' line")
    for sid in IDS:
        d = OUT / f"id_{sid:02d}"
        d.mkdir(parents=True, exist_ok=True)
        out = pat.sub(lambda m: f"{m.group(1)}{sid}", text)
        (d / f"id_{sid:02d}.ino").write_bytes(out.encode("utf-8"))
    print(f"wrote {len(IDS)} sketches to {OUT}/id_XX/")


if __name__ == "__main__":
    main()
