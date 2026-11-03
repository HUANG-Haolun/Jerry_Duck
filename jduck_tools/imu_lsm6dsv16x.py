"""LSM6DSV16X IMU over Linux I2C (/dev/i2c-N) + a gravity estimate for the policy.

The policy needs, in the trunk_base frame (x forward, y left, z up):
  - base_ang_vel: gyro, rad/s
  - projected_gravity: unit gravity direction (z ≈ -1 when upright)

The chip's own axes depend on how the board is mounted, so `axes` in the config
maps chip axes to body axes, e.g. `axes: ["-y", "x", "z"]` means
body_x = -chip_y, body_y = chip_x, body_z = chip_z.

Gravity is estimated with a complementary filter: integrate the gyro, and pull
slowly towards the accelerometer direction. Training uses perfect gravity plus
±0.01 noise and up to 6° of misalignment, so a few degrees of filter error is
inside what the policy has seen.

Config (YAML):
    bus: 1                 # /dev/i2c-1
    address: 0x6A          # 0x6A (SA0 low) or 0x6B
    axes: ["x", "y", "z"]  # chip -> body
    gyro_bias: [0, 0, 0]   # rad/s, measured at rest (see `python3 imu_lsm6dsv16x.py calib`)
    alpha: 0.02            # accel weight per 20 ms tick

Register values follow the LSM6DSV16X datasheet: WHO_AM_I = 0x70, CTRL1/2
set ODR, CTRL6/8 set full scale, and data starts at OUTX_L_G (0x22). Not tested
on hardware yet. Check WHO_AM_I first with `python3 imu_lsm6dsv16x.py probe`.
"""

from __future__ import annotations

import fcntl
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

I2C_SLAVE = 0x0703
WHO_AM_I, WHO_AM_I_VAL = 0x0F, 0x70
CTRL1, CTRL2, CTRL3, CTRL6, CTRL8 = 0x10, 0x11, 0x12, 0x15, 0x17
OUTX_L_G = 0x22                    # gyro x,y,z then accel x,y,z (12 bytes)
GYRO_SENS = 0.070 * math.pi / 180  # ±2000 dps → 70 mdps/LSB
ACC_SENS = 0.000122 * 9.80665      # ±4 g → 0.122 mg/LSB


class I2CDev:
    def __init__(self, bus: int, addr: int):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)

    def write(self, reg: int, val: int) -> None:
        os.write(self.fd, bytes([reg, val]))

    def read(self, reg: int, n: int) -> bytes:
        os.write(self.fd, bytes([reg]))
        return os.read(self.fd, n)


def _axis_map(spec):
    m = np.zeros((3, 3))
    for body_i, s in enumerate(spec):
        sign = -1.0 if s.startswith("-") else 1.0
        m[body_i, "xyz".index(s[-1])] = sign
    return m


class Imu:
    def __init__(self, bus=1, address=0x6A, axes=("x", "y", "z"), gyro_bias=(0, 0, 0), alpha=0.02):
        self.dev = I2CDev(bus, address)
        who = self.dev.read(WHO_AM_I, 1)[0]
        if who != WHO_AM_I_VAL:
            raise RuntimeError(f"WHO_AM_I = 0x{who:02X}, expected 0x{WHO_AM_I_VAL:02X} (wrong bus/address?)")
        self.dev.write(CTRL3, 0x44)   # BDU=1, IF_INC=1
        self.dev.write(CTRL1, 0x07)   # accel ODR 480 Hz, high-performance
        self.dev.write(CTRL2, 0x07)   # gyro ODR 480 Hz
        self.dev.write(CTRL6, 0x04)   # gyro ±2000 dps
        self.dev.write(CTRL8, 0x01)   # accel ±4 g
        time.sleep(0.05)
        self.M = _axis_map(axes)
        self.bias = np.asarray(gyro_bias, float)
        self.alpha = alpha
        self.g = None
        self.t = None

    @classmethod
    def from_config(cls, path: Path) -> "Imu":
        import yaml

        c = yaml.safe_load(Path(path).read_text()) or {}
        return cls(c.get("bus", 1), int(c.get("address", 0x6A)), c.get("axes", ["x", "y", "z"]),
                   c.get("gyro_bias", [0, 0, 0]), c.get("alpha", 0.02))

    def raw(self):
        b = self.dev.read(OUTX_L_G, 12)
        v = np.frombuffer(b, dtype="<i2").astype(float)
        gyro = self.M @ (v[0:3] * GYRO_SENS)
        acc = self.M @ (v[3:6] * ACC_SENS)
        return gyro, acc

    def read(self):
        """(ang_vel rad/s, projected_gravity unit) in the trunk_base frame."""
        gyro, acc = self.raw()
        w = gyro - self.bias
        g_acc = -acc / max(1e-6, np.linalg.norm(acc))   # accel reads +g upward at rest
        now = time.monotonic()
        if self.g is None:
            self.g = g_acc
        else:
            dt = min(0.1, now - self.t)
            # gravity is fixed in the world, so in the body frame dg/dt = -w × g
            self.g = self.g - np.cross(w, self.g) * dt
            self.g = (1 - self.alpha) * self.g + self.alpha * g_acc
            self.g /= np.linalg.norm(self.g)
        self.t = now
        return w.astype(np.float32), self.g.astype(np.float32)


def main():
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("cmd", choices=["probe", "show", "calib"])
    p.add_argument("--config", type=Path, default=Path(__file__).with_name("imu.yaml"))
    a = p.parse_args()
    imu = Imu.from_config(a.config) if a.config.exists() else Imu()
    if a.cmd == "probe":
        print("WHO_AM_I ok (0x70)")
    elif a.cmd == "calib":
        print("保持机器人静止 3 秒 ...")
        s = np.zeros(3)
        n = 0
        t_end = time.time() + 3
        while time.time() < t_end:
            s += imu.raw()[0]
            n += 1
            time.sleep(0.005)
        print(f"gyro_bias: [{', '.join(f'{x:.5f}' for x in s / n)}]   ← 填进 imu.yaml")
    else:
        # Signs checked in MuJoCo against the trunk_base frame (x fwd, y left, z up).
        print("机器人平放：gravity 应约为 [0, 0, -1]；低头（机头朝下）时 x 变正；左侧抬起时 y 变负。Ctrl+C 退出")
        try:
            while True:
                w, g = imu.read()
                sys.stdout.write(f"\rang_vel {np.round(w, 2)}  gravity {np.round(g, 2)}   ")
                sys.stdout.flush()
                time.sleep(0.02)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
