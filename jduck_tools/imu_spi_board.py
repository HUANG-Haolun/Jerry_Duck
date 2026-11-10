import ctypes
from pathlib import Path

import numpy as np
import yaml
from imu_lsm6dsv16x import Imu as GravityFilter, _axis_map


class Imu(GravityFilter):
    @classmethod
    def from_config(cls, path):
        cfg = yaml.safe_load(Path(path).read_text())
        if cfg.get("chip") != "ICM42688P":
            raise ValueError("Expected ICM42688P SPI configuration")
        return cls(cfg)

    def __init__(self, cfg):
        self.lib = ctypes.CDLL(str(Path(__file__).with_name("libjduck_icm.so")))
        self.lib.sample_icm.argtypes = [ctypes.POINTER(ctypes.c_int16)]
        if self.lib.open_icm() != 0:
            raise OSError("SPI IMU ID/configuration mismatch")
        self.buf = (ctypes.c_int16 * 6)()
        self.M = _axis_map(cfg["axes"])
        self.bias = np.array(cfg["gyro_bias"], dtype=float)
        self.alpha = cfg.get("alpha", 0.02)
        self.g = self.t = None

    def raw(self):
        if self.lib.sample_icm(self.buf) < 0:
            raise OSError("SPI IMU sample failed")
        v = np.asarray(self.buf, dtype=float)
        return self.M @ np.deg2rad(v[3:] / 32.8), self.M @ (v[:3] * (9.80665 / 4096))
