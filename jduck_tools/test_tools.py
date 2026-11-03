"""Offline tests for the protocol, calibration math and URDF patching.

    python3 -m pytest jduck_tools/test_tools.py -q     (or: python3 jduck_tools/test_tools.py)

No hardware needed. A fake half-duplex port answers per the UBT-12HC spec and
also echoes our own command first, as single-wire adapters do.
"""

import math
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibration import Calibration, JointCal, apply_to_urdf, read_urdf_limits  # noqa: E402
from servo_bus import (  # noqa: E402
    CMD_MOVE, CMD_RELAX, Reply, ServoBus, decode_reply, encode_command, find_replies,
)


def test_encode_matches_spec_example():
    # Spec §2.2: ID=01 CMD=01 Pos=0x012C → FA AF 01 01 01 2C 2F ED
    assert encode_command(1, 0x01, 300) == bytes.fromhex("FAAF0101012C2FED")


def test_decode_matches_spec_examples():
    # Spec §6: FA AF 01 01 F4 00 57 4D ED → pos 500, speed +87
    assert decode_reply(bytes.fromhex("FAAF0101F400574DED")) == Reply(1, 500, 87)
    # negative speed -87 = 0xFFA9
    sid, ph, pl, sh, sl = 1, 0x01, 0xF4, 0xFF, 0xA9
    frame = bytes([0xFA, 0xAF, sid, ph, pl, sh, sl, (sid + ph + pl + sh + sl) & 0xFF, 0xED])
    assert decode_reply(frame).speed_dps == -87
    assert decode_reply(frame[:-1] + b"\x00") is None          # bad end byte
    assert decode_reply(frame[:7] + b"\x00" + frame[8:]) is None  # bad checksum


def _reply_bytes(sid, pos, speed=0):
    s = speed & 0xFFFF
    b = [sid, pos >> 8, pos & 0xFF, s >> 8, s & 0xFF]
    return bytes([0xFA, 0xAF, *b, sum(b) & 0xFF, 0xED])


def test_find_replies_skips_echo_and_garbage():
    echo = encode_command(3, CMD_RELAX)
    buf = b"\x00\x12" + echo + _reply_bytes(3, 512, -5) + _reply_bytes(4, 600)[:5]
    replies, rest = find_replies(buf)
    assert replies == [Reply(3, 512, -5)]
    assert rest == _reply_bytes(4, 600)[:5]


class FakePort:
    """Stands in for serial.Serial: echoes TX, then the addressed servo replies."""

    def __init__(self, positions):
        self.positions = dict(positions)
        self.rx = b""
        self.rts = False

    def reset_input_buffer(self):
        self.rx = b""

    def write(self, data):
        self.rx += data  # single-wire echo
        sid, cmd, ph, pl = data[2], data[3], data[4], data[5]
        if cmd == CMD_MOVE and sid in self.positions:
            self.positions[sid] = max(50, min(950, (ph << 8) | pl))
        if sid in self.positions:
            self.rx += _reply_bytes(sid, self.positions[sid])

    def flush(self):
        pass

    def read(self, n):
        out, self.rx = self.rx[:n], self.rx[n:]
        return out

    def close(self):
        pass


def _bus(positions):
    bus = ServoBus.__new__(ServoBus)
    bus.baud, bus.timeout_s, bus.direction = 230400, 0.01, "none"
    bus.ser = FakePort(positions)
    return bus


def test_bus_scan_and_move():
    bus = _bus({1: 500, 7: 300})
    found = bus.scan(range(1, 10))
    assert set(found) == {1, 7} and found[7].pos == 300
    assert bus.transact(7, CMD_MOVE, 420).pos == 420
    assert bus.relax(5) is None


def test_joint_math_roundtrip_and_range():
    k = 2 * math.pi / 1024
    c = JointCal(id=3, adc_zero=500, sign=-1, adc_min=300, adc_max=800)
    q = c.to_rad(560, k)
    assert math.isclose(q, -60 * k)
    assert math.isclose(c.to_adc(q, k), 560)
    lo, hi = c.urdf_range(k, math.radians(5))
    # sign -1: adc 800 → -300k, adc 300 → +200k
    assert math.isclose(lo, -300 * k + math.radians(5))
    assert math.isclose(hi, 200 * k - math.radians(5))


def test_save_load_and_apply_to_urdf_keeps_crlf():
    urdf_src = HERE.parent / "jduck_description" / "urdf" / "jduck_description.urdf"
    with tempfile.TemporaryDirectory() as td:
        urdf = Path(td) / "jduck_description.urdf"
        shutil.copy2(urdf_src, urdf)
        cal = Calibration()
        cal.joints["left_knee"] = JointCal(id=4, adc_zero=512, sign=1, adc_min=300, adc_max=760)
        cal_path = Path(td) / "calibration.yaml"
        cal.save(cal_path)
        cal2 = Calibration.load(cal_path)
        assert cal2.joints["left_knee"] == cal.joints["left_knee"]

        before = read_urdf_limits(urdf)
        assert apply_to_urdf(cal2, urdf) == ["left_knee"]
        after = read_urdf_limits(urdf)
        lo, hi = cal2.joints["left_knee"].urdf_range(cal2.k_rad_per_count, cal2.margin_rad)
        assert math.isclose(after["left_knee"][0], lo, abs_tol=1e-4)
        assert math.isclose(after["left_knee"][1], hi, abs_tol=1e-4)
        assert after["right_knee"] == before["right_knee"]       # untouched
        assert b"\r\n" in urdf.read_bytes()                       # CRLF preserved
        assert (Path(td) / "jduck_description.urdf.precal.bak").exists()


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-q"]))
