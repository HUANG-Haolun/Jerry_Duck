"""UBT-12HC bus-servo protocol and a minimal half-duplex bus driver.

Frame formats come from `~/Downloads/UBT12HC_总线舵机通讯协议说明.pdf`, which
describes firmware `send_rev_230400_ult.ino`. The firmware has since been
re-flashed with per-servo IDs, so the ID byte is assumed to be honoured.
Nothing here has been checked against real hardware yet.

  command (8 B):  FA AF ID CMD PosH PosL SUM ED     SUM = (ID+CMD+PosH+PosL) & 0xFF
  reply   (9 B):  FA AF ID PosH PosL SpdH SpdL SUM ED
                                                    SUM = (ID+PosH+PosL+SpdH+SpdL) & 0xFF

Every valid command is answered with the current potentiometer ADC (uint16,
big-endian) and a signed speed in deg/s. The speed is a 40 ms position
difference, so it is coarse and lagged.

CMD 0x03 (relax) turns the motor off and still replies, so it is the safe way to
read a position while moving a joint by hand.
"""

from __future__ import annotations

import os
import select
import time
from dataclasses import dataclass

HEADER = b"\xFA\xAF"
END = 0xED
CMD_MOVE = 0x01   # set target ADC (firmware clamps to 50..950), enable motor
CMD_HOLD = 0x02   # hold the current position, enable motor
CMD_RELAX = 0x03  # motor off
ADC_MIN = 50
ADC_MAX = 950
REPLY_LEN = 9


def encode_command(servo_id: int, cmd: int, pos: int = 0) -> bytes:
    if not 0 <= servo_id <= 0xFF:
        raise ValueError(f"servo_id out of range: {servo_id}")
    if not 0 <= pos <= 0xFFFF:
        raise ValueError(f"pos out of range: {pos}")
    ph, pl = (pos >> 8) & 0xFF, pos & 0xFF
    checksum = (servo_id + cmd + ph + pl) & 0xFF
    return bytes([0xFA, 0xAF, servo_id, cmd, ph, pl, checksum, END])


@dataclass(frozen=True)
class Reply:
    servo_id: int
    pos: int          # raw potentiometer ADC
    speed_dps: int    # signed, deg/s, 40 ms difference


def decode_reply(frame: bytes) -> Reply | None:
    if len(frame) != REPLY_LEN or frame[:2] != HEADER or frame[8] != END:
        return None
    sid, ph, pl, sh, sl, checksum = frame[2:8]
    if (sid + ph + pl + sh + sl) & 0xFF != checksum:
        return None
    raw = (sh << 8) | sl
    speed = raw - 0x10000 if raw & 0x8000 else raw
    return Reply(sid, (ph << 8) | pl, speed)


def find_replies(buf: bytes) -> tuple[list[Reply], bytes]:
    """Extract every valid reply frame from `buf`.

    Returns (replies, leftover). `leftover` is a possible partial frame to
    prepend to the next read. Garbage, and the echo of our own 8-byte command
    on single-wire adapters, fails the checksum/end-byte test and is skipped.
    """
    replies: list[Reply] = []
    i = 0
    while True:
        j = buf.find(HEADER, i)
        if j < 0:
            return replies, (buf[-1:] if buf.endswith(b"\xFA") else b"")
        if len(buf) - j < REPLY_LEN:
            return replies, buf[j:]
        reply = decode_reply(buf[j:j + REPLY_LEN])
        if reply is not None:
            replies.append(reply)
            i = j + REPLY_LEN
        else:
            i = j + 1


class ServoBus:
    """One request, one reply, on a half-duplex UART.

    direction:
      "none"     adapter switches direction by itself (most USB-TTL half-duplex boards)
      "rts"      RTS high while transmitting
      "rts-inv"  RTS low while transmitting
    """

    def __init__(self, port: str, baud: int = 230400, timeout_s: float = 0.02,
                 direction: str = "none"):
        import serial  # pyserial

        if direction not in ("none", "rts", "rts-inv"):
            raise ValueError(f"unknown direction mode: {direction}")
        self.baud = baud
        self.timeout_s = timeout_s
        self.direction = direction
        self.ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0)
        self.ser.reset_input_buffer()
        self._rxbuf = b""
        if direction != "none":
            self.ser.rts = direction == "rts-inv"   # idle = receive

    @property
    def _fast(self) -> bool:
        """Raw fd + select on the board's UART (no Python 0.3 ms polling sleeps). Ports without
        a real file descriptor (the tests' FakePort) and RTS direction control keep pyserial."""
        return self.direction == "none" and os.name == "posix" and hasattr(self.ser, "fileno")

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> "ServoBus":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def transact(self, servo_id: int, cmd: int, pos: int = 0,
                 timeout_s: float | None = None) -> Reply | None:
        frame = encode_command(servo_id, cmd, pos)
        if self.direction != "none":
            self.ser.rts = self.direction == "rts"
        if self._fast:
            # The board UART is a real file descriptor.  Waiting for it with
            # select avoids Python's 0.3 ms polling sleeps (14 per control tick).
            os.write(self.ser.fileno(), frame)
        else:
            self.ser.write(frame)
        if self.direction != "none":
            self.ser.flush()
            time.sleep(len(frame) * 10 / self.baud)  # last stop bit out of the shifter
            self.ser.rts = self.direction == "rts-inv"
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            if self._fast:
                remaining = max(0.0, deadline - time.monotonic())
                ready, _, _ = select.select([self.ser.fileno()], [], [], remaining)
                chunk = os.read(self.ser.fileno(), 128) if ready else b""
            else:
                chunk = self.ser.read(64)
            if not chunk:
                if not self._fast:
                    time.sleep(0.0003)
                continue
            replies, self._rxbuf = find_replies(getattr(self, "_rxbuf", b"") + chunk)
            for r in replies:
                if r.servo_id == servo_id:
                    return r
        return None

    def relax(self, servo_id: int) -> Reply | None:
        """Motor off and read position. Safe while moving the joint by hand."""
        return self.transact(servo_id, CMD_RELAX)

    def scan(self, ids, retries: int = 2) -> dict[int, Reply]:
        found: dict[int, Reply] = {}
        for sid in ids:
            for _ in range(retries):
                r = self.relax(sid)
                if r is not None:
                    found[sid] = r
                    break
        return found
