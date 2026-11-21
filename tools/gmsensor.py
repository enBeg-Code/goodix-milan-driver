"""Minimal userspace access to the Goodix Milan F (GXFP3200) sensor through spidev.

Mirrors the capture sequence of src/goodixmilan.c; see docs/PROTOCOL.md.
The sensor must be bound to spidev (the udev rule does it) and fprintd must not be
using it at the same time:   sudo systemctl stop fprintd
"""
import ctypes
import fcntl
import os
import struct
import time

import numpy as np

DEV = '/dev/spidev0.0'
SPEED = 4_000_000            # as declared by the ACPI _CRS of the FPRT device
W, H = 108, 88
RAWLEN = 0x37B4
FIFO_CHUNK = 4092            # fits the default spidev buffer (4096)
MODE_CAPTURE = ((0x005C, 0x0080), (0x0120, 0x0408), (0x0052, 0x0008), (0x0050, 0x0501))
REG_IRQ_CLEAR, REG_IRQ_STATUS, IRQ_IMAGE = 0x124, 0x126, 0x08

RIDGE_PRESENT = 40.0         # ridge energy of a raw frame: ~31 empty, ~52-56 with a finger
RIDGE_ABSENT = 35.0
QUALITY_MIN = 300.0          # minimum mean |background - frame| of a usable image


class SensorError(Exception):
    pass


def _crc_table():
    t = []
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = ((c << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if c & 0x80000000 else (c << 1) & 0xFFFFFFFF
        t.append(c)
    return t


_CRC = _crc_table()


def crc32_mpeg2(data):
    c = 0xFFFFFFFF
    for b in data:
        c = ((c << 8) & 0xFFFFFFFF) ^ _CRC[(c >> 24) ^ b]
    return c


def unpack12(buf):
    """6 bytes -> 4 pixels of 12 bit."""
    b = np.frombuffer(buf[:len(buf) // 6 * 6], dtype=np.uint8).reshape(-1, 6).astype(np.uint16)
    out = np.empty((b.shape[0], 4), dtype=np.uint16)
    out[:, 0] = b[:, 1] + ((b[:, 0] & 0xF) << 8)
    out[:, 1] = (b[:, 3] << 4) + (b[:, 0] >> 4)
    out[:, 2] = b[:, 2] + ((b[:, 5] & 0xF) << 8)
    out[:, 3] = (b[:, 4] << 4) + (b[:, 5] >> 4)
    return out.reshape(-1)[:W * H].reshape(H, W).astype(np.float32)


def ridge_energy(frame):
    """Same measure as gm_ridge_energy() in src/gm-match.c (needs OpenCV)."""
    import cv2
    a = frame - frame.mean(axis=1, keepdims=True)
    hp = a - cv2.GaussianBlur(a, (0, 0), 2.0)
    return float(cv2.GaussianBlur(hp, (0, 0), 0.8)[3:-3, 3:-3].std())


class Sensor:
    def __init__(self, dev=DEV, speed=SPEED):
        self.fd = os.open(dev, os.O_RDWR)
        self.speed = speed
        fcntl.ioctl(self.fd, 0x40016B01, struct.pack('B', 0))       # SPI mode 0
        fcntl.ioctl(self.fd, 0x40016B03, struct.pack('B', 8))       # 8 bits per word
        cid = self.chip_id()
        if cid >> 8 != 0x2202:
            self.close()
            raise SensorError(f'unexpected chip ID 0x{cid:08x}')
        self.bg = None

    def close(self):
        if self.fd is not None:
            try:
                self.cmd(0xC0)
            except OSError:
                pass
            os.close(self.fd)
            self.fd = None

    # ---- SPI primitives
    def _xfer(self, parts):
        """parts: list of (tx_bytes or None, rx_len), executed with CS held low."""
        keep, desc = [], b''
        for tx, n in parts:
            ln = len(tx) if tx is not None else n
            tb = ctypes.create_string_buffer(bytes(tx), ln) if tx is not None else None
            rb = ctypes.create_string_buffer(ln) if tx is None else None
            keep.append((tb, rb))
            desc += struct.pack('QQIIHBBBBBB', ctypes.addressof(tb) if tb else 0,
                                ctypes.addressof(rb) if rb else 0, ln, self.speed, 0, 8, 0, 0, 0, 0, 0)
        buf = ctypes.create_string_buffer(desc, len(desc))
        fcntl.ioctl(self.fd, (1 << 30) | ((32 * len(parts)) << 16) | (ord('k') << 8), buf)
        return [rb.raw if rb else None for _, rb in keep]

    def set_addr(self, addr):
        self._xfer([(bytes([0xF0, addr >> 8 & 0xFF, addr & 0xFF]), 0)])

    def read(self, addr, n):
        self.set_addr(addr)
        return self._xfer([(b'\xF1', 0), (None, n)])[1]

    def read16(self, addr):
        b = self.read(addr, 2)
        return b[0] << 8 | b[1]

    def write16(self, addr, val):
        self._xfer([(bytes([0xF0, addr >> 8 & 0xFF, addr & 0xFF, 0, 1, val >> 8 & 0xFF, val & 0xFF]), 0)])

    def cmd(self, c):
        self._xfer([(bytes([c]), 0)])

    def chip_id(self):
        b = self.read(0x0000, 4)
        return (b[2] << 24) | (b[3] << 16) | (b[0] << 8) | b[1]

    # ---- capture
    def frame(self, timeout=1.0):
        """One raw frame, H x W float32."""
        self.cmd(0xC0); time.sleep(0.001)
        st = self.read16(REG_IRQ_STATUS)
        if st:
            self.write16(REG_IRQ_CLEAR, st)
        self.write16(REG_IRQ_CLEAR, 0x00FF)
        for a, v in MODE_CAPTURE:
            self.write16(a, v)
        self.cmd(0xC2); time.sleep(0.001)
        self.write16(0x0054, 0x0110)
        t0 = time.monotonic()
        while not (st := self.read16(REG_IRQ_STATUS)) & IRQ_IMAGE:
            if time.monotonic() - t0 > timeout:
                self.cmd(0xC0)
                raise SensorError(f'capture timeout (status 0x{st:04x})')
            time.sleep(0.002)
        self.write16(REG_IRQ_CLEAR, st)
        self.set_addr(0xAAAA)
        raw = b''.join(self._xfer([(b'\xF1', 0), (None, min(FIFO_CHUNK, RAWLEN - off))])[1]
                       for off in range(0, RAWLEN, FIFO_CHUNK))
        seed = self.read16(0x0074) << 16 | self.read16(0x0072)
        self.cmd(0xC0)
        crc = crc32_mpeg2(raw[:-4])
        if raw[-4:] != bytes([crc >> 8 & 0xFF, crc & 0xFF, crc >> 24, crc >> 16 & 0xFF]):
            raise SensorError('frame CRC mismatch')
        if seed:
            raise SensorError(f'scrambled frame (seed 0x{seed:08x}) not supported')
        return unpack12(raw[:-4])

    def frame_retry(self, tries=3):
        for i in range(tries):
            try:
                return self.frame()
            except SensorError:
                if i == tries - 1:
                    raise
                time.sleep(0.02)

    def wait_lift(self):
        calm = 0
        while calm < 2:
            f = self.frame_retry()
            calm = calm + 1 if ridge_energy(f) < RIDGE_ABSENT else 0
            if calm:
                self.bg = f if self.bg is None else 0.7 * self.bg + 0.3 * f
            time.sleep(0.05)

    def wait_finger(self):
        """Waits for a finger; returns (difference image background - frame, quality)."""
        hits = 0
        while True:
            f = self.frame_retry()
            r = ridge_energy(f)
            if r < RIDGE_ABSENT:
                hits = 0
                self.bg = f if self.bg is None else 0.7 * self.bg + 0.3 * f
            elif r > RIDGE_PRESENT and self.bg is not None:
                hits += 1
                if hits >= 2:
                    time.sleep(0.25)
                    diff = max((self.bg - self.frame_retry() for _ in range(2)),
                               key=lambda d: float(np.abs(d).mean()))
                    return diff, float(np.abs(diff).mean())
            time.sleep(0.03)


if __name__ == '__main__':
    s = Sensor()
    print(f'chip ID 0x{s.chip_id():08x}')
    t = time.monotonic()
    f = s.frame()
    print(f'frame {f.shape[1]}x{f.shape[0]} in {(time.monotonic() - t) * 1000:.0f} ms, '
          f'ridge energy {ridge_energy(f):.1f} (empty ~31, finger >50)')
    s.close()
