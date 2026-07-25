#!/usr/bin/env python3
"""Republish the 8BitDo Ultimate 2.4G (D-input mode) as a virtual Xbox 360 pad
with the two rear paddles wired to L3/R3, and with working rumble.

Why this exists: in X-input mode the controller is a vendor-specific USB device
(class ff/5d/01) with no HID interface, and its firmware never puts the paddles
on the wire. D-input mode does expose them, but declares no force feedback, so
the kernel creates no FF device and games get no rumble.

Rumble is restored by driving the controller's vendor CMD_VIBRATE command over
hidraw directly:

    81 11 04 08 00 <dur_lo> <dur_hi> <left> <right>

(report 0x81, transfer type 0x04, cmd 0x08, duration in ms as uint16 LE, then
the two motor intensities 0-255). Protocol credit: s8n/ultimatecontroller-rs,
reverse-engineered from 8BitDoAdvance.dll and verified here acoustically.

The source device is grabbed exclusively (EVIOCGRAB) so games never see both
the real pad and this virtual one; that double-vision is the usual cause of
every input registering twice.
"""

import errno
import fcntl
import glob
import os
import select
import struct
import sys
import time

import evdev
from evdev import AbsInfo, InputDevice, UInput, ecodes as e

SRC_VENDOR = 0x2DC8
SRC_PRODUCT = 0x3013  # Ultimate 2.4G, D-input mode

DST_NAME = "Microsoft X-Box 360 pad"
DST_VENDOR = 0x045E
DST_PRODUCT = 0x028E
DST_VERSION = 0x0114

BUTTON_MAP = {
    e.BTN_A: e.BTN_A,
    e.BTN_B: e.BTN_B,
    e.BTN_X: e.BTN_X,
    e.BTN_Y: e.BTN_Y,
    e.BTN_TL: e.BTN_TL,
    e.BTN_TR: e.BTN_TR,
    e.BTN_SELECT: e.BTN_SELECT,
    e.BTN_START: e.BTN_START,
    e.BTN_MODE: e.BTN_MODE,
    e.BTN_THUMBL: e.BTN_THUMBL,
    e.BTN_THUMBR: e.BTN_THUMBR,
    e.BTN_TRIGGER_HAPPY8: e.BTN_THUMBL,   # left rear paddle  -> L3
    e.BTN_TRIGGER_HAPPY4: e.BTN_THUMBR,   # right rear paddle -> R3
}

AXIS_MAP = {
    e.ABS_X: (e.ABS_X, True),
    e.ABS_Y: (e.ABS_Y, True),
    e.ABS_Z: (e.ABS_RX, True),
    e.ABS_RZ: (e.ABS_RY, True),
    e.ABS_BRAKE: (e.ABS_Z, False),
    e.ABS_GAS: (e.ABS_RZ, False),
    e.ABS_HAT0X: (e.ABS_HAT0X, False),
    e.ABS_HAT0Y: (e.ABS_HAT0Y, False),
}

STICK = AbsInfo(value=0, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)
TRIGGER = AbsInfo(value=0, min=0, max=255, fuzz=0, flat=0, resolution=0)
HAT = AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)

CAPS = {
    e.EV_KEY: sorted(set(BUTTON_MAP.values())),
    e.EV_ABS: [
        (e.ABS_X, STICK), (e.ABS_Y, STICK),
        (e.ABS_RX, STICK), (e.ABS_RY, STICK),
        (e.ABS_Z, TRIGGER), (e.ABS_RZ, TRIGGER),
        (e.ABS_HAT0X, HAT), (e.ABS_HAT0Y, HAT),
    ],
    e.EV_FF: [e.FF_RUMBLE, e.FF_PERIODIC, e.FF_SQUARE, e.FF_TRIANGLE,
              e.FF_SINE, e.FF_GAIN],
}

# ---- uinput force-feedback ioctls (not exposed by python-evdev) ----
# struct ff_effect is 48 bytes on x86_64; uinput_ff_upload is
# request_id(4) + retval(4) + effect(48) + old(48) = 104 bytes.
SIZEOF_FF_EFFECT = 48
SIZEOF_FF_UPLOAD = 104
SIZEOF_FF_ERASE = 12
UINPUT_IOCTL_BASE = ord('U')


def _iowr(nr, size):
    return (3 << 30) | (size << 16) | (UINPUT_IOCTL_BASE << 8) | nr


def _iow(nr, size):
    return (1 << 30) | (size << 16) | (UINPUT_IOCTL_BASE << 8) | nr


UI_BEGIN_FF_UPLOAD = _iowr(200, SIZEOF_FF_UPLOAD)
UI_END_FF_UPLOAD = _iow(201, SIZEOF_FF_UPLOAD)
UI_BEGIN_FF_ERASE = _iowr(202, SIZEOF_FF_ERASE)
UI_END_FF_ERASE = _iow(203, SIZEOF_FF_ERASE)

EV_SIZE = struct.calcsize("llHHi")   # struct input_event on 64-bit

VIBRATE_MAX_MS = 0xFFFF  # duration field is uint16 ms
HOLD_MS = 1000           # chunk length used for open-ended effects
REFRESH_S = 0.75         # re-arm an open-ended effect before HOLD_MS expires
IDLE_TIMEOUT = 0.2       # select timeout when no effect is playing


def log(msg):
    print(msg, flush=True)


def find_source():
    for path in evdev.list_devices():
        try:
            dev = InputDevice(path)
        except OSError:
            continue
        if dev.info.vendor == SRC_VENDOR and dev.info.product == SRC_PRODUCT:
            return dev
        dev.close()
    return None


def find_hidraw():
    """Resolve by device id - the hidraw number changes on every reconnect."""
    for p in glob.glob("/dev/input/by-id/*8BitDo*hidraw*"):
        return os.path.realpath(p)
    return None


class Rumble:
    """Drives the controller's vendor vibrate command."""

    def __init__(self):
        self.fd = None
        self.left = 0
        self.right = 0
        self.until = 0.0
        self.next_refresh = 0.0
        self.gain = 1.0          # set by FF_GAIN

    def open(self):
        path = find_hidraw()
        if not path:
            return False
        try:
            self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        except OSError as exc:
            log(f"rumble: cannot open {path}: {exc}")
            self.fd = None
            return False
        log(f"rumble: using {path}")
        return True

    def close(self):
        if self.fd is not None:
            try:
                self._send(0, 0)
            except Exception:
                pass
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def _send(self, left, right, duration_ms=HOLD_MS):
        if self.fd is None:
            return
        pkt = bytearray(64)
        pkt[0] = 0x81          # report id
        pkt[1] = 0x11          # payload length (17)
        pkt[2] = 0x04          # transfer type
        pkt[3] = 0x08          # cmd = CMD_VIBRATE (uint16 LE)
        pkt[4] = 0x00
        struct.pack_into("<H", pkt, 5, min(duration_ms, VIBRATE_MAX_MS))
        pkt[7] = left
        pkt[8] = right
        try:
            os.write(self.fd, bytes(pkt))
        except OSError as exc:
            if exc.errno in (errno.ENODEV, errno.EBADF, errno.EIO):
                try:
                    os.close(self.fd)
                except OSError:
                    pass
                self.fd = None

    def play(self, strong, weak, duration_ms):
        # ff magnitudes are uint16; the controller takes 0-255 per motor
        self.left = min(255, int((strong >> 8) * self.gain))
        self.right = min(255, int((weak >> 8) * self.gain))
        if self.fd is None and not self.open():
            return
        now = time.time()
        if duration_ms and duration_ms <= VIBRATE_MAX_MS:
            # Let the controller time itself out; that is far more precise than
            # waiting for our own loop to notice and send a stop.
            self.until = now + duration_ms / 1000.0
            self.next_refresh = float("inf")
            self._send(self.left, self.right, duration_ms)
        else:
            # duration 0 means "until stopped": hold it in chunks and re-arm
            self.until = float("inf")
            self.next_refresh = now + REFRESH_S
            self._send(self.left, self.right, HOLD_MS)

    def stop(self):
        self.until = 0.0
        self.next_refresh = float("inf")
        self.left = self.right = 0
        self._send(0, 0, 0)

    def deadline(self):
        """Seconds until this needs attention, for the select() timeout."""
        if self.until <= 0:
            return IDLE_TIMEOUT
        now = time.time()
        return max(0.002, min(self.until, self.next_refresh) - now)

    def tick(self):
        """Expire a finished effect, or re-arm an open-ended one."""
        if self.until <= 0:
            return
        now = time.time()
        if now >= self.until:
            self.stop()
            return
        if now >= self.next_refresh:
            self._send(self.left, self.right, HOLD_MS)
            self.next_refresh = now + REFRESH_S


class FFHandler:
    """Services force-feedback upload/erase/play requests from the uinput fd."""

    def __init__(self, ui, rumble):
        self.ui = ui
        self.rumble = rumble
        self.effects = {}      # effect id -> (strong, weak, length_ms)

    def _upload(self, request_id):
        buf = bytearray(SIZEOF_FF_UPLOAD)
        struct.pack_into("<I", buf, 0, request_id)
        try:
            fcntl.ioctl(self.ui.fd, UI_BEGIN_FF_UPLOAD, buf)
        except OSError as exc:
            log(f"ff: BEGIN_FF_UPLOAD failed: {exc}")
            return
        # struct ff_effect starts at offset 8 within uinput_ff_upload
        base = 8
        etype, eid = struct.unpack_from("<Hh", buf, base)
        length = struct.unpack_from("<H", buf, base + 10)[0]     # replay.length
        strong, weak = struct.unpack_from("<HH", buf, base + 16)  # union: rumble
        if etype == e.FF_RUMBLE:
            self.effects[eid] = (strong, weak, length)
        else:
            # Approximate non-rumble effects so games still get something.
            self.effects[eid] = (0x8000, 0x8000, length)
        struct.pack_into("<i", buf, 4, 0)      # retval = success
        try:
            fcntl.ioctl(self.ui.fd, UI_END_FF_UPLOAD, buf)
        except OSError as exc:
            log(f"ff: END_FF_UPLOAD failed: {exc}")

    def _erase(self, request_id):
        buf = bytearray(SIZEOF_FF_ERASE)
        struct.pack_into("<I", buf, 0, request_id)
        try:
            fcntl.ioctl(self.ui.fd, UI_BEGIN_FF_ERASE, buf)
        except OSError as exc:
            log(f"ff: BEGIN_FF_ERASE failed: {exc}")
            return
        eid = struct.unpack_from("<I", buf, 8)[0]
        self.effects.pop(eid, None)
        struct.pack_into("<i", buf, 4, 0)
        try:
            fcntl.ioctl(self.ui.fd, UI_END_FF_ERASE, buf)
        except OSError as exc:
            log(f"ff: END_FF_ERASE failed: {exc}")

    def pump(self):
        try:
            data = os.read(self.ui.fd, EV_SIZE * 32)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            raise
        for i in range(0, len(data) - EV_SIZE + 1, EV_SIZE):
            _s, _us, etype, code, value = struct.unpack_from("llHHi", data, i)
            if etype == e.EV_UINPUT:
                if code == e.UI_FF_UPLOAD:
                    self._upload(value)
                elif code == e.UI_FF_ERASE:
                    self._erase(value)
            elif etype == e.EV_FF:
                if code == e.FF_GAIN:
                    # value is 0..0xFFFF; games use this as a master volume
                    self.rumble.gain = max(0.0, min(1.0, value / 65535.0))
                    continue
                if value:
                    strong, weak, length = self.effects.get(code, (0xFFFF, 0xFFFF, 0))
                    self.rumble.play(strong, weak, length)
                else:
                    self.rumble.stop()


def release_all(dst, held):
    """Drop every held button and recentre the sticks.

    Without this, a paddle held at the moment the controller disconnects stays
    latched down on the virtual pad forever.
    """
    changed = False
    for code, n in list(held.items()):
        if n > 0:
            dst.write(e.EV_KEY, code, 0)
            changed = True
    held.clear()
    for code in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY,
                 e.ABS_Z, e.ABS_RZ, e.ABS_HAT0X, e.ABS_HAT0Y):
        dst.write(e.EV_ABS, code, 0)
        changed = True
    if changed:
        dst.syn()


def main():
    log("8bitdo paddle shim starting")
    dst = UInput(CAPS, name=DST_NAME, vendor=DST_VENDOR, product=DST_PRODUCT,
                 version=DST_VERSION, bustype=e.BUS_USB, max_effects=16)
    log(f"virtual pad created: {dst.device.path} ({DST_NAME}) with FF_RUMBLE")

    rumble = Rumble()
    ff = FFHandler(dst, rumble)

    # Track how many physical inputs hold each virtual button, so releasing a
    # paddle doesn't cancel a simultaneously-held stick click.
    held = {}
    src = None
    try:
        while True:
            if src is None:
                src = find_source()
                if src is None:
                    rumble.close()
                    time.sleep(2)
                    continue
                try:
                    src.grab()
                except OSError as exc:
                    log(f"cannot grab {src.path}: {exc}; retrying")
                    src.close()
                    src = None
                    time.sleep(2)
                    continue
                log(f"grabbed {src.path} ({src.name})")
                held.clear()
                rumble.open()

            try:
                r, _, _ = select.select([src.fd, dst.fd], [], [], rumble.deadline())
            except OSError:
                r = []

            rumble.tick()

            if dst.fd in r:
                try:
                    ff.pump()
                except OSError as exc:
                    log(f"ff pump error: {exc}")

            if src.fd not in r:
                continue

            try:
                events = list(src.read())
            except OSError as exc:
                if exc.errno not in (errno.ENODEV, errno.EBADF):
                    raise
                log("source disconnected; waiting for it to return")
                release_all(dst, held)
                rumble.close()
                try:
                    src.ungrab()
                except Exception:
                    pass
                try:
                    src.close()
                except Exception:
                    pass
                src = None
                continue

            # Forward a whole batch, then emit one SYN_REPORT, mirroring the
            # source's framing. Emitting a syn per event would split an atomic
            # snapshot (e.g. a diagonal stick move) across several frames.
            dirty = False
            for event in events:
                if event.type == e.EV_SYN:
                    if event.code == e.SYN_REPORT and dirty:
                        dst.syn()
                        dirty = False
                    continue
                if event.type == e.EV_KEY:
                    target = BUTTON_MAP.get(event.code)
                    if target is None or event.value == 2:
                        continue
                    n = held.get(target, 0)
                    if event.value == 1:
                        held[target] = n + 1
                        if n == 0:
                            dst.write(e.EV_KEY, target, 1)
                            dirty = True
                    else:
                        n = max(0, n - 1)
                        held[target] = n
                        if n == 0:
                            dst.write(e.EV_KEY, target, 0)
                            dirty = True
                elif event.type == e.EV_ABS:
                    mapped = AXIS_MAP.get(event.code)
                    if mapped is None:
                        continue
                    target, rescale = mapped
                    value = event.value * 257 - 32768 if rescale else event.value
                    dst.write(e.EV_ABS, target, value)
                    dirty = True
            if dirty:
                dst.syn()
    except KeyboardInterrupt:
        log("stopping")
    finally:
        rumble.close()
        dst.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
