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
import glob
import os
import select
import signal
import struct
import sys
import time

import evdev
from evdev import AbsInfo, InputDevice, UInput, ecodes as e
from evdev.uinput import UInputError

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

VIBRATE_MAX_MS = 0xFFFF  # duration field is uint16 ms
HOLD_MS = 1000           # chunk length used for open-ended effects
REFRESH_S = 0.75         # re-arm an open-ended effect before HOLD_MS expires


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
                self._send(0, 0, 0)
            except OSError as exc:
                log(f"rumble: stop on close failed: {exc}")
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
        # Reset scheduling state too. Leaving `until`/`next_refresh` set would
        # keep deadline() short forever (waking the loop while the device is
        # gone) and would replay the stale motor values on reconnect.
        self.until = 0.0
        self.next_refresh = float("inf")
        self.left = self.right = 0
        self.gain = 1.0

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
        """select() timeout, or None to block until an fd is readable.

        Nothing needs doing on a timer while no effect is playing, so idling
        blocks indefinitely rather than waking several times a second.
        """
        if self.until <= 0:
            return None
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
        self.playing = set()   # effect ids the game currently has running

    def _upload(self, request_id):
        try:
            upload = self.ui.begin_upload(request_id)
        except (OSError, UInputError) as exc:
            log(f"ff: begin_upload failed: {exc}")
            return
        eff = upload.effect
        length = eff.ff_replay.length
        if eff.type == e.FF_RUMBLE:
            rumble = eff.u.ff_rumble_effect
            self.effects[eff.id] = (rumble.strong_magnitude,
                                    rumble.weak_magnitude, length)
        else:
            # Approximate non-rumble effects so games still get something.
            self.effects[eff.id] = (0x8000, 0x8000, length)
        upload.retval = 0
        try:
            self.ui.end_upload(upload)
        except (OSError, UInputError) as exc:
            log(f"ff: end_upload failed: {exc}")

    def _erase(self, request_id):
        try:
            erase = self.ui.begin_erase(request_id)
        except (OSError, UInputError) as exc:
            log(f"ff: begin_erase failed: {exc}")
            return
        self.effects.pop(erase.effect_id, None)
        self.playing.discard(erase.effect_id)
        erase.retval = 0
        try:
            self.ui.end_erase(erase)
        except (OSError, UInputError) as exc:
            log(f"ff: end_erase failed: {exc}")
        self._refresh()

    def _refresh(self):
        """Drive the motors from whichever effects are currently playing.

        Games routinely run more than one effect at once (a sustained rumble
        plus short impact pulses). Stopping any single effect must not silence
        the others, so the strongest active magnitude wins rather than the most
        recent event.
        """
        if not self.playing:
            self.rumble.stop()
            return
        active = [self.effects.get(eid, (0xFFFF, 0xFFFF, 0))
                  for eid in self.playing]
        strong = max(s for s, _, _ in active)
        weak = max(w for _, w, _ in active)
        lengths = [ln for _, _, ln in active]
        # 0 means "until stopped", so it dominates: if any active effect is
        # open-ended the combined effect is too. Otherwise run until the
        # longest one would have finished.
        length = 0 if any(ln == 0 for ln in lengths) else max(lengths)
        self.rumble.play(strong, weak, length)

    def pump(self):
        try:
            events = list(self.ui.read())
        except BlockingIOError:
            return
        for ev in events:
            if ev.type == e.EV_UINPUT:
                if ev.code == e.UI_FF_UPLOAD:
                    self._upload(ev.value)
                elif ev.code == e.UI_FF_ERASE:
                    self._erase(ev.value)
            elif ev.type == e.EV_FF:
                if ev.code == e.FF_GAIN:
                    # value is 0..0xFFFF; games use this as a master volume
                    self.rumble.gain = max(0.0, min(1.0, ev.value / 65535.0))
                    continue
                if ev.value:
                    self.playing.add(ev.code)
                else:
                    self.playing.discard(ev.code)
                self._refresh()


def create_pad():
    """Publish the virtual Xbox 360 pad."""
    dst = UInput(CAPS, name=DST_NAME, vendor=DST_VENDOR, product=DST_PRODUCT,
                 version=DST_VERSION, bustype=e.BUS_USB, max_effects=16)
    log(f"virtual pad created: {dst.device.path} ({DST_NAME}) with FF_RUMBLE")
    return dst


def teardown(src, dst, held, rumble):
    """Release the controller and destroy the virtual pad.

    Destroying the pad (rather than leaving it published) is what makes games
    see a real disconnect. It also means any buttons held at that moment vanish
    with the device, so no explicit release is needed.

    Returns the (src, dst, ff) triple as None so callers can reset their state
    in the same statement, leaving no window where a SIGTERM-driven second
    teardown could run against already-closed objects.
    """
    held.clear()
    rumble.close()
    if src is not None:
        for step in (src.ungrab, src.close):
            try:
                step()
            except OSError as exc:
                log(f"cleanup: {step.__name__} failed: {exc}")
    if dst is not None:
        try:
            dst.close()
            log("virtual pad removed")
        except (OSError, UInputError) as exc:
            log(f"cleanup: closing virtual pad failed: {exc}")
    return None, None, None


def main():
    log("8bitdo paddle shim starting")

    # systemctl stop/restart sends SIGTERM, whose default disposition kills the
    # interpreter outright. Without this the motors are never told to stop and
    # the pad keeps buzzing for up to HOLD_MS after the service goes away.
    def on_term(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    rumble = Rumble()

    # The virtual pad only exists while a real controller is connected, so
    # games see genuine hotplug rather than a pad that is permanently present.
    dst = None
    ff = None

    # Track how many physical inputs hold each virtual button, so releasing a
    # paddle doesn't cancel a simultaneously-held stick click.
    held = {}
    src = None
    try:
        while True:
            if src is None:
                src = find_source()
                if src is None:
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
                # Publish the pad only now that a real controller is behind it.
                try:
                    dst = create_pad()
                except OSError as exc:
                    log(f"cannot create virtual pad: {exc}; retrying")
                    src, dst, ff = teardown(src, None, held, rumble)
                    time.sleep(2)
                    continue
                ff = FFHandler(dst, rumble)
                rumble.open()

            try:
                r, _, _ = select.select([src.fd, dst.fd], [], [], rumble.deadline())
            except OSError as exc:
                # A bad fd here would otherwise spin at 100% CPU with no output,
                # since select() on a dead fd returns immediately. Drop the
                # source and re-acquire rather than looping on it.
                log(f"select failed: {exc}; dropping source")
                src, dst, ff = teardown(src, dst, held, rumble)
                time.sleep(1)
                continue

            rumble.tick()

            if dst.fd in r:
                try:
                    ff.pump()
                except (OSError, UInputError) as exc:
                    # A bad uinput fd stays permanently "readable", so logging
                    # and carrying on would busy-loop. Rebuild instead.
                    log(f"ff pump failed ({exc}); rebuilding virtual pad")
                    src, dst, ff = teardown(src, dst, held, rumble)
                    continue

            if src.fd not in r:
                continue

            try:
                events = list(src.read())
            except OSError as exc:
                # Any I/O error means the pad is gone or the dongle is
                # re-associating. Treat them all as "reconnect" - letting an
                # unexpected errno escape would crash the daemon and can trip
                # systemd's start-rate limit into leaving the unit failed.
                log(f"source read failed ({exc}); waiting for it to return")
                src, dst, ff = teardown(src, dst, held, rumble)
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
        teardown(src, dst, held, rumble)
    return 0


if __name__ == "__main__":
    sys.exit(main())
