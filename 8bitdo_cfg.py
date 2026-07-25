#!/usr/bin/env python3
"""Read/write 8BitDo Ultimate 2.4G onboard configuration over hidraw.

Protocol per https://github.com/TheJayMann/8bitdo-spec (Pro2 family).
The controller must be in D-input mode (physical X/D switch on D) for a
hidraw node to exist; in X-input mode it is a vendor-specific XInput
device with no HID interface at all.
"""

import os
import select
import struct
import sys

CONFIG_SIZE = 1652
CHUNK = 45
PKT = 64

REQ_WRITE = 1
REQ_READ = 2
REQ_REPORT_STATE = 7   # "enter config mode" - required before read/write
REQ_COMMIT = 6


def crc16_kermit(data):
    """CRC-16/KERMIT as used by the 8BitDo config protocol.

    Poly 0x8408 (reflected 0x1021), init 0xFFFF, no final XOR. This is the
    variant also known as CRC-16/MCRF4XX; check value for "123456789" is 0x6F91.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def config_crc(cfg):
    """CRC over the whole 1652-byte config with the CRC field itself zeroed."""
    buf = bytearray(cfg)
    struct.pack_into("<I", buf, OFF_CRC, 0)
    return crc16_kermit(buf)


def apply_config_crc(cfg):
    buf = bytearray(cfg)
    struct.pack_into("<I", buf, OFF_CRC, config_crc(buf))
    return bytes(buf)

ENABLED = 0x20190911

# Physical buttons, in the order their function slots appear in a profile.
BUTTONS = [
    "A", "B", "X", "Y", "L", "R", "L2", "R2", "L3", "R3",
    "Select", "Start", "Share", "Home", "Up", "Down", "Left", "Right",
    "P1", "P2",
]

# Function bit -> name.
FUNCTIONS = {
    0: "Start", 1: "L3", 2: "R3", 3: "Select", 4: "X", 5: "Y",
    6: "Right", 7: "Left", 8: "Down", 9: "Up", 10: "L1", 11: "R1",
    12: "B", 13: "A", 14: "L2", 15: "R2", 16: "Menu", 17: "Home",
    18: "BT Connect", 22: "Screenshot", 23: "Turbo Single",
    24: "Turbo Auto", 25: "P1", 26: "P2", 27: "Dynamic button swap",
}

MODES = {0: "Switch", 1: "DInput", 2: "Mac", 3: "XInput"}

# Offsets within the 1652-byte blob.
OFF_PROFILE_ENABLE = [0x00, 0x04, 0x08]
OFF_CRC = 0x0C
OFF_MODE = 0x10
OFF_SLOT = 0x12
OFF_MAPPING = [0x0E0, 0x134, 0x188]  # per-profile enable flag; slots follow


def func_name(val):
    if val == 0:
        return "(disabled)"
    bits = [i for i in range(32) if val & (1 << i)]
    if len(bits) == 1:
        return FUNCTIONS.get(bits[0], f"unknown bit {bits[0]}")
    return f"raw 0x{val:08X}"


def request(req_type, sub, data_len, offset, payload=b""):
    """Build a 64-byte request packet.

    Bytes 9-10 carry a CRC-16/KERMIT over this packet's data payload. Writing
    zero there (as an earlier version of this tool did) makes the firmware
    silently discard the config.
    """
    pkt = bytearray(PKT)
    pkt[0] = 0x81
    pkt[1] = data_len + 17
    pkt[2] = 0x04
    struct.pack_into("<HH", pkt, 3, req_type, sub)
    struct.pack_into("<H", pkt, 7, data_len)
    struct.pack_into("<H", pkt, 9, crc16_kermit(payload) if payload else 0)
    struct.pack_into("<I", pkt, 11, CONFIG_SIZE if req_type not in (REQ_COMMIT, REQ_REPORT_STATE) else 0)
    struct.pack_into("<I", pkt, 15, offset)
    pkt[19:19 + len(payload)] = payload
    return bytes(pkt)


def enter_config_mode(fd):
    """Send the report-state command. Some firmware ignores config without it."""
    try:
        exchange(fd, request(REQ_REPORT_STATE, 0, 0, 0), REQ_REPORT_STATE,
                 timeout=1.0)
    except TimeoutError:
        # This model does not acknowledge it; the command still primes the
        # device, so carry on rather than aborting.
        pass


def exchange(fd, pkt, want_type, timeout=3.0):
    """Send a request; return the matching response, skipping input reports.

    In D-input mode the device streams gamepad input reports on the same
    hidraw node, so responses must be filtered by their 02 04 04 00 header.
    """
    os.write(fd, pkt)
    deadline = select.select
    import time
    end = time.time() + timeout
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], max(0.05, end - time.time()))
        if not r:
            continue
        resp = os.read(fd, PKT)
        if len(resp) < 18:
            continue
        if resp[0:4] != b"\x02\x04\x04\x00":
            continue  # gamepad input report
        rtype = struct.unpack_from("<H", resp, 4)[0]
        if rtype != want_type:
            continue
        return resp
    raise TimeoutError(f"no response to request type {want_type}")


def read_config(path):
    fd = os.open(path, os.O_RDWR)
    try:
        enter_config_mode(fd)
        out = bytearray()
        for offset in range(0, CONFIG_SIZE, CHUNK):
            size = min(CHUNK, CONFIG_SIZE - offset)
            resp = exchange(fd, request(REQ_READ, 0, size, offset), REQ_READ)
            got_off = struct.unpack_from("<I", resp, 14)[0]
            if got_off != offset:
                raise IOError(f"offset mismatch: asked {offset}, got {got_off}")
            out += resp[18:18 + size]
        if len(out) != CONFIG_SIZE:
            raise IOError(f"short read: {len(out)} of {CONFIG_SIZE}")
        return bytes(out)
    finally:
        os.close(fd)


def write_config(path, cfg):
    if len(cfg) != CONFIG_SIZE:
        raise ValueError(f"config must be {CONFIG_SIZE} bytes, got {len(cfg)}")
    fd = os.open(path, os.O_RDWR)
    try:
        enter_config_mode(fd)
        for offset in range(0, CONFIG_SIZE, CHUNK):
            size = min(CHUNK, CONFIG_SIZE - offset)
            chunk = cfg[offset:offset + size]
            exchange(fd, request(REQ_WRITE, 0, size, offset, chunk), REQ_WRITE)
        exchange(fd, request(REQ_COMMIT, 21, 0, 0), REQ_COMMIT)
    finally:
        os.close(fd)


def dump(cfg):
    print(f"config: {len(cfg)} bytes")
    for i, off in enumerate(OFF_PROFILE_ENABLE):
        flag = struct.unpack_from("<I", cfg, off)[0]
        state = "enabled" if flag == ENABLED else ("blank" if flag == 0 else "?")
        print(f"  profile {i + 1} enable flag: 0x{flag:08X}  ({state})")
    print(f"  crc16 field  : 0x{struct.unpack_from('<I', cfg, OFF_CRC)[0]:08X}")
    mode = struct.unpack_from("<H", cfg, OFF_MODE)[0]
    print(f"  gamepad mode : {mode} ({MODES.get(mode, '?')})")
    print(f"  current slot : {struct.unpack_from('<H', cfg, OFF_SLOT)[0]}")

    for p, base in enumerate(OFF_MAPPING):
        flag = struct.unpack_from("<I", cfg, base)[0]
        state = "enabled" if flag == ENABLED else ("blank" if flag == 0 else "?")
        print(f"\n  profile {p + 1} button mapping: 0x{flag:08X} ({state})")
        for j, name in enumerate(BUTTONS):
            off = base + 4 + j * 4
            val = struct.unpack_from("<I", cfg, off)[0]
            mark = "  <<<" if name in ("P1", "P2") else ""
            print(f"    0x{off:03X}  {name:<7} -> {func_name(val)}{mark}")


# Identity mapping: each physical button drives its own function, except the
# paddles. Bit numbers come from the FUNCTIONS table above; note the spec's
# warning that R2/Home use bits 15/17 (swapped) in button-mapping context.
IDENTITY = {
    "A": 13, "B": 12, "X": 4, "Y": 5, "L": 10, "R": 11, "L2": 14, "R2": 15,
    "L3": 1, "R3": 2, "Select": 3, "Start": 0, "Share": 22, "Home": 17,
    "Up": 9, "Down": 8, "Left": 7, "Right": 6,
    "P1": 1,   # <-- paddle 1 emits L3
    "P2": 2,   # <-- paddle 2 emits R3
}


def patch(cfg, profiles=(0,), slot=None, mode=None, overrides=None):
    """Enable the given profiles with an identity map plus P1->L3 / P2->R3.

    Every other section is left exactly as read. Their enable flags are not
    the 0x20190911 magic, so the firmware treats them as unconfigured and
    applies its defaults, which is the current (working) behaviour.
    """
    mapping = dict(IDENTITY)
    mapping.update(overrides or {})
    buf = bytearray(cfg)
    for p in profiles:
        struct.pack_into("<I", buf, OFF_PROFILE_ENABLE[p], ENABLED)
        base = OFF_MAPPING[p]
        struct.pack_into("<I", buf, base, ENABLED)
        for j, name in enumerate(BUTTONS):
            struct.pack_into("<I", buf, base + 4 + j * 4, 1 << mapping[name])
    if slot is not None:
        struct.pack_into("<H", buf, OFF_SLOT, slot)
    if mode is not None:
        struct.pack_into("<H", buf, OFF_MODE, mode)
    return apply_config_crc(buf)


def diff(a, b):
    print(f"{'offset':>8}  {'before':>10}  {'after':>10}")
    runs = []
    i = 0
    while i < len(a):
        if a[i] != b[i]:
            j = i
            while j < len(a) and a[j] != b[j]:
                j += 1
            runs.append((i, j))
            i = j
        else:
            i += 1
    total = 0
    for start, end in runs:
        for off in range(start, end, 4):
            n = min(4, end - off)
            if n == 4:
                x = struct.unpack_from("<I", a, off)[0]
                y = struct.unpack_from("<I", b, off)[0]
                print(f"  0x{off:04X}  0x{x:08X}  0x{y:08X}")
            total += n
    print(f"{total} bytes changed")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        print("usage: 8bitdo_cfg.py read  <hidraw> <outfile>")
        print("       8bitdo_cfg.py dump  <file>")
        print("       8bitdo_cfg.py patch <infile> <outfile>")
        print("       8bitdo_cfg.py write <hidraw> <infile>")
        return 1
    cmd = sys.argv[1]
    if cmd == "read":
        cfg = read_config(sys.argv[2])
        with open(sys.argv[3], "wb") as f:
            f.write(cfg)
        print(f"saved {len(cfg)} bytes -> {sys.argv[3]}\n")
        dump(cfg)
    elif cmd == "dump":
        with open(sys.argv[2], "rb") as f:
            dump(f.read())
    elif cmd == "patch":
        with open(sys.argv[2], "rb") as f:
            orig = f.read()
        opts = sys.argv[4:]
        profiles = (0,)
        slot = mode = None
        if "--all-profiles" in opts:
            profiles = (0, 1, 2)
        if "--slot" in opts:
            slot = int(opts[opts.index("--slot") + 1])
        if "--mode" in opts:
            mode = int(opts[opts.index("--mode") + 1])
        overrides = {}
        if "--test-ab" in opts:
            overrides["A"] = IDENTITY["B"]  # physical A should emit B
        new = patch(orig, profiles=profiles, slot=slot, mode=mode,
                    overrides=overrides)
        with open(sys.argv[3], "wb") as f:
            f.write(new)
        diff(orig, new)
        print()
        dump(new)
    elif cmd == "write":
        with open(sys.argv[3], "rb") as f:
            cfg = f.read()
        write_config(sys.argv[2], cfg)
        print("write + commit complete")
    else:
        print(f"unknown command: {cmd}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
