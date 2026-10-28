#!/usr/bin/env python3
"""Driver for Standa 8SMC1-USBhF stepper motor controllers (USB 10c4:0230).

The controller has no data endpoints: every command is a vendor control
transfer on EP0.  The packet layouts follow the XInstruments Linux driver
(usmcdrv / libusmc, 2007), which mirrors USMCDLL.dll documented in the manual.

Units: positions are in 1/8 full steps (as USMC_State.CurPos), speeds in
1/8 steps per second as accepted by USMC_Start (16..5000; measured on fw 2.4.10).
"""

import argparse
import struct
import sys
import time

import usb.core

VID, PID = 0x10C4, 0x0230

REQ_SET_CURRENT_POSITION = 0x01
REQ_STOP_STEP_MOTOR = 0x07
REQ_EMULATE_BUTTONS = 0x0D
REQ_GO_TO = 0x80
REQ_SET_MODE = 0x81
REQ_GET_STATE = 0x82
REQ_SET_PARAMETERS = 0x83
REQ_SAVE_PARAMETERS = 0x84
REQ_GET_ENCODER_STATE = 0x85
REQ_GET_SERIAL = 0xC9

RT_IN = 0xC0   # device-to-host, vendor, device
RT_OUT = 0x40  # host-to-device, vendor, device

MICROSTEP_BITS = {1: 0b00, 2: 0b01, 4: 0b10, 8: 0b11}

STATE_FLAGS = (
    # byte 6
    "M1", "M2", "Loft", "FullPower", "CW_CCW", "Power", "FullSpeed", "AReset",
    # byte 7
    "RUN", "SyncIN", "SyncOUT", "RotTr", "RotTrErr", "EmReset", "Trailer1", "Trailer2",
)

MODE_BITS = (
    # byte 0
    "PMode", "PReg", "ResetD", "EMReset", "Tr1T", "Tr2T", "RotTrT", "TrSwap",
    # byte 1
    "Tr1En", "Tr2En", "RotTeEn", "RotTrOp", "Butt1T", "Butt2T", "ButSwap", "ResetRT",
    # byte 2
    "SyncOUTEn", "SyncOUTR", "SyncINOp", "SyncInvert", "EncoderEn", "EncoderInv",
    "ResBEnc", "ResEnc",
)

# USMC_Mode defaults used by libusmc/USMCDLL (ResetD=False -> motor powered).
DEFAULT_MODE = dict(PReg=True, Tr1En=True, Tr2En=True, RotTrOp=True,
                    SyncOUTEn=True, SyncINOp=True, SyncCount=4)

# USMC_Parameters defaults used by libusmc/USMCDLL.
DEFAULT_PARAMETERS = dict(
    AccelT=200.0, DecelT=200.0, PTimeout=100.0,
    BTimeout1=500.0, BTimeout2=500.0, BTimeout3=500.0, BTimeout4=500.0,
    BTimeoutR=500.0, BTimeoutD=500.0, MinP=500.0,
    BTO1P=200.0, BTO2P=300.0, BTO3P=400.0, BTO4P=500.0,
    MaxLoft=32, StartPos=0, RTDelta=200, RTMinError=15, MaxTemp=70.0,
    SynOUTP=1, LoftPeriod=500.0, EncMult=2.5,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _speed_to_period(speed):
    """Timer reload for GO_TO (USMC_Start conversion)."""
    return int(65536.0 - 1000000.0 / _clamp(speed, 16.0, 5000.0) + 0.5)


def _period_to_speed(period):
    return 1000000.0 / (65536.0 - period)


def _button_period(speed):
    """Timer reload for the parameter-packet speeds (MinP, BTOxP)."""
    return int(65536.0 - 125000.0 / _clamp(speed, 2.0, 625.0) + 0.5)


def _timeout(ms):
    return int(_clamp(ms, 1.0, 9961.0) / 0.152 + 0.5)


def _bits(values, names):
    """Pack booleans named in `names` (LSB first, 8 per byte) into bytes."""
    out = bytearray((len(names) + 7) // 8)
    for i, name in enumerate(names):
        if values.get(name):
            out[i // 8] |= 1 << (i % 8)
    return bytes(out)


class USMCError(Exception):
    pass


class USMC:
    """One 8SMC1-USBhF controller."""

    def __init__(self, dev, timeout_ms=1000):
        self.dev = dev
        self.timeout = timeout_ms
        try:
            if dev.is_kernel_driver_active(0):
                # cp210x is sometimes force-bound via new_id; it cannot drive this device.
                dev.detach_kernel_driver(0)
        except (usb.core.USBError, NotImplementedError):
            pass
        self.serial = self.get_serial()
        self.version = dev.bcdDevice

    # ------------------------------------------------------------ discovery
    @staticmethod
    def usb_path(dev):
        return "%d-%s" % (dev.bus, ".".join(map(str, dev.port_numbers or ())))

    @classmethod
    def find_all(cls):
        devs = usb.core.find(find_all=True, idVendor=VID, idProduct=PID)
        devs = sorted(devs, key=lambda d: (d.bus, d.port_numbers or ()))
        return [cls(d) for d in devs]

    @classmethod
    def open(cls, ident):
        """Open by serial (full or trailing digits), USB path, or index."""
        ctrls = cls.find_all()
        for c in ctrls:
            if ident in (c.serial, c.serial.lstrip("0"), cls.usb_path(c.dev)):
                return c
        if ident.isdigit() and int(ident) < len(ctrls) and len(ident) <= 2:
            return ctrls[int(ident)]
        raise USMCError("controller %r not found (have: %s)" % (
            ident, ", ".join(c.serial.lstrip("0") for c in ctrls) or "none"))

    # ------------------------------------------------------------ transport
    def _in(self, req, length):
        data = bytes(self.dev.ctrl_transfer(RT_IN, req, 0, 0, length, self.timeout))
        if len(data) != length:
            raise USMCError("short reply to 0x%02x: %d/%d bytes" % (req, len(data), length))
        return data

    def _out(self, req, value=0, index=0, data=None):
        self.dev.ctrl_transfer(RT_OUT, req, value, index, data, self.timeout)

    def _out_packet(self, req, packet, value_swapped=True, index_swapped=True):
        """Send a packet the way usmcdrv does: bytes 0-3 in wValue/wIndex, the rest as data."""
        b = packet
        value = (b[0] << 8 | b[1]) if value_swapped else (b[1] << 8 | b[0])
        index = (b[2] << 8 | b[3]) if index_swapped else (b[3] << 8 | b[2])
        self._out(req, value, index, b[4:] or None)

    # ------------------------------------------------------------ queries
    def get_serial(self):
        return self._in(REQ_GET_SERIAL, 16).decode("ascii", "replace").rstrip("\0")

    def get_state(self):
        raw = self._in(REQ_GET_STATE, 11)
        cur, temp, flags, b8, volt = struct.unpack("<iHHBH", raw)
        st = {name: bool(flags >> i & 1) for i, name in enumerate(STATE_FLAGS)}
        st["CurPos"] = int(cur / 8)  # C truncation, as libusmc
        st["SDivisor"] = 1 << (st["M2"] << 1 | st["M1"])
        if self.version < 0x2400:
            import math
            t = temp * 3.3 / 65536.0
            t = t * 10.0 / (5.0 - t)
            t = 1.0 / (1.0 / 298.0 + math.log(t / 10.0) / 3950.0) - 273.0
        else:
            t = temp * 3.3 * 100.0 / 65536.0 - 50.0
        st["Temp"] = round(t, 2)
        v = volt / 65536.0 * 3.3 * 20.0
        st["Voltage"] = round(v if v >= 5.0 else 0.0, 2)
        st["USBPower"] = bool(b8 & 0x01)
        st["Working"] = bool(b8 & 0x80)
        for k in ("M1", "M2"):
            del st[k]
        return st

    def get_encoder(self):
        ecur, enc = struct.unpack("<ii", self._in(REQ_GET_ENCODER_STATE, 8))
        return {"ECurPos": ecur, "EncoderPos": enc}

    # ------------------------------------------------------------ commands
    def move_to(self, position, speed=200.0, divisor=8, slow_start=True,
                loft_en=False, def_dir=False, force_loft=False,
                wait_sync=False, sync_out_reset=False):
        """Start a move to `position` (1/8 steps). Returns the actual speed."""
        if divisor not in MICROSTEP_BITS:
            raise ValueError("divisor must be 1, 2, 4 or 8")
        raw = (int(position) * 8) & 0xFFFFFFFF
        period = _speed_to_period(speed)
        flags = (MICROSTEP_BITS[divisor] | def_dir << 2 | loft_en << 3 | slow_start << 4
                 | wait_sync << 5 | sync_out_reset << 6 | force_loft << 7)
        self._out(REQ_GO_TO, value=raw >> 16, index=raw & 0xFFFF,
                  data=bytes((period >> 8, period & 0xFF, flags)))
        return _period_to_speed(period)

    def move_relative(self, delta, **kw):
        """Start a relative move. Returns (destination, actual speed)."""
        dest = self.get_state()["CurPos"] + delta
        return dest, self.move_to(dest, **kw)

    def stop(self):
        self._out(REQ_STOP_STEP_MOTOR)

    def set_position(self, position):
        """Redefine the current position (resolution: 4 units)."""
        raw = (int(position) * 8) & 0xFFFFFFE0
        self._out(REQ_SET_CURRENT_POSITION, value=raw >> 16, index=raw & 0xFFFF)

    def set_mode(self, **mode):
        """Send USMC_Mode. Unspecified fields take the libusmc defaults.

        The controller cannot report its mode, so every call sets all of it.
        Power off: ResetD=True.  Limit switches: Tr1En/Tr2En, polarity Tr1T/Tr2T.
        """
        m = dict(DEFAULT_MODE)
        m.update(mode)
        unknown = set(m) - set(MODE_BITS) - {"SyncCount"}
        if unknown:
            raise ValueError("unknown mode fields: %s" % ", ".join(sorted(unknown)))
        packet = _bits(m, MODE_BITS) + struct.pack(">I", int(m["SyncCount"]) & 0xFFFFFFFF)
        self._out_packet(REQ_SET_MODE, packet)
        return m

    def set_parameters(self, **params):
        """Send USMC_Parameters (RAM only; see save_parameters)."""
        p = dict(DEFAULT_PARAMETERS)
        p.update(params)
        unknown = set(p) - set(DEFAULT_PARAMETERS)
        if unknown:
            raise ValueError("unknown parameters: %s" % ", ".join(sorted(unknown)))
        if self.version < 0x2400:
            raise USMCError("MaxTemp conversion for firmware < 2.4 not implemented")
        start = (int(p["StartPos"]) * 8 & 0xFFFFFF00) if self.version >= 0x2407 else 0
        max_temp = int((_clamp(p["MaxTemp"], 0.0, 100.0) + 50.0) / 330.0 * 65536.0 + 0.5)
        loft = 0 if p["LoftPeriod"] == 0 else int(
            65536.0 - 125000.0 / _clamp(p["LoftPeriod"], 16.0, 5000.0) + 0.5)
        packet = struct.pack(
            "<BBH", _clamp(int(p["AccelT"] / 98.0 + 0.5), 1, 15),
            _clamp(int(p["DecelT"] / 98.0 + 0.5), 1, 15), _timeout(p["PTimeout"]))
        packet += struct.pack(
            ">6H5HHIHHHBHB",
            *(_timeout(p[k]) for k in ("BTimeout1", "BTimeout2", "BTimeout3", "BTimeout4",
                                       "BTimeoutR", "BTimeoutD")),
            *(_button_period(p[k]) for k in ("MinP", "BTO1P", "BTO2P", "BTO3P", "BTO4P")),
            _clamp(int(p["MaxLoft"]), 1, 1023) * 64, start,
            _clamp(int(p["RTDelta"]), 4, 1023) * 64,
            _clamp(int(p["RTMinError"]), 4, 1023) * 64,
            max_temp, int(p["SynOUTP"]) & 0xFF, loft, int(p["EncMult"] * 4.0 + 0.5) & 0xFF)
        packet += bytes(15)
        assert len(packet) == 57
        # wValue = DELAY1:DELAY2, wIndex = RefINTimeout as stored (little endian)
        self._out_packet(REQ_SET_PARAMETERS, packet, index_swapped=False)
        return p

    def save_parameters(self):
        """Write the current mode and parameters to the controller's flash."""
        self._out(REQ_SAVE_PARAMETERS)

    def power(self, on, **mode):
        return self.set_mode(ResetD=not on, **mode)

    def wait(self, dest=None, timeout=60.0, poll=0.05, verbose=False, start_latency=0.3):
        """Wait until the move is over. Returns the final state.

        RUN is raised ~50 ms after GO_TO, so a stopped motor only counts as done
        once it is at `dest` or `start_latency` has passed (e.g. blocked by a limit).
        """
        t0 = time.time()
        while True:
            st = self.get_state()
            if verbose:
                print("  pos=%-8d run=%d" % (st["CurPos"], st["RUN"]))
            if not st["RUN"] and (st["CurPos"] == dest or time.time() - t0 > start_latency):
                return st
            if time.time() - t0 > timeout:
                self.stop()
                raise USMCError("move timeout, motor stopped")
            time.sleep(poll)


# ------------------------------------------------------------------ CLI
def _fmt_state(st):
    flags = " ".join(k for k in STATE_FLAGS if st.get(k))
    return ("pos=%d div=1/%d T=%.1fC V=%.1fV  [%s]"
            % (st["CurPos"], st["SDivisor"], st["Temp"], st["Voltage"], flags))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-d", "--device", help="serial, USB path or index (default: all / first)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="list controllers")
    sub.add_parser("status", help="print state")
    p = sub.add_parser("power", help="motor power on/off (sets the whole mode)")
    p.add_argument("onoff", choices=("on", "off"))
    p.add_argument("--no-limits", action="store_true", help="disable limit switches")
    p.add_argument("--tr1t", action="store_true", help="limit 1 active high")
    p.add_argument("--tr2t", action="store_true", help="limit 2 active high")
    for name, hlp in (("move", "absolute move"), ("rmove", "relative move")):
        p = sub.add_parser(name, help=hlp + " (1/8 steps)")
        p.add_argument("pos", type=int)
        p.add_argument("-s", "--speed", type=float, default=200.0, help="1/8 steps/s (16..5000)")
        p.add_argument("--no-wait", action="store_true")
    sub.add_parser("stop", help="stop motion")
    p = sub.add_parser("setpos", help="redefine current position")
    p.add_argument("pos", type=int)
    p = sub.add_parser("params", help="send motion parameters (RAM)")
    p.add_argument("--accel", type=float, default=200.0, help="ms")
    p.add_argument("--decel", type=float, default=200.0, help="ms")
    a = ap.parse_args(argv)

    if a.cmd in ("list", "status") and not a.device:
        ctrls = USMC.find_all()
    else:
        ctrls = [USMC.open(a.device) if a.device else USMC.find_all()[0]]

    for c in ctrls:
        tag = "%s sn=%s fw=%x.%x.%x" % (USMC.usb_path(c.dev), c.serial.lstrip("0"),
                                        c.version >> 12, c.version >> 8 & 0xF, c.version & 0xFF)
        if a.cmd == "list":
            print(tag)
        elif a.cmd == "status":
            print(tag + "  " + _fmt_state(c.get_state()) + "  enc=%(EncoderPos)d" % c.get_encoder())
        elif a.cmd == "power":
            c.power(a.onoff == "on", Tr1En=not a.no_limits, Tr2En=not a.no_limits,
                    Tr1T=a.tr1t, Tr2T=a.tr2t)
            time.sleep(0.1)
            print(tag + "  " + _fmt_state(c.get_state()))
        elif a.cmd in ("move", "rmove"):
            if a.cmd == "move":
                dest, v = a.pos, c.move_to(a.pos, speed=a.speed)
            else:
                dest, v = c.move_relative(a.pos, speed=a.speed)
            print("moving to %d at %.1f (1/8 steps)/s" % (dest, v))
            if not a.no_wait:
                try:
                    st = c.wait(dest, verbose=True)
                except KeyboardInterrupt:
                    c.stop()
                    st = c.get_state()
                print(_fmt_state(st))
        elif a.cmd == "stop":
            c.stop()
            print(_fmt_state(c.get_state()))
        elif a.cmd == "setpos":
            c.set_position(a.pos)
            print(_fmt_state(c.get_state()))
        elif a.cmd == "params":
            c.set_parameters(AccelT=a.accel, DecelT=a.decel)
            print("parameters sent")


if __name__ == "__main__":
    sys.exit(main())
