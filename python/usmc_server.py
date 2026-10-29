#!/usr/bin/env python3
"""TCP bridge to Standa 8SMC1-USBhF controllers (USB control transfers over the network).

Runs on the machine the controllers are plugged into, so the EPICS IOC can run
anywhere (e.g. in Kubernetes) and reach them through an asyn IP port.

Protocol: one ASCII line per request, one line per reply ("OK ..." or "ERR ...").
Numbers are hex unless noted; <sn> is the 16-char serial reported by LIST.

    LIST                                 -> OK <sn>:<bcdDevice> ...
    IN  <sn> <req> <value> <index> <len> -> OK <hexdata>   (len is decimal)
    OUT <sn> <req> <value> <index> [<hexdata>] -> OK

Only the requests the IOC needs are accepted: firmware download (0xC8),
set serial (0xCA) and save to flash (0x84) are refused.
"""

import argparse
import logging
import os
import socketserver
import sys
import threading

import usb.core

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from usmc import (USMC, REQ_GET_STATE, REQ_GET_ENCODER_STATE, REQ_GET_SERIAL,  # noqa: E402
                  REQ_SET_CURRENT_POSITION, REQ_STOP_STEP_MOTOR, REQ_GO_TO, REQ_SET_MODE,
                  REQ_SET_PARAMETERS, RT_IN, RT_OUT)

ALLOWED_IN = {REQ_GET_STATE, REQ_GET_ENCODER_STATE, REQ_GET_SERIAL}
ALLOWED_OUT = {REQ_SET_CURRENT_POSITION, REQ_STOP_STEP_MOTOR, REQ_GO_TO, REQ_SET_MODE,
               REQ_SET_PARAMETERS}

log = logging.getLogger("usmc_server")


class Bridge:
    """Controllers by serial; all USB access goes through one lock."""

    def __init__(self, timeout_ms=1000):
        self.lock = threading.Lock()
        self.timeout = timeout_ms
        self.ctrls = {}

    def rescan(self):
        old = self.ctrls
        self.ctrls = {}
        for c in USMC.find_all():
            self.ctrls[c.serial] = c
        if set(old) != set(self.ctrls):
            log.info("controllers: %s", " ".join("%s:%04x" % (s, c.version)
                                                 for s, c in sorted(self.ctrls.items())) or "none")

    def device(self, sn):
        if sn not in self.ctrls:
            self.rescan()
        if sn not in self.ctrls:
            raise LookupError("no controller %s" % sn)
        return self.ctrls[sn].dev

    def transfer(self, sn, rt, req, value, index, data_or_len):
        try:
            return self.device(sn).ctrl_transfer(rt, req, value, index, data_or_len, self.timeout)
        except usb.core.USBError:
            self.ctrls.pop(sn, None)  # unplugged or reset: find it again next time
            raise

    def handle(self, line):
        f = line.split()
        if not f:
            return "ERR empty request"
        cmd = f[0].upper()
        with self.lock:
            if cmd == "LIST" and len(f) == 1:
                self.rescan()
                return "OK " + " ".join("%s:%04x" % (s, c.version) for s, c in sorted(self.ctrls.items()))
            if cmd == "IN" and len(f) == 6:
                sn, req, value, index, n = f[1], int(f[2], 16), int(f[3], 16), int(f[4], 16), int(f[5])
                if req not in ALLOWED_IN or not 0 < n <= 64:
                    return "ERR request 0x%02x/%d not allowed" % (req, n)
                data = bytes(self.transfer(sn, RT_IN, req, value, index, n))
                if len(data) != n:
                    return "ERR short reply %d/%d" % (len(data), n)
                return "OK " + data.hex().upper()
            if cmd == "OUT" and len(f) in (5, 6):
                sn, req, value, index = f[1], int(f[2], 16), int(f[3], 16), int(f[4], 16)
                data = bytes.fromhex(f[5]) if len(f) == 6 else None
                if req not in ALLOWED_OUT:
                    return "ERR request 0x%02x not allowed" % req
                self.transfer(sn, RT_OUT, req, value, index, data)
                return "OK"
        return "ERR bad request"


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        peer = "%s:%d" % self.client_address
        log.info("connect %s", peer)
        for raw in self.rfile:
            line = raw.decode("ascii", "replace").strip()
            try:
                reply = self.server.bridge.handle(line)
            except (ValueError, LookupError, usb.core.USBError) as e:
                reply = "ERR %s" % e
            if reply.startswith("ERR"):
                log.warning("%s: %r -> %s", peer, line, reply)
            self.wfile.write(reply.encode("ascii") + b"\n")
        log.info("disconnect %s", peer)


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bind", default="0.0.0.0", help="address to listen on")
    ap.add_argument("--port", type=int, default=5080)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    srv = Server((a.bind, a.port), Handler)
    srv.bridge = Bridge()
    with srv.bridge.lock:
        srv.bridge.rescan()
    log.info("listening on %s:%d", a.bind, a.port)
    srv.serve_forever()


if __name__ == "__main__":
    sys.exit(main())
