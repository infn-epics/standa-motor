# Standa 8SMC1-USBhF motor control

EPICS asyn motor support for Standa 8SMC1-USBhF stepper motor controllers
(USB `10c4:0230`), plus a Python driver and a USB-to-TCP bridge.

```
 8SMC1 ×4 ──USB── Raspberry Pi ──TCP:5080── EPICS IOC (anywhere: epik8s pod, PC, ...)
                  usmc_server.py            standa-motor support (libUSMC)
```

The controllers speak USB vendor control transfers, not serial, so ser2net
cannot be used. `python/usmc_server.py` runs on the host they are plugged into
and forwards the transfers over TCP. The IOC holds all the 8SMC1 protocol logic
and reaches the bridge through an asyn IP port.

| Axis | PV          | Serial | USB path    |
|------|-------------|--------|-------------|
| 0    | `STANDA:m1` | 6183   | 1-1.3.1     |
| 1    | `STANDA:m2` | 5912   | 1-1.3.2.1   |
| 2    | `STANDA:m3` | 6098   | 1-1.3.3.1   |
| 3    | `STANDA:m4` | 6140   | 1-1.3.3.3.1 |

## Protocol notes

The manual (`docs/`) only documents the Windows `USMCDLL.dll`. The device has
**no data endpoints**: every command is a vendor control transfer on EP0. The
packet layouts come from the XInstruments Linux driver `usmcdrv`/`libusmc`
(2007) and were verified on firmware 2.4.10:

| Request | Code | Notes |
|---|---|---|
| GET_SERIAL | `0xC9` in, 16 B | ASCII serial |
| GET_STATE | `0x82` in, 11 B | pos (LE int32, /8 → 1/8 steps), temp, flags, voltage |
| GET_ENCODER_STATE | `0x85` in, 8 B | |
| GO_TO | `0x80` out | wValue/wIndex = raw position hi/lo, data = timer period (BE) + flags |
| SET_MODE | `0x81` out | power (ResetD), limit switches; the mode cannot be read back |
| SET_PARAMETERS | `0x83` out | acceleration etc. (RAM) |
| SAVE_PARAMETERS | `0x84` out | writes flash: not used by the IOC |
| SET_CURRENT_POSITION | `0x01` out | |
| STOP_STEP_MOTOR | `0x07` out | |

Measured behaviour:
- Positions are in 1/8 full steps, speeds in (1/8 steps)/s, range 16..5000.
- RUN rises about 50 ms after GO_TO; don't treat "not running" right after a move as done.
- Set position only keeps multiples of 32 units (4 full steps): 1000 → 1024.

The `cp210x` serial driver gets force-bound to these devices (`new_id`). That
creates `/dev/ttyUSB*` / `standa_motorN` nodes, which are of no use here. The bridge
and the Python driver detach it.

## Python driver

```sh
python3 python/usmc.py list
python3 python/usmc.py status
python3 python/usmc.py -d 6183 power on          # limit switches enabled, active low
python3 python/usmc.py -d 6183 rmove 80 -s 100   # +10 full steps at 12.5 steps/s
python3 python/usmc.py -d 6183 stop
```

Needs `python3-usb` (pyusb). Don't run it while the IOC is moving the same axis.

## USB-to-TCP bridge (usmc-server)

`python/usmc_server.py` runs on the host the controllers are plugged into. It
needs `python3-usb` and `usmc.py` in the same directory. Options are `--bind`
(default `0.0.0.0`), `--port` (default `5080`) and `-v` for debug logging.

### Build

There is nothing to compile: the bridge is two Python files. It needs Python 3
(tested with 3.13) and pyusb (tested with 1.2.1) on the host. It runs as root
so it can open the USB devices and detach `cp210x`.

```sh
# on the Pi: no direct internet, so apt goes through the proxy
export http_proxy=http://squid.lnf.infn.it:3128 https_proxy=http://squid.lnf.infn.it:3128
apt-get update && apt-get install -y python3-usb
```

If apt complains that signatures are "not live yet" or expired, the Pi's
clock is wrong. Set it, for example from your machine with
`ssh root@<pi> date -s "@$(date +%s)"`, then retry.

Try it in the foreground before installing it:

```sh
python3 python/usmc_server.py -v --bind 127.0.0.1
```

### Install

```sh
PI=root@192.168.104.89
ssh $PI mkdir -p /opt/standa
scp python/usmc.py python/usmc_server.py python/usmc-server.service $PI:/opt/standa/
ssh $PI 'cp /opt/standa/usmc-server.service /etc/systemd/system/ &&
         systemctl daemon-reload && systemctl enable --now usmc-server'
```

The unit (`python/usmc-server.service`) runs
`/usr/bin/python3 /opt/standa/usmc_server.py --port 5080`. It restarts on
failure and starts at boot. To listen on a single interface instead of all
of them, override `ExecStart` with `systemctl edit usmc-server`:

```ini
[Service]
ExecStart=
ExecStart=/usr/bin/python3 /opt/standa/usmc_server.py --bind 192.168.104.89 --port 5080
```

### Verify

```sh
ssh $PI 'systemctl status usmc-server --no-pager; ss -ltn | grep 5080'
printf 'LIST\n' | nc -w2 192.168.104.89 5080   # from the IOC network
# OK 0000000000005912:2410 0000000000006098:2410 0000000000006140:2410 0000000000006183:2410
ssh $PI journalctl -u usmc-server -f            # connects, refused requests, USB errors
```

`LIST` must show every controller the IOC expects. A controller that is
unplugged and plugged back in is found again on the next request.

### Update

Copy the new `.py` files to `/opt/standa/` again, then run
`systemctl restart usmc-server`. IOCs reconnect by themselves. Their axes show
`:Connected` = 0 for a moment and then come back.

### Protocol

One line per request:

```
LIST                                 -> OK <sn>:<bcdDevice> ...
IN  <sn> <req> <value> <index> <len> -> OK <hexdata>
OUT <sn> <req> <value> <index> [<hexdata>] -> OK
```

Only the requests the IOC uses are accepted (state, encoder, serial, go-to,
mode, parameters, set position, stop). Firmware download, set serial and
save-to-flash are refused. There is no authentication, so bind the bridge to an
interface that only the IOC network can reach. The bridge serialises USB access,
so several clients (the IOC, `nc` for debugging) can share it. The Python
CLI talks to USB directly; don't use it while the bridge is driving the same
axis.

## EPICS support module

This repository is an EPICS support module: library `USMC`, `USMCSupport.dbd`,
`db/standaAxis.db`. Each axis of an `asynMotorController` is one 8SMC1, picked by
serial number among those the bridge reports.

```
drvAsynIPPortConfigure("STANDA_BRIDGE", "<pi>:5080", 0, 0, 0)
USMCCreateController(port, "STANDA_BRIDGE", "serial1 serial2 ...", movingPollMs, idlePollMs)
USMCConfigLimits(port, enable, limit1ActiveHigh, limit2ActiveHigh)
```

- Units: motor records use EGU = full steps and MRES = 0.125, so VELO ranges
  from 2 to 625 steps/s.
- CNEN switches motor power.
- Limits: switch 1 is the low limit and switch 2 the high limit.
- Homing: not supported, since the controller has no home input.
- Extra PVs: `$(P)$(M):Temperature`, `:Voltage`, `:Serial`, `:Connected`.
- Motor power at connect: when the IOC connects to an axis, it powers the motor
  and sends the default mode (limits enabled, active low, current reduction on).

### epik8s / ibek

The `standa-motor` ibek support lives in `ibek-support-infn`, and the
`devtype: standa` branch is in the ibek-templates `motor` template. Example
beamline entry:

```yaml
standa-mot01:
  template: motor
  devtype: standa
  devgroup: mot
  image: ghcr.io/infn-epics/infn-epics-ioc-runtime:v26.10.29   # first release with standa-motor
  iocprefix: SPARC:MOT:STANDA
  server: 192.168.104.89      # host running usmc_server.py
  port: 5080
  devices:
    - {name: M1, serial: "6183"}
    - {name: M2, serial: "5912", egu: mm, mres: 0.000625}
```

Optional settings: `mres`, `egu`, `velo`, `vbas` and `accl` can be set on a
device or on the whole IOC. `limitsEnable`, `limit1ActiveHigh` and
`limit2ActiveHigh` apply to the whole IOC.

### Deploying a change

1. Merge to `main` here. The `infn-epics-ioc` image builds `standa-motor` from
   `main` (see `ibek-support-infn/standa-motor/standa-motor.install.yml`).
2. If `ibek-support-infn` or `ibek-templates` changed, bump those submodules in
   `infn-epics-ioc`.
3. Tag a release in `infn-epics-ioc` (`vYY.MM.DD`). The tag build publishes
   `infn-epics-ioc-runtime:<tag>`.
4. Point `image:` in the beamline `deploy/values.yaml` (for example
   `epik8-sparc`) at the new tag, then push.
5. ArgoCD creates or updates the app `<beamline>-<name>-ioc`, for example
   `sparc-standa-mot01-ioc`. Motor IOCs have no automatic sync, so sync the app
   yourself:

   ```sh
   argocd app sync sparc-standa-mot01-ioc
   kubectl -n sparc logs -f statefulset/standa-mot01   # expect all axes connected
   ```

The pod needs TCP access to the bridge. If it can't reach the bridge, the IOC
stops at startup and the pod restarts in a loop.

Don't leave the standalone test IOC (below) running on the same motors as the
cluster IOC. Both work through the bridge, so a move from one shows up in the
other.

### Standalone test IOC

`iocBoot/iocStanda` is built when `BUILD_IOCS = YES`. `docker/` builds it as an
arm64 image (EPICS base 7.0.8.1, asyn R4-45, motor R7-3-1) for the Pi:

```sh
docker/deploy.sh root@192.168.104.89   # build, copy the image, start the container
docker exec standa-ioc caget STANDA:m1.RBV
```

`USMC_BRIDGE` defaults to `127.0.0.1:5080`, i.e. the bridge on the same host.
The container needs `--ulimit nofile=4096`; with Docker's default limit the CA
tools spin when they fork `caRepeater`.

## Raspberry Pi notes

The Pi has no direct internet access. apt works through the proxy
`http://squid.lnf.infn.it:3128`. Its clock is not NTP-synchronised; if it drifts,
apt signature checks fail.
