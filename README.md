# Standa 8SMC1-USBhF motor control

Python driver and EPICS asyn motor IOC for Standa 8SMC1-USBhF stepper motor
controllers (USB `10c4:0230`), running on a Raspberry Pi (Debian 13, aarch64).

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
creates `/dev/ttyUSB*` / `standa_motorN` nodes, which are of no use here. Both
drivers detach it.

## Python driver

```sh
python3 python/usmc.py list
python3 python/usmc.py status
python3 python/usmc.py -d 6183 power on          # limit switches enabled, active low
python3 python/usmc.py -d 6183 rmove 80 -s 100   # +10 full steps at 12.5 steps/s
python3 python/usmc.py -d 6183 stop
```

Needs `python3-usb` (pyusb). Don't run it while the IOC is moving the same axis.

## EPICS IOC

`ioc/` is a standard EPICS application. `USMCDriver.cpp` is an
`asynMotorController` that uses libusb-1.0, with one axis per USB controller
selected by serial number.

```
USMCCreateController(port, "serial1 serial2 ...", movingPollMs, idlePollMs)
USMCConfigLimits(port, enable, limit1ActiveHigh, limit2ActiveHigh)
```

Motor records: EGU = full steps, MRES = 0.125, so VELO ranges from 2 to 625 steps/s.
- Mode bits: CNEN switches motor power. Limit switch 1 maps to the low limit and
  limit switch 2 to the high limit.
- Homing: not supported, since the controller has no home input.
- Extra PVs: `$(P)$(M):Temperature`, `:Voltage`, `:Serial`, `:Connected`.
- Power on IOC start: when the IOC connects to an axis, it switches motor power on
  and sends a SET_MODE with the default mode (limits enabled, active low, current
  reduction on).

### Docker

The image is built on an arm64 host: EPICS base 7.0.8.1, asyn R4-45, motor R7-3-1.

```sh
docker/deploy.sh root@192.168.104.89   # build, copy the image, start the container
```

On the Pi (`docker/run-ioc.sh` is installed as `standa-ioc-run`):

```sh
docker logs standa-ioc
docker attach standa-ioc                  # IOC shell, detach with Ctrl-P Ctrl-Q
docker exec standa-ioc caget STANDA:m1.RBV
```

The Pi has no direct internet access. apt works through the proxy
`http://squid.lnf.infn.it:3128`. Its clock is not NTP-synchronised; if it drifts,
apt signature checks fail.
