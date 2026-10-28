#!/bin/sh
# (Re)start the Standa IOC container on the Raspberry Pi.
#   PREFIX        PV prefix                   (default STANDA:)
#   USMC_SERIALS  serials, one per axis m1..  (default "6183 5912 6098 6140")
#
# --ulimit nofile: CA clients fork caRepeater, which closes every fd up to the
#   limit; with Docker's default (~1e9) caget/camonitor spin for minutes.
# /dev/bus/usb + cgroup rule (USB major 189) keep re-plugged controllers usable.
# Attach to the IOC shell with: docker attach standa-ioc  (detach: Ctrl-P Ctrl-Q)
set -e
docker rm -f standa-ioc >/dev/null 2>&1 || true
docker run -dit --name standa-ioc --restart unless-stopped \
    --network host --ulimit nofile=4096:4096 \
    -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
    -e PREFIX="${PREFIX:-STANDA:}" \
    -e USMC_SERIALS="${USMC_SERIALS:-6183 5912 6098 6140}" \
    standa-ioc:latest
