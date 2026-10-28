#!/bin/sh
# Build the IOC image for linux/arm64 locally and load it on the Raspberry Pi.
# Usage: docker/deploy.sh [user@host]
set -e
HOST=${1:-root@192.168.104.89}
cd "$(dirname "$0")/.."
docker build --platform linux/arm64 -t standa-ioc:latest -f docker/Dockerfile .
docker save standa-ioc:latest | gzip -1 | ssh "$HOST" 'gunzip | docker load'
scp -q docker/run-ioc.sh "$HOST":/usr/local/bin/standa-ioc-run
ssh "$HOST" /usr/local/bin/standa-ioc-run
