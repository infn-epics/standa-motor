# EPICS IOC for Standa 8SMC1-USBhF stepper motor controllers.
# Build context: repository root.  Target platform: linux/arm64 (Raspberry Pi, Debian 13).

ARG DEBIAN=trixie

# ---------------------------------------------------------------- EPICS base + asyn + motor
FROM debian:${DEBIAN} AS epics
ARG EPICS_BASE_TAG=R7.0.8.1
ARG ASYN_TAG=R4-45
ARG MOTOR_TAG=R7-3-1
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential perl git ca-certificates libreadline-dev re2c \
        libtirpc-dev libusb-1.0-0-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*
ENV EPICS_ROOT=/opt/epics
ENV EPICS_BASE=${EPICS_ROOT}/base
WORKDIR ${EPICS_ROOT}

RUN git clone --depth 1 -b ${EPICS_BASE_TAG} https://github.com/epics-base/epics-base.git base \
    && make -C base -j"$(nproc)" \
    && make -C base clean >/dev/null

RUN git clone --depth 1 -b ${ASYN_TAG} https://github.com/epics-modules/asyn.git support/asyn \
    && printf 'SUPPORT=%s\nEPICS_BASE=%s\n' "${EPICS_ROOT}/support" "${EPICS_BASE}" \
        > support/asyn/configure/RELEASE.local \
    && printf 'TIRPC=YES\n' > support/asyn/configure/CONFIG_SITE.local \
    && make -C support/asyn -j"$(nproc)"

RUN git clone --depth 1 -b ${MOTOR_TAG} https://github.com/epics-modules/motor.git support/motor \
    && printf 'SUPPORT=%s\nEPICS_BASE=%s\nASYN=%s\n' "${EPICS_ROOT}/support" "${EPICS_BASE}" \
        "${EPICS_ROOT}/support/asyn" > support/motor/configure/RELEASE.local \
    && printf 'BUILD_IOCS=NO\n' > support/motor/configure/CONFIG_SITE.local \
    && make -C support/motor -j"$(nproc)"

# ---------------------------------------------------------------- Standa IOC
FROM epics AS build
COPY ioc /opt/epics/ioc/standa
RUN printf 'SUPPORT=%s\nEPICS_BASE=%s\nASYN=%s\nMOTOR=%s\n' "${EPICS_ROOT}/support" "${EPICS_BASE}" \
        "${EPICS_ROOT}/support/asyn" "${EPICS_ROOT}/support/motor" \
        > /opt/epics/ioc/standa/configure/RELEASE.local \
    && make -C /opt/epics/ioc/standa -j"$(nproc)"

# ---------------------------------------------------------------- runtime
FROM debian:${DEBIAN}-slim AS runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
        libreadline8t64 libtirpc3t64 libusb-1.0-0 python3 python3-usb \
    && rm -rf /var/lib/apt/lists/*
ENV EPICS_ROOT=/opt/epics
ENV EPICS_BASE=${EPICS_ROOT}/base
ENV PATH=${EPICS_BASE}/bin/linux-aarch64:${PATH}
COPY --from=build /opt/epics/base/bin    /opt/epics/base/bin
COPY --from=build /opt/epics/base/lib    /opt/epics/base/lib
COPY --from=build /opt/epics/base/dbd    /opt/epics/base/dbd
COPY --from=build /opt/epics/support/asyn/lib  /opt/epics/support/asyn/lib
COPY --from=build /opt/epics/support/asyn/db   /opt/epics/support/asyn/db
COPY --from=build /opt/epics/support/motor/lib /opt/epics/support/motor/lib
COPY --from=build /opt/epics/support/motor/db  /opt/epics/support/motor/db
COPY --from=build /opt/epics/ioc/standa  /opt/epics/ioc/standa
COPY python/usmc.py /usr/local/bin/usmc.py
WORKDIR /opt/epics/ioc/standa/iocBoot/iocStanda
CMD ["../../bin/linux-aarch64/standa", "st.cmd"]
