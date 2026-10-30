FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates git cmake ninja-build build-essential python3 \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    libusb-1.0-0-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Pico SDK 2.3.1, including its exact lwIP and TinyUSB submodule revisions.
ARG PICO_SDK_REV=079c6f39023649b154152db30f1d781e884879bc
RUN git init /opt/pico-sdk \
    && cd /opt/pico-sdk \
    && git remote add origin https://github.com/raspberrypi/pico-sdk.git \
    && git fetch --depth 1 origin ${PICO_SDK_REV} \
    && git checkout --detach FETCH_HEAD \
    && git submodule update --init --depth 1 lib/tinyusb lib/lwip
ENV PICO_SDK_PATH=/opt/pico-sdk

# Backport TinyUSB PR #3914: Windows 11 requires NCM NTB divisors >= 4.
RUN python3 - <<'PY'
from pathlib import Path
import re
p = Path('/opt/pico-sdk/lib/tinyusb/src/class/net/ncm_device.c')
s = p.read_text()
for field in ('wNdbInDivisor', 'wNdbOutDivisor'):
    pattern = rf'(\.{field}\s*=\s*)[14](\s*,)'
    s, count = re.subn(pattern, rf'\g<1>4\2', s)
    assert count == 1, f'Unexpected TinyUSB source: {field}'
p.write_text(s)
PY

WORKDIR /project

# Rebased TinyUSB PR #3630: iOS NCM packet-filter and NTB input-size negotiation.
# Preserve this SDK's descriptor macro signature and the Windows divisors above.
COPY patches/tinyusb-ncm-ios.patch /tmp/tinyusb-ncm-ios.patch
RUN git -C /opt/pico-sdk/lib/tinyusb apply --check /tmp/tinyusb-ncm-ios.patch \
    && git -C /opt/pico-sdk/lib/tinyusb apply /tmp/tinyusb-ncm-ios.patch
