#
# Build a Docker image with tools to build a zephyr firmware image
# Update the container image by running:
# $ docker buildx build --platform linux/arm64,linux/amd64 -t ghcr.io/mouse-gym/auto-trainer-hardware/zephyr-sdk --push .
#
# The command above publishes both platform images and their shared manifest.
#

FROM ubuntu:22.04 AS build

ARG TARGETARCH
ARG ZEPHYR_SDK_VERSION=0.16.8

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    git \
    cmake \
    ninja-build \
    gperf \
    ccache \
    dfu-util \
    device-tree-compiler \
    wget \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-tk \
    python3-wheel \
    xz-utils \
    file \
    libpython3-dev \
    make \
    gcc \
    libsdl2-dev \
    libmagic1 \
    protobuf-compiler \
    build-essential \
    python3-pyelftools && \
    if [ "${TARGETARCH}" = "amd64" ]; then \
        DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
        gcc-multilib \
        g++-multilib; \
    fi && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install \
    west \
    protobuf \
    pyelftools \
    click \
    cryptography \
    cbor \
    intelhex

RUN case "${TARGETARCH}" in \
        amd64) sdk_arch=x86_64 ;; \
        arm64) sdk_arch=aarch64 ;; \
        *) echo "Unsupported target architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    sdk_archive="zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-${sdk_arch}_minimal.tar.xz" && \
    cd /opt && \
    wget "https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/${sdk_archive}" && \
    tar xf "${sdk_archive}" && \
    rm -f "${sdk_archive}" && \
    cd "zephyr-sdk-${ZEPHYR_SDK_VERSION}" && \
    ./setup.sh -t arm-zephyr-eabi -c -h
