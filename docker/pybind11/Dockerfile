#
# Build a Docker image with tools to build a pybind11 package
# Update the container image by running:
# $ docker buildx build --platform linux/arm64,linux/amd64 -t ghcr.io/mouse-gym/auto-trainer-hardware/pybind11 --push .
#

FROM ubuntu:24.04 AS build

ARG TARGETARCH
ARG CMAKE_VERSION=3.31.2

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    build-essential \
    git \
    ninja-build \
    wget \
    python3 \
    python3-dev \
    python3-pip

# Install a recent version of cmake
RUN case "${TARGETARCH}" in \
        amd64) cmake_arch=x86_64 ;; \
        arm64) cmake_arch=aarch64 ;; \
        *) echo "Unsupported target architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    cmake_installer="cmake-${CMAKE_VERSION}-linux-${cmake_arch}.sh" && \
    cd /usr/local && \
    wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${cmake_installer}" && \
    /bin/sh "${cmake_installer}" --skip-license && \
    rm "${cmake_installer}"

RUN pip3 install --break-system-packages pybind11 mypy pytest
