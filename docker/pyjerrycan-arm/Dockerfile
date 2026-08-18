FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH
ARG CMAKE_VERSION=3.31.2

RUN apt-get update && apt-get install -y \
    build-essential \
    python3.8 \
    python3.8-dev \
    python3-pip \
    git \
    ninja-build \
    wget

# Install a recent CMake while retaining Ubuntu 20.04 as the target userspace.
RUN case "${TARGETARCH}" in \
        arm64) cmake_arch=aarch64 ;; \
        *) echo "Unsupported target architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    cmake_installer="cmake-${CMAKE_VERSION}-linux-${cmake_arch}.sh" && \
    cd /usr/local && \
    wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${cmake_installer}" && \
    /bin/sh "${cmake_installer}" --skip-license && \
    rm "${cmake_installer}"

RUN apt-get install -y crossbuild-essential-arm64

RUN pip3 install --upgrade pip && \
    pip3 install wheel setuptools setuptools-scm pybind11
