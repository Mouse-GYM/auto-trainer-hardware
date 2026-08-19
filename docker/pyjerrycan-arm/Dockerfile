FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3.8 \
    python3.8-dev \
    python3-pip \
    git \
    ninja-build

RUN apt-get install -y crossbuild-essential-arm64

RUN pip3 install --upgrade pip && \
    pip3 install wheel setuptools setuptools-scm pybind11
