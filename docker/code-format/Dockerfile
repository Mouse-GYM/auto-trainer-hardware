#
# Build a Docker image with clang-format
# Update the container image by running:
# $ docker buildx build -t ghcr.io/mouse-gym/auto-trainer-hardware/code-format .
#
# Update the container registry by running:
# $ docker push ghcr.io/mouse-gym/auto-trainer-hardware/code-format
#

FROM ubuntu AS build

RUN apt update && \
    apt install -y git cmake build-essential python3

# Clone the latest 18.x LLVM version
RUN git clone --branch release/18.x --depth=1 https://github.com/llvm/llvm-project.git
RUN cmake -B build -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release llvm-project/llvm
RUN make -j$(nproc) -C build clang-format
RUN make -C build install-clang-format

FROM ubuntu

RUN apt update && \
    apt install -y git python3-minimal

COPY --from=build /usr/local /usr/local
