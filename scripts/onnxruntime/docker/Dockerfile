# FROM must be called before other ARGS except for ARG BASE_IMAGE
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM ${BASE_IMAGE} AS builder

# For the rest of this Dockerfile
SHELL ["/bin/bash", "-c"]

ARG CMAKE_VERSION=cmake-3.31.8-linux-aarch64
ARG CUDA_ARCH=87
ARG ORT_PARALLEL
ARG ORT_NVCC_THREADS
ARG ONNXRUNTIME_GIT_REPO=https://github.com/microsoft/onnxruntime.git
ARG ONNXRUNTIME_GIT_TAG=v1.23.2
ARG EIGEN_GIT_REPO=https://gitlab.com/libeigen/eigen.git
ARG EIGEN_GIT_TAG=

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Los_Angeles
ENV HOME=/root

# Build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    wget \
    git \
    build-essential \
    ninja-build \
    pkg-config \
    patchelf \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# CMake
# PWD is /, cmake will be installed to /root/${CMAKE_VERSION}/bin/cmake
RUN CMAKE_VER_NUMBERS=$(echo "${CMAKE_VERSION}" | cut -d"-" -f2) \
 && wget -q https://github.com/Kitware/CMake/releases/download/v${CMAKE_VER_NUMBERS}/${CMAKE_VERSION}.tar.gz \
 && tar -xf ${CMAKE_VERSION}.tar.gz \
 && cp -ar ${CMAKE_VERSION} ${HOME}
ENV PATH=${HOME}/${CMAKE_VERSION}/bin:${PATH}

ENV CUDA_HOME=/usr/local/cuda
ENV CUDNN_HOME=/usr/lib/aarch64-linux-gnu
ENV TENSORRT_HOME=/usr/lib/aarch64-linux-gnu
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

WORKDIR /work

# Clone ONNX Runtime source
RUN git clone --recursive ${ONNXRUNTIME_GIT_REPO} src \
 && cd src \
 && git checkout "${ONNXRUNTIME_GIT_TAG}" \
 && git submodule update --init --recursive

# If EIGEN_GIT_TAG is specified, use pre-cloned Eigen via --use_preinstalled_eigen.
RUN if [ -n "${EIGEN_GIT_TAG}" ]; then \
      git clone ${EIGEN_GIT_REPO} eigen \
      && cd eigen \
      && git checkout "${EIGEN_GIT_TAG}"; \
    else \
      echo "Skip pre-cloned Eigen (EIGEN_GIT_TAG is empty)"; \
    fi

# Python via deadsnakes PPA (pre-built binaries, no compilation needed)
ARG PYTHON_VERSION=3.12
RUN if [ -z "${PYTHON_VERSION}" ]; then echo "Error: ARG PYTHON_VERSION not specified."; exit 1; fi
ENV VIRTUAL_ENV=/opt/onnxruntime-venv
ENV VIRTUAL_ENV_PYTHON=/opt/onnxruntime-venv/bin/python
RUN add-apt-repository -y ppa:deadsnakes/ppa \
 && apt-get update && apt-get install -y \
    python${PYTHON_VERSION} \
    python${PYTHON_VERSION}-dev \
    python${PYTHON_VERSION}-venv \
 && rm -rf /var/lib/apt/lists/* \
 && /usr/bin/python${PYTHON_VERSION} -m venv "${VIRTUAL_ENV}" \
 && "${VIRTUAL_ENV_PYTHON}" -m ensurepip --upgrade \
 && "${VIRTUAL_ENV_PYTHON}" -m pip install --upgrade pip setuptools wheel
RUN "${VIRTUAL_ENV_PYTHON}" --version
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}

# Python build dependencies
RUN "${VIRTUAL_ENV_PYTHON}" -m pip install -U \
    numpy \
    packaging \
    flatbuffers \
    psutil

# Build as non-root because ONNX Runtime disallows root builds.
RUN useradd -m -u 1000 builder \
 && cp -ar /root/${CMAKE_VERSION} /home/builder/ \
 && chown -R builder:builder /home/builder/${CMAKE_VERSION} \
 && chown -R builder:builder /work \
 && mkdir -p /artifacts \
 && chown -R builder:builder /artifacts
USER builder
ENV HOME=/home/builder
ENV PATH=${HOME}/${CMAKE_VERSION}/bin:${PATH}

WORKDIR /work/src

RUN build_args=( \
      --config Release \
      --update --build \
      --parallel "${ORT_PARALLEL:-$(nproc)}" \
      --nvcc_threads "${ORT_NVCC_THREADS:-$(nproc)}" \
      --build_wheel \
      --use_tensorrt \
      --cuda_home "${CUDA_HOME}" \
      --cudnn_home "${CUDNN_HOME}" \
      --tensorrt_home "${TENSORRT_HOME}" \
      --skip_tests \
      --cmake_extra_defines CMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} \
      --cmake_extra_defines onnxruntime_CUDA_MINIMAL=ON \
      --cmake_extra_defines onnxruntime_BUILD_UNIT_TESTS=OFF \
      --cmake_extra_defines onnxruntime_USE_FLASH_ATTENTION=OFF \
      --cmake_extra_defines onnxruntime_USE_MEMORY_EFFICIENT_ATTENTION=OFF \
    ) \
 && if [ -n "${EIGEN_GIT_TAG}" ]; then \
      build_args+=(--use_preinstalled_eigen --eigen_path /work/eigen); \
    fi \
 && ./build.sh "${build_args[@]}"

USER root

RUN cp -av /work/src/build/Linux/Release/dist/*.whl /artifacts/ \
 && cp -av /work/src/LICENSE /artifacts/LICENSE.txt \
 && cp -av /work/src/ThirdPartyNotices.txt /artifacts/ \
 && ls -lh /artifacts

FROM scratch AS export

COPY --from=builder /artifacts/*.whl /wheels/
COPY --from=builder /artifacts/LICENSE.txt /LICENSE.txt
COPY --from=builder /artifacts/ThirdPartyNotices.txt /ThirdPartyNotices.txt
