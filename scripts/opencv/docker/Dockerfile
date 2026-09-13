# FROM must be called before other ARGS except for ARG BASE_IMAGE
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM ${BASE_IMAGE} AS builder

# For the rest of this Dockerfile
SHELL ["/bin/bash", "-c"]

ARG CMAKE_VERSION=cmake-3.31.8-linux-aarch64
ARG CUDA_ARCH=8.7
ARG OPENCV_PARALLEL
ARG OPENCV_GIT_REPO=https://github.com/opencv/opencv.git
ARG OPENCV_GIT_TAG=4.13.0
ARG OPENCV_CONTRIB_GIT_REPO=https://github.com/opencv/opencv_contrib.git
ARG OPENCV_CONTRIB_GIT_TAG=4.13.0
ARG OPENCV_PYTHON_GIT_REPO=https://github.com/opencv/opencv-python.git
ARG OPENCV_PYTHON_GIT_TAG=92

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Los_Angeles
ENV HOME=/root
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

RUN rm /var/lib/dpkg/info/libc-bin.* || true
RUN apt-get clean

# Build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    wget \
    git \
    build-essential \
    ninja-build \
    pkg-config \
    software-properties-common \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libwebp-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libtbb-dev \
 && rm -rf /var/lib/apt/lists/*

# CMake
# PWD is /, cmake will be installed to /root/${CMAKE_VERSION}/bin/cmake
RUN CMAKE_VER_NUMBERS=$(echo "${CMAKE_VERSION}" | cut -d"-" -f2) \
 && wget -q https://github.com/Kitware/CMake/releases/download/v${CMAKE_VER_NUMBERS}/${CMAKE_VERSION}.tar.gz \
 && tar -xf ${CMAKE_VERSION}.tar.gz \
 && cp -ar ${CMAKE_VERSION} ${HOME}
ENV PATH=${HOME}/${CMAKE_VERSION}/bin:${PATH}

ARG CUDA_STUB_LIB_DIR=/usr/local/cuda/targets/aarch64-linux/lib/stubs
ARG CUDA_CUDA_LIBRARY=/usr/local/cuda/targets/aarch64-linux/lib/stubs/libcuda.so
ENV CUDA_STUB_LIB_DIR=${CUDA_STUB_LIB_DIR}
ENV CUDA_CUDA_LIBRARY=${CUDA_CUDA_LIBRARY}

WORKDIR /work

# Build OpenCV C++ install artifacts
RUN git clone --depth 1 --branch "${OPENCV_GIT_TAG}" "${OPENCV_GIT_REPO}" opencv
RUN git clone --depth 1 --branch "${OPENCV_CONTRIB_GIT_TAG}" "${OPENCV_CONTRIB_GIT_REPO}" opencv_contrib
WORKDIR /work/opencv
RUN cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/root/opencv_install \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
    -DCMAKE_INSTALL_RPATH=\$ORIGIN:/usr/local/cuda/lib64 \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=OFF \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_DOCS=OFF \
    -DBUILD_opencv_apps=OFF \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_opencv_highgui=OFF \
    -DOPENCV_GENERATE_PKGCONFIG=ON \
    -DOPENCV_EXTRA_MODULES_PATH=/work/opencv_contrib/modules \
    -DWITH_CUDA=ON \
    -DWITH_CUDNN=ON \
    -DWITH_CUBLAS=ON \
    -DOPENCV_DNN_CUDA=ON \
    -DWITH_GTK=OFF \
    -DWITH_QT=OFF \
    -DWITH_OPENGL=OFF \
    -DCUDA_ARCH_BIN="${CUDA_ARCH}" \
    -DCUDA_FAST_MATH=ON \
    -DENABLE_FAST_MATH=ON
RUN cmake --build build --parallel "${OPENCV_PARALLEL:-$(nproc)}"
RUN cmake --install build

# Build wheel from opencv-python with CUDA-enabled OpenCV
WORKDIR /work
RUN git clone --recursive "${OPENCV_PYTHON_GIT_REPO}" opencv-python \
 && cd opencv-python \
 && if [ "${OPENCV_PYTHON_GIT_TAG}" != "main" ]; then git checkout "${OPENCV_PYTHON_GIT_TAG}"; fi \
 && git submodule update --init --recursive \
 && cd opencv \
 && git fetch --depth 1 origin "${OPENCV_GIT_TAG}" || true \
 && git checkout "${OPENCV_GIT_TAG}" \
 && cd /work/opencv-python \
 && if [ -d opencv_contrib ]; then \
      cd opencv_contrib \
      && git fetch --depth 1 origin "${OPENCV_CONTRIB_GIT_TAG}" \
      && git checkout "${OPENCV_CONTRIB_GIT_TAG}"; \
    fi

WORKDIR /work/opencv-python

# Python via deadsnakes PPA (pre-built binaries, no compilation needed)
ARG PYTHON_VERSION=3.12
RUN if [ -z "${PYTHON_VERSION}" ]; then echo "Error: ARG PYTHON_VERSION not specified."; exit 1; fi
ENV VIRTUAL_ENV=/opt/opencv-venv
ENV VIRTUAL_ENV_PYTHON=/opt/opencv-venv/bin/python
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

# Python build dependencies for opencv-python wheel
RUN "${VIRTUAL_ENV_PYTHON}" -m pip install -U \
    numpy \
    build \
    scikit-build \
    packaging

ENV ENABLE_CONTRIB=1
ENV ENABLE_JAVA=0
RUN ENABLE_HEADLESS=1 \
    CMAKE_ARGS="-DWITH_CUDA=ON -DWITH_CUDNN=ON -DWITH_CUBLAS=ON -DOPENCV_DNN_CUDA=ON -DCUDA_ARCH_BIN=${CUDA_ARCH} -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_highgui=OFF -DWITH_GTK=OFF -DWITH_QT=OFF -DWITH_OPENGL=OFF" \
    "${VIRTUAL_ENV_PYTHON}" -m pip wheel . --verbose --no-deps -w /root/wheels

RUN mkdir -p /artifacts/install /artifacts/wheels /artifacts/licenses/cpp /artifacts/licenses/python \
 && cp -a /root/opencv_install/. /artifacts/install/ \
 && cp -av /root/wheels/*.whl /artifacts/wheels/ \
 && cp -av /work/opencv/LICENSE /artifacts/licenses/cpp/LICENSE \
 && if [ -f /work/opencv/LICENSE-3RD-PARTY.txt ]; then cp -av /work/opencv/LICENSE-3RD-PARTY.txt /artifacts/licenses/cpp/; fi \
 && if [ -d /root/opencv_install/share/licenses/opencv4 ]; then \
      mkdir -p /artifacts/licenses/cpp/3rdparty \
      && cp -a /root/opencv_install/share/licenses/opencv4/. /artifacts/licenses/cpp/3rdparty/; \
    fi \
 && find /work/opencv-python -maxdepth 1 -type f \
      \( -iname 'LICENSE*' -o -iname 'COPYING*' -o -iname 'NOTICE*' \) \
      -exec cp -av {} /artifacts/licenses/python/ \; \
 && ls -lh /artifacts/wheels

FROM scratch AS export

COPY --from=builder /artifacts/install/ /install/
COPY --from=builder /artifacts/wheels/ /wheels/
COPY --from=builder /artifacts/licenses/ /licenses/
