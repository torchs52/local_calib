#!/usr/bin/env bash

set -eux

cuda_nvcc() {
    if command -v nvcc >/dev/null 2>&1; then
        command -v nvcc
        return
    fi

    if [ -x /usr/local/cuda/bin/nvcc ]; then
        echo /usr/local/cuda/bin/nvcc
        return
    fi

    find /usr/local -maxdepth 3 -path '/usr/local/cuda-*/bin/nvcc' -type f -executable 2>/dev/null | sort -V | tail -n 1
}

python_toolchain_available() {
    local python_bin="python${PYTHON_VERSION}"

    command -v "${python_bin}" >/dev/null 2>&1 || return 1
    command -v pip3 >/dev/null 2>&1 || return 1
    "${python_bin}" -m venv --help >/dev/null 2>&1 || return 1
    "${python_bin}" - <<'PY' >/dev/null 2>&1
import pathlib
import sysconfig

include_dir = pathlib.Path(sysconfig.get_paths()["include"])
raise SystemExit(0 if (include_dir / "Python.h").exists() else 1)
PY
}

setup_python() {
    if python_toolchain_available; then
        echo "Python ${PYTHON_VERSION} toolchain already available: $(command -v "python${PYTHON_VERSION}")"
        return
    fi

    sudo apt install -y software-properties-common
    sudo add-apt-repository -y ppa:deadsnakes/ppa
    sudo apt update
    sudo apt install -y \
        python3 python3-venv python3-pip python3-dev \
        python${PYTHON_VERSION} python${PYTHON_VERSION}-venv python${PYTHON_VERSION}-dev
}

is_wsl() {
    grep -qi microsoft /proc/sys/kernel/osrelease /proc/version 2>/dev/null
}

cuda_repo_distro() {
    if is_wsl; then
        echo wsl-ubuntu
        return
    fi

    . /etc/os-release
    echo "ubuntu${VERSION_ID/./}"
}

set_cuda_env() {
    export CUDA_HOME=/usr/local/cuda
    export PATH=/usr/local/cuda/bin:${PATH}
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
}

setup_cuda_toolkit() {
    local nvcc_path
    local repo_distro
    local keyring_deb
    local cuda_toolkit_package

    if [ "$(uname -m)" != "x86_64" ]; then
        return
    fi

    nvcc_path="$(cuda_nvcc)"
    if [ -n "${nvcc_path}" ]; then
        echo "CUDA toolkit already available: ${nvcc_path}"
        return
    fi

    repo_distro="$(cuda_repo_distro)"
    keyring_deb="/tmp/cuda-keyring_${repo_distro}_1.1-1_all.deb"
    cuda_toolkit_package="${CUDA_TOOLKIT_PACKAGE:-cuda-toolkit-12-6}"

    sudo apt install -y wget ca-certificates
    wget -q -O "${keyring_deb}" "https://developer.download.nvidia.com/compute/cuda/repos/${repo_distro}/x86_64/cuda-keyring_1.1-1_all.deb"
    sudo dpkg -i "${keyring_deb}"
    sudo apt update
    sudo apt install -y "${cuda_toolkit_package}"

    if [ -d /usr/local/cuda/bin ]; then
        set_cuda_env
        printf '%s\n' \
            'export CUDA_HOME=/usr/local/cuda' \
            'export PATH=/usr/local/cuda/bin:${PATH}' \
            'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}' |
            sudo tee /etc/profile.d/cuda-toolkit.sh >/dev/null
    fi
}

docker_rootless_available() {
    local docker_host

    if ! command -v docker >/dev/null 2>&1; then
        return 1
    fi

    docker_host="${DOCKER_HOST:-unix:///run/user/$(id -u)/docker.sock}"
    DOCKER_HOST="${docker_host}" docker info >/dev/null 2>&1 || return 1
    DOCKER_HOST="${docker_host}" docker info --format '{{join .SecurityOptions "\n"}}' | grep -q rootless
}

install_docker_packages() {
    local ubuntu_codename

    if command -v docker >/dev/null 2>&1 &&
        command -v dockerd-rootless-setuptool.sh >/dev/null 2>&1; then
        sudo apt install -y uidmap dbus-user-session slirp4netns fuse-overlayfs
        return
    fi

    . /etc/os-release
    ubuntu_codename="${UBUNTU_CODENAME:-$VERSION_CODENAME}"

    sudo apt install -y ca-certificates curl uidmap dbus-user-session slirp4netns fuse-overlayfs
    sudo install -m 0755 -d /etc/apt/keyrings
    if [ ! -f /etc/apt/keyrings/docker.asc ]; then
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc
    fi
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${ubuntu_codename} stable" |
        sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
    sudo apt update
    sudo apt install -y \
        docker-ce docker-ce-cli containerd.io \
        docker-buildx-plugin docker-compose-plugin docker-ce-rootless-extras
}

setup_rootless_docker() {
    if docker_rootless_available; then
        echo "Rootless Docker already available"
        return
    fi

    install_docker_packages

    sudo systemctl disable --now docker.service docker.socket || true
    if [ ! -f "${HOME}/.config/systemd/user/docker.service" ]; then
        dockerd-rootless-setuptool.sh install
    fi
    systemctl --user enable --now docker.service
    sudo loginctl enable-linger "$(id -un)"

    export DOCKER_HOST="${DOCKER_HOST:-unix:///run/user/$(id -u)/docker.sock}"
}

sudo apt update

# git lfs
sudo apt install -y \
    git git-lfs
git lfs install && git lfs pull || true

# Python
PYTHON_VERSION="3.12"
setup_python

## cpp
sudo apt install -y \
    build-essential cmake wget libidn2-dev clang-format

sudo apt install -y pybind11-dev

# Open3D deps
sudo apt install -y \
    xorg-dev libxcb-shm0 libglu1-mesa-dev libxkbcommon-dev \
    libc++-11-dev libc++abi-11-dev libsdl2-dev libxi-dev \
    libtbb-dev \
    libosmesa6-dev \
    libudev-dev autoconf libtool \
    clang-11 ninja-build \
    gfortran

# OpenCV deps (Jetson含め3rdpartyビルド前提)
sudo apt install -y \
    pkg-config \
    libjpeg-dev libpng-dev libtiff-dev libwebp-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

# CUDA toolkit (WSL x86_64 / Ubuntu x86_64)
setup_cuda_toolkit

# Rootless Docker
setup_rootless_docker

# QEMU
sudo apt install -y qemu binfmt-support qemu-user-static

# Argus Synchro
sudo apt install -y feh

# for AppImage
sudo apt-get install -y apt-rdepends
