#!/usr/bin/env bash

set -euo pipefail

export BUILDKIT_PROGRESS=plain

# OpenCV repository settings
GIT_REPO="https://github.com/opencv/opencv.git"
GIT_TAG="${GIT_TAG:-4.13.0}"
OPENCV_CONTRIB_GIT_REPO="https://github.com/opencv/opencv_contrib.git"
OPENCV_CONTRIB_GIT_TAG="${OPENCV_CONTRIB_GIT_TAG:-${GIT_TAG}}"
OPENCV_PYTHON_GIT_REPO="https://github.com/opencv/opencv-python.git"
OPENCV_PYTHON_GIT_TAG="${OPENCV_PYTHON_GIT_TAG:-92}"

# Directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PACKAGE_DIR="${SCRIPT_DIR}/.."
PROJECT_ROOT="${SCRIPT_DIR}/../../.."
EXPORT_DIR="${PACKAGE_DIR}/out"
WHEEL_OUTPUT_DIR="${PROJECT_ROOT}/requirements/wheels"
WHEEL_METADATA_OUTPUT_DIR="${PROJECT_ROOT}/requirements/wheels/opencv"
WHEEL_LICENSE_OUTPUT_DIR="${WHEEL_METADATA_OUTPUT_DIR}/3rdparty"
WHEEL_WRAPPER_LICENSE_OUTPUT_DIR="${WHEEL_METADATA_OUTPUT_DIR}/opencv-python"
OPENCV_LICENSE_OUTPUT_FILE="${WHEEL_METADATA_OUTPUT_DIR}/LICENSE"
OPENCV_THIRD_PARTY_LICENSE_OUTPUT_FILE="${WHEEL_METADATA_OUTPUT_DIR}/LICENSE-3RD-PARTY.txt"
INSTALL_OUTPUT_ROOT_DIR="${PROJECT_ROOT}/3rdparty/opencv_jetson"
INSTALL_OUTPUT_DIR="${INSTALL_OUTPUT_ROOT_DIR}/install"
LICENSE_OUTPUT_DIR="${INSTALL_OUTPUT_ROOT_DIR}/3rdparty"

PYTHON_VERSION="${PYTHON_VERSION:-3.12}"
BUILDX_BUILDER_NAME=opencv-builder

# JetPack 6.x
BASE_IMAGE="${BASE_IMAGE:-nvcr.io/nvidia/l4t-jetpack:r36.4.0}"
CUDA_ARCH="${CUDA_ARCH:-8.7}"

fix_artifact_permissions() {
    local path="$1"

    if [ ! -e "${path}" ]; then
        return 0
    fi

    if [ -d "${path}" ]; then
        chmod -R u=rwX,go=rX "${path}"
    elif [ -f "${path}" ]; then
        chmod u=rw,go=r "${path}"
    fi
}

setup_rootless_docker_client() {
    export DOCKER_HOST="${DOCKER_HOST:-unix:///run/user/$(id -u)/docker.sock}"

    docker info >/dev/null
    if ! docker info --format '{{join .SecurityOptions "\n"}}' | grep -q rootless; then
        echo "Docker daemon is not rootless: ${DOCKER_HOST}" >&2
        exit 1
    fi

    if ! docker buildx inspect "${BUILDX_BUILDER_NAME}" >/dev/null 2>&1; then
        docker buildx create --name "${BUILDX_BUILDER_NAME}" --driver docker-container >/dev/null
    fi
    docker buildx inspect --bootstrap "${BUILDX_BUILDER_NAME}" >/dev/null
}

opencv_print_env() {
    echo "[opencv_print_env()] DOCKER_TAG: ${DOCKER_TAG}"
    echo "[opencv_print_env()] BASE_IMAGE: ${BASE_IMAGE}"
    echo "[opencv_print_env()] CMAKE_VERSION: ${CMAKE_VERSION}"
    echo "[opencv_print_env()] PYTHON_VERSION: ${PYTHON_VERSION}"
    echo "[opencv_print_env()] CUDA_ARCH: ${CUDA_ARCH}"
    echo "[opencv_print_env()] OPENCV_PARALLEL: ${OPENCV_PARALLEL}"
    echo "[opencv_print_env()] GIT_REPO: ${GIT_REPO}"
    echo "[opencv_print_env()] GIT_TAG: ${GIT_TAG}"
    echo "[opencv_print_env()] OPENCV_CONTRIB_GIT_REPO: ${OPENCV_CONTRIB_GIT_REPO}"
    echo "[opencv_print_env()] OPENCV_CONTRIB_GIT_TAG: ${OPENCV_CONTRIB_GIT_TAG}"
    echo "[opencv_print_env()] OPENCV_PYTHON_GIT_REPO: ${OPENCV_PYTHON_GIT_REPO}"
    echo "[opencv_print_env()] OPENCV_PYTHON_GIT_TAG: ${OPENCV_PYTHON_GIT_TAG}"
}

opencv_export_env() {
    export DOCKER_TAG="${DOCKER_TAG:-opencv:jetpack-arm64-cuda-release}"
    export BASE_IMAGE="${BASE_IMAGE}"
    export CMAKE_VERSION="${CMAKE_VERSION:-cmake-3.31.8-linux-aarch64}"
    export PYTHON_VERSION="${PYTHON_VERSION}"
    export CUDA_ARCH="${CUDA_ARCH}"
    export OPENCV_PARALLEL="${OPENCV_PARALLEL:-$(nproc)}"
}

opencv_build() {
    local wheels=()

    opencv_print_env

    rm -rf "${EXPORT_DIR}"
    mkdir -p "${EXPORT_DIR}"

    pushd "${PACKAGE_DIR}" >/dev/null
    docker buildx build \
        --builder "${BUILDX_BUILDER_NAME}" \
        --platform linux/arm64 \
        --output type=local,dest="${EXPORT_DIR}" \
        --build-arg BASE_IMAGE="${BASE_IMAGE}" \
        --build-arg CMAKE_VERSION="${CMAKE_VERSION}" \
        --build-arg PYTHON_VERSION="${PYTHON_VERSION}" \
        --build-arg CUDA_ARCH="${CUDA_ARCH}" \
        --build-arg OPENCV_PARALLEL="${OPENCV_PARALLEL}" \
        --build-arg OPENCV_GIT_REPO="${GIT_REPO}" \
        --build-arg OPENCV_GIT_TAG="${GIT_TAG}" \
        --build-arg OPENCV_CONTRIB_GIT_REPO="${OPENCV_CONTRIB_GIT_REPO}" \
        --build-arg OPENCV_CONTRIB_GIT_TAG="${OPENCV_CONTRIB_GIT_TAG}" \
        --build-arg OPENCV_PYTHON_GIT_REPO="${OPENCV_PYTHON_GIT_REPO}" \
        --build-arg OPENCV_PYTHON_GIT_TAG="${OPENCV_PYTHON_GIT_TAG}" \
        -f docker/Dockerfile .
    popd >/dev/null

    fix_artifact_permissions "${EXPORT_DIR}"

    if [ ! -d "${EXPORT_DIR}/install" ]; then
        echo "OpenCV C++ install was not exported to ${EXPORT_DIR}/install/" >&2
        exit 1
    fi
    if [ ! -d "${EXPORT_DIR}/licenses/cpp" ]; then
        echo "OpenCV C++ licenses were not exported to ${EXPORT_DIR}/licenses/cpp/" >&2
        exit 1
    fi
    if [ ! -f "${EXPORT_DIR}/licenses/cpp/LICENSE" ]; then
        echo "OpenCV LICENSE was not exported to ${EXPORT_DIR}/licenses/cpp/LICENSE" >&2
        exit 1
    fi
    if [ ! -d "${EXPORT_DIR}/licenses/python" ]; then
        echo "OpenCV Python licenses were not exported to ${EXPORT_DIR}/licenses/python/" >&2
        exit 1
    fi
    if [ ! -f "${EXPORT_DIR}/licenses/python/LICENSE.txt" ]; then
        echo "OpenCV Python LICENSE was not exported to ${EXPORT_DIR}/licenses/python/LICENSE.txt" >&2
        exit 1
    fi

    if [ -d "${INSTALL_OUTPUT_DIR}" ]; then
        echo "Cleaning previous C++ install: ${INSTALL_OUTPUT_DIR}/"
        rm -rf "${INSTALL_OUTPUT_DIR}"
    fi
    if [ -d "${LICENSE_OUTPUT_DIR}" ]; then
        echo "Cleaning previous C++ licenses: ${LICENSE_OUTPUT_DIR}/"
        rm -rf "${LICENSE_OUTPUT_DIR}"
    fi
    mkdir -p "${INSTALL_OUTPUT_ROOT_DIR}"
    mkdir -p "${WHEEL_OUTPUT_DIR}"
    mkdir -p "${WHEEL_METADATA_OUTPUT_DIR}"
    rm -rf "${WHEEL_LICENSE_OUTPUT_DIR}" "${WHEEL_WRAPPER_LICENSE_OUTPUT_DIR}"
    mkdir -p "${WHEEL_LICENSE_OUTPUT_DIR}"
    rm -f "${OPENCV_LICENSE_OUTPUT_FILE}" "${OPENCV_THIRD_PARTY_LICENSE_OUTPUT_FILE}"

    mv "${EXPORT_DIR}/install" "${INSTALL_OUTPUT_ROOT_DIR}/"
    mv "${EXPORT_DIR}/licenses/cpp" "${LICENSE_OUTPUT_DIR}"
    mv "${EXPORT_DIR}/licenses/python" "${WHEEL_WRAPPER_LICENSE_OUTPUT_DIR}"

    cp "${LICENSE_OUTPUT_DIR}/LICENSE" "${OPENCV_LICENSE_OUTPUT_FILE}"
    if [ -f "${LICENSE_OUTPUT_DIR}/LICENSE-3RD-PARTY.txt" ]; then
        cp "${LICENSE_OUTPUT_DIR}/LICENSE-3RD-PARTY.txt" "${OPENCV_THIRD_PARTY_LICENSE_OUTPUT_FILE}"
    elif [ -f "${WHEEL_WRAPPER_LICENSE_OUTPUT_DIR}/LICENSE-3RD-PARTY.txt" ]; then
        cp "${WHEEL_WRAPPER_LICENSE_OUTPUT_DIR}/LICENSE-3RD-PARTY.txt" "${OPENCV_THIRD_PARTY_LICENSE_OUTPUT_FILE}"
    fi
    if [ -d "${LICENSE_OUTPUT_DIR}/3rdparty" ]; then
        cp -a "${LICENSE_OUTPUT_DIR}/3rdparty/." "${WHEEL_LICENSE_OUTPUT_DIR}/"
    fi

    shopt -s nullglob
    wheels=(
        "${EXPORT_DIR}"/wheels/*.whl
    )
    shopt -u nullglob
    if [ ${#wheels[@]} -eq 0 ]; then
        echo "No OpenCV wheel was exported to ${EXPORT_DIR}/wheels/" >&2
        exit 1
    fi
    mv "${wheels[@]}" "${WHEEL_OUTPUT_DIR}/"

    fix_artifact_permissions "${INSTALL_OUTPUT_DIR}"
    fix_artifact_permissions "${LICENSE_OUTPUT_DIR}"
    fix_artifact_permissions "${WHEEL_OUTPUT_DIR}"
    fix_artifact_permissions "${WHEEL_METADATA_OUTPUT_DIR}"

    echo ""
    echo "=== Build outputs ==="
    echo "Export directory: ${EXPORT_DIR}/"
    echo "C++ library: ${INSTALL_OUTPUT_DIR}/"
    ls -lh "${INSTALL_OUTPUT_DIR}/lib/"libopencv_core* 2>/dev/null || true
    echo ""
    echo "Python wheel: ${WHEEL_OUTPUT_DIR}/"
    ls -lh "${WHEEL_OUTPUT_DIR}"/*.whl 2>/dev/null || true
    echo ""
    echo "C++ licenses: ${LICENSE_OUTPUT_DIR}/"
    find "${LICENSE_OUTPUT_DIR}" -type f \
        \( -iname 'LICENSE*' -o -iname 'COPYING*' -o -iname 'NOTICE*' \) | sort | sed -n '1,20p'
    echo ""
    echo "Wheel metadata: ${WHEEL_METADATA_OUTPUT_DIR}/"
    find "${WHEEL_METADATA_OUTPUT_DIR}" -type f \
        \( -iname 'LICENSE*' -o -iname 'COPYING*' -o -iname 'NOTICE*' \) | sort | sed -n '1,20p'
}

main() {
    echo "[$(basename "$0")] building OpenCV (ARM64, CUDA, headless wheel + C++ lib)"

    setup_rootless_docker_client
    opencv_export_env
    opencv_build
}

if [ "$0" = "${BASH_SOURCE[0]}" ]; then
    main "$@"
fi
