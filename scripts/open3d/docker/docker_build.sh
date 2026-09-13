#!/usr/bin/env bash

set -euo pipefail

export BUILDKIT_PROGRESS=plain

# Open3D repository settings
GIT_REPO="https://github.com/isl-org/Open3D.git"
GIT_TAG="${GIT_TAG:-main}"

# Directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PACKAGE_DIR="${SCRIPT_DIR}/.."
PROJECT_ROOT="${SCRIPT_DIR}/../../.."
EXPORT_DIR="${PACKAGE_DIR}/out"
WHEEL_OUTPUT_DIR="${PROJECT_ROOT}/requirements/wheels"
WHEEL_METADATA_OUTPUT_DIR="${PROJECT_ROOT}/requirements/wheels/open3d"
WHEEL_LICENSE_OUTPUT_DIR="${WHEEL_METADATA_OUTPUT_DIR}/3rdparty"
OPEN3D_LICENSE_OUTPUT_FILE="${WHEEL_METADATA_OUTPUT_DIR}/LICENSE"
INSTALL_OUTPUT_ROOT_DIR="${PROJECT_ROOT}/3rdparty/open3d_jetson"
INSTALL_OUTPUT_DIR="${INSTALL_OUTPUT_ROOT_DIR}/install"
LICENSE_OUTPUT_DIR="${INSTALL_OUTPUT_ROOT_DIR}/3rdparty"

PYTHON_VERSION="${PYTHON_VERSION:-3.12}"
BUILDX_BUILDER_NAME=o3d-builder
CUDA_ARCH="${CUDA_ARCH:-87}"

# - JetPack 6.2 / 6.1 (L4T r36.4.0, CUDA 12.6): ONNX Runtime v1.23.2
# - JetPack 6.0 (L4T r36.3.0, CUDA 12.2): ONNX Runtime v1.19.0
BASE_IMAGE="${BASE_IMAGE:-nvcr.io/nvidia/l4t-jetpack:r36.4.0}"

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

openblas_print_env() {
    echo "[openblas_print_env()] DOCKER_TAG: ${DOCKER_TAG}"
    echo "[openblas_print_env()] BASE_IMAGE: ${BASE_IMAGE}"
    echo "[openblas_print_env()] CMAKE_VERSION: ${CMAKE_VERSION}"
    echo "[openblas_print_env()] PYTHON_VERSION: ${PYTHON_VERSION}"
    echo "[openblas_print_env()] CUDA_ARCH: ${CUDA_ARCH}"
    echo "[openblas_print_env()] BUILD_CUDA_MODULE: ON"
    echo "[openblas_print_env()] CMAKE_CUDA_ARCHITECTURES: ${CUDA_ARCH}"
    echo "[openblas_print_env()] GIT_REPO: ${GIT_REPO}"
    echo "[openblas_print_env()] GIT_TAG: ${GIT_TAG}"
}

openblas_export_env() {
    export DOCKER_TAG="${DOCKER_TAG:-open3d:jetpack-arm64-cuda-release}"
    export BASE_IMAGE="${BASE_IMAGE}"
    export CMAKE_VERSION="${CMAKE_VERSION:-cmake-3.31.8-linux-aarch64}"
    export PYTHON_VERSION="${PYTHON_VERSION}"
    export CUDA_ARCH="${CUDA_ARCH}"
}

openblas_build() {
    local license_file
    local license_dest_dir
    local rel_path
    local wheels=()

    openblas_print_env

    # Recreate the host-side export directory before running buildx.
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
        --build-arg OPEN3D_GIT_REPO="${GIT_REPO}" \
        --build-arg OPEN3D_GIT_TAG="${GIT_TAG}" \
        -f docker/Dockerfile.openblas .
    popd >/dev/null

    fix_artifact_permissions "${EXPORT_DIR}"

    # Clean previous build artifacts and prepare output directories
    if [ -d "${INSTALL_OUTPUT_DIR}" ]; then
        echo "Cleaning previous C++ install: ${INSTALL_OUTPUT_DIR}/"
        rm -rf "${INSTALL_OUTPUT_DIR}"
    fi
    if [ -d "${LICENSE_OUTPUT_DIR}" ]; then
        echo "Cleaning previous 3rdparty licenses: ${LICENSE_OUTPUT_DIR}/"
        rm -rf "${LICENSE_OUTPUT_DIR}"
    fi
    mkdir -p "${INSTALL_OUTPUT_ROOT_DIR}"
    mkdir -p "${WHEEL_OUTPUT_DIR}"
    mkdir -p "${WHEEL_METADATA_OUTPUT_DIR}"
    rm -rf "${WHEEL_LICENSE_OUTPUT_DIR}"
    mkdir -p "${WHEEL_LICENSE_OUTPUT_DIR}"
    rm -f "${OPEN3D_LICENSE_OUTPUT_FILE}"

    mv "${EXPORT_DIR}/install" "${INSTALL_OUTPUT_ROOT_DIR}/"
    mv "${EXPORT_DIR}/3rdparty" "${INSTALL_OUTPUT_ROOT_DIR}/"
    mv "${EXPORT_DIR}/LICENSE" "${OPEN3D_LICENSE_OUTPUT_FILE}"

    while IFS= read -r -d '' license_file; do
        rel_path="${license_file#"${LICENSE_OUTPUT_DIR}/"}"
        license_dest_dir="${WHEEL_LICENSE_OUTPUT_DIR}/$(dirname "${rel_path}")"
        mkdir -p "${license_dest_dir}"
        cp "${license_file}" "${license_dest_dir}/"
    done < <(
        find "${LICENSE_OUTPUT_DIR}" -type f \
            \( -iname 'LICENSE*' -o -iname 'COPYING*' -o -iname 'NOTICE*' \) \
            -print0
    )

    shopt -s nullglob
    wheels=("${EXPORT_DIR}"/wheels/*.whl)
    shopt -u nullglob
    if [ ${#wheels[@]} -gt 0 ]; then
        mv "${wheels[@]}" "${WHEEL_OUTPUT_DIR}/"
    fi

    fix_artifact_permissions "${INSTALL_OUTPUT_DIR}"
    fix_artifact_permissions "${LICENSE_OUTPUT_DIR}"
    fix_artifact_permissions "${OPEN3D_LICENSE_OUTPUT_FILE}"
    fix_artifact_permissions "${WHEEL_OUTPUT_DIR}"
    fix_artifact_permissions "${WHEEL_LICENSE_OUTPUT_DIR}"

    echo ""
    echo "=== Build outputs ==="
    echo "Export directory: ${EXPORT_DIR}/"
    echo "3rdparty licenses: ${LICENSE_OUTPUT_DIR}/"
    find "${LICENSE_OUTPUT_DIR}" -maxdepth 2 -type f \
        \( -iname 'LICENSE*' -o -iname 'COPYING*' -o -iname 'NOTICE*' \) | sort | sed -n '1,10p'
    echo ""
    echo "C++ library: ${INSTALL_OUTPUT_DIR}/"
    ls -lh "${INSTALL_OUTPUT_DIR}/lib/"libOpen3D* 2>/dev/null || true
    echo ""
    echo "Python wheel: ${WHEEL_OUTPUT_DIR}/"
    ls -lh "${WHEEL_OUTPUT_DIR}"/*.whl 2>/dev/null || true
    echo "Open3D license: ${OPEN3D_LICENSE_OUTPUT_FILE}"
    ls -lh "${OPEN3D_LICENSE_OUTPUT_FILE}" 2>/dev/null || true
    echo "Wheel licenses: ${WHEEL_LICENSE_OUTPUT_DIR}/"
    find "${WHEEL_LICENSE_OUTPUT_DIR}" -type f | sort | sed -n '1,10p'
}

main() {
    echo "[$(basename "$0")] building Open3D (ARM64, wheel + C++ lib)"

    setup_rootless_docker_client
    openblas_export_env
    openblas_build
}

if [ "$0" = "${BASH_SOURCE[0]}" ]; then
    main "$@"
fi
