#!/usr/bin/env bash

set -euo pipefail

export BUILDKIT_PROGRESS=plain

# ONNX Runtime repository settings
GIT_REPO="https://github.com/microsoft/onnxruntime.git"
GIT_TAG="${GIT_TAG:-}"
EIGEN_GIT_REPO="https://gitlab.com/libeigen/eigen.git"
EIGEN_GIT_TAG="${EIGEN_GIT_TAG:-}"

# Directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PACKAGE_DIR="${SCRIPT_DIR}/.."
PROJECT_ROOT="${SCRIPT_DIR}/../../.."
EXPORT_DIR="${PACKAGE_DIR}/out"
WHEEL_OUTPUT_DIR="${PROJECT_ROOT}/requirements/wheels"
LICENSE_OUTPUT_DIR="${PROJECT_ROOT}/requirements/wheels/onnxruntime"
LICENSE_OUTPUT_FILE="${LICENSE_OUTPUT_DIR}/LICENSE.txt"
THIRD_PARTY_NOTICES_OUTPUT_FILE="${LICENSE_OUTPUT_DIR}/ThirdPartyNotices.txt"

# - JetPack 6.2 / 6.1 (L4T r36.4.0, TensorRT 10.3): ONNX Runtime v1.23.2
# - JetPack 6.0 (L4T r36.3.0, TensorRT 8.6): ONNX Runtime v1.19.0
#
BASE_IMAGE="${BASE_IMAGE:-nvcr.io/nvidia/l4t-jetpack:r36.4.0}"
CUDA_ARCH="${CUDA_ARCH:-87}"
PYTHON_VERSION="${PYTHON_VERSION:-3.12}"
BUILDX_BUILDER_NAME=ort-builder

base_image_tag() {
    echo "${BASE_IMAGE##*:}"
}

default_git_tag_for_base_image() {
    case "$(base_image_tag)" in
        r36.3.*)
            echo "v1.19.0"
            ;;
        r36.4.*)
            echo "v1.23.2"
            ;;
        *)
            echo "Unsupported BASE_IMAGE for ONNX Runtime build: ${BASE_IMAGE}" >&2
            echo "Supported L4T tags: r36.3.0, r36.4.0" >&2
            exit 1
            ;;
    esac
}

default_eigen_git_tag_for_ort_tag() {
    case "${GIT_TAG:-$(default_git_tag_for_base_image)}" in
        v1.19.0)
            echo "e7248b26a1ed53fa030c5c459f7ea095dfd276ac"
            ;;
        v1.23.2)
            echo ""
            ;;
        *)
            echo "Unsupported ONNX Runtime tag for Eigen checkout: ${GIT_TAG:-}" >&2
            echo "Supported ONNX Runtime tags: v1.19.0, v1.23.2" >&2
            exit 1
            ;;
    esac
}

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

ort_print_env() {
    echo "[ort_print_env()] DOCKER_TAG: ${DOCKER_TAG}"
    echo "[ort_print_env()] BASE_IMAGE: ${BASE_IMAGE}"
    echo "[ort_print_env()] CMAKE_VERSION: ${CMAKE_VERSION}"
    echo "[ort_print_env()] PYTHON_VERSION: ${PYTHON_VERSION}"
    echo "[ort_print_env()] CUDA_ARCH: ${CUDA_ARCH}"
    echo "[ort_print_env()] ORT_PARALLEL: ${ORT_PARALLEL}"
    echo "[ort_print_env()] ORT_NVCC_THREADS: ${ORT_NVCC_THREADS}"
    echo "[ort_print_env()] GIT_REPO: ${GIT_REPO}"
    echo "[ort_print_env()] GIT_TAG: ${GIT_TAG}"
    echo "[ort_print_env()] EIGEN_GIT_REPO: ${EIGEN_GIT_REPO}"
    echo "[ort_print_env()] EIGEN_GIT_TAG: ${EIGEN_GIT_TAG}"
}

ort_export_env() {
    export DOCKER_TAG=onnxruntime:jetpack-arm64-release
    export BASE_IMAGE="${BASE_IMAGE}"
    export CMAKE_VERSION=cmake-3.31.8-linux-aarch64
    export PYTHON_VERSION="${PYTHON_VERSION}"
    export CUDA_ARCH="${CUDA_ARCH}"
    export GIT_TAG="${GIT_TAG:-$(default_git_tag_for_base_image)}"
    export EIGEN_GIT_REPO="${EIGEN_GIT_REPO}"
    export EIGEN_GIT_TAG="${EIGEN_GIT_TAG:-$(default_eigen_git_tag_for_ort_tag)}"

    export ORT_PARALLEL="${ORT_PARALLEL:-$(nproc)}"
    export ORT_NVCC_THREADS="${ORT_NVCC_THREADS:-$(nproc)}"
}

ort_build() {
    local wheels=()

    ort_print_env

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
        --build-arg ORT_PARALLEL="${ORT_PARALLEL}" \
        --build-arg ORT_NVCC_THREADS="${ORT_NVCC_THREADS}" \
        --build-arg ONNXRUNTIME_GIT_REPO="${GIT_REPO}" \
        --build-arg ONNXRUNTIME_GIT_TAG="${GIT_TAG}" \
        --build-arg EIGEN_GIT_REPO="${EIGEN_GIT_REPO}" \
        --build-arg EIGEN_GIT_TAG="${EIGEN_GIT_TAG}" \
        -f docker/Dockerfile .
    popd >/dev/null

    fix_artifact_permissions "${EXPORT_DIR}"

    mkdir -p "${LICENSE_OUTPUT_DIR}"
    mkdir -p "${WHEEL_OUTPUT_DIR}"
    rm -f "${LICENSE_OUTPUT_FILE}" "${THIRD_PARTY_NOTICES_OUTPUT_FILE}"

    mv "${EXPORT_DIR}/LICENSE.txt" "${LICENSE_OUTPUT_FILE}"
    mv "${EXPORT_DIR}/ThirdPartyNotices.txt" "${THIRD_PARTY_NOTICES_OUTPUT_FILE}"
    shopt -s nullglob
    wheels=("${EXPORT_DIR}"/wheels/*.whl)
    shopt -u nullglob
    if [ ${#wheels[@]} -eq 0 ]; then
        echo "No ONNX Runtime wheel was exported to ${EXPORT_DIR}/wheels/" >&2
        exit 1
    fi
    mv "${wheels[@]}" "${WHEEL_OUTPUT_DIR}/"

    fix_artifact_permissions "${LICENSE_OUTPUT_FILE}"
    fix_artifact_permissions "${THIRD_PARTY_NOTICES_OUTPUT_FILE}"
    fix_artifact_permissions "${WHEEL_OUTPUT_DIR}"

    echo ""
    echo "=== Build outputs ==="
    echo "Export directory: ${EXPORT_DIR}/"
    echo "Wheel output: ${WHEEL_OUTPUT_DIR}/"
    ls -lh "${WHEEL_OUTPUT_DIR}" 2>/dev/null || true
}

main() {
    echo "[$(basename "$0")] building ONNX Runtime (ARM64, wheel)"

    setup_rootless_docker_client
    ort_export_env
    ort_build
}

if [ "$0" = "${BASH_SOURCE[0]}" ]; then
    main "$@"
fi
