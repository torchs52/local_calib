#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

PYTHON_VERSION=3.10 "${SCRIPT_DIR}/onnxruntime/docker/docker_build.sh" || true
PYTHON_VERSION=3.11 "${SCRIPT_DIR}/onnxruntime/docker/docker_build.sh" || true
PYTHON_VERSION=3.12 "${SCRIPT_DIR}/onnxruntime/docker/docker_build.sh" || true

PYTHON_VERSION=3.10 BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.3.0 "${SCRIPT_DIR}/onnxruntime/docker/docker_build.sh" || true
PYTHON_VERSION=3.11 BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.3.0 "${SCRIPT_DIR}/onnxruntime/docker/docker_build.sh" || true
PYTHON_VERSION=3.12 BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.3.0 "${SCRIPT_DIR}/onnxruntime/docker/docker_build.sh" || true

PYTHON_VERSION=3.10 "${SCRIPT_DIR}/open3d/docker/docker_build.sh" || true
PYTHON_VERSION=3.11 "${SCRIPT_DIR}/open3d/docker/docker_build.sh" || true
PYTHON_VERSION=3.12 "${SCRIPT_DIR}/open3d/docker/docker_build.sh" || true

PYTHON_VERSION=3.10 "${SCRIPT_DIR}/opencv/docker/docker_build.sh" || true
PYTHON_VERSION=3.11 "${SCRIPT_DIR}/opencv/docker/docker_build.sh" || true
PYTHON_VERSION=3.12 "${SCRIPT_DIR}/opencv/docker/docker_build.sh" || true
