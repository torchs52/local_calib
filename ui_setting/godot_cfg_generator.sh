#!/bin/bash
set -euo pipefail

MODE="dev"

APPIMAGE_TEMPLATE="./godotSettings_appimage.cfg.template"
ENGINE_TEMPLATE="./godotSettings.cfg.template"

APPIMAGE_OUT="./godotSettings_appimage.cfg"
ENGINE_OUT="./godotSettings.cfg"

ARGUS3D_LIB_BASE="/var/lib/argus3d"
RUNTIME_BASE="/run/argus3d"
#RUNTIME_BASE="~/argus_pipe_filter/log"

usage() {
    cat <<EOF
使い方:
  ./godot_cfg_generator.sh [option]

Options:
  --dev          settings.ini を ~/argus_pipe_filter/config/settings.ini にする
  --production   settings.ini を /opt/argus3d/config/core/settings.ini にする
  -h, --help     ヘルプ表示
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dev)
            MODE="dev"
            ;;
        --production)
            MODE="production"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "不明なオプション: $1"
            usage
            exit 1
            ;;
    esac
    shift
done

APP_USER="$(id -un)"
APP_HOME="$(getent passwd "$APP_USER" | cut -d: -f6)"

if [[ -z "$APP_HOME" ]]; then
    echo "ERROR: APP_HOME を取得できません"
    exit 1
fi

ARGUS3D_CORE_DEV="${APP_HOME}/argus_pipe_filter"
ARGUS3D_CORE_PROD="/opt/argus3d/core"

JETSON_MONITOR_PATH_ENGINE="${APP_HOME}/argus_godot_ui/monitorSample/metrics.json"

if mountpoint -q /mnt/nvme; then
    LOG_BASE_APPIMAGE="/mnt/nvme/log/argus3d"
    JETSON_MONITOR_PATH_APPIMAGE="/mnt/nvme/log/argus3d/jetson_monitor/test/metrics_snapshot.json"
    echo "[INFO] NVMe detected"
else
    LOG_BASE_APPIMAGE="/var/log/argus3d"
    JETSON_MONITOR_PATH_APPIMAGE="${APP_HOME}/jetson_monitor/test/metrics_snapshot.json"
    echo "[INFO] NVMe not detected"
fi

LOG_BASE_ENGINE="${APP_HOME}/argus_godot_ui/log"

if [[ "$MODE" == "production" ]]; then
    # 量産時向けは確定ではない
    CONFIG_BASE_APPIMAGE="/opt/argus3d/config/core"
    ARGUS3D_CORE_APPIMAGE="${ARGUS3D_CORE_PROD}"
else
    CONFIG_BASE_APPIMAGE="${APP_HOME}/argus_pipe_filter/config"
    ARGUS3D_CORE_APPIMAGE="${ARGUS3D_CORE_DEV}"
fi

CONFIG_BASE_ENGINE="${APP_HOME}/argus_pipe_filter/config"
ARGUS3D_CORE_ENGINE="${ARGUS3D_CORE_DEV}"

check_exists() {
    local f="$1"
    if [[ ! -f "$f" ]]; then
        echo "ERROR: file not found: $f"
        exit 1
    fi
}

force_lf() {
    local f="$1"
    sed -i 's/\r$//' "$f"
}

check_no_crlf() {
    local f="$1"
    if file "$f" | grep -q "CRLF"; then
        echo "ERROR: CRLF line terminators detected: $f"
        exit 1
    fi
}

replace_template() {
    local template="$1"
    local out="$2"
    local log_base="$3"
    local config_base="$4"
    local runtime_base="$5"
    local argus3d_core="$6"
    local jetson_monitor_path="$7"
    local argus3d_lib_base="$8"

    sed \
        -e "s|__LOG_BASE__|${log_base}|g" \
        -e "s|__CONFIG_BASE__|${config_base}|g" \
        -e "s|__RUNTIME_BASE__|${runtime_base}|g" \
        -e "s|__ARGUS3D_CORE__|${argus3d_core}|g" \
        -e "s|__JETSON_MONITOR_PATH__|${jetson_monitor_path}|g" \
        -e "s|__ARGUS3D_LIB_BASE__|${argus3d_lib_base}|g" \
        "$template" > "$out"

    force_lf "$out"
    check_no_crlf "$out"

    echo "[INFO] generated: $out"
    file "$out"
}

echo "[INFO] MODE=${MODE}"
echo "[INFO] APP_USER=${APP_USER}"
echo "[INFO] APP_HOME=${APP_HOME}"
echo "[INFO] LOG_BASE_APPIMAGE=${LOG_BASE_APPIMAGE}"
echo "[INFO] LOG_BASE_ENGINE=${LOG_BASE_ENGINE}"
echo "[INFO] CONFIG_BASE_APPIMAGE=${CONFIG_BASE_APPIMAGE}"
echo "[INFO] CONFIG_BASE_ENGINE=${CONFIG_BASE_ENGINE}"
echo "[INFO] RUNTIME_BASE=${RUNTIME_BASE}"
echo "[INFO] ARGUS3D_CORE_APPIMAGE=${ARGUS3D_CORE_APPIMAGE}"
echo "[INFO] ARGUS3D_CORE_ENGINE=${ARGUS3D_CORE_ENGINE}"

check_exists "$APPIMAGE_TEMPLATE"
check_exists "$ENGINE_TEMPLATE"

force_lf "$APPIMAGE_TEMPLATE"
force_lf "$ENGINE_TEMPLATE"

check_no_crlf "$APPIMAGE_TEMPLATE"
check_no_crlf "$ENGINE_TEMPLATE"

echo "[INFO] generate config: ${APPIMAGE_OUT}"
replace_template \
    "$APPIMAGE_TEMPLATE" \
    "$APPIMAGE_OUT" \
    "$LOG_BASE_APPIMAGE" \
    "$CONFIG_BASE_APPIMAGE" \
    "$RUNTIME_BASE" \
    "$ARGUS3D_CORE_APPIMAGE" \
    "$JETSON_MONITOR_PATH_APPIMAGE" \
    "$ARGUS3D_LIB_BASE"

echo "[INFO] generate config: ${ENGINE_OUT}"
replace_template \
    "$ENGINE_TEMPLATE" \
    "$ENGINE_OUT" \
    "$LOG_BASE_ENGINE" \
    "$CONFIG_BASE_ENGINE" \
    "$RUNTIME_BASE" \
    "$ARGUS3D_CORE_ENGINE" \
    "$JETSON_MONITOR_PATH_ENGINE" \
    "$ARGUS3D_LIB_BASE"

echo "[INFO] done"