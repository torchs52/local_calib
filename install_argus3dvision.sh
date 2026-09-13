#!/bin/bash
set -euo pipefail

# ============================================
# Argus3D Vision UI/Core config installer
# ============================================

ARGUS_UISET_DIR_NAME="argus_pipe_filter/ui_setting"
ARGUS_CORE_CONFIG_SRC_NAME="argus_pipe_filter/config"

APPIMAGE_DST_DIR="/opt/argus3d/distribution/ui"
CONFIG_UI_DST_DIR="/opt/argus3d/config/ui"
CONFIG_CORE_DST_DIR="/opt/argus3d/config/core"

LIB_DST_DIR="/var/lib/argus3d"
RUNTIME_DST_DIR="${LIB_DST_DIR}/uptime"

APPIMAGE_DST_NAME="CraneViewer-aarch64.AppImage"
GODOT_CFG_DST_NAME="godotSettings_appimage.cfg"
CORE_SETTINGS_NAME="settings.ini"

RUN_USER=""
RUN_GROUP=""
INSTALL_MODE="all"

if mountpoint -q /mnt/nvme; then
    LOG_DST_DIR="/mnt/nvme/log/argus3d"
    echo "[INFO] NVMe detected"
else
    LOG_DST_DIR="/var/log/argus3d"
    echo "[INFO] NVMe not detected"
fi

usage() {
    cat <<EOF
使い方:
  sudo ./install_argus3dvision.sh [option]

Options:
  --all                 AppImage + UI config + Core config を更新
  --appimage-only       AppImageだけ更新
  --config-only         UI configだけ更新
  --core-config-only    Core configだけ更新
  --dirs-only           ディレクトリ作成と権限設定だけ
  --user USER           実行ユーザー指定
  --group GROUP         実行グループ指定
  -h, --help            ヘルプ表示

例:
  sudo ./install_argus3dvision.sh --all --user shiuser
  sudo ./install_argus3dvision.sh --config-only --user shiuser
  sudo ./install_argus3dvision.sh --core-config-only --user shiuser
EOF
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --all)
                INSTALL_MODE="all"
                ;;
            --appimage-only)
                INSTALL_MODE="appimage"
                ;;
            --config-only)
                INSTALL_MODE="config"
                ;;
            --core-config-only)
                INSTALL_MODE="core-config"
                ;;
            --dirs-only)
                INSTALL_MODE="dirs"
                ;;
            --user)
                RUN_USER="${2:-}"
                shift
                ;;
            --group)
                RUN_GROUP="${2:-}"
                shift
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
}

print_step() {
    echo
    echo "=================================================="
    echo "$1"
    echo "=================================================="
}

check_root() {
    if [[ "${EUID}" -ne 0 ]]; then
        echo "このスクリプトは sudo で実行してください。"
        exit 1
    fi
}

resolve_run_user_group() {
    if [[ -z "${RUN_USER}" ]]; then
        if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
            RUN_USER="${SUDO_USER}"
        else
            RUN_USER="nvidia"
            echo "WARNING: --user 未指定かつ SUDO_USER が取得できないため、RUN_USER=nvidia を使用します。"
        fi
    fi

    if ! id "${RUN_USER}" >/dev/null 2>&1; then
        echo "実行ユーザーが存在しません: ${RUN_USER}"
        exit 1
    fi

    if [[ -z "${RUN_GROUP}" ]]; then
        RUN_GROUP="$(id -gn "${RUN_USER}")"
    fi

    if ! getent group "${RUN_GROUP}" >/dev/null 2>&1; then
        echo "実行グループが存在しません: ${RUN_GROUP}"
        exit 1
    fi
}

check_lf_file() {
    local f="$1"

    if file "$f" | grep -q "CRLF"; then
        echo "ERROR: CRLF改行のファイルです: $f"
        echo "修正例: sed -i 's/\\r$//' '$f'"
        exit 1
    fi
}

check_source_files() {
    if [[ "${INSTALL_MODE}" == "all" || "${INSTALL_MODE}" == "appimage" ]]; then
        [[ -f "${APPIMAGE_SRC}" ]] || { echo "AppImage が見つかりません: ${APPIMAGE_SRC}"; exit 1; }
    fi

    if [[ "${INSTALL_MODE}" == "all" || "${INSTALL_MODE}" == "config" ]]; then
        [[ -f "${GODOT_CFG_SRC}" ]] || { echo "UI config が見つかりません: ${GODOT_CFG_SRC}"; exit 1; }
        check_lf_file "${GODOT_CFG_SRC}"
    fi

    if [[ "${INSTALL_MODE}" == "all" || "${INSTALL_MODE}" == "core-config" ]]; then
        [[ -d "${CORE_CONFIG_SRC_DIR}" ]] || { echo "Core config dir が見つかりません: ${CORE_CONFIG_SRC_DIR}"; exit 1; }
        [[ -f "${CORE_CONFIG_SRC_DIR}/${CORE_SETTINGS_NAME}" ]] || {
            echo "settings.ini が見つかりません: ${CORE_CONFIG_SRC_DIR}/${CORE_SETTINGS_NAME}"
            exit 1
        }
    fi
}

create_directories() {
    mkdir -p "${APPIMAGE_DST_DIR}"
    mkdir -p "${CONFIG_UI_DST_DIR}"
    mkdir -p "${CONFIG_CORE_DST_DIR}"

    mkdir -p "${LOG_DST_DIR}/frameLog"
    mkdir -p "${LOG_DST_DIR}/calibrationHistory/checkHistory"
    mkdir -p "${LOG_DST_DIR}/calibrationHistory/history"
    mkdir -p "${LOG_DST_DIR}/ui"
    mkdir -p "${LOG_DST_DIR}/tmpCalib/2d-3d"
    mkdir -p "${LOG_DST_DIR}/tmpCalib/3d-3d"

    mkdir -p "${RUNTIME_DST_DIR}"
}

install_appimage() {
    cp -f "${APPIMAGE_SRC}" "${APPIMAGE_DST_DIR}/${APPIMAGE_DST_NAME}"
}

install_ui_config() {
    cp -f "${GODOT_CFG_SRC}" "${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
    check_lf_file "${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
    file "${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
}

install_core_config() {
    rm -rf "${CONFIG_CORE_DST_DIR:?}/"*
    cp -a "${CORE_CONFIG_SRC_DIR}/." "${CONFIG_CORE_DST_DIR}/"

    find "${CONFIG_CORE_DST_DIR}" -type f | while read -r f; do
        check_lf_file "$f"
    done
}

set_permissions() {
    # /opt/argus3d 全体の基本権限
    mkdir -p /opt/argus3d
    chown root:root /opt/argus3d
    chmod 755 /opt/argus3d

    mkdir -p /opt/argus3d/distribution
    chown root:root /opt/argus3d/distribution
    chmod 755 /opt/argus3d/distribution

    chown root:root "${APPIMAGE_DST_DIR}"
    chmod 755 "${APPIMAGE_DST_DIR}"

    if [[ -f "${APPIMAGE_DST_DIR}/${APPIMAGE_DST_NAME}" ]]; then
        chown root:root "${APPIMAGE_DST_DIR}/${APPIMAGE_DST_NAME}"
        chmod 755 "${APPIMAGE_DST_DIR}/${APPIMAGE_DST_NAME}"
    fi

    # UI config
    mkdir -p /opt/argus3d/config
    chown root:root /opt/argus3d/config
    chmod 755 /opt/argus3d/config

    chown root:root "${CONFIG_UI_DST_DIR}"
    chmod 755 "${CONFIG_UI_DST_DIR}"

    # Godotが実行中に更新するUI cfgは書き込み可
    if [[ -f "${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}" ]]; then
        chown "${RUN_USER}:${RUN_GROUP}" "${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
        chmod 664 "${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
    fi

    # Core config
    chown -R root:root "${CONFIG_CORE_DST_DIR}"
    find "${CONFIG_CORE_DST_DIR}" -type d -exec chmod 755 {} \;
    find "${CONFIG_CORE_DST_DIR}" -type f -exec chmod 644 {} \;

    # settings.iniだけGodot/実行ユーザーが書き込み可能
    if [[ -f "${CONFIG_CORE_DST_DIR}/${CORE_SETTINGS_NAME}" ]]; then
        chown "${RUN_USER}:${RUN_GROUP}" "${CONFIG_CORE_DST_DIR}/${CORE_SETTINGS_NAME}"
        chmod 664 "${CONFIG_CORE_DST_DIR}/${CORE_SETTINGS_NAME}"
    fi

    # log
    chown -R "${RUN_USER}:${RUN_GROUP}" "${LOG_DST_DIR}"
    find "${LOG_DST_DIR}" -type d -exec chmod 755 {} \;
    find "${LOG_DST_DIR}" -type f -exec chmod 644 {} \; 2>/dev/null || true

    # runtime
    mkdir -p "${RUNTIME_DST_DIR}"
    chown -R "${RUN_USER}:${RUN_GROUP}" /var/lib/argus3d
    chmod 755 /var/lib/argus3d
    chmod 755 "${RUNTIME_DST_DIR}"
}

show_result() {
    echo
    echo "インストール完了: mode=${INSTALL_MODE}"
    echo
    echo "AppImage        : ${APPIMAGE_DST_DIR}/${APPIMAGE_DST_NAME}"
    echo "UI Config       : ${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
    echo "Core Config Dir : ${CONFIG_CORE_DST_DIR}"
    echo "Core settings   : ${CONFIG_CORE_DST_DIR}/${CORE_SETTINGS_NAME}"
    echo "Log dir         : ${LOG_DST_DIR}"
    echo "Runtime         : ${RUNTIME_DST_DIR}"
    echo "Run user        : ${RUN_USER}:${RUN_GROUP}"
    echo
    echo "権限確認:"
    ls -l "${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}" 2>/dev/null || true
    ls -l "${CONFIG_CORE_DST_DIR}/${CORE_SETTINGS_NAME}" 2>/dev/null || true
    echo
    echo "起動確認:"
    echo "  ${APPIMAGE_DST_DIR}/${APPIMAGE_DST_NAME} --headless --ui-version --settings ${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
    echo "  ${APPIMAGE_DST_DIR}/${APPIMAGE_DST_NAME} --settings ${CONFIG_UI_DST_DIR}/${GODOT_CFG_DST_NAME}"
}

main() {
    parse_args "$@"

    print_step "Argus3D Vision installer start"
    echo "mode=${INSTALL_MODE}"

    check_root
    resolve_run_user_group

    HOME_DIR="/home/${RUN_USER}"
    ARGUS_UISET_DIR="${HOME_DIR}/${ARGUS_UISET_DIR_NAME}"

    APPIMAGE_SRC="${ARGUS_UISET_DIR}/${APPIMAGE_DST_NAME}"
    GODOT_CFG_SRC="${ARGUS_UISET_DIR}/${GODOT_CFG_DST_NAME}"
    CORE_CONFIG_SRC_DIR="${HOME_DIR}/${ARGUS_CORE_CONFIG_SRC_NAME}"

    echo "run user=${RUN_USER}:${RUN_GROUP}"
    echo "home dir=${HOME_DIR}"
    echo "ui setting dir=${ARGUS_UISET_DIR}"
    echo "core config src=${CORE_CONFIG_SRC_DIR}"

    check_source_files

    print_step "create directories"
    create_directories

    case "${INSTALL_MODE}" in
        all)
            print_step "install AppImage"
            install_appimage

            print_step "install UI config"
            install_ui_config

            #print_step "install Core config"
            #install_core_config
            ;;
        appimage)
            print_step "install AppImage only"
            install_appimage
            ;;
        config)
            print_step "install UI config only"
            install_ui_config
            ;;
        core-config)
            print_step "install Core config only"
            install_core_config
            ;;
        dirs)
            print_step "directories only"
            ;;
        *)
            echo "不明な INSTALL_MODE: ${INSTALL_MODE}"
            exit 1
            ;;
    esac

    print_step "set permissions"
    set_permissions

    print_step "done"
    show_result
}

main "$@"