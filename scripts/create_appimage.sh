#!/usr/bin/env bash
set -euo pipefail

is_jetson() {
    [[ -f /etc/nv_tegra_release ]] && return 0
    uname -a | grep -qi "tegra"
}

log() {
    echo "[INFO] $*"
}

warn() {
    echo "[WARN] $*" >&2
}

err() {
    echo "[ERROR] $*" >&2
    exit 1
}

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || err "required command not found: $1"
}

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${PROJECT_ROOT}"

APP_NAME="argus_synchro"
APP_DIR="${APP_NAME}.AppDir"
SOURCE_ENTRY="./argus_synchro/__main__.py"

ARCHITECTURE="x86_64"
LIB_ARCH_DIR="x86_64-linux-gnu"
if is_jetson; then
    ARCHITECTURE="aarch64"
    LIB_ARCH_DIR="aarch64-linux-gnu"
fi

APPIMAGE_TOOL="appimagetool-${ARCHITECTURE}.AppImage"
TEMP_DEB_DIR="${PROJECT_ROOT}/temp_deb"
BUILD_DIR="${PROJECT_ROOT}/build"
DIST_DIR="${PROJECT_ROOT}/dist"
OUTPUT_DIR="${PROJECT_ROOT}/output"
OUTPUT_APPIMAGE="${APP_NAME}-${ARCHITECTURE}.AppImage"

# インストールするパッケージはここに追加する。
APT_PACKAGES=(
    build-essential
    cmake
    wget
    libidn2-dev
    clang-format
    pybind11-dev
    xorg-dev
    libxcb-shm0
    libglu1-mesa-dev
    libc++-11-dev
    libc++abi-11-dev
    libsdl2-dev
    libxi-dev
    libtbb-dev
    libosmesa6-dev
    libudev-dev
    autoconf
    libtool
    clang-11
    ninja-build
    gfortran
    feh
)

log "作業ディレクトリ: ${PROJECT_ROOT}"
log "アーキテクチャ: ${ARCHITECTURE}"
log "ライブラリディレクトリ: ${LIB_ARCH_DIR}"
log "追加 apt パッケージ: ${APT_PACKAGES[*]}"

need_cmd wget
need_cmd pyinstaller
need_cmd dpkg-deb
need_cmd apt-get
need_cmd apt-cache
need_cmd grep
need_cmd sort
need_cmd find

if ! command -v apt-rdepends >/dev/null 2>&1; then
    err "apt-rdepends が必要です。sudo apt-get update && sudo apt-get install -y apt-rdepends"
fi


# ==========================================
# 1. appimagetool
# ==========================================
if [[ ! -f "${APPIMAGE_TOOL}" ]]; then
    log "appimagetool が見つかりません。ダウンロードします..."
    wget "https://github.com/AppImage/appimagetool/releases/download/1.9.1/appimagetool-${ARCHITECTURE}.AppImage"
    chmod +x "${APPIMAGE_TOOL}"
else
    log "appimagetool は既に存在します。"
fi
chmod +x "${APPIMAGE_TOOL}"

# ==========================================
# 2. AppDir 初期化
# ==========================================
log "AppDir を初期化します..."
rm -rf "${APP_DIR}/usr"
mkdir -p "${APP_DIR}/usr/bin"
mkdir -p "${APP_DIR}/usr/lib/${LIB_ARCH_DIR}"
mkdir -p "${APP_DIR}/usr/share/applications"

# ==========================================
# 3. PyInstaller
# ==========================================
log "PyInstaller でアプリをビルドします..."
rm -rf "${BUILD_DIR}" "${DIST_DIR}"

pyinstaller "${SOURCE_ENTRY}" --noconfirm

if [[ ! -d "${DIST_DIR}/__main__" ]]; then
    err "PyInstaller の出力 ${DIST_DIR}/__main__ が見つかりません。"
fi

cp -r "${DIST_DIR}/__main__" "${APP_DIR}/usr/bin/${APP_NAME}"
log "PyInstaller 成果物を ${APP_DIR}/usr/bin/${APP_NAME} にコピーしました。"

# ==========================================
# 4. apt パッケージ取得
# ==========================================
log "aptパッケージと依存を取得します..."
rm -rf "${TEMP_DEB_DIR}"
mkdir -p "${TEMP_DEB_DIR}"
pushd "${TEMP_DEB_DIR}" >/dev/null

: > packages_all.txt

for root_pkg in "${APT_PACKAGES[@]}"; do
    log "依存解析: ${root_pkg}"
    apt-rdepends "${root_pkg}" 2>/dev/null \
        | grep -v "^ " \
        >> packages_all.txt || true
done

sort -u packages_all.txt > packages_unique.txt

grep -vE '^(libc6|libgcc-s1|libstdc\+\+6|gcc-.*-base|dpkg|debconf|install-info|bash|coreutils|systemd|apt|perl-base|login|passwd|adduser|tar|gzip|sed|grep|findutils|mount|init-system-helpers|sysvinit-utils|util-linux|base-files|base-passwd|dash|diffutils|hostname|ncurses-base|sensible-utils)$' \
    packages_unique.txt > packages_download.txt || true

if [[ ! -s packages_download.txt ]]; then
    warn "依存一覧が空です。指定パッケージ本体のみダウンロードします。"
    for pkg in "${APT_PACKAGES[@]}"; do
        echo "${pkg}" >> packages_download.txt
    done
fi

log "ダウンロード対象一覧:"
cat packages_download.txt

while read -r pkg; do
    [[ -z "${pkg}" ]] && continue
    log "download: ${pkg}"
    apt-get download "${pkg}" || warn "download失敗: ${pkg}"
done < packages_download.txt

# ==========================================
# 5. .deb 展開
# ==========================================
log ".deb を展開します..."
for deb in ./*.deb; do
    [[ -e "${deb}" ]] || continue
    log "extract: $(basename "${deb}")"
    dpkg-deb -x "${deb}" .
done
# ==========================================
# 6. AppDir にコピー
# ==========================================
log "展開したファイルを AppDir にコピーします..."

if [[ -d "./usr/bin" ]]; then
    mkdir -p "${PROJECT_ROOT}/${APP_DIR}/usr/bin"
    cp -a ./usr/bin/* "${PROJECT_ROOT}/${APP_DIR}/usr/bin/" 2>/dev/null || true
fi

if [[ -d "./usr/lib" ]]; then
    mkdir -p "${PROJECT_ROOT}/${APP_DIR}/usr/lib"
    cp -a ./usr/lib/* "${PROJECT_ROOT}/${APP_DIR}/usr/lib/" 2>/dev/null || true
fi

if [[ -d "./usr/share" ]]; then
    mkdir -p "${PROJECT_ROOT}/${APP_DIR}/usr/share"
    cp -a ./usr/share/* "${PROJECT_ROOT}/${APP_DIR}/usr/share/" 2>/dev/null || true
fi

if [[ -d "./lib" ]]; then
    mkdir -p "${PROJECT_ROOT}/${APP_DIR}/usr/lib"
    cp -a ./lib/* "${PROJECT_ROOT}/${APP_DIR}/usr/lib/" 2>/dev/null || true
fi

popd >/dev/null
rm -rf "${TEMP_DEB_DIR}"
log "依存パッケージの組み込み完了"


# ==========================================
# 9. AppRun
# ==========================================
log "AppRun を作成します..."
cat > "${APP_DIR}/AppRun" << EOF
#!/usr/bin/env bash
set -e

APPDIR="\$(cd "\$(dirname "\$0")" && pwd)"
export PATH="\${APPDIR}/usr/bin:\${PATH}"
export LD_LIBRARY_PATH="\${APPDIR}/usr/lib/${LIB_ARCH_DIR}:\${APPDIR}/usr/lib:\${LD_LIBRARY_PATH}"

if [[ "\${APPIMAGE_DEBUG:-0}" == "1" ]]; then
    echo "[DEBUG] APPDIR=\${APPDIR}"
    echo "[DEBUG] PATH=\${PATH}"
    echo "[DEBUG] LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}"
    for cmd in ${APT_PACKAGES[*]}; do
        if command -v "\$cmd" >/dev/null 2>&1; then
            echo "[DEBUG] which \$cmd = \$(which "\$cmd")"
            echo "[DEBUG] realpath \$cmd = \$(readlink -f "\$(which "\$cmd")")"
            ldd "\$(readlink -f "\$(which "\$cmd")")" || true
        else
            echo "[DEBUG] \$cmd not found in PATH"
        fi
    done
fi

exec "\${APPDIR}/usr/bin/${APP_NAME}/__main__" "\$@"
EOF
chmod +x "${APP_DIR}/AppRun"

# ==========================================
# 10. AppImage ビルド
# ==========================================

log "AppImage をビルドします..."
ARCH="${ARCHITECTURE}" "./${APPIMAGE_TOOL}" "${APP_DIR}" "${OUTPUT_APPIMAGE}"

chmod +x "${OUTPUT_APPIMAGE}"
log "ビルド完了: ${OUTPUT_APPIMAGE}"

gpg --detach-sign --armor "${OUTPUT_APPIMAGE}"
log "署名付与完了 ${OUTPUT_APPIMAGE}.asc"

# ==========================================
# 11. output ディレクトリへ配置
# ==========================================
log "output ディレクトリを準備します..."
mkdir -p "${OUTPUT_DIR}"

log "成果物を output ディレクトリへ移動します..."
mv -f "${OUTPUT_APPIMAGE}" "${OUTPUT_DIR}/"
mv -f "${OUTPUT_APPIMAGE}.asc" "${OUTPUT_DIR}/"

log "配置完了:"
log " - ${OUTPUT_DIR}/${OUTPUT_APPIMAGE}"
log " - ${OUTPUT_DIR}/${OUTPUT_APPIMAGE}.asc"