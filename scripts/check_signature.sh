#!/usr/bin/env bash
set -euo pipefail

is_jetson() {
    [[ -f /etc/nv_tegra_release ]] && return 0
    uname -a | grep -qi "tegra"
}


ARCHITECTURE="x86_64"
if is_jetson; then
    ARCHITECTURE="aarch64"
fi
KEYRING="$HOME/.verify-gnupg/pubring.kbx"
APPIMAGE="argus_synchro-${ARCHITECTURE}.AppImage"
SIGNATURE="${APPIMAGE}.asc"

# 前提条件の確認
if [[ ! -f "$KEYRING" ]]; then
    echo "キーリングが見つかりません: $KEYRING" >&2
    exit 1
fi

if [[ ! -f "$APPIMAGE" ]]; then
    echo "AppImage が見つかりません: $APPIMAGE" >&2
    exit 1
fi

if [[ ! -f "$SIGNATURE" ]]; then
    echo "署名ファイルが見つかりません: $SIGNATURE" >&2
    exit 1
fi

RESULT=$(gpgv --keyring "$HOME/.verify-gnupg/pubring.kbx" --status-fd 1 argus_synchro-${ARCHITECTURE}.AppImage.asc argus_synchro-${ARCHITECTURE}.AppImage 2>/dev/null)
echo "$RESULT"

if echo "$RESULT" | grep -q "^\[GNUPG:\] VALIDSIG"; then
    echo "検証成功: 信頼された有効な署名です。"
else
    echo "検証失敗: 署名が不正か、公開鍵が見つかりません。"
    exit 1
fi

exit 0