#!/usr/bin/env bash
set -e

is_jetson() {
    [[ -f /etc/nv_tegra_release ]] && return 0
    uname -a | grep -qi "tegra"
}


ARCHITECTURE="x86_64"
if is_jetson; then
    ARCHITECTURE="aarch64"
fi

# importした公開鍵を配置するディレクトリ
KEY_DIR="$HOME/.verify-gnupg"
# importする公開鍵のパス
IMPORT_KEY_PATH="./release-signing-key.asc"

if [ ! -f "${IMPORT_KEY_PATH}" ]; then
    echo "エラー: 公開鍵のimportに使用する${IMPORT_KEY_PATH}が見つかりません。"
    exit 1
fi

if [ -d "${KEY_DIR}" ]; then
    echo "公開鍵を保存予定のディレクトリ${KEY_DIR}が既にあるため削除し、再生成します。"
    sudo chattr -R -i "${KEY_DIR}"
    chmod 700 "${KEY_DIR}"
    # TODO エラーが出た場合はsudo rmに変更 (NSW)
    rm "${KEY_DIR}" -rdf
fi

mkdir -p "${KEY_DIR}"
chmod 700 "${KEY_DIR}"
gpg --homedir "${KEY_DIR}" --import ${IMPORT_KEY_PATH}
gpg --homedir "${KEY_DIR}" --fingerprint
chmod -R a-w "${KEY_DIR}"
# 読み書きの権限書換えをroot権限必須とする
sudo chattr -R +i "${KEY_DIR}"

echo "暗号鍵のimportが完了しました。"

exit 0