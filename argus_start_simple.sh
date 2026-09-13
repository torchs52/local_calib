#!/bin/bash
set -e

# プロジェクトディレクトリに移動
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

# 仮想環境（.venv）のPythonのパスを動的に定義（ベタ書き回避）
VENV_PYTHON="$SCRIPT_DIR/.venv/bin/python"

# settings.ini のデータパスの書き換えをここで行う場合
DATA_DIR=/mnt/nvme
#DATA_DIR=/home/matsuoka/data/X6T304
CAL_DATA_DIR=/mnt/nvme
#CAL_DATA_DIR=/home/matsuoka
sed -i "s|^data_dir *=.*|data_dir = ${DATA_DIR}|g" config/settings.ini
sed -i "s|^data_dir *=.*|data_dir = ${CAL_DATA_DIR}|g" config/calib_settings.ini

CONFIG_DIR="./config"
LOG_DIR="./log"
MMAP_DIR="/dev/shm"

RUN_USER="$(id -un)"
RUN_GROUP="$(id -gn)"

"$VENV_PYTHON" ./scripts/prepare_argus.py \
    --config-dir "$CONFIG_DIR" \
    --log-dir "$LOG_DIR" \
    --mmap-dir "$MMAP_DIR" \
    --user "$RUN_USER" \
    --group "$RUN_GROUP"

"$VENV_PYTHON" -m argus_synchro \
    --config-dir "$CONFIG_DIR" \
    --log-dir "$LOG_DIR" \
    --mmap-dir "$MMAP_DIR"
