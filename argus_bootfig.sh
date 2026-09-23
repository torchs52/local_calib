#!/bin/bash
set -e
source ~/.profile

# ARGUS3D開発者モード
# Godot UIにて画面ログの記録時間の上限がなくなる
# （製品仕様は30分で強制終了）
export ARGUS3D_DEV=1
echo "$ARGUS3D_DEV"

# ----------------------------------------------------------------------
# プロジェクトディレクトリに移動
# ----------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

# ----------------------------------------------------------------------
# 二重起動対策
#
# 前回の Argus がまだ動作している場合は、既存の argus_synchro(Main) へ SIGTERM を送り、
# 前回の argus_bootfig が cleanup を完了してflock を解放するまで待つ。
#
# flock を使用するため、VS Code が argus_bootfig.sh を開いているだけでは二重起動とは判定されない。
# argus_bootfig.sh / argus_bootfig_jetson.sh で共通ロックを使用できるよう、ロックファイル名は argus3d.lock とする。
# ----------------------------------------------------------------------
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/argus3d.lock"

exec 9>"$LOCK_FILE"

if ! flock -n 9; then
    echo "<<run_all>> 既存の Argus 起動を検出しました"
    echo "<<run_all>> 既存プロセスを終了してから再起動します"

    # --------------------------------------------------------------
    # 既存 Main(argus_synchro) を探す
    #
    # MonitorArgus:
    #   -m argus_synchro.SystemMonitor.MonitorArgus
    #
    # Main:
    #   -m argus_synchro
    #
    # Main のみを対象にする。
    # --------------------------------------------------------------
    OLD_MAIN_PIDS="$(
        pgrep -f '[p]ython.*-m argus_synchro([[:space:]]|$)' || true
    )"

    if [ -n "$OLD_MAIN_PIDS" ]; then
        echo "<<run_all>> 既存 Main に SIGTERM を送信: PID=${OLD_MAIN_PIDS}"

        # Main の shutdown handler に正常終了を要求する。
        kill -TERM $OLD_MAIN_PIDS 2>/dev/null || true
    else
        echo "<<run_all>> WARN: 既存 Main が見つかりません"
        echo "<<run_all>> 前回 argus_bootfig の cleanup 完了を待ちます"
    fi

    # --------------------------------------------------------------
    # 前回の argus_bootfig が cleanup を完了して、
    # flock を解放するまで最大40秒待つ。
    #
    # 既存 stop_service() は、
    #   SIGINT
    #     ↓
    #   最大30秒待機
    #     ↓
    #   SIGTERM
    #     ↓
    #   3秒
    #     ↓
    #   SIGKILL
    #
    # という動作なので、それより少し長く待つ。
    # --------------------------------------------------------------
    LOCK_ACQUIRED=0

    for i in {1..40}; do
        if flock -n 9; then
            LOCK_ACQUIRED=1
            echo "<<run_all>> 既存 Argus の終了を確認しました"
            break
        fi

        echo "<<run_all>> 既存 Argus の終了待ち... (${i}/40)"
        sleep 1
    done

    # --------------------------------------------------------------
    # 40秒経過してもロックが解放されない場合
    #
    # 新しい側から無理に bootfig 自体を SIGKILL すると、
    # cleanup を飛ばしてプロセスや MMAP を残す可能性があるため、ここでは安全側に倒して起動を中止する。
    # --------------------------------------------------------------
    if [ "$LOCK_ACQUIRED" -ne 1 ]; then
        echo "<<run_all>> ERROR: 既存 Argus が正常終了しませんでした"
        echo "<<run_all>> ERROR: 安全のため新しい Argus は起動しません"
        exit 1
    fi
fi

# 仮想環境（.venv）のPython
VENV_PYTHON="$SCRIPT_DIR/.venv/bin/python"

############### 要設定 ##################
# 仮想環境を有効化
# source p312/bin/activate
source .venv/bin/activate
###########################################

# ----------------------------------------------------------------------
# データディレクトリ
# ----------------------------------------------------------------------
# settings.ini のデータパスの書き換えをここで行う場合
#DATA_DIR=/mnt/nvme
DATA_DIR=../data

#DATA_DIR=/home/matsuoka/data/X6T304

#CAL_DATA_DIR=/mnt/nvme
CAL_DATA_DIR=../data

#CAL_DATA_DIR=/home/matsuoka

sed -i "s|^data_dir *=.*|data_dir = ${DATA_DIR}|g" config/settings.ini
sed -i "s|^data_dir *=.*|data_dir = ${CAL_DATA_DIR}|g" config/calib_settings.ini

# ----------------------------------------------------------------------
# 各種ディレクトリ
# ----------------------------------------------------------------------
CONFIG_DIR="./config"
LOG_DIR="./log"
MMAP_DIR="/dev/shm"

# ステータスMMAP
MMAP_FILE="${MMAP_DIR}/status.mmap"

RUN_USER="$(id -un)"
RUN_GROUP="$(id -gn)"

# ----------------------------------------------------------------------
# 起動モード
#
# engine   : Godot Engine + project_dir
# appimage : AppImage 単体起動
#
# 起動画像モード
# production  : 全画面表示し、Godotの背面に常駐
# development : 1920x1080で表示し、RUNNING時に終了
# ----------------------------------------------------------------------
UI_MODE="${1:-appimage}"
BOOT_IMAGE_MODE="${2:-development}"

case "$UI_MODE" in
    engine|appimage)
        ;;
    *)
        echo "Usage: $0 [engine|appimage] [production|development]"
        exit 1
        ;;
esac

case "$BOOT_IMAGE_MODE" in
    production|development)
        ;;
    *)
        echo "Usage: $0 [engine|appimage] [production|development]"
        exit 1
        ;;
esac

export ARGUS_UI_MODE="$UI_MODE"

# PYTHONPATHを設定（相対import対策）
export PYTHONPATH="$PWD"

# 起動時画像
BOOT_IMAGE="./config/fig/Splash_booting.png"

# ----------------------------------------------------------------------
# monitor_boot_img 終了通知用ファイル
#
# monitor_boot_img はバックグラウンドのサブシェルで動くため、
# 親シェルの変数変更だけでは終了通知できない。
# そのため一時ファイルを終了フラグとして使用する。
# ----------------------------------------------------------------------
MONITOR_STOP_FILE="/tmp/argus_bootfig_monitor_stop_$$"
rm -f "$MONITOR_STOP_FILE"

# PID初期化
MAIN_PID=""
MONITOR_PID=""
BOOT_IMAGE_PID=""

# ----------------------------------------------------------------------
# プロセス停止
# ----------------------------------------------------------------------
stop_service() {
    local pid="$1"
    local name="$2"

    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return
    fi

    echo "<<run_all>> ${name} に終了要求を送信 (PID=${pid})"
    kill -INT "$pid" 2>/dev/null || true

    for _ in {1..30}; do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
            return
        fi
        sleep 1
    done

    echo "<<run_all>> WARN: ${name} が停止しないためプロセスグループを終了"

    kill -TERM -- "-${pid}" 2>/dev/null || true
    sleep 3

    kill -KILL -- "-${pid}" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}

# ----------------------------------------------------------------------
# cleanup
# ----------------------------------------------------------------------
cleanup() {
    # cleanup自身から再度trapが呼ばれないようにする
    trap - EXIT INT TERM

    echo "<<run_all>> cleanup start"

    # --------------------------------------------------------------
    # Main停止
    # --------------------------------------------------------------
    if [ -n "${MAIN_PID:-}" ]; then
        stop_service "$MAIN_PID" "Main"
    fi

    # --------------------------------------------------------------
    # MonitorArgus停止
    # --------------------------------------------------------------
    if [ -n "${MONITOR_PID:-}" ]; then
        stop_service "$MONITOR_PID" "MonitorArgus"
    fi

    # UI停止中も背景を隠すため、起動画像監視は最後に終了する。
    touch "$MONITOR_STOP_FILE" 2>/dev/null || true

    # --------------------------------------------------------------
    # 起動画像監視停止
    #
    # monitor_boot_img の子として feh が存在する可能性があるため、
    # 子プロセスも先に停止する。
    # --------------------------------------------------------------
    if [ -n "${BOOT_IMAGE_PID:-}" ]; then
        pkill -TERM -P "$BOOT_IMAGE_PID" 2>/dev/null || true
        kill -TERM "$BOOT_IMAGE_PID" 2>/dev/null || true
        wait "$BOOT_IMAGE_PID" 2>/dev/null || true
    fi

    rm -f "$MONITOR_STOP_FILE"

    echo "<<run_all>> cleanup end"
}

trap cleanup EXIT INT TERM

# ----------------------------------------------------------------------
# RAM領域にMMAPを準備
#
# /dev/shm は一般ユーザーがファイルを作成可能。
# status.mmap は Argus 実行ユーザー所有で使用するため、
# sudo は付けない。
#
# fs.protected_regular=2 環境では、
# sudo(root) から一般ユーザー所有の /dev/shm/status.mmap を
# write_bytes("wb") で開き直すと PermissionError になる場合がある。
# ----------------------------------------------------------------------
"$VENV_PYTHON" ./scripts/prepare_argus.py \
    --config-dir "$CONFIG_DIR" \
    --log-dir "$LOG_DIR" \
    --mmap-dir "$MMAP_DIR" \
    --user "$RUN_USER" \
    --group "$RUN_GROUP"

# ----------------------------------------------------------------------
# status.mmap 初期確認
#
# monitor_boot_img を起動する前に1回確認しておく。
# ----------------------------------------------------------------------
STATUS_OUTPUT="$(
    "$VENV_PYTHON" ./argus_synchro/SystemMonitor/get_status.py \
        --config-dir "$CONFIG_DIR" \
        --log-dir "$LOG_DIR" \
        --mmap-dir "$MMAP_DIR"
)"

read STATUS STATUS_NAME <<< "$STATUS_OUTPUT"

if ! [[ "$STATUS" =~ ^-?[0-9]+$ ]]; then
    echo "<<run_all>> ERROR: status.mmap の初期確認に失敗: $STATUS_OUTPUT"
    exit 1
fi

echo "<<run_all>> status.mmap 初期状態: $STATUS_NAME ($STATUS)"

# ----------------------------------------------------------------------
# 起動画像表示監視
#
# productionでは起動画像をGodotの背面に常時表示する。
# developmentではRUNNINGになるまでサイズ指定で表示する。
# 表示中のfehが予期せず終了した場合は再起動する。
#
# MONITOR_STOP_FILE が作成された場合は終了する。
#
# また、同じステータスを0.5秒ごとにログへ出さないよう、
# 状態が変化した時だけ present status を表示する。
# ----------------------------------------------------------------------
monitor_boot_img() {
    local FEH_PID=0
    local LAST_STATUS="__UNSET__"
    local STATUS=""
    local STATUS_NAME=""
    local STATUS_OUTPUT=""

    # --------------------------------------------------------------
    # monitor_boot_img 自身が終了するとき feh も停止
    # --------------------------------------------------------------
    monitor_boot_img_cleanup() {
        if [ "$FEH_PID" -ne 0 ] && kill -0 "$FEH_PID" 2>/dev/null; then
            kill -TERM "$FEH_PID" 2>/dev/null || true
            wait "$FEH_PID" 2>/dev/null || true
        fi
    }

    trap monitor_boot_img_cleanup EXIT INT TERM

    while [ ! -e "$MONITOR_STOP_FILE" ]; do

        if [ -f "$MMAP_FILE" ]; then

            STATUS_OUTPUT="$(
                "$VENV_PYTHON" ./argus_synchro/SystemMonitor/get_status.py \
                    --config-dir "$CONFIG_DIR" \
                    --log-dir "$LOG_DIR" \
                    --mmap-dir "$MMAP_DIR"
            )"

            read STATUS STATUS_NAME <<< "$STATUS_OUTPUT"

            # ------------------------------------------------------
            # 不正値
            #
            # 同じ異常を0.5秒ごとに出さない。
            # ------------------------------------------------------
            if ! [[ "$STATUS" =~ ^-?[0-9]+$ ]]; then
                if [ "$LAST_STATUS" != "__INVALID__" ]; then
                    echo "<<monitor_status>> ERROR: invalid status: $STATUS_OUTPUT"
                    LAST_STATUS="__INVALID__"
                fi

                sleep 0.5
                continue
            fi

            # ------------------------------------------------------
            # ステータスが変化した時だけログ出力
            # ------------------------------------------------------
            if [ "$STATUS" != "$LAST_STATUS" ]; then
                echo "<<monitor_status>> present status: $STATUS_NAME ($STATUS)"
                LAST_STATUS="$STATUS"
            fi

            if { [ "$BOOT_IMAGE_MODE" = "production" ] || [ "$STATUS" -ne 3 ]; } &&
               [ -f "$BOOT_IMAGE" ] &&
               { [ "$FEH_PID" -eq 0 ] || ! kill -0 "$FEH_PID" 2>/dev/null; }; then

                if [ "$BOOT_IMAGE_MODE" = "production" ]; then
                    echo "<<monitor_status>> 起動画像を全画面表示 (production)"
                    feh --fullscreen --auto-zoom --image-bg black --hide-pointer \
                        "$BOOT_IMAGE" &
                else
                    echo "<<monitor_status>> 起動画像をサイズ指定表示 (development)"
                    feh --geometry 1920x1080 "$BOOT_IMAGE" &
                fi

                FEH_PID=$!
            fi

            if [ "$BOOT_IMAGE_MODE" = "development" ] &&
               [ "$STATUS" -eq 3 ] &&
               [ "$FEH_PID" -ne 0 ] &&
               kill -0 "$FEH_PID" 2>/dev/null; then

                echo "<<monitor_status>> 起動画像を非表示 (development)"
                kill -INT "$FEH_PID" 2>/dev/null || true
                wait "$FEH_PID" 2>/dev/null || true
                FEH_PID=0
            fi
        fi

        sleep 0.5
    done

    echo "<<monitor_status>> 監視終了"
}

# ----------------------------------------------------------------------
# 起動画像監視をバックグラウンド実行
# ----------------------------------------------------------------------
monitor_boot_img &
BOOT_IMAGE_PID=$!

echo "<<run_all>> 起動画像監視開始 (PID=$BOOT_IMAGE_PID, MODE=$BOOT_IMAGE_MODE)"

# ----------------------------------------------------------------------
# MonitorArgus 起動 (taskset指定なし)
# ----------------------------------------------------------------------
setsid "$VENV_PYTHON" -m argus_synchro.SystemMonitor.MonitorArgus \
    --config-dir "$CONFIG_DIR" \
    --log-dir "$LOG_DIR" \
    --mmap-dir "$MMAP_DIR" &
MONITOR_PID=$!

echo "<<run_all>> MonitorArgus 起動 (PID=$MONITOR_PID, UI_MODE=$ARGUS_UI_MODE)"

# ----------------------------------------------------------------------
# MonitorArgus が起動直後に落ちていないか確認
#
# AppImage不存在などの場合、MonitorArgus は sys.exit(1) する。
# この場合、親スクリプトも終了し cleanup へ進む。
# ----------------------------------------------------------------------
sleep 1

if ! kill -0 "$MONITOR_PID" 2>/dev/null; then
    wait "$MONITOR_PID" 2>/dev/null || true
    echo "<<run_all>> ERROR: MonitorArgus が起動直後に終了しました"
    exit 1
fi

# ----------------------------------------------------------------------
# mmapファイルが作成されるまで最大10秒待機
# ----------------------------------------------------------------------
MMAP_FOUND=0

for i in {1..10}; do
    if [ -f "$MMAP_FILE" ]; then
        echo "<<run_all>> status.mmap 検出"
        MMAP_FOUND=1
        break
    fi

    echo "<<run_all>> waiting mmap ... ($i)"
    sleep 1
done

if [ "$MMAP_FOUND" -ne 1 ]; then
    echo "<<run_all>> ERROR: status.mmap が見つかりません: $MMAP_FILE"
    exit 1
fi

# ----------------------------------------------------------------------
# Main.py 起動
# ----------------------------------------------------------------------
setsid "$VENV_PYTHON" -m argus_synchro \
    --config-dir "$CONFIG_DIR" \
    --log-dir "$LOG_DIR" \
    --mmap-dir "$MMAP_DIR" &
MAIN_PID=$!

echo "<<run_all>> Main.py 起動 (PID=$MAIN_PID)"

# ----------------------------------------------------------------------
# BOOTING 状態になるまで最大10秒待機
# ----------------------------------------------------------------------
BOOTING_FOUND=0

for i in {1..10}; do

    STATUS_OUTPUT="$(
        "$VENV_PYTHON" ./argus_synchro/SystemMonitor/get_status.py \
            --config-dir "$CONFIG_DIR" \
            --log-dir "$LOG_DIR" \
            --mmap-dir "$MMAP_DIR"
    )"

    read STATUS STATUS_NAME <<< "$STATUS_OUTPUT"

    if ! [[ "$STATUS" =~ ^-?[0-9]+$ ]]; then
        echo "<<run_all>> invalid status: $STATUS_OUTPUT"
        sleep 1
        continue
    fi

    if [ "$STATUS" -eq 2 ]; then
        echo "<<run_all>> ステータスが BOOTING に遷移"
        BOOTING_FOUND=1
        break
    fi

    echo "<<run_all>> ステータス待機中... (現在: $STATUS_NAME $STATUS)"
    sleep 1
done

if [ "$BOOTING_FOUND" -ne 1 ]; then
    echo "<<run_all>> WARN: 10秒以内にBOOTINGを確認できませんでした"
fi

# ----------------------------------------------------------------------
# Main.py の終了待ち
# ----------------------------------------------------------------------
set +e
wait "$MAIN_PID"
MAIN_EXIT_CODE=$?
set -e

echo "<<run_all>> Main.py 終了 (exit=$MAIN_EXIT_CODE)"

# ----------------------------------------------------------------------
# MonitorArgus停止
# ----------------------------------------------------------------------
if [ -n "${MONITOR_PID:-}" ]; then
    stop_service "$MONITOR_PID" "MonitorArgus"
    MONITOR_PID=""
fi

# ----------------------------------------------------------------------
# 起動画像監視停止
# ----------------------------------------------------------------------
touch "$MONITOR_STOP_FILE" 2>/dev/null || true

if [ -n "${BOOT_IMAGE_PID:-}" ]; then
    pkill -TERM -P "$BOOT_IMAGE_PID" 2>/dev/null || true
    kill -TERM "$BOOT_IMAGE_PID" 2>/dev/null || true
    wait "$BOOT_IMAGE_PID" 2>/dev/null || true
    BOOT_IMAGE_PID=""
fi

rm -f "$MONITOR_STOP_FILE"

# Main はすでに wait 済み
MAIN_PID=""

# 正常終了部分まで来たのでEXIT trapを解除
trap - EXIT INT TERM

echo "<<run_all>> 全プロセス終了"

exit "$MAIN_EXIT_CODE"