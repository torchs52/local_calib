from __future__ import annotations

import atexit
import datetime as _dt
import os
import pickle
import re
import time
from pathlib import Path
from typing import Any, Literal, Union

# =========================
# Types
# =========================
IndexArg = Union[int, None]
LoadIndexArg = Union[int, None, Literal["latest"]]


# =========================
# Module-global state (プロセス内共有)
# =========================
_STATE: dict[str, Any] = {
    "initialized": False,
    "closed": False,
    # identity
    "pid": None,
    "date_str": None,
    "base_dir": None,
    # paths
    "contents_path": None,
    "delta_path": None,
    "snapshot_path": None,
    # open file handles (buffered)
    "contents_fp": None,  # append binary
    "delta_fp": None,  # append binary
    # in-memory table
    "table": None,  # dict
    # scheduling / counters
    "version": 2,
    "event_count": 0,
    "last_flush_t": 0.0,
    "last_snapshot_t": 0.0,
    # config (default tuned for 30fps*multi-call)
    "flush_interval_sec": 0.5,  # flush contents/delta every 0.5 sec
    "snapshot_interval_sec": 5.0,  # write snapshot every 5 sec
    "snapshot_on_close": True,  # write final snapshot on close
    "fsync_snapshot": False,  # allow losing few seconds -> default False
    "fsync_flush": False,  # default False (flush only)
    # options
    "allow_mixed_mode": True,  # allow both indexed & single per key
    "key_blacklist": [],
    "deny_all": False,  # Deny all data:  default False
}

_KEY_RE = re.compile(r"^[A-Za-z0-9_]+$")


# =========================
# Public APIs
# =========================
def debug_config(
    *,
    base_dir: str | Path | None = None,
    flush_interval_sec: float | None = None,
    snapshot_interval_sec: float | None = None,
    snapshot_on_close: bool | None = None,
    fsync_snapshot: bool | None = None,
    fsync_flush: bool | None = None,
) -> None:
    """
    設定。初回 debug_store 前に呼ぶのが推奨（呼ばなくてもデフォルトで動作）。
    初期化後でも interval 等は変更可能。
    """
    if base_dir is not None and not _STATE["initialized"]:
        _STATE["base_dir"] = Path(base_dir)
    if flush_interval_sec is not None:
        _STATE["flush_interval_sec"] = float(flush_interval_sec)
    if snapshot_interval_sec is not None:
        _STATE["snapshot_interval_sec"] = float(snapshot_interval_sec)
    if snapshot_on_close is not None:
        _STATE["snapshot_on_close"] = bool(snapshot_on_close)
    if fsync_snapshot is not None:
        _STATE["fsync_snapshot"] = bool(fsync_snapshot)
    if fsync_flush is not None:
        _STATE["fsync_flush"] = bool(fsync_flush)


def debug_apply_blacklist(blpath: str, deny_all: bool = False):
    with open(blpath) as rf:
        for line in rf.readlines():
            lst = line.strip()
            if lst == "" or lst[0] == "#":
                continue
            debug_append_blacklist(lst)
    if deny_all:
        _STATE["deny_all"] = True


def debug_append_blacklist(blkey: str):
    if blkey not in _STATE["key_blacklist"]:
        _STATE["key_blacklist"].append(blkey)


def debug_store(
    key: str,
    value: Any,
    index: IndexArg = -1,
    *,
    base_dir: str | Path = ".",
) -> tuple[str, int | None]:
    """
    高頻度デバッグ格納（contents=append, table=delta+snapshot）。

    index:
      - None : キーにつき単一スロット（上書き）
      - -1   : 自動割り当て（0スタート、キーごとnext_index）
      - >=0  : 指定indexに保存（既存があれば参照更新）
    """
    if _STATE["deny_all"]:
        return key, None
    if key in _STATE["key_blacklist"]:
        return key, None

    _validate_key(key)
    _ensure_initialized(base_dir)

    if _STATE["closed"]:
        raise RuntimeError(
            "debug_close() 済みです。再利用する場合はプロセスをやり直すか、実装を拡張してください。"
        )

    if index is not None:
        if not isinstance(index, int):
            raise TypeError("index は int か None である必要があります")
        if index < -1:
            raise ValueError("index は -1 / 0以上 / None のみ許可します")

    table = _STATE["table"]
    assert isinstance(table, dict)

    # key entry
    keys = table.setdefault("keys", {})
    ent = keys.get(key)

    mode = "single" if index is None else "indexed"
    if ent is None:
        ent = _new_key_entry(mode=mode)
        keys[key] = ent
    # 混在許容：必要なフィールドを足す
    elif index is None:
        ent.setdefault("single", None)
    else:
        ent.setdefault("items", {})

    now = _now_iso()

    # determine stored_index
    stored_index: int | None
    if index is None:
        stored_index = None
    elif index == -1:
        next_index = int(ent.get("next_index", 0))
        stored_index = next_index
        ent["next_index"] = next_index + 1
    else:
        stored_index = int(index)
        next_index = int(ent.get("next_index", 0))
        if stored_index >= next_index:
            ent["next_index"] = stored_index + 1

    # 1) append to contents -> offset
    offset = _append_contents_record(
        key=key, stored_index=stored_index, value=value, time_iso=now
    )

    # 2) update in-memory table
    if stored_index is None:
        ent["single"] = {"offset": offset, "time": now}
    else:
        items = ent.setdefault("items", {})
        items[int(stored_index)] = {"offset": offset, "time": now}
        ent["last_index"] = max(int(ent.get("last_index", -1)), int(stored_index))
        ent["latest_index"] = int(stored_index)
        ent["latest_offset"] = int(offset)

    table["total_records"] = int(table.get("total_records", 0)) + 1
    table["last_write_time"] = now

    # 3) append delta event
    _append_delta_event(("PUT", key, stored_index, int(offset), now))

    # 4) periodic flush/snapshot
    _maybe_flush_and_snapshot()

    return key, stored_index


def debug_load(
    key: str,
    index: LoadIndexArg = "latest",
    *,
    base_dir: str | Path = ".",
) -> Any:
    """
    保存したデータを読み出す（tableからoffsetを引いてcontentsへseek）。
    """
    _validate_key(key)
    _ensure_initialized(base_dir)

    table = _STATE["table"]
    assert isinstance(table, dict)
    ent = table.get("keys", {}).get(key)
    if ent is None:
        raise KeyError(f"key '{key}' は存在しません")

    if index is None:
        single = ent.get("single")
        if not single:
            raise KeyError(f"key '{key}' の単一スロットが存在しません")
        offset = int(single["offset"])
        return _read_contents_value_at(offset)

    if index == "latest":
        items = ent.get("items") or {}
        if not items:
            raise KeyError(f"key '{key}' に indexed データが存在しません")
        latest_index = ent.get("latest_index")
        if latest_index is None:
            latest_index = max(map(int, items.keys()))
        meta = items.get(int(latest_index))
        if not meta:
            raise KeyError(f"key '{key}' の latest が不正です")
        return _read_contents_value_at(int(meta["offset"]))

    if not isinstance(index, int) or index < 0:
        raise ValueError("debug_load の index は None / 'latest' / 0以上int のみ")

    items = ent.get("items") or {}
    meta = items.get(int(index))
    if not meta:
        raise KeyError(f"key '{key}' の index={index} は存在しません")
    return _read_contents_value_at(int(meta["offset"]))


def debug_paths(*, base_dir: str | Path = ".") -> tuple[Path, Path, Path]:
    """
    (snapshot, delta, contents) のパス
    """
    _ensure_initialized(base_dir)
    return _STATE["snapshot_path"], _STATE["delta_path"], _STATE["contents_path"]


def debug_force_snapshot(*, base_dir: str | Path = ".") -> None:
    """
    今すぐスナップショットを保存する。
    contents/delta も flush して、直後の debug_load で EOF にならないようにする。
    """
    _ensure_initialized(base_dir)
    _flush_files()  # ← 追加：contents/delta をディスクへ反映
    _write_snapshot_atomic()


def debug_close(*, base_dir: str | Path = ".") -> None:
    """
    ファイルハンドルを閉じ、必要なら最終スナップショットを書き出す。
    """
    if not _STATE["initialized"]:
        return

    if _STATE["closed"]:
        return

    # 最終 flush
    _flush_files()

    # 最終 snapshot（設定に従う）
    if _STATE.get("snapshot_on_close", True):
        _write_snapshot_atomic()

    # close handles
    fp = _STATE.get("contents_fp")
    if fp:
        try:
            fp.close()
        except Exception:
            pass

    fp = _STATE.get("delta_fp")
    if fp:
        try:
            fp.close()
        except Exception:
            pass

    _STATE["contents_fp"] = None
    _STATE["delta_fp"] = None
    _STATE["closed"] = True


# =========================
# Internal helpers
# =========================
def _validate_key(key: str) -> None:
    if not isinstance(key, str):
        raise TypeError("key は str である必要があります")
    if not _KEY_RE.match(key):
        raise ValueError(f"key '{key}' は英数字・アンダースコアのみ許可です")


def _ensure_initialized(base_dir: str | Path) -> None:
    """
    lazy init:
      - snapshot をロード
      - delta を snapshot の delta_pos からリプレイ
      - contents/delta のファイルハンドルを open して保持
    """
    if _STATE["initialized"]:
        return

    # apply base_dir from config if set before init
    if _STATE.get("base_dir") is not None:
        base = Path(_STATE["base_dir"])
    else:
        base = Path(base_dir)

    base.mkdir(parents=True, exist_ok=True)

    pid = os.getpid()
    date_str = _dt.datetime.now().strftime("%y%m%d")

    contents_path = base / f"{date_str}_{pid}_debugdata_contents.pickle"
    delta_path = base / f"{date_str}_{pid}_debugdata_table_delta.pickle"
    snapshot_path = base / f"{date_str}_{pid}_debugdata_table_snapshot.pickle"

    # 1) load snapshot if exists
    table, delta_pos = _load_snapshot(snapshot_path)

    # 2) replay delta from delta_pos
    table = _replay_delta(delta_path, table, start_pos=delta_pos)

    # 3) open file handles (append mode, buffered)
    #    open once, reuse
    contents_fp = contents_path.open("ab")
    delta_fp = delta_path.open("ab")

    # set state
    _STATE.update(
        {
            "initialized": True,
            "closed": False,
            "pid": pid,
            "date_str": date_str,
            "base_dir": base,
            "contents_path": contents_path,
            "delta_path": delta_path,
            "snapshot_path": snapshot_path,
            "contents_fp": contents_fp,
            "delta_fp": delta_fp,
            "table": table,
            "event_count": 0,
            "last_flush_t": time.time(),
            "last_snapshot_t": time.time(),
        }
    )

    # atexit close
    atexit.register(debug_close)


def _new_table(
    *, pid: int, date_str: str, contents_path: Path, delta_pos: int
) -> dict[str, Any]:
    now = _now_iso()
    return {
        "version": int(_STATE["version"]),
        "created_at": now,
        "pid": int(pid),
        "date_str": date_str,
        "contents_path": str(contents_path),
        "keys": {},
        "total_records": 0,
        "last_write_time": now,
        # snapshot metadata
        "delta_pos": int(
            delta_pos
        ),  # delta file byte position applied in this snapshot
    }


def _new_key_entry(*, mode: str) -> dict[str, Any]:
    ent: dict[str, Any] = {
        "created_at": _now_iso(),
        "next_index": 0,
        "last_index": -1,
        "latest_index": None,
        "latest_offset": None,
    }
    if mode == "single":
        ent["single"] = None
    elif mode == "indexed":
        ent["items"] = {}
    else:
        raise ValueError("mode must be 'single' or 'indexed'")
    return ent


def _append_contents_record(
    *, key: str, stored_index: int | None, value: Any, time_iso: str
) -> int:
    """
    contents_fp に追記して offset を返す。
    """
    fp = _STATE["contents_fp"]
    if fp is None:
        raise RuntimeError("contents_fp が未初期化です")

    record = {
        "key": key,
        "index": stored_index,
        "time": time_iso,
        "value": value,
    }

    offset = fp.tell()
    pickle.dump(record, fp, protocol=pickle.HIGHEST_PROTOCOL)
    _STATE["event_count"] += 1
    return int(offset)


def _append_delta_event(event: tuple) -> None:
    """
    delta_fp に追記（軽量イベント）。
    event例: ("PUT", key, stored_index_or_None, offset, time_iso)
    """
    fp = _STATE["delta_fp"]
    if fp is None:
        raise RuntimeError("delta_fp が未初期化です")

    pickle.dump(event, fp, protocol=pickle.HIGHEST_PROTOCOL)


def _maybe_flush_and_snapshot() -> None:
    """
    flush_interval_sec ごとに flush。
    snapshot_interval_sec ごとに snapshot を原子的に保存。
    """
    now_t = time.time()

    # flush
    if now_t - float(_STATE["last_flush_t"]) >= float(_STATE["flush_interval_sec"]):
        _flush_files()
        _STATE["last_flush_t"] = now_t

    # snapshot
    if now_t - float(_STATE["last_snapshot_t"]) >= float(
        _STATE["snapshot_interval_sec"]
    ):
        _write_snapshot_atomic()
        _STATE["last_snapshot_t"] = now_t


def _flush_files() -> None:
    """
    contents/delta を flush（fsyncは通常しない）。
    """
    for k in ("contents_fp", "delta_fp"):
        fp = _STATE.get(k)
        if fp:
            try:
                fp.flush()
                if _STATE.get("fsync_flush", False):
                    os.fsync(fp.fileno())
            except Exception:
                # デバッグ用途：ここで落とさない
                pass


def _write_snapshot_atomic() -> None:
    """
    in-memory table を snapshot に丸ごと保存（tmp->replace）。
    snapshotには delta_pos（deltaファイルの現在位置）を記録する。
    """
    table = _STATE["table"]
    if not isinstance(table, dict):
        return

    snap_path: Path = _STATE["snapshot_path"]
    delta_fp = _STATE.get("delta_fp")
    if delta_fp is None:
        return

    # deltaの現在位置までを「適用済み」として記録
    try:
        delta_pos = int(delta_fp.tell())
    except Exception:
        delta_pos = int(table.get("delta_pos", 0))

    # copy table shallow (avoid mutating while writing)
    table_to_save = dict(table)
    table_to_save["delta_pos"] = delta_pos
    table_to_save["last_snapshot_time"] = _now_iso()

    tmp_path = snap_path.with_suffix(snap_path.suffix + ".tmp")
    try:
        with tmp_path.open("wb") as f:
            pickle.dump(table_to_save, f, protocol=pickle.HIGHEST_PROTOCOL)
            f.flush()
            if _STATE.get("fsync_snapshot", False):
                os.fsync(f.fileno())
        os.replace(tmp_path, snap_path)
    except Exception:
        # 失敗してもデバッグ用途：落とさない（ただしtmpが残る可能性）
        try:
            if tmp_path.exists():
                tmp_path.unlink(missing_ok=True)
        except Exception:
            pass


def _load_snapshot(snapshot_path: Path) -> tuple[dict[str, Any], int]:
    """
    snapshotを読み、(table, delta_pos) を返す。
    なければ新規table。
    """
    pid = os.getpid()
    date_str = _dt.datetime.now().strftime("%y%m%d")
    # contents_path はここでは確定してないが、tableに入れても参照用なので後で上書きしない前提。
    # 新規作成時のみ、呼び出し側が作るパスに合わせて差し替える必要があるため、
    # ここでは仮のPathを入れておき、_ensure_initializedで確定パスをセットする。
    dummy_contents = (
        snapshot_path.parent / f"{date_str}_{pid}_debugdata_contents.pickle"
    )

    if snapshot_path.exists():
        try:
            with snapshot_path.open("rb") as f:
                table = pickle.load(f)
            if not isinstance(table, dict):
                raise ValueError("snapshot format invalid")
            delta_pos = int(table.get("delta_pos", 0))
            return table, delta_pos
        except Exception:
            # 壊れていたら新規
            return _new_table(
                pid=pid, date_str=date_str, contents_path=dummy_contents, delta_pos=0
            ), 0

    return _new_table(
        pid=pid, date_str=date_str, contents_path=dummy_contents, delta_pos=0
    ), 0


def _replay_delta(
    delta_path: Path, table: dict[str, Any], start_pos: int
) -> dict[str, Any]:
    """
    deltaログを start_pos から読み、tableに適用。
    末尾が途切れている場合は例外を握りつぶして止める（デバッグ用途）。
    """
    if not delta_path.exists():
        # contents_pathは確定後に上書きしたいが、ここでは呼び出し側で整合させる
        return table

    try:
        with delta_path.open("rb") as f:
            # seek to last applied position
            if start_pos > 0:
                f.seek(int(start_pos))

            while True:
                try:
                    ev = pickle.load(f)
                except EOFError:
                    break
                except Exception:
                    # 末尾破損など：ここで止める
                    break

                if not isinstance(ev, tuple) or len(ev) < 1:
                    continue

                if ev[0] == "PUT":
                    # ("PUT", key, stored_index_or_None, offset, time_iso)
                    try:
                        _, key, idx, offset, tiso = ev
                    except Exception:
                        continue
                    # apply
                    keys = table.setdefault("keys", {})
                    ent = keys.get(key)
                    if ent is None:
                        ent = _new_key_entry(
                            mode=("single" if idx is None else "indexed")
                        )
                        keys[key] = ent
                    elif idx is None:
                        ent.setdefault("single", None)
                    else:
                        ent.setdefault("items", {})

                    if idx is None:
                        ent["single"] = {"offset": int(offset), "time": tiso}
                    else:
                        items = ent.setdefault("items", {})
                        idxi = int(idx)
                        items[idxi] = {"offset": int(offset), "time": tiso}
                        ent["last_index"] = max(int(ent.get("last_index", -1)), idxi)
                        ent["latest_index"] = idxi
                        ent["latest_offset"] = int(offset)
                        # next_indexの推定：明示indexが出てくる場合を考慮
                        next_index = int(ent.get("next_index", 0))
                        if idxi >= next_index:
                            ent["next_index"] = idxi + 1

                    table["total_records"] = int(table.get("total_records", 0)) + 1
                    table["last_write_time"] = tiso

                # 将来拡張用：他イベントが来ても無視

    except Exception:
        # 読めない場合は諦める
        pass

    return table


def _read_contents_value_at(offset: int) -> Any:
    """
    contentsを seek して value を読む。
    ※読み出しは open/close で良い（頻度が低い想定）。
    """
    contents_path: Path = _STATE["contents_path"]
    with contents_path.open("rb") as f:
        f.seek(int(offset))
        record = pickle.load(f)
    if not isinstance(record, dict) or "value" not in record:
        raise ValueError("contents record format invalid")
    return record["value"]


def _now_iso() -> str:
    return _dt.datetime.now().isoformat(timespec="seconds")
