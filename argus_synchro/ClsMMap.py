"""---------------------------
共有メモリ管理クラス
---------------------------"""

from __future__ import annotations

import mmap
import os
import struct

import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class ByteSize:
    UNIT = 1
    INT8 = 1  # Char
    INT16 = 2  # Short
    INT32 = 4  # long
    INT64 = 8  # longlong
    FLOAT = 4
    MAP_ALL = 10 * 1024 * 1024
    # MAP_ALL = 10 * 1024 * 15  # damp時に巨大すぎないような一時指定


class classMMap:
    def __init__(self, mapFilePath: str, app_logger_factory: AppLoggerFactory) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        # self.endian = sys.byteorder
        self.endian = "little"
        self.UNIT_SIZE = ByteSize.UNIT
        self.mapFilePath: str = mapFilePath
        self.size: int = ByteSize.MAP_ALL
        self._fp: object | None = None
        self._mm: mmap.mmap | None = None

        # ファイルをサイズ固定で作成／補正
        self._ensure_file(self.mapFilePath, self.size)
        # マップを開く（書込可）
        self._open_map()

        self._logger.info("__init__ comp.")

    # ---------- internal ----------
    def _ensure_file(self, path: str, size: int) -> None:
        # 既存サイズと違う場合も truncate で合わせる
        fd = os.open(path, os.O_RDWR | os.O_CREAT)
        try:
            cur = os.path.getsize(path)
            if cur != size:
                os.ftruncate(fd, size)
        finally:
            os.close(fd)
        self._logger.info(f"_ensure_file ok: {path} ({size} bytes)")

    def _open_map(self) -> None:
        # Windows での共有を考慮した変更.
        self._fp = open(self.mapFilePath, "r+b", buffering=0)
        # 明示サイズ＋書込可でマップ（length=0は避ける）
        self._mm = mmap.mmap(
            self._fp.fileno(), length=self.size, access=mmap.ACCESS_WRITE
        )
        self._mm.seek(0)
        self._logger.info("_open_map fin.")

    # ---------- public ----------
    def flush(self) -> None:
        if self._mm:
            self._mm.flush()

    def dispose(self) -> None:
        try:
            if self._mm:
                self._mm.flush()
                self._mm.close()
        finally:
            self._mm = None
            if self._fp:
                self._fp.close()
                self._fp = None
        self._logger.info("dispose fin.")

    # ---------- read / write ----------
    def ReadInt(self, adr: int, byte_size: int, sign: bool) -> int | None:
        try:
            assert self._mm is not None
            self._mm.seek(adr)
            bytes: bytes = self._mm.read(byte_size)
            val: int = int.from_bytes(bytes, self.endian, signed=sign)
            self._mm.seek(0)
            return val
        except Exception as e:
            self._logger.info("ReadInt except: " + str(e).replace("\n", ""))
            return None

    def ReadInt64(self, adr: int) -> int | None:
        return self.ReadInt(adr, ByteSize.INT64, False)

    def ReadInt32(self, adr: int) -> int | None:
        return self.ReadInt(adr, ByteSize.INT32, False)

    def ReadInt16(self, adr: int) -> int | None:
        return self.ReadInt(adr, ByteSize.INT16, False)

    def ReadInt8(self, adr: int) -> int | None:
        return self.ReadInt(adr, ByteSize.INT8, False)

    def WriteInt(self, adr: int, data: int, byte_size: int, sign: bool) -> None:
        try:
            assert self._mm is not None
            # 正しい範囲チェック（符号あり／なし）
            if sign:
                min_v = -(1 << (8 * byte_size - 1))
                max_v = (1 << (8 * byte_size - 1)) - 1
            else:
                min_v = 0
                max_v = (1 << (8 * byte_size)) - 1
            if not (min_v <= int(data) <= max_v):
                self._logger.info(
                    f"Error. Out of range for {byte_size}B sign={sign}: {data}"
                )
                return

            bytes: bytes = int(data).to_bytes(byte_size, self.endian, signed=sign)
            self._mm[adr : adr + byte_size] = bytes
        except Exception as e:
            self._logger.info("WriteInt except " + str(e).replace("\n", ""))

    def WriteInt64(self, adr: int, data: int) -> None:
        self.WriteInt(adr, data, ByteSize.INT64, False)

    def WriteInt32(self, adr: int, data: int) -> None:
        self.WriteInt(adr, data, ByteSize.INT32, False)

    def WriteInt16(self, adr: int, data: int) -> None:
        self.WriteInt(adr, data, ByteSize.INT16, False)

    def WriteSignedInt16(self, adr: int, data: int) -> None:
        self.WriteInt(adr, data, ByteSize.INT16, True)

    def WriteInt8(self, adr: int, data: int) -> None:
        self.WriteInt(adr, data, ByteSize.INT8, False)

    def WriteFloat(self, adr: int, data: float) -> None:
        try:
            assert self._mm is not None
            struct.pack_into("<f", self._mm, adr, float(data))
        except Exception as e:
            self._logger.info("WriteFloat except " + str(e).replace("\n", ""))

    def ReadFloat(self, adr: int) -> float | None:
        try:
            assert self._mm is not None
            return struct.unpack_from("<f", self._mm, adr)[0]
        except Exception as e:
            self._logger.info("ReadFloat except " + str(e).replace("\n", ""))
            return None

    # 配列データ書き込み & Byteデータ直接書き込み.(JPEG画像書き込み等に使用)
    def WriteBytes(
        self, adr: int, data: NDArray[np.uint8] | bytes | bytearray | memoryview
    ) -> None:
        try:
            assert self._mm is not None
            mv: memoryview[int] = memoryview(data)
            self._mm[adr : adr + len(mv)] = mv
        except Exception as e:
            self._logger.info("WriteBytes except " + str(e).replace("\n", ""))

    def ReadBytes(self, adr: int, length: int) -> bytes | None:
        try:
            assert self._mm is not None
            return self._mm[adr : adr + length]
        except Exception as e:
            self._logger.info("ReadBytes except " + str(e).replace("\n", ""))
            return None
