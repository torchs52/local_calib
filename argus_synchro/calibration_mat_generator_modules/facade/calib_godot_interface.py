from __future__ import annotations

import datetime
import json

# 標準ライブラリ
import os
import platform
import sys
import time
from io import TextIOWrapper

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.ClsMMap import ByteSize as BS
from argus_synchro.ClsMMap import classMMap  # 共有メモリ操作クラス

# Argus3Dライブラリ
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_excepts import SharedExcepts

_logger: AppLogger = AppLoggerFactory.from_name("UI_IF")


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)


def W_Null(clsMMap: classMMap, Adr: int, repeat_num: int) -> None:
    WBin = bytes(repeat_num)
    clsMMap.WriteBytes(Adr, WBin)


def check_damp_file(fname: str, start: int, length: int) -> None:
    np.set_printoptions(edgeitems=10)
    damped = np.loadtxt(fname, dtype="unicode")
    _logger.info(damped[start : start + length])


def log_mmap(damp_fp: TextIOWrapper, bin_data: bytes) -> None:
    # 16 進数表記に変換して文字列化
    # 各バイトを 2 桁の大文字 16 進数で表記
    hex_string: str = " ".join(f"{byte_data:02X}" for byte_data in bin_data)
    hex_string = hex_string + "\n"
    # AppLogger.info("ui_if",hex_string)
    # テキストファイルに書き込み
    damp_fp.write(hex_string)


class CalibMMapMaintainer:  # 共有メモリをMainから作る場合はこちらの使用を検討。仕様が不必要に複雑になるため一度やめることにした
    def __init__(
        self,
        sac: SharedAppConfig,
        mmap_assign_json_path: str,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        # 前回異常終了していると情報が残るので、最初に共有メモリファイル削除.
        self.datPathList = sac.read().CalibUI_IF.UI_mmap
        for dataPathIndex in range(len(self.datPathList)):
            # data_file
            if os.path.exists(self.datPathList[dataPathIndex]):
                os.remove(self.datPathList[dataPathIndex])

        # 共有メモリ管理クラス インスタンス生成
        self.clsMMap_list: list[classMMap] = []
        for dataPathIndex in range(len(self.datPathList)):
            self._logger.info(
                "dataPathIndex: "
                + str(dataPathIndex)
                + ", "
                + self.datPathList[dataPathIndex],
            )
            self.clsMMap_list.append(
                classMMap(
                    self.datPathList[dataPathIndex],
                    app_logger_factory=app_logger_factory,
                )
            )

            # self.clsMMap_list[dataPathIndex].~~~  ここで初期化を実行
            self._logger.info(
                f"{datetime.datetime.now()} - classMMap {self.datPathList[dataPathIndex]} created",
            )

        with open(mmap_assign_json_path, encoding="utf-8") as f:
            mmap_assign_db: dict = json.load(f)

        self.IsReading_ADR: int = mmap_assign_db["IsReading"]["ADDR"]
        self.IsWriting_ADR: int = mmap_assign_db["IsWriting"]["ADDR"]
        self.UNIX_TIME_ADDR: int = mmap_assign_db["UNIX_TIME"]["ADDR"]
        self.ERROR_ADDR: int = mmap_assign_db["ERROR"]["ADDR"]
        self.Start_ADR: int = mmap_assign_db["Start_ADDR"]

        # テストここまで *****

    def __del__(self) -> None:
        for dataPathIndex in range(len(self.datPathList)):
            self.clsMMap_list[dataPathIndex].dispose()


class CalibGodotInterface:
    def __init__(
        self,
        # calib_ui_if: CalibUIIFConf,
        is_damp_out: bool,
        datPathList: list[str],
        dampPathList: list[str],
        s_frame: int,
        mmap_assign_json_path: str,
        app_logger_factory: AppLoggerFactory,
        output_log: bool = False,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        pf = platform.system()
        self._logger.info("platform: " + pf)
        self._logger.info("endian: " + sys.byteorder)
        self.s_frame = s_frame
        self.mapIndex: int = 0
        now: float = time.time()
        self.sTime: float = now
        self.preProcessTime: float = now

        self.damp_out: bool = is_damp_out
        self.datPathList: list[str] = datPathList
        self.dampPathList: list[str] = dampPathList
        self.output_log: bool = output_log

        with open(mmap_assign_json_path, encoding="utf-8") as f:
            mmap_assign_db: dict = json.load(f)

        self.IsReading_ADR: int = mmap_assign_db["IsReading"]["ADDR"]
        self.IsWriting_ADR: int = mmap_assign_db["IsWriting"]["ADDR"]
        self.UNIX_TIME_ADDR: int = mmap_assign_db["UNIX_TIME"]["ADDR"]
        self.ERROR_ADDR: int = mmap_assign_db["ERROR"]["ADDR"]
        self.Start_ADR: int = mmap_assign_db["Start_ADDR"]

        # 前回異常終了していると情報が残るので、最初に共有メモリファイル削除.
        for dataPathIndex in range(len(self.datPathList)):
            # data_file
            if os.path.exists(self.datPathList[dataPathIndex]):
                os.remove(self.datPathList[dataPathIndex])

        # 共有メモリ管理クラス インスタンス生成
        self.clsMMap_list: list[classMMap] = []
        for dataPathIndex in range(len(self.datPathList)):
            self._logger.info(
                "dataPathIndex: "
                + str(dataPathIndex)
                + ", "
                + self.datPathList[dataPathIndex],
            )
            self.clsMMap_list.append(
                classMMap(self.datPathList[dataPathIndex], app_logger_factory)
            )
            self._logger.info(
                f"{datetime.datetime.now()} - classMMap {self.datPathList[dataPathIndex]} created",
            )

        if self.damp_out:
            for dataPathIndex in range(len(self.datPathList)):
                # damp_file
                if os.path.exists(self.dampPathList[dataPathIndex]):
                    os.remove(self.dampPathList[dataPathIndex])

            # dampファイルポインタ生成
            self.damp_fp_list: list[TextIOWrapper] = []
            for dataPathIndex in range(len(self.datPathList)):
                damp_fp: TextIOWrapper = open(self.dampPathList[dataPathIndex], "w")
                self.damp_fp_list.append(damp_fp)

        self.clsMMap: classMMap = self.clsMMap_list[self.mapIndex]
        self.writtenAdr: int = self.Start_ADR

        self.ref_t_count: int | None = None

    def __del__(self) -> None:
        for dataPathIndex in range(len(self.datPathList)):
            self.clsMMap_list[dataPathIndex].dispose()
            if self.damp_out:
                self.damp_fp_list[dataPathIndex].close()

    def preprocess_info(self) -> None:
        self.sTime = time.time()
        if self.output_log:
            self._logger.debug(f"Adr(IsWrite): {self.writtenAdr!s}")
        # 書き込み中フラグ
        self.clsMMap.WriteInt8(self.writtenAdr, 1)

        # self.writtenAdr += BS.INT8
        self.writtenAdr = (
            self.UNIX_TIME_ADDR
        )  # 直前にReadOnlyアドレスがあるため明示的に指定.
        if self.output_log:
            self._logger.debug(f"Adr(unix_time): {self.writtenAdr!s}")
        now_unix_time = int(time.time() * 1000 - 1732600000000)
        # 書込時間
        self.clsMMap.WriteInt64(self.writtenAdr, now_unix_time)
        if self.output_log:
            self._logger.debug(
                f"now_unix_time: {now_unix_time!s}, {self.mapIndex = }"
            )
        self.writtenAdr += BS.INT64

    def error_info(self, sec: SharedExcepts, errorcode_pre: int) -> None:
        if self.output_log:
            self._logger.debug(f"Adr(error): {self.writtenAdr!s}")
        # エラー情報は専用MMAPで通知するため、この旧領域は0固定とする。
        _ = sec, errorcode_pre
        code = 0
        if self.output_log:
            self._logger.debug(f"{code = }")
        self.clsMMap.WriteInt32(self.writtenAdr, code)
        self.writtenAdr += BS.INT32

    def WriteUInt8(self, val: int):
        self.clsMMap.WriteInt8(self.writtenAdr, val)
        self.writtenAdr += BS.INT8

    def WriteUInt16(self, val: int):
        self.clsMMap.WriteInt16(self.writtenAdr, val)
        self.writtenAdr += BS.INT16

    def WriteSInt16(self, val: int):
        self.clsMMap.WriteSignedInt16(self.writtenAdr, val)
        self.writtenAdr += BS.INT16

    def WriteUInt32(self, val: int):
        self.clsMMap.WriteInt32(self.writtenAdr, val)
        self.writtenAdr += BS.INT32

    def WriteFloat32(self, val: float):
        self.clsMMap.WriteFloat(self.writtenAdr, val)
        self.writtenAdr += BS.INT32

    def WriteFloat32Batch(self, values: NDArray[np.float32]) -> None:
        """
        複数の float32 を一括で書き込む
        :param values: list[float] or np.ndarray
        """
        # numpy配列に変換（float32）
        if not isinstance(values, np.ndarray):
            arr = np.array(values, dtype=np.float32)
        else:
            arr = values.astype(np.float32)

        # struct.pack_into("<f") と同じリトルエンディアン形式でバイト列化
        byte_data = arr.tobytes(order="C")  # C-orderで連続メモリ

        # classMMap.WriteBytesで一括書き込み
        self.clsMMap.WriteBytes(self.writtenAdr, byte_data)

        # アドレス更新
        self.writtenAdr += len(byte_data)

    def WriteBytes_size32b(self, bdata: bytes, datasize: int):
        self.clsMMap.WriteInt8(self.writtenAdr, datasize)
        self.writtenAdr += BS.INT32
        self.clsMMap.WriteBytes(self.writtenAdr, bdata)
        self.writtenAdr += datasize * BS.INT8

    def camera_img(
        self,
        frame: NDArray[np.uint8],
        bboxes: NDArray[np.float32],
    ) -> None:
        frame_b: NDArray[np.uint8] = np.asarray(frame).astype(np.uint8)
        try:
            if frame_b.size > 0:
                ret, encoded = cv2.imencode(
                    ".jpg", frame_b, (cv2.IMWRITE_JPEG_QUALITY, 10)
                )
            else:
                encoded = np.zeros(0, np.uint8)
        except Exception as e:
            self._logger.warning(f"camera_img convert exception: {e}")
            encoded = np.zeros(0, np.uint8)

        if self.output_log:
            self._logger.debug(
                f"encoded.size: {encoded.size}" + f", writtenAdr: {self.writtenAdr}",
            )

        # カメラ画像バイト数
        if self.output_log:
            self._logger.debug(f"Adr(cam.size): {self.writtenAdr}")
        self.clsMMap.WriteInt32(self.writtenAdr, encoded.size)
        self.writtenAdr += BS.INT32

        # カメラ画像の実体（圧縮データ）
        if self.output_log:
            self._logger.debug(f"Adr(cam.data): {self.writtenAdr}")
        self.clsMMap.WriteBytes(self.writtenAdr, encoded)
        self.writtenAdr += encoded.size * BS.INT8
        if self.output_log:
            self._logger.debug(encoded)

        # 人検知結果（四角形BB個数、座標）
        # self.write_2d_object_detection_result(frame, LS_cam_det)
        bboxes_transmit: NDArray[np.float32] = bboxes.reshape(-1, 4)
        dsize = len(bboxes_transmit)

        if self.output_log:
            self._logger.info(
                f"Adr(bboxes_transmit): {self.writtenAdr}, len: {len(bboxes_transmit)}, data:{bboxes_transmit}, {dsize}",
            )
        self.clsMMap.WriteInt16(self.writtenAdr, dsize)
        self.writtenAdr += BS.INT16

        for d in bboxes_transmit:
            for ix, elem in enumerate(d):
                if self.output_log:
                    self._logger.info(
                        f"bboxes_transmit {ix} : {elem} @ {self.writtenAdr}"
                    )
                self.clsMMap.WriteInt16(self.writtenAdr, elem)
                self.writtenAdr += BS.INT16

    """def write_2d_object_detection_result(
        self,
        frame: NDArray[np.uint8],
        num_boxes: int,
        out_boxes: NDArray,
        out_classes: NDArray, 
        num_boxes: NDArray
    ) -> None:
        
        person_num: int = 0
        person_num_Adr: int = copy.copy(
            self.writtenAdr,
        )  # 書き込み場所を記憶して、最後に戻って書く.
        # 四角形BB個数
        self._logger.info( f"Adr(person_num) (Skip): {person_num_Adr!s}")
        self.writtenAdr += BS.INT8

        image_h, image_w, _ = frame.shape

        coordinate: NDArray[np.int16] = np.zeros(4).astype(np.int16)
        # num_boxes[0] が0の時は一度も処理をせずにpassする.
        for i in range(num_boxes[0]):
            # score = out_scores[0][i]
            class_ind = int(out_classes[0][i])
            if class_ind < 0 or class_ind > NUM_CLASSES:
                continue
            if class_ind == 0:
                person_num += 1

                coordinate[0] = int(out_boxes[i][1] * image_w)  # x 始点
                coordinate[1] = int(out_boxes[i][0] * image_h)  # y 始点
                coordinate[2] = int(out_boxes[i][3] * image_w)  # x 終点
                coordinate[3] = int(out_boxes[i][2] * image_h)  # y 終点

                self._logger.info(
                    self, f"Adr(person_coordinate): {self.writtenAdr!s}"
                )
                # 四角形BB頂点1x, 1y, 2x, 2y （対角線）
                for i in range(len(coordinate)):
                    self.clsMMap.WriteSignedInt16(self.writtenAdr, coordinate[i])
                    self.writtenAdr += BS.INT16
                    self._logger.info( coordinate[i])

        # 四角形BB個数
        self._logger.info( f"Adr(person_num): {person_num_Adr!s}")
        self.clsMMap.WriteInt8(person_num_Adr, person_num)
        self._logger.info( f"{person_num = }")
        # tmpBytes = self.clsMMap.ReadBytes(person_num_Adr, int(1 + 2 * person_num + 3))
        # AppLogger.info("ui_if", tmpBytes)"""

    def postprocess_info(
        self,
        ref_t_arg: int | None,
        is_firstframe: bool = False,
        force_changepage: bool = False,
        mmap_erase_rest: bool = False,
    ) -> None:
        if ref_t_arg is None:
            if self.ref_t_count is None or is_firstframe:
                ref_t = self.s_frame
            else:
                ref_t = self.ref_t_count + 1
        else:
            ref_t = ref_t_arg

        self.ref_t_count = ref_t

        # 処理フレーム番号をデバッグ用に記述
        if self.output_log:
            self._logger.info(f"Adr(frame_num): {self.writtenAdr!s}")
        self.clsMMap.WriteInt64(self.writtenAdr, ref_t)
        if self.output_log:
            self._logger.info(f"{ref_t = }")
        self.writtenAdr += BS.INT64

        if mmap_erase_rest:
            self.write_blankdata(
                beginaddr=self.writtenAdr, endaddr=BS.MAP_ALL, data=bytes([0])
            )
            self._logger.info(
                f"** write_blankdata called!! ** address: {self.writtenAdr=}"
            )

        if self.writtenAdr > BS.MAP_ALL:
            self._logger.warning(
                f"**** SHARED MEMORY OVERFLOW!!!! **** {self.writtenAdr = }"
            )

        # 必要に応じてMMAP切り替え
        isRead = self.clsMMap.ReadInt8(self.IsReading_ADR) > 0  # 読み込み中フラグ
        if isRead or ref_t == self.s_frame or force_changepage:
            if isRead:
                if self.output_log:
                    self._logger.debug(
                        f"{isRead = } or {ref_t = } == {self.s_frame = }, MMAP Index will be changed!",
                    )
            if self.output_log:
                self._logger.debug(f"{self.clsMMap.ReadInt8(self.IsReading_ADR) = }")
                self._logger.debug(f"{self.clsMMap.ReadInt8(self.IsWriting_ADR) = }")
                self._logger.debug(f"{self.clsMMap.ReadInt64(self.UNIX_TIME_ADDR) = }")
            self.clsMMap.WriteInt8(self.IsWriting_ADR, 0)
            self.mapIndex = (self.mapIndex + 1) % len(self.clsMMap_list)
            self.clsMMap = self.clsMMap_list[self.mapIndex]
            self.clsMMap.WriteInt8(self.IsWriting_ADR, 1)
            if self.output_log:
                self._logger.debug("MMAP Index changed!")

                self._logger.debug(f"{self.clsMMap.ReadInt8(self.IsReading_ADR) = }")
                self._logger.debug(f"{self.clsMMap.ReadInt8(self.IsWriting_ADR) = }")
                self._logger.debug(f"{self.clsMMap.ReadInt64(self.UNIX_TIME_ADDR) = }")

        now: float = time.time()
        MMapdelta: float = (now - self.sTime) * 1000
        Alldelta: float = (now - self.preProcessTime) * 1000
        if self.output_log:
            self._logger.info(
                f"Adr(last): {self.writtenAdr!s}"
                f", MMapdelta: {MMapdelta:.2f} msec"
                f", Alldelta: {Alldelta:.2f} msec",
            )
        self.preProcessTime = now

        self.writtenAdr = self.Start_ADR
        if self.output_log:
            self._logger.debug("writtenAdr Reset!")

    def damp_info(self) -> None:
        # damp of mmap file.
        for dataPathIndex in range(len(self.datPathList)):
            # Start_ADRを起点にして、全てダンプ（メモリサイズを負の値に設定）する指定。
            bin_data: bytes = self.clsMMap_list[dataPathIndex].ReadBytes(
                self.Start_ADR, -1
            )
            log_mmap(self.damp_fp_list[dataPathIndex], bin_data)

    def get_rawmemdata(self) -> list[bytes]:
        return [cm.ReadBytes(self.Start_ADR, -1) for cm in self.clsMMap_list]

    def write_blankdata(
        self,
        beginaddr: Optional[int] = None,
        endaddr: Optional[int] = None,
        data=bytes([0]),
    ):
        if beginaddr is None:
            beginaddr = self.writtenAdr

        if endaddr is None:
            endaddr = BS.MAP_ALL

        self._logger.info(
            f"{beginaddr = }, {endaddr = } mem clear start at write_blankdata"
        )
        for addr in range(beginaddr, endaddr - len(data), len(data)):
            self.clsMMap_list[self.mapIndex].WriteBytes(addr, data)
            self.writtenAdr = addr

        datalen_rest = endaddr - self.writtenAdr - 1
        self._logger.info(f"{datalen_rest = }, additional mem clear at write_blankdata")
        if datalen_rest > 0:
            self.clsMMap_list[self.mapIndex].WriteBytes(addr, data[:datalen_rest])
        self.writtenAdr = addr + datalen_rest
        self._logger.info(f"write_blankdata end, {self.writtenAdr = }")

    def write_notinit_mesg(
        self,
        beginaddr: Optional[int] = None,
        endaddr: Optional[int] = None,
    ):
        self.write_blankdata(
            beginaddr=beginaddr,
            endaddr=endaddr,
            data=bytes([0xBA, 0xAD, 0xF0, 0x0D]),
        )
