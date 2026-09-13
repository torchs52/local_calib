"""
UIインターフェースを定義(facade) boss/ctrl上位クラスに対してスレッドセーフな情報共有手段を提供して情報を収集し、内外UIアプリへの橋渡しを行う。
UIアプリ起動。終了はこのクラスの管轄とする。
簡易UIをこちらに定義
"""

from __future__ import annotations

# import multiprocessing.synchronize
from abc import ABC, abstractmethod
from configparser import ConfigParser
from enum import IntEnum
from pathlib import Path
import time
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

# from open3d.cpu.pybind.geometry import LineSet
from numpy.typing import NDArray

# 自作クラス等
from argus_synchro.calibration_mat_generator_modules.facade.calib_godot_interface import (
    CalibGodotInterface,
)
from argus_synchro.common import paths

# ARGUSシステム制御関連
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import (
    FacadeConf,
    parse_list,
)
from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CameraCalibCheckStatus,
    validate_camera_calibcheck_status,
)
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_excepts import SharedExcepts

ALIVE_LOG_INTERVAL_SEC = 5.0

""" dataの値の範囲を[val_min, val_max]に変換する t3daの内容と同じなので避けたいところではある"""


def scale_transform(
    data: NDArray,
    val_min: float = 0,
    val_max: float = 1,
    allclose: float = 0.001,
) -> NDArray:
    if (data.max() - data.min()) < allclose:
        return np.array([(val_max + val_min) / 2] * len(data))
    return (val_max - val_min) / (data.max() - data.min()) * (
        data - data.min()
    ) + val_min


class FacadeUIClass_Base(ABC):
    @abstractmethod
    def __init__(
        self,
        facadeConf: FacadeConf,
        sec: SharedExcepts,
        sac: SharedAppConfig,
    ):
        raise NotImplementedError("FacadeUIClass_Base - __init__: Not implemented!")

    @abstractmethod
    def put_data(self, sectionname: str, keystr: str, content: Any) -> None:
        raise NotImplementedError("FacadeUIClass_Base - put_data: Not implemented!")

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError("FacadeUIClass_Base - put_data: Not implemented!")


CAM_CALIBLATION_STATUS_INIT = 0
CURRENTCAMERA_INIT = 255  # 未選択状態を示す値


class CalibrationCommonStatus(IntEnum):
    """status_calibcommonへ書き込む各校正モード共通の状態。"""

    INACTIVE = 0
    RUNNING = 1
    CALCULATING = 2
    COMPLETED = 3


class CalibrationUIGodot(FacadeUIClass_Base):
    def __init__(
        self,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        output_log: bool = False,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ):
        self._directory_config = directory_config
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

        self.calibGodotInterfaceInst = CalibGodotInterface(
            is_damp_out=sac.read().CalibUI_IF.damp_out,
            datPathList=sac.read().CalibUI_IF.UI_mmap,
            dampPathList=sac.read().CalibUI_IF.damp_mmap,
            s_frame=0,
            mmap_assign_json_path=sac.read().CalibUI_IF.mmap_assign_json_path,
            app_logger_factory=app_logger_factory,
            output_log=output_log,
        )
        self.output_log = output_log
        self._last_dummy_log_mono: float = 0.0
        self._last_dummy_log_state: tuple[object, ...] | None = None
        self.update_configuration(sac=sac)
        self.initialize_internal_values(
            errorcode_pre=0
        )  # TODO: エラーコードは初期化して良いのか?
        self.sac: SharedAppConfig = sac

    def apply_config(
        self,
        FacadeConfInst: FacadeConf,
    ):
        self.write_dummydata: bool = FacadeConfInst.write_dummydata

    def update_configuration(
        self,
        sac: SharedAppConfig,
    ) -> None:
        # Config読み込み・反映　(後で再起動無しで更新できるものはupdate関数に分ける)
        self.camera_num: int = sac.read().camera.count

    def initialize_internal_values(
        self, errorcode_pre: int
    ) -> None:  # A0突入時呼び出し
        self.reset_internal_values(
            errorcode_pre=errorcode_pre,
            status_calibcommon=CalibrationCommonStatus.INACTIVE,
        )
        self.camera_calibstatus_values: list[int] = [
            CAM_CALIBLATION_STATUS_INIT for _ in range(self.camera_num)
        ]
        self.yaw_value = 0

    def reset_internal_values(
        self,
        errorcode_pre: int,
        status_calibcommon: int | CalibrationCommonStatus,
        currentmode: int = 0,
        currentcamera: int = CURRENTCAMERA_INIT,
    ) -> None:  # B1, C1, D1突入時呼び出し
        if self.output_log:
            self._logger.info("reset_internal_values called")
        self.clear_internal_values(status_calibcommon=status_calibcommon)
        self.datadict: dict[str, dict[str, Any]] = {}

        self.currentmode: int = currentmode
        self.currentcamera: int = currentcamera
        self.errors_calibcommon: int = 0
        self._yaw_from_input = False

        self.camera_calibcheck_values: list[int] = [
            int(CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA)
            for _ in range(self.camera_num)
        ]

        self.errorcode_pre: int = errorcode_pre  # TODO: センサ異常等のコード

    def clear_internal_values(
        self, status_calibcommon: int | CalibrationCommonStatus
    ) -> None:  # A1突入時呼び出し
        if self.output_log:
            self._logger.info("clear_internal_values called")
        # 一時self._logger.info(から受け取って共有メモリに書き込みたいが管理上このような形式が合理的?

        self.is_end_calmode: int = 0
        self.status_calibcommon: int = int(CalibrationCommonStatus(status_calibcommon))

        self.cameradata: list[NDArray[np.uint8]] = [
            np.zeros(0, dtype=np.uint8) for _ in range(self.camera_num)
        ]
        self.camera_bbox: list[NDArray[np.int32]] = [
            np.zeros((0, 4), dtype=np.int32) for _ in range(self.camera_num)
        ]

        self.pointdata: NDArray[np.float32] = np.zeros((0, 3), dtype=np.float32)
        self.cornerpointdata: NDArray[np.float32] = np.zeros((0, 3), dtype=np.float32)

        self.progress = 0
        self.calibration_ready = False

        self.blockprogress_status: list[float] = [0 for _ in range(8)]
        self.subblockprogress_status: list[float] = [0 for _ in range(8 * 9)]

        self.pointcolordata: NDArray[np.float32] = np.zeros((0, 3), dtype=np.float32)
        self.cornerpointcolordata: NDArray[np.float32] = np.zeros(
            (0, 3), dtype=np.float32
        )

    def close(self):
        pass

    def put_data(self, sectionname: str, keystr: str, content: Any) -> None:
        if sectionname not in self.datadict:
            self.datadict[sectionname] = dict()
        self.datadict[sectionname][keystr] = content

    def transmit(
        self,
        sec: SharedExcepts,
        ref_t: int | None,
        is_end_calmode: int,
        status_calibcommon: int,
        errors_calibcommon: int,
        currentmode: int,
        currentcamera: int,
        frames: list[NDArray[np.uint8]],
        YOLOresults: list[NDArray[np.float32]],  # これは現状使わず人検知枠ゼロで出力
        yaw: float,
        points: NDArray[np.float32],
        corner3d: NDArray[np.float32],
        progress_summary: float,
        is_calib_available: bool,
        calibcheck_status: list[int],
        calib_status: list[int],
        mblock_progress_status: list[int],
        sblock_progress_status: list[int],
        write_dummydata: bool = False,
        is_firstframe: bool = False,
        force_changepage: bool = False,
        mmap_erase_rest: bool = False,
    ):
        self.errorcode_pre = 0
        # 書き込み中フラグ、書き込み時間
        self.calibGodotInterfaceInst.preprocess_info()  # 書き込み中フラグ、pass、書き込み時間
        if write_dummydata:
            self.calibGodotInterfaceInst.write_notinit_mesg(
                self.calibGodotInterfaceInst.ERROR_ADDR
            )  # デバッグ用: 未初期化を示すパターン(0xBAADF00D)で埋める

        if self.output_log:
            self._logger.debug(
                f"after preprocess_info, addr:{self.calibGodotInterfaceInst.writtenAdr}",
            )
        self.calibGodotInterfaceInst.error_info(
            sec=sec, errorcode_pre=self.errorcode_pre
        )  # エラー種別
        if self.output_log:
            self._logger.debug(
                f"after error_info, addr:{self.calibGodotInterfaceInst.writtenAdr}",
            )
        self.calibGodotInterfaceInst.WriteUInt8(is_end_calmode)  # 全体終了フラグ
        if self.output_log:
            self._logger.info(
                f"after CalMatGen_ex-IsFinished, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{is_end_calmode}",
            )

        self.calibGodotInterfaceInst.WriteUInt8(
            status_calibcommon
        )  # 稼働状態(各校正共通)
        if self.output_log:
            self._logger.info(
                f"after status_calibcommon, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{status_calibcommon}",
            )

        self.calibGodotInterfaceInst.WriteUInt8(
            currentmode
        )  # currentmode(各校正共通)
        if self.output_log:
            self._logger.info(
                f"after currentmode, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{currentmode}",
            )
        self.calibGodotInterfaceInst.WriteUInt8(
            currentcamera
        )  # currentcamera(各校正共通)
        if self.output_log:
            self._logger.info(
                f"after currentcamera, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{currentcamera}",
            )

        self.calibGodotInterfaceInst.WriteUInt32(
            errors_calibcommon
        )  # エラー番号(各校正共通)
        if self.output_log:
            self._logger.debug(
                f"after errors_calibcommon, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{errors_calibcommon}",
            )
        """for i in range(4):
            x = errors_camera[i] if i < len(errors_camera) else 0
            self.calibGodotInterfaceInst.WriteUInt8(x)
            if self.output_log:
                self._logger.info(
                    f"after errors_camera[{i}], addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{x}",
                )"""

        # 画像表示の有無: currentmode値が2(2D3D校正)の時のみ適用。それ以外は常時表示
        if currentmode != 2 or self.sac.read().CalibUI_IF.show_image2d3d:
            for i in range(self.camera_num):
                self.calibGodotInterfaceInst.camera_img(
                    frames[i], YOLOresults[i]
                )  # カメラ画像
        else:
            for i in range(self.camera_num):
                self.calibGodotInterfaceInst.camera_img(
                    np.zeros((0), np.uint8), np.zeros((0, 0), np.float32)
                )  # カメラ画像
        if self.output_log:
            self._logger.debug(
                f"after frames, addr:{self.calibGodotInterfaceInst.writtenAdr}"
            )
        self.calibGodotInterfaceInst.WriteFloat32(yaw)  # 上部旋回体回転角度
        if self.output_log:
            self._logger.info(
                f"after yaw, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{yaw}",
            )
        self._transmit_3dfmat(points)
        if self.output_log:
            self._logger.info(
                f"after points, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{points.shape}",
            )

        if self.sac.read().CalibUI_IF.show_trajectory:
            self._transmit_3dfmat(corner3d)
            if self.output_log:
                self._logger.debug(
                    f"after corner3d, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{corner3d.shape}",
                )
        else:
            corner3d_zeromat = np.zeros((0, 3), np.float32)
            self._transmit_3dfmat(corner3d_zeromat)
            if self.output_log:
                self._logger.debug(
                    f"after corner3d, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{corner3d_zeromat.shape}",
                )

        self.calibGodotInterfaceInst.WriteFloat32(progress_summary)
        if self.output_log:
            self._logger.debug(
                f"after progress_summary, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{progress_summary}",
            )

        self.calibGodotInterfaceInst.WriteUInt8(
            1 if is_calib_available else 0
        )  # 校正可能フラグ
        if self.output_log:
            self._logger.info(
                f"after is_calib_available, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{is_calib_available}",
            )

        self._transmit_calibcheck_status(calibcheck_status)
        if self.output_log:
            self._logger.info(
                f"after calibcheck_status, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{calibcheck_status}",
            )
        # self._transmit_calibstatus_list_to_bitlist(mblock_progress_status)

        for x in calib_status:
            self.calibGodotInterfaceInst.WriteUInt8(x)
        if self.output_log:
            self._logger.info(
                f"after calib_status, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{calib_status}",
            )

        for x in mblock_progress_status:
            self.calibGodotInterfaceInst.WriteUInt8(x)
        if self.output_log:
            self._logger.debug(
                f"after mblock_progress_status, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{mblock_progress_status}",
            )

        # self._transmit_calibstatus_list_to_bitlist(sblock_progress_status)
        for x in sblock_progress_status:
            self.calibGodotInterfaceInst.WriteUInt8(x)
        if self.output_log:
            self._logger.debug(
                f"after sblock_progress_status, addr:{self.calibGodotInterfaceInst.writtenAdr}, content:{sblock_progress_status}",
            )
        self.calibGodotInterfaceInst.postprocess_info(
            ref_t,
            is_firstframe=is_firstframe,
            force_changepage=force_changepage,
            mmap_erase_rest=mmap_erase_rest,
        )
        if self.output_log:
            self._logger.debug(
                f"after postprocess_info, addr:{self.calibGodotInterfaceInst.writtenAdr}, ref_t:{ref_t}",
            )

    def _transmit_3dfmat(self, mat3d: NDArray[np.float32]):
        self.calibGodotInterfaceInst.WriteUInt32(len(mat3d))
        self.calibGodotInterfaceInst.WriteFloat32Batch(mat3d.reshape(-1))
        # for X in mat3d:
        #    self.calibGodotInterfaceInst.WriteFloat32(X[0])
        #    self.calibGodotInterfaceInst.WriteFloat32(X[1])
        #    self.calibGodotInterfaceInst.WriteFloat32(X[2])

    @staticmethod
    def _calc_subblockpos_to_id(blockid: int, sub_x: int, sub_y: int) -> int:
        return blockid * (3 * 3) + sub_x * 3 + sub_y

    def set_dummydata(
        self,
        enable_systemerrorflag: bool = False,
        enable_errorflag: bool = False,
        enable_yawangle: bool = False,
        overwrite_checkresult: bool = False,
        overwrite_calibresult: bool = False,
        overwrite_caliberror: bool = True,
    ):
        config_dir: Path = paths.get_config_dir(
            self._directory_config, "calibration_mat_generator_modules"
        )

        dummy_senddata_path: str = str(
            paths.normalize_path("dummy_senddata.ini", config_dir)
        )

        try:  # ダミー書き込み関数なのでここでエラーが発生したとしても落としたくない。ファイル読み込みが絡むためConfigParser周りでのエラーが懸念。
            confparser_dummy = ConfigParser()
            confparser_dummy.read(
                filenames=dummy_senddata_path,
                encoding="utf8",
            )
            applied = False

            if enable_errorflag:
                if confparser_dummy.has_option("DEFAULT", "errors_calibcommon_bin"):
                    errors_calibcommon_str = confparser_dummy.get(
                        "DEFAULT", "errors_calibcommon_bin"
                    )
                    try:
                        self.errors_calibcommon = int(errors_calibcommon_str, base=2)
                        applied = True
                    except ValueError as ev:
                        self._logger.warning(
                            f"Applying [DEFAULT] errors_calibcommon_bin {ev}"
                        )
                if confparser_dummy.has_option("DEFAULT", "errors_calibcommon"):
                    errors_calibcommon = confparser_dummy.get(
                        "DEFAULT", "errors_calibcommon"
                    )
                    try:
                        self.errors_calibcommon = int(errors_calibcommon)
                        applied = True
                    except ValueError as ev:
                        self._logger.warning(
                            f"Applying [DEFAULT] errors_calibcommon {ev}"
                        )

            if enable_systemerrorflag:
                # self._logger.info( f"{enable_systemerrorflag = }")
                if confparser_dummy.has_option("DEFAULT", "errors_system_bin"):
                    errors_system_bin_str = confparser_dummy.get(
                        "DEFAULT", "errors_system_bin"
                    )
                    try:
                        self.errorcode_pre = int(errors_system_bin_str, base=2)
                        applied = True
                    except ValueError as ev:
                        self._logger.warning(
                            f"Applying [DEFAULT] errors_system_bin {ev}"
                        )

            if enable_yawangle and (
                getattr(self, "write_dummydata", False)
                or not getattr(self, "_yaw_from_input", False)
            ):
                if confparser_dummy.has_option("DEFAULT", "yaw"):
                    applied = True
                    yaw_str = confparser_dummy.get("DEFAULT", "yaw")
                    self.yaw_value = float(yaw_str)

            if overwrite_checkresult:
                if confparser_dummy.has_section(
                    "CalibCheckOverwrite"
                ) and confparser_dummy.has_option("CalibCheckOverwrite", "values"):
                    camera_values_str = parse_list(
                        confparser_dummy.get("CalibCheckOverwrite", "values")
                    )

                    for ix, dmy_stat in enumerate(camera_values_str):
                        if "na" in dmy_stat:
                            pass
                        elif "ng" in dmy_stat:
                            applied = True
                            self.camera_calibcheck_values[ix] = int(
                                CameraCalibCheckStatus.CALIBRATION_REQUIRED
                            )
                        elif "ok" in dmy_stat:
                            applied = True
                            self.camera_calibcheck_values[ix] = int(
                                CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED
                            )
                        elif "un" in dmy_stat:
                            applied = True
                            self.camera_calibcheck_values[ix] = int(
                                CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
                            )
                        # 数値ならMMAPへ書き込み可能なstatusとして検証して代入
                        elif dmy_stat.isdigit():
                            status = validate_camera_calibcheck_status(int(dmy_stat))
                            applied = True
                            self.camera_calibcheck_values[ix] = int(status)

            if overwrite_calibresult:
                if confparser_dummy.has_section(
                    "CalibStatusOverwrite"
                ) and confparser_dummy.has_option("CalibStatusOverwrite", "values"):
                    camera_values_str: list[str] = parse_list(
                        confparser_dummy.get("CalibStatusOverwrite", "values")
                    )

                    for ix, dmy_stat in enumerate(camera_values_str):
                        # un(→unknown): 0 / ok:3 / ng:1 / na
                        if "na" in dmy_stat:
                            pass
                        else:
                            try:
                                val = int(dmy_stat)
                                self.camera_calibstatus_values[ix] = val
                                applied = True
                            except Exception as e:
                                self._logger.warning(f"Exception: {e}")
                    # self._logger.info( f"Applying [CalibCheckOverwrite] values -> {self.camera_calibstatus_values}")

            if applied:
                state = (
                    self.errorcode_pre,
                    self.yaw_value,
                    tuple(self.camera_calibcheck_values),
                    self.errors_calibcommon,
                    tuple(self.camera_calibstatus_values),
                )
                now = time.monotonic()
                last_state = getattr(self, "_last_dummy_log_state", None)
                last_log_mono = getattr(self, "_last_dummy_log_mono", 0.0)
                if (
                    state != last_state
                    or now - last_log_mono >= ALIVE_LOG_INTERVAL_SEC
                ):
                    self._last_dummy_log_state = state
                    self._last_dummy_log_mono = now
                    self._logger.info(
                        f"set_dummydata applied (alive): {self.errorcode_pre=}, "
                        f"{self.yaw_value=}, "
                        f"camera_calibcheck_values={self.camera_calibcheck_values}, "
                        f"self.errors_calibcommon={self.errors_calibcommon}, "
                        f"camera_calibstatus_values={self.camera_calibstatus_values}"
                    )

        except Exception as ef:
            self._logger.warning(f"Exception: {ef}")

    def transmit_setdata(
        self,
        sec: SharedExcepts,
        ref_t: int | None,
        is_firstframe: bool = False,
        force_changepage: bool = False,
        mmap_erase_rest: bool = False,
    ) -> None:
        status_calibcommon = self.status_calibcommon
        errors_calibcommon = self.errors_calibcommon
        yaw_value = self.yaw_value

        """
        with open("argus_synchro/calibration_mat_generator_modules/temp/facade_frame_info.pickle", mode="ab") as wbf:
            pickle.dump((
                ref_t,
                status_calibcommon,
                errors_calibcommon,
                self.cameradata,
                self.camera_bbox, #これは現状使わず人検知枠ゼロで出力
                self.yaw_value,
                self.pointdata,
                self.cornerpointdata,
                self.progress,
                self.calibration_ready,
                self.camera_calibcheck_values,
                self.blockprogress_status,
                self.subblockprogress_status,
                self.datadict
            ), wbf)"""

        self.transmit(
            sec=sec,
            ref_t=ref_t,
            is_end_calmode=self.is_end_calmode,
            status_calibcommon=status_calibcommon,
            errors_calibcommon=errors_calibcommon,
            currentmode=self.currentmode,
            currentcamera=self.currentcamera,
            frames=self.cameradata,
            YOLOresults=self.camera_bbox,  # これは現状使わず人検知枠ゼロで出力
            yaw=yaw_value,
            points=self.pointdata,
            corner3d=self.cornerpointdata,
            progress_summary=self.progress,
            is_calib_available=self.calibration_ready,
            calibcheck_status=self.camera_calibcheck_values,
            calib_status=self.camera_calibstatus_values,
            mblock_progress_status=self.blockprogress_status,
            sblock_progress_status=self.subblockprogress_status,
            write_dummydata=self.write_dummydata,
            is_firstframe=is_firstframe,
            force_changepage=force_changepage,
            mmap_erase_rest=mmap_erase_rest,
        )

        # if ref_t % 100 == 0:
        #    dump_rawmem: list[bytes] = self.calibGodotInterfaceInst.get_rawmemdata()
        #    for ix in range(len(dump_rawmem)):
        #        with open(f"log/CalibUIIF_rawmemdump{ix}_{ref_t}.bin", mode="wb") as wbf:
        #            wbf.write(dump_rawmem[ix])

    def set_status_calibcommon(
        self, status_calibcommon_val: int | CalibrationCommonStatus
    ) -> None:
        if self.output_log:
            self._logger.info(
                f"UI value set by status_calibcommon: {status_calibcommon_val}"
            )
        self.status_calibcommon = int(CalibrationCommonStatus(status_calibcommon_val))

    def set_currentmode(self, currentmode: int):
        if self.output_log:
            self._logger.info(f"UI value set by set_currentmode: {currentmode}")
        self.currentmode: int = currentmode

    def set_currentcamera(self, currentcamera: int):
        if self.output_log:
            self._logger.info(f"UI value set by set_currentcamera: {currentcamera}")
        self.currentcamera: int = currentcamera

    def set_errors_calibcommon(self, errors_calibcommon_val: int):
        if self.output_log:
            self._logger.info("UI value set by errors_calibcommon")
        self.errors_calibcommon = int(errors_calibcommon_val)

    def _transmit_calibcheck_status(self, calibcheck_status: list[int]):
        assert len(calibcheck_status) == self.camera_num
        for value in calibcheck_status:
            status = validate_camera_calibcheck_status(value)
            self.calibGodotInterfaceInst.WriteUInt8(int(status))

    def _transmit_calibstatus_list_to_bitlist(self, block_progress_status: list[int]):
        datalist_temp: list[int] = []
        for ix, X in enumerate(block_progress_status):
            if (ix % 8) == 0:
                datalist_temp.append(0)
            if X > 0:
                datalist_temp[-1] |= 1 << (ix % 8)

        self.calibGodotInterfaceInst.WriteBytes_size32b(
            bytes(datalist_temp), len(datalist_temp)
        )

    def set_end_calibration_flag(self, end_calibration_flag: int):
        if self.output_log:
            self._logger.info(f"set_end_calibration_flag, {end_calibration_flag}")
        self.is_end_calmode = end_calibration_flag

    def set_image(self, camera_id: int, data: NDArray[np.uint8]):
        if self.output_log:
            self._logger.debug(
                f"UI value set by set_image, camera {camera_id}, {data.shape}"
            )
        self.cameradata[camera_id] = data

    def set_2Dbbox(self, camera_id: int, data: NDArray[np.int32]):
        if self.output_log:
            self._logger.info(
                f"UI value set by set_2Dbbox, camera {camera_id}, {data.shape}"
            )
        self.camera_bbox[camera_id] = data

    def set_yaw(self, value: float):
        if self.output_log:
            self._logger.info(f"UI value set by set_yaw, value: {value}")
        self.yaw_value: float = value
        self._yaw_from_input = True

    def set_points(self, data: NDArray[np.float32], colordata: NDArray[np.float32]):
        if self.output_log:
            self._logger.debug(
                f"UI value set by set_points, {data.shape}, {colordata.shape}"
            )
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data)
        pcd = pcd.voxel_down_sample(0.1)
        self.pointdata = np.array(pcd.points)
        self.pointcolordata = colordata

    def set_cornerpoints(
        self, data: NDArray[np.float32], colordata: NDArray[np.float32]
    ):
        if self.output_log:
            self._logger.debug(
                f"UI value set by set_cornerpoints, {data.shape}, {colordata.shape}",
            )
        self.cornerpointdata = data
        self.cornerpointcolordata = colordata

    def set_progress(self, value: float):
        if self.output_log:
            self._logger.debug(f"UI value set by set_progress, {value}")
        self.progress: float = value

    def set_calibration_ready(self, value: int):
        if self.output_log:
            self._logger.info(f"UI value set by set_calibration_ready, {value}")
        self.calibration_ready: bool = value > 0

    def set_camera_calibcheck_status(
        self, camera_id: int, value: int | CameraCalibCheckStatus
    ) -> None:
        status = validate_camera_calibcheck_status(value)
        if self.output_log:
            self._logger.info(
                f"UI value set by set_camera_calibcheck_status, camera {camera_id} : {int(status)}",
            )
        self.camera_calibcheck_values[camera_id] = int(status)

    def set_camera_calibration_status(self, camera_id: int, value: int):
        if self.output_log:
            self._logger.info(
                f"UI value set by set_camera_calibration_status, camera {camera_id} : {value}",
            )
        self.camera_calibstatus_values[camera_id] = value

    def set_blockprogress_status(self, blockprogress_status: NDArray[np.float32]):
        if self.output_log:
            self._logger.debug(
                f"UI value set by set_blockprogress_status, {blockprogress_status}",
            )
        self.blockprogress_status: list[float] = [int(x) for x in blockprogress_status]

    def set_subblockprogress_status(self, subblockprogress_status: NDArray[np.float32]):
        if self.output_log:
            self._logger.debug(
                f"UI value set by set_subblockprogress_status, {subblockprogress_status}",
            )
        self.subblockprogress_status: list[float] = [
            int(x) for x in subblockprogress_status
        ]

    def set_boxes(
        self,
        points_multipoints: NDArray[np.float32],
        points_multi_lines: NDArray[np.int32],
    ):
        if self.output_log:
            self._logger.debug("UI value set by set_boxes")
        self.points_multipoints = points_multipoints
        self.points_multi_lines = points_multi_lines
        self.put_data("dataproc", f"3dobj_{0}_multipoints", points_multipoints)
        self.put_data("dataproc", f"3dobj_{0}_multilines", points_multi_lines)

    def set_errorcode_unexpected_exception(self, value: bool):
        if value:
            self.errorcode_pre |= 0x00000001
        else:
            self.errorcode_pre &= 0xFFFFFFFE

    @staticmethod
    def convert_intensity_to_color(
        intensities: NDArray[np.float32],
    ) -> NDArray[np.float32]:
        intensities_line: NDArray[np.float32] = intensities.ravel()
        cmap_plt = plt.get_cmap("jet")
        intensities_line = np.where(
            (intensities_line < 0),
            intensities_line + 256,
            intensities_line,
        )
        color: NDArray[np.float32] = cmap_plt(
            scale_transform(intensities_line, val_min=0, val_max=1)
        )
        return color


def printable_dict(d: dict[str, Any], indent: int = 0) -> str:
    """
    ネストされた辞書のすべてのキーと値を再帰的に表示用文字列にする関数。
    :param d: 表示対象の辞書
    :param indent: インデントの深さ（内部用）
    """
    returnstr = ""
    for key, value in d.items():
        prefix = " " * indent
        if isinstance(value, dict):
            returnstr += f"{prefix}{key}: ** dict **" + "\n"
            ret = printable_dict(value, indent + 4)
            returnstr += ret
        else:
            valuestr_temp: str = f"{prefix}{key}:"
            if isinstance(value, np.ndarray):
                valuestr_temp += (
                    f"NDArray {value.dtype}, shape: {value.shape}, contents:{value}"
                )
            else:
                valuestr_temp += f"{value}"

            returnstr += (
                valuestr_temp[: min(200, len(valuestr_temp))]
                + ("..." if len(valuestr_temp) > 200 else "")
                + "\n"
            )
    return returnstr
