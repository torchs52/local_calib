# 画像・点群同期読み込み

# 画像対応点抽出
from datetime import datetime
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture import (
    data_capture,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.datacapture_local import (
    tools2Dcapture as t2dc,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.datacapture_local import (
    tools3Dcapture as t3dc,
)

# from ctrl.calibration2d3d.datacapture_local import pickle_2ddataio as p2io
# from datacapture.videofilereader_fromtimestamp import videofilereader_fromtimestamp_onefile
# 型定義でのみ使用
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.calib_fifo_message import FIFOData
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors, StateErrorDIndex


class datacapture_class:
    def _report_file_io_error(
        self, path: str, operation: str, error: Exception
    ) -> None:
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        result = file_io_error.errors_diagnosis(True)
        file_io_error.log_output(
            *result,
            StateErrorDIndex.FILE_IO_ERROR,
            path,
            operation,
            f"{type(error).__name__}: {error}",
        )

    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors,
    ) -> None:
        """dataCaptureConf: DataCaptureConf,
        dataConverter2D3DConf: DataConverter2D3DConf,
        calib2d3dConf: Calib2d3dConf,
        camerasel: int"""
        self._app_logger_factory = app_logger_factory
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.app_config_calib = app_config_calib
        self.dataConverter2D3DConf = self.app_config_calib.dataConverter2D3D
        self.dataCaptureConf = self.app_config_calib.dataCapture
        self.calib2d3dConf = self.app_config_calib.calib2d3d
        self.sac = sac
        self.camerasel = sac.read().CalibMode.cameraID
        self._ser: SharedErrors = shared_errors

        self.verbose = False  # TODO config追加

        self.init_settings(camerasel=self.camerasel)
        self.cvcap_false_count_limit: int | None = (
            1000  # デバッグ向け定数 100f連続上記ならアプリ自体を落とす Noneで無効
        )

        self.cvcap_false_count = 0

    def close(self) -> None:
        pass

    def init_settings(self, camerasel: int) -> None:
        begintime = datetime.now()

        accumulate_length = self.dataConverter2D3DConf.Lidar.accumulate_length

        self.cap2d = t2dc.capture2d(
            app_config_calib=self.app_config_calib, sac=self.sac
        )
        self.cap3d = t3dc.capture3d(
            app_config_calib=self.app_config_calib,
            sac=self.sac,
            app_logger_factory=self._app_logger_factory,
            file_io_error_reporter=self._report_file_io_error,
        )

        self.ts = 0

        self.accumulate_length = accumulate_length

        self.prev_timestamp: None | int = None

        self._logger.info(
            f"skip to {self.dataCaptureConf.s_frame - 3 * int(self.dataConverter2D3DConf.Lidar.accumulate_length)}"
        )

        self._logger.info(f"init time spent: {(datetime.now() - begintime)}")
        self.frame_prevtime = datetime.now()

    def get_cameramatrix(self):
        return self.cap2d.get_cameramatrix()

    def release(self):
        pass

    def isOpened(self):
        return self.cap2d.isOpened() and self.cap3d.isOpened()

    def input_data_diagnosis(
        self,
        data_cameras: list[tuple[NDArray[np.uint8], int, float]],
        data_lidars: list[tuple[NDArray[np.float32], int, float]],
        can_data: tuple[float, float],
        framecounter: int,
    ) -> bool:
        invalid_data_input = self._ser.state_errors_D[
            StateErrorDIndex.INVALID_DATA_INPUT
        ]
        result, failsafe_result = invalid_data_input.errors_diagnosis(
            (data_cameras, data_lidars, can_data, framecounter)
        )
        invalid_data_input.log_output(
            result, failsafe_result, StateErrorDIndex.INVALID_DATA_INPUT
        )
        if result == ResultDiagnosis.DETECTION:
            return True

        images = tuple(camera_data[0] for camera_data in data_cameras)
        min_xyz_columns = 3
        pcds_point_cloud = tuple(
            lidar_data[0][:, :min_xyz_columns] for lidar_data in data_lidars
        )

        array_shape_error = self._ser.state_errors_D[StateErrorDIndex.ARRAY_SHAPE_ERROR]
        result, failsafe_result = array_shape_error.errors_diagnosis(
            ("images", images),
            ("pcds_point_cloud", pcds_point_cloud),
        )
        array_shape_error.log_output(
            result, failsafe_result, StateErrorDIndex.ARRAY_SHAPE_ERROR
        )
        return result == ResultDiagnosis.DETECTION

    # 1F分のデータを取得
    # 返り値：取得失敗でNone、または
    """
    ret, frame = readresults[0]
    pcdframe = readresults[1]
    timestamp_img = readresults[2]
    timestamp_pcd = readresults[3] #点群蓄積の影響で点群のタイムスタンプは画像タイムスタンプより遅れる
    """

    def read(
        self, readresult_pop: FIFOData
    ) -> tuple[
        tuple[NDArray[np.uint8], int | None], tuple[NDArray[np.float32], int | None]
    ]:
        dontread = False
        if (
            self.prev_timestamp is not None
            and (
                self.prev_timestamp
                + 3 * self.dataConverter2D3DConf.Lidar.accumulate_length
            )
            < self.dataCaptureConf.s_frame
        ):  # 読み込み処理だけでそれなりに重いので読み取りフレームまで遠い場合は一気にループを進める
            dontread = True

        if not self.isOpened():
            return None

        prev_timestamp_img = 0
        prev_timestamp_pcd = 0
        readresults = self._read_worker(
            readresult_pop=readresult_pop, dontread=dontread
        )
        ret, frame = readresults[0]
        pcdframe = readresults[1]
        timestamp_img = readresults[2]
        timestamp_pcd = readresults[
            3
        ]  # 点群蓄積の影響で点群のタイムスタンプは画像タイムスタンプより遅れる
        self.prev_timestamp = timestamp_img
        if not ret:
            self._logger.warning(
                "Warning!! cv cap returned False. Video or stream ends?"
            )
            self.cvcap_false_count += 1
            if self.cvcap_false_count_limit is not None:
                if (
                    self.cvcap_false_count > self.cvcap_false_count_limit
                    and self.app_config_calib.default.File_Input
                ):
                    raise RuntimeError(
                        f"画像がロードされないまま規定数{self.cvcap_false_count_limit}フレーム経過しました。エラー終了。"
                    )
            return None
        if len(pcdframe) == 0:
            self._logger.warning(
                "warning: pcd size is zero"
            )  # LiDARの一時的なデータ欠損かも?
            return (frame, prev_timestamp_img), (pcdframe, prev_timestamp_pcd)

        # 読み込み時間表示
        if not dontread:
            self._logger.info(
                f"read: {timestamp_img}size: 3d, 2d:{pcdframe.shape},{frame.shape},dtime:{(datetime.now() - self.frame_prevtime)}"
            )
            if self.verbose:
                self._logger.info(
                    self,
                    f"{np.max(pcdframe, axis=0) = }, {np.min(pcdframe, axis=0) = }",
                )
            self.frame_prevtime = datetime.now()

        elif timestamp_img % 1000 == 0:
            self._logger.info(
                "(skipping) read:{timestamp_img}size: 3d, 2d:,{pcdframe.shape},{frame.shape}"
            )

        # index指定で入力を移動するため、読み込みスキップは廃止
        # if timestamp_img < self.dataCaptureConf.s_frame:
        #     continue
        if timestamp_img > self.dataCaptureConf.e_frame:
            return None

        prev_timestamp_pcd = timestamp_pcd
        prev_timestamp_img = timestamp_img

        return (frame, timestamp_img), (pcdframe, timestamp_pcd)

    def _read_worker(
        self, readresult_pop: FIFOData, dontread=False
    ) -> tuple[
        tuple[bool, NDArray[np.uint8]],
        NDArray[np.float32],
        int | None,
        int | None,
    ]:
        # dontread: 点群の読み込みをパスして空の点群を返す。フレームスキップ機能実装のため。

        if readresult_pop is None:
            return (
                (False, np.zeros((0, 0, 0), dtype=np.uint8)),
                np.zeros((0, 4), dtype=np.float32),
                None,
                None,
            )
        data_cameras, data_lidars, can_data, framecounter = readresult_pop
        if self.input_data_diagnosis(
            data_cameras,
            data_lidars,
            can_data,
            framecounter,
        ):
            return (
                (False, np.zeros((0, 0, 0), dtype=np.uint8)),
                np.zeros((0, 4), dtype=np.float32),
                None,
                None,
            )

        ret, frame, ts_img_raw = self.cap2d.read(data_cameras)
        if ret is None or ts_img_raw is None:
            return (ret, frame), np.zeros((0, 4), dtype=np.float32), None, None
        ts_img = framecounter  # ts_img_raw[0]
        pcd, ts_lidar_raw = self.cap3d.read(data_lidars, dontread=dontread)
        if ts_lidar_raw is None:
            return (ret, frame), np.zeros((0, 4), dtype=np.float32), None, None
        # ts_lidar = ts_lidar_raw[0] - int(
        #    (self.accumulate_length - 1) / 2
        # )  # インデックスで計算
        ts_lidar = framecounter - int((self.accumulate_length - 1) / 2)
        # ts_img = self.ts
        # ts_lidar = self.ts - int( (self.accumulate_length-1)/2 )
        self.ts += 1
        if self.verbose:
            self._logger.info(
                f"_read_worker image ts:{ts_img}, lidar ts:{ts_lidar}, internal ts:{self.ts}",
            )
        return [(ret, frame), pcd, ts_img, ts_lidar]

    def read_all(self, data_capture_inst: data_capture):
        dontread = False
        if (
            self.prev_timestamp is not None
            and (
                self.prev_timestamp
                + 3 * self.dataConverter2D3DConf.Lidar.accumulate_length
            )
            < self.dataCaptureConf.s_frame
        ):  # 読み込み処理だけでそれなりに重いので読み取りフレームまで遠い場合は一気にループを進める
            dontread = True

        if not self.isOpened():
            return None

        # 1フレーム分データ取得 無限ループはフレームスキップ機能のため（インデックス指定して直に飛べるならそちらの方が良い）
        while True:
            readresults = self._read_all_worker(
                data_capture_inst=data_capture_inst, dontread=dontread
            )
            if readresults is None:
                return None

            ret = all([x[0] for x in readresults[0]])
            frames = [x[1] for x in readresults[0]]
            pcdframe = readresults[1]
            timestamp_img = readresults[2]
            timestamp_pcd = readresults[
                3
            ]  # 点群蓄積の影響で点群のタイムスタンプは画像タイムスタンプより遅れる
            self.prev_timestamp = timestamp_img
            if not ret:
                self._logger.warning(
                    "Warning!! cv cap returned False. Video or stream ends?"
                )
                return None
            if len(pcdframe) == 0:
                self._logger.warning(
                    "warning: pcd list size is zero"
                )  # LiDARの一時的なデータ欠損かも?

            # 読み込み時間表示
            if not dontread:
                self._logger.info(
                    "read: {timestamp_img}size: 3d, 2d:{pcdframe.shape},{frames[0].shape},dtime:{(datetime.now() - self.frame_prevtime)}"
                )
                self.frame_prevtime = datetime.now()

            elif timestamp_img % 1000 == 0:
                self._logger.info(
                    f"(skipping) read: {timestamp_img}size: 3d, 2d:{pcdframe.shape},{frames[0].shape}"
                )

            # 読み込みスキップ
            if timestamp_img < self.dataCaptureConf.s_frame:
                continue
            if timestamp_img > self.dataCaptureConf.e_frame:
                return None

            return (frames, timestamp_img), (pcdframe, timestamp_pcd)

    def _read_all_worker(self, data_capture_inst: data_capture, dontread=False):
        try:
            # dontread: 点群の読み込みをパスして空の点群を返す。フレームスキップ機能実装のため。
            data_cameras, data_lidars, framecounter = data_capture_inst.pop()
            returns_and_frames_and_timestamps = [
                self.cap2d.read(data_cameras, ix) for ix in range(len(data_cameras))
            ]

            ts_imgs = returns_and_frames_and_timestamps[0][2][0]
            pcd, ts_lidar_raw = self.cap3d.read(data_lidars, dontread=dontread)
            ts_lidar = ts_lidar_raw[0] - int(
                (self.accumulate_length - 1) / 2
            )  # インデックスで計算
            # ts_img = self.ts
            # ts_lidar = self.ts - int( (self.accumulate_length-1)/2 )
            self.ts += 1

            return [returns_and_frames_and_timestamps, pcd, ts_imgs, ts_lidar]
        except TypeError as e:
            self._logger.error(f"at _read_all_worker, {e}, skip ")
            return None

    def points_adjust_coord_calib2normal(self, points3d, convvec_index):
        return self.cap3d.adjust_rev_coordinate(
            points3d=points3d, convvec_index=convvec_index
        )

    def points_adjust_coord_normal2argus(self, points3d):
        return self.cap3d.Adjust_Lidar_data(points3d)
