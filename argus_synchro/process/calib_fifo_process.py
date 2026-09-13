from __future__ import annotations

import datetime
import time
from contextlib import ExitStack
from queue import Empty
from typing import cast

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.sensor_sync import (
    sensor_sync_filter,
)
from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.message.calib_fifo_message import FIFOData
from argus_synchro.message.input_message import CameraData, CanData, PointCloudData
from argus_synchro.process import ProcessBase
from argus_synchro.process.message import Consumer, MessageFlow, Producer
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_fps import ProfFps
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.shared_app_config import SharedAppConfig, SharedAppConfigCalibration
from argus_synchro.shared_excepts import SharedGetDataExcept

ALIVE_LOG_INTERVAL_SEC = 5.0


class CalibFIFOProcess(ProcessBase):
    __slots__ = (
        "_app_config",
        "_begin_time",
        "_buffersize_for_file",
        "_camera_count",
        "_camera_datalist",
        "_camera_inputs",
        "_camera_tsfilter_diff",
        "_can_input",
        "_debugmesg_sensors",
        "_end_frame",
        "_fifo_output",
        "_final_tsfilter",
        "_fps_prof",
        "_framethinning_bufferlen_threshold",
        "_input_readdone_ret",
        "_ispass_frame",
        "_last_print_time",
        "_last_queue_log_mono",
        "_last_updated",
        "_lidar_count",
        "_lidar_datalist",
        "_lidar_tsfilter_diff",
        "_loop_count",
        "_pcd_inputs",
        "_ref_t",
        "_sac",
        "_sac_calib",
    )

    def __init__(
        self,
        sec_get_data: SharedGetDataExcept,
        sac: SharedAppConfig,
        sac_calib: SharedAppConfigCalibration,
        camera_inputs: tuple[MessageFlow[CameraData], ...],
        pcd_inputs: tuple[MessageFlow[PointCloudData], ...],
        can_input: MessageFlow[CanData],
        fifo_output: MessageFlow[FIFOData],
        activator: ProcessActivator,
    ) -> None:
        super().__init__(sec_get_data, activator, "FIFOProcess")
        self._camera_inputs: tuple[MessageFlow[CameraData], ...] = tuple(
            self._subscribe(camera) for camera in camera_inputs
        )
        self._pcd_inputs: tuple[MessageFlow[PointCloudData], ...] = tuple(
            self._subscribe(pcd) for pcd in pcd_inputs
        )
        self._can_input: MessageFlow[CanData] = self._subscribe(
            can_input,
        )
        self._fifo_output: MessageFlow[FIFOData] = self._subscribe(
            fifo_output,
        )
        self._sac: SharedAppConfig = sac
        self._fps_prof: ProfFps = ProfFps(self.__class__.__name__)
        self._sac_calib: SharedAppConfigCalibration = sac_calib

        self._camera_count = len(camera_inputs)
        self._lidar_count = len(pcd_inputs)

        # _startupで初期化
        self._ref_t: int
        self._buffersize_for_file: int
        self._framethinning_bufferlen_threshold: int
        # bufferを超えたときに、frame rateを半分にするために使用
        self._ispass_frame: bool

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self._sac_calib.log_register(self._app_logger_factory)

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._app_config_calib: AppConfigCalibration = self._sac_calib.read()
        self._debugmesg_sensors: bool = (
            not self._app_config_calib.default.print_disabled
        )
        self._last_updated: int = self._sac.last_updated
        self._end_frame: int = self._app_config_calib.dataCapture.e_frame

    def _startup(self) -> None:
        self._config_load()
        self._ref_t = 1
        self.sensor_sync_filter_inst = sensor_sync_filter(
            synctype_select=self._app_config_calib.dataCapture.sync_type,
            app_logger_factory=self._app_logger_factory,
        )
        self._fps_prof.start()
        self._pre_sync()
        if self._app_config_calib.default.File_Input:
            self._buffersize_for_file = 10
        else:
            self._buffersize_for_file = (
                self._app_config_calib.dataCapture.Camera.data_buffersize
            )
        self._framethinning_bufferlen_threshold = (
            self._app_config_calib.dataCapture.Lidar.framethinning_bufferlen_threshold
        )
        self._ispass_frame = False
        self._last_queue_log_mono = 0.0
        self.create_producer_and_consumer()

    def create_producer_and_consumer(self) -> None:
        self.camera_consumers: tuple[Consumer[CameraData], ...] = tuple(
            camera.create_consumer() for camera in self._camera_inputs
        )
        self.pcd_consumers: tuple[Consumer[PointCloudData], ...] = tuple(
            pcd.create_consumer() for pcd in self._pcd_inputs
        )
        self.can_consumer: Consumer[CanData] = self._can_input.create_consumer()

        self.fifo_producer: Producer[FIFOData] = self._fifo_output.create_producer()

    def restart_completed(self) -> None:
        for i in self.camera_consumers:
            i.restart_completed()
        for i in self.pcd_consumers:
            i.restart_completed()
        self.can_consumer.restart_completed()

        self.fifo_producer.restart_completed()

    def _start_restart(self) -> None:
        self._config_load()
        # TODO """必要に応じて実際にプロセスを落とさないで再起動で実行する処理を記載""" (NSW)
        while True:
            try:
                time.sleep(0.1)
                self._fifo_output._message.read()
            except Empty:
                break
        self._ref_t = 1

        for i in self.camera_consumers:
            i.require_restart()
        del self.camera_consumers
        for i in self.pcd_consumers:
            i.require_restart()
        del self.pcd_consumers
        self.can_consumer.require_restart()
        del self.can_consumer

        self.fifo_producer.require_restart()
        del self.fifo_producer

    def _shutdown(self) -> None:
        self._fps_prof.export()
        # TODO privateを使用しないようにリファクタリング　(NSW)
        while True:
            try:
                self._fifo_output._message.read()
            except Empty:
                if self._fifo_output.qsize() > 0:
                    time.sleep(0.1)
                    continue
                break

    @log_main()
    def _loop(self) -> None:
        while self.enable:
            if self._sac.last_updated > self._last_updated:
                self._config_load()

            # TODO バッファが空くまでの時間待ち 時間は要検討　(NSW)
            qsize = self._fifo_output.qsize()
            if qsize is not None and qsize >= self._buffersize_for_file:
                time.sleep(0.1)
                continue
            now_mono = time.monotonic()
            if (
                not self._debugmesg_sensors
                and now_mono - self._last_queue_log_mono >= ALIVE_LOG_INTERVAL_SEC
            ):
                self._last_queue_log_mono = now_mono
                self._logger.info(
                    f"calibration FIFO alive: qsize={qsize}",
                )

            if not self.fifo_producer.wait():
                continue

            if (
                any(not c.wait() for c in self.camera_consumers)
                or any(not i.wait() for i in self.pcd_consumers)
                or not self.can_consumer.wait()
            ):
                continue

            if qsize is not None and qsize >= self._framethinning_bufferlen_threshold:
                # 閾値を超えていたら受け取らずに半分廃棄する。
                if self._ispass_frame:
                    # 次は読み込む
                    self._ispass_frame = False
                    continue
                # 次は読み込まない
                self._ispass_frame = True

            with ExitStack() as stack:
                # 入力処理
                cameras: tuple[CameraData, ...] = tuple(
                    stack.enter_context(c.consume()) for c in self.camera_consumers
                )
                pcds: tuple[PointCloudData, ...] = tuple(
                    stack.enter_context(c.consume()) for c in self.pcd_consumers
                )
                candata: CanData = stack.enter_context(self.can_consumer.consume())
                (
                    copied_cameras,
                    copied_pcds,
                    copied_candata,
                ) = self._copy_consumed_inputs(cameras, pcds, candata)

            # 前のloopで同期が成功した場合は、同期に必要な値を初期化する
            if self._input_readdone_ret:
                self._pre_sync()

            # 実際の処理
            output_data: FIFOData = self._update(
                copied_pcds,
                copied_candata,
                copied_cameras,
            )

            # NOTE 同期失敗時はFIFOに格納しないで次のFrameを読む
            if not self._input_readdone_ret:
                continue

            self._ref_t += 1

            self.fifo_producer.produce(output_data)

    def _copy_consumed_inputs(
        self,
        cameras: tuple[CameraData, ...],
        pcds: tuple[PointCloudData, ...],
        can_data: CanData,
    ) -> tuple[tuple[CameraData, ...], tuple[PointCloudData, ...], CanData]:
        copied_cameras: tuple[CameraData, ...] = tuple(
            CameraData(
                index=int(camera.index),
                frame=int(camera.frame),
                time=float(camera.time),
                image=camera.image.copy(),
            )
            for camera in cameras
        )
        copied_pcds: tuple[PointCloudData, ...] = tuple(
            PointCloudData(
                frame=int(pcd.frame),
                time=float(pcd.time),
                point_cloud=pcd.point_cloud.copy(),
            )
            for pcd in pcds
        )
        copied_can_data = CanData(
            yaw_angle_deg=float(can_data.yaw_angle_deg),
            lever_pressure=can_data.lever_pressure.copy(),
            frame=int(can_data.frame),
            time=float(can_data.time),
        )
        return copied_cameras, copied_pcds, copied_can_data

    @log_target("データ取得", ProfCategory.Process)
    def _update(
        self,
        pcd_input_data: tuple[PointCloudData, ...],
        can_input_data: CanData,
        camera_input_data: tuple[CameraData, ...],
    ) -> FIFOData:
        self._fps_prof.enter()
        if datetime.datetime.now() - self._begin_time > datetime.timedelta(seconds=5.0):
            self._logger.warning(
                f"data_capture.pop(): too long sync time: {datetime.datetime.now() - self._begin_time}, {self._final_tsfilter=}",
            )
            self._begin_time: datetime.datetime = datetime.datetime.now()
        if self._loop_count > 30:
            self._logger.warning(
                f"data_capture.pop(): too many frame to sync: {self._loop_count}, {self._final_tsfilter=}",
            )
            self._loop_count = 0

        # ---- カメラ ----
        for camera_ix, camera_input_data_ix in enumerate(camera_input_data):
            if self._camera_tsfilter_diff[camera_ix] == 0:
                continue
            self._camera_datalist[camera_ix] = (
                camera_input_data_ix.image,
                camera_input_data_ix.index,
                camera_input_data_ix.time,
            )

        # ---- LiDAR ----
        for lidar_ix, lidar_input_data_ix in enumerate(pcd_input_data):
            if self._lidar_tsfilter_diff[lidar_ix] == 0:
                continue
            self._lidar_datalist[lidar_ix] = (
                lidar_input_data_ix.point_cloud,
                lidar_input_data_ix.frame,
                lidar_input_data_ix.time,
            )
        self.can_data = (can_input_data.yaw_angle_deg, can_input_data.time)

        # ---- 同期評価 ----
        input_readdone_ret, input_tsfilter_diff, input_evaluate_value = (
            self.sensor_sync_filter_inst.datasync_filtering(
                self._camera_datalist + self._lidar_datalist,
                verbose=(not self._app_config_calib.default.print_disabled),
            )
        )
        self._input_readdone_ret = input_readdone_ret
        self._camera_tsfilter_diff[:] = input_tsfilter_diff[
            : len(self._camera_datalist)
        ]
        self._lidar_tsfilter_diff[:] = input_tsfilter_diff[len(self._camera_datalist) :]

        self._final_tsfilter = input_tsfilter_diff

        # self._logger.info(self, f"{input_readdone_ret=},{input_tsfilter_diff=},{input_evaluate_value=}")

        # if self._debugmesg_sensors:
        # self._logger.info( f"After compare: camera_tsfilter_diff ({id(camera_tsfilter_diff)}) = {camera_tsfilter_diff}")
        # self._logger.info(ter compare: self.lidar_tsfilter_diff ({id(self.lidar_tsfilter_diff)}) = {self.lidar_tsfilter_diff}")
        # self._logger.info(nput_readdone = }")

        current_time = datetime.datetime.now()

        if (current_time - self._last_print_time).total_seconds() >= 3:
            try:
                self._logger.warning(
                    f"入力待ち 3秒経過: {current_time.strftime('%H:%M:%S')}, timestamp:{[d[1:] for d in (self._camera_datalist + self._lidar_datalist)]}",
                )
            except Exception as e:
                self._logger.warning(f"入力待ち 3秒経過: タイムスタンプ取得失敗 {e}")
            self._last_print_time = current_time

        # self._logger.info( f"{input_readdone = }, {id(input_readdone) = }")

        # 指定した終了フレームまで進んだらプロセスを終了する
        if self._ref_t > self._end_frame:
            self._unsubscribe()

        self._fps_prof.prof(
            pcd_frame=self._ref_t,
            camera_frame=self._ref_t,
            pcd_s_time=pcd_input_data[0].time,
            camera_s_time=camera_input_data[0].time,
        )
        assert all(x is not None for x in self._camera_datalist)
        assert all(x is not None for x in self._lidar_datalist)
        camera: list[tuple[NDArray[np.uint8], int, float]] = cast(
            list[tuple[NDArray[np.uint8], int, float]], self._camera_datalist
        )
        lidar: list[tuple[NDArray[np.float64], int, float]] = cast(
            list[tuple[NDArray[np.float64], int, float]], self._lidar_datalist
        )
        return (
            camera,
            lidar,
            self.can_data,
            self._ref_t,
        )

    def _pre_sync(self) -> None:
        """同期前に必要な変数を初期化"""

        if self._debugmesg_sensors:
            self._logger.info("data_capture put() start")
        self._last_print_time: datetime.datetime = datetime.datetime.now()

        self._camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None] = [
            None for _ in range(self._camera_count)
        ]
        self._camera_tsfilter_diff = np.ones(
            self._camera_count, np.float64
        )  # <=0 0で最新データ。負：他のカメラに比べて遅れているので読み進める必要がある。初期状態は全部1として読ませる

        self._lidar_datalist: list[tuple[NDArray[np.float64], int, float] | None] = [
            None for _ in range(self._lidar_count)
        ]
        self._lidar_tsfilter_diff = np.ones(
            self._lidar_count, np.float64
        )  # <=0 0で最新データ。負：他のLiDARに比べて遅れているので読み進める必要がある。初期状態は全部1として読ませる

        self._input_readdone_ret = False

        if self._debugmesg_sensors:
            self._logger.info(f"qsizes:{self._fifo_output.qsize()}")

        self._final_tsfilter = []
        self._begin_time = datetime.datetime.now()
        self._loop_count = 0
