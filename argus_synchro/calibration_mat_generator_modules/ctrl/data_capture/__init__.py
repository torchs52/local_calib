import datetime
from queue import Empty

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.camera_capture import (
    MultiCameraManager,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.lidar_capture import (
    MultiLidarManager,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.sensor_sync import (
    sensor_sync_filter,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import (
    DataCaptureConf,
    DefaultConf,
)
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors
from argus_synchro.shared_excepts import SharedExcepts


class data_capture:
    # カメラ・LiDAR・CANセンサ信号を同期を取りながら入力。
    # このクラスは各センサのインスタンス（別プロセス）を持ち、sensor_syncクラスの関数でタイムスタンプ同期を取りながらデータを入力。
    def __init__(
        self,
        DefaultConfig: DefaultConf,
        DataCaptureConfig: DataCaptureConf,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        verbose: bool,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors | None = None,
    ):
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        print("data_capture __init__")
        self.cameras = MultiCameraManager(
            DefaultConfig,
            DataCaptureConfig,
            sec=sec,
            sac=sac,
            app_logger_factory=app_logger_factory,
        )
        self.lidars = MultiLidarManager(
            DefaultConfig,
            DataCaptureConfig,
            sec=sec,
            sac=sac,
            app_logger_factory=app_logger_factory,
            shared_errors=shared_errors,
        )
        self.sensor_sync_filter_inst = sensor_sync_filter(
            synctype_select=DataCaptureConfig.sync_type,
            app_logger_factory=app_logger_factory,
        )
        self.sec: SharedExcepts = sec
        self.sac: SharedAppConfig = sac

        self.cameras.run()
        self.lidars.run()
        self.DefaultConfig: DefaultConf = DefaultConfig
        self.verbose = verbose

        self.debugmesg_sensors: bool = not DefaultConfig.print_disabled
        self.framecounter: int = 0

        self.end_reading_mesgcount: int = 0

        # 別プロセス実行対応後ここで_task_independent_process起動
        print("data_capture __init__ end")

    def __del__(self) -> None:
        self.force_close()

    def _task_independent_process(self):
        raise NotImplementedError("別プロセス実行は未サポート")

    def is_end(self):
        return self.cameras.is_end()[0] and self.lidars.is_end()[0]

    def is_end_any(self) -> bool:
        returnflag: bool = self.cameras.is_end_any()[0] or self.lidars.is_end_any()[0]
        if self.verbose and returnflag:
            self._logger.info(
                f"is_end_any: {self.cameras.is_end_any() = }, {self.lidars.is_end_any() = }",
            )
        return returnflag

    def graceful_close(self):
        self.cameras.prepare_close(wait_for_worker=True, wait_timeout=0.5)
        self.lidars.prepare_close(wait_for_worker=True, wait_timeout=0.5)

        self.cameras.join_all(timeout=1)
        self.lidars.join_all(timeout=1)

        self.cameras.terminate_all()
        self.lidars.terminate_all()

    def force_close(self):
        # 1) 強制終了（ブロッキングI/Oでも落ちる）
        self.cameras.terminate_all()
        self.lidars.terminate_all()

        # 2) 終了待ち（短時間）
        self.cameras.join_all(timeout=1)
        self.lidars.join_all(timeout=1)

        # 3) キュー片付け（短時間だけ or スキップ可）
        try:
            self.cameras.prepare_close(wait_for_worker=False)
            self.lidars.prepare_close(wait_for_worker=False)
        except Exception:
            pass

    def bufferclear_and_reset_index(self):
        if not self.DefaultConfig.print_disabled:
            self._logger.info("bufferclear_and_reset_index")

        if self.DefaultConfig.File_Input:
            self.graceful_close()

        self.cameras.reset_all_index()
        self.lidars.reset_all_index()

        # バッファを空にする：　タイムスタンプが継続して大きくなっていたら読み飛ばす 0になったら終了（次読むときは1からになる）
        for camera_ix, queue in enumerate(self.cameras.QueueList):
            pref_ts = 0
            while queue.qsize() > 1:
                dat: tuple[np.ndarray, int, float] = queue.get()
                if dat[1] < pref_ts:
                    break
                pref_ts = dat[1]

        for lidar_ix, queue in enumerate(self.lidars.QueueList):
            pref_ts = 0
            while queue.qsize() > 1:
                dat: tuple[np.ndarray, int, float] = queue.get()
                if dat[1] < pref_ts:
                    break
                pref_ts = dat[1]

        if self.DefaultConfig.File_Input:
            self.cameras.run()
            self.lidars.run()

    def keep_loop_condition(self) -> bool:
        returnflag: bool = not (
            self.sec.CalMatGen_ex.IsFinished.value or self.is_end_any()
        )
        if self.verbose and (not returnflag):
            self._logger.info(
                f"{self.sec.CalMatGen_ex.IsFinished.value = }, {self.is_end_any() = }",
            )
        return returnflag

    def drain_allqueue(self):
        for queue in self.cameras.QueueList:
            try:
                queue.get_nowait()
            except Empty:
                pass

        # ---- LiDAR ----
        for queue in self.lidars.QueueList:
            try:
                queue.get_nowait()
            except Empty:
                pass

    def pop(
        self,
    ) -> tuple[
        list[tuple[NDArray[np.uint8], int, float] | None],
        list[tuple[NDArray[np.float32], int, float] | None],
        int,
    ]:  # camera datalistとlidar datalistを返す。データが無い（読み終わり等）場合はNone、それ以外は各カメラ・LiDARごとに(data, ts_ix, ts_time)を返す
        if not self.DefaultConfig.print_disabled:
            self._logger.info("data_capture pop() start")
        last_print_time: datetime.datetime = datetime.datetime.now()

        camera_datalist: list[tuple[NDArray[np.uint8], int, float] | None] = [
            None for _ in range(len(self.cameras.QueueList))
        ]
        camera_tsfilter_diff = np.ones(
            len(self.cameras.QueueList), np.float64
        )  # <=0 0で最新データ。負：他のカメラに比べて遅れているので読み進める必要がある。初期状態は全部1として読ませる

        lidar_datalist: list[tuple[NDArray[np.float32], int, float] | None] = [
            None for _ in range(len(self.lidars.QueueList))
        ]

        lidar_tsfilter_diff = np.ones(
            len(self.lidars.QueueList), np.float64
        )  # <=0 0で最新データ。負：他のLiDARに比べて遅れているので読み進める必要がある。初期状態は全部1として読ませる

        input_readdone = False

        if self.debugmesg_sensors:
            self._logger.info(
                f"qsizes:{[f'{camera_ix = }:{queue.qsize()}' for camera_ix, queue in enumerate(self.cameras.QueueList)]}{[f'{lidar_ix = }:{queue.qsize()}' for lidar_ix, queue in enumerate(self.lidars.QueueList)]}"
            )

        # このループで長くとどまっているのは何か異常が疑われる
        final_tsfilter = []
        begin_time = datetime.datetime.now()
        loop_count = 0
        while (not input_readdone) and self.keep_loop_condition():
            if datetime.datetime.now() - begin_time > datetime.timedelta(seconds=5.0):
                self._logger.warning(
                    f"data_capture.pop(): too long sync time: {datetime.datetime.now() - begin_time}, {final_tsfilter=}",
                )
                begin_time = datetime.datetime.now()
            if loop_count > 30:
                self._logger.warning(
                    f"data_capture.pop(): too many frame to sync: {loop_count}, {final_tsfilter=}",
                )
                loop_count = 0

            # ---- カメラ ----
            for camera_ix, queue in enumerate(self.cameras.QueueList):
                if camera_tsfilter_diff[camera_ix] == 0:
                    continue
                try:
                    # まずは即時
                    camera_datalist[camera_ix] = queue.get_nowait()
                except Empty:
                    # 来てなければ超短タイムアウトでブロック
                    try:
                        camera_datalist[camera_ix] = queue.get(timeout=0.003)  # 3ms
                    except Empty:
                        pass

            # ---- LiDAR ----
            for lidar_ix, queue in enumerate(self.lidars.QueueList):
                if lidar_tsfilter_diff[lidar_ix] == 0:
                    continue
                try:
                    lidar_datalist[lidar_ix] = queue.get_nowait()
                except Empty:
                    try:
                        lidar_datalist[lidar_ix] = queue.get(timeout=0.003)
                    except Empty:
                        pass

            # ---- 同期評価 ----
            input_readdone_ret, input_tsfilter_diff, input_evaluate_value = (
                self.sensor_sync_filter_inst.datasync_filtering(
                    camera_datalist + lidar_datalist,
                    verbose=(not self.DefaultConfig.print_disabled),
                )
            )
            input_readdone = input_readdone_ret
            camera_tsfilter_diff[:] = input_tsfilter_diff[: len(camera_datalist)]
            lidar_tsfilter_diff[:] = input_tsfilter_diff[len(camera_datalist) :]

            final_tsfilter = input_tsfilter_diff

            # self._logger.info(self, f"{input_readdone_ret=},{input_tsfilter_diff=},{input_evaluate_value=}")

            # if self.debugmesg_sensors:
            # self._logger.info( f"After compare: camera_tsfilter_diff ({id(camera_tsfilter_diff)}) = {camera_tsfilter_diff}")
            # self._logger.info(ter compare: lidar_tsfilter_diff ({id(lidar_tsfilter_diff)}) = {lidar_tsfilter_diff}")
            # self._logger.info(nput_readdone = }")

            current_time = datetime.datetime.now()

            if (current_time - last_print_time).total_seconds() >= 3:
                try:
                    self._logger.warning(
                        f"入力待ち 3秒経過: {current_time.strftime('%H:%M:%S')}, timestamp:{[d[1:] for d in (camera_datalist + lidar_datalist)]}",
                    )
                except Exception as e:
                    self._logger.warning(
                        f"入力待ち 3秒経過: タイムスタンプ取得失敗 {e}"
                    )
                last_print_time = current_time

            # self._logger.info( f"{input_readdone = }, {id(input_readdone) = }")
            continue

        if not self.keep_loop_condition():
            if self.end_reading_mesgcount > 30:
                self._logger.info("*** end reading ***")
                self.end_reading_mesgcount = 0
            else:
                self.end_reading_mesgcount += 1
        else:
            self.end_reading_mesgcount = 0

        self.framecounter += 1
        if camera_datalist[0] is not None:
            self.framecounter = camera_datalist[0][1]

        return camera_datalist, lidar_datalist, self.framecounter
