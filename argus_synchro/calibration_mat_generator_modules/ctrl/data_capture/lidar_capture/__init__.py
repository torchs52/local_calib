from __future__ import annotations

import datetime
import multiprocessing
import multiprocessing.sharedctypes
import time
import traceback
from queue import Empty

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.lidar_capture.lidar_capture_tool import (
    read_singleLidar_file,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.lidar_capture.mid360 import (
    mid360,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.data_capture.MultiSensorBase import (
    MultiSensorManagerBase,
    MultiSensorWorkerBase,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import (
    DataCaptureConf,
    DefaultConf,
)
from argus_synchro.config.fileinput_pathselector import (
    lidar_filepath_loader,
)
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors, StateErrorDIndex
from argus_synchro.shared_excepts import SharedExcepts

SENTINEL = (np.zeros((0, 0), dtype=np.float32), -1, -1.0)


class MultiLidarManager(MultiSensorManagerBase):
    def __init__(
        self,
        DefaultConfig: DefaultConf,
        DataCaptureConfig: DataCaptureConf,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors | None = None,
    ):
        # タスク管理関係
        self.DefaultConfig: DefaultConf = DefaultConfig
        self.DataCaptureConfig: DataCaptureConf = DataCaptureConfig
        self.LidarConfig: DataCaptureConf.LidarConf = DataCaptureConfig.Lidar
        super().__init__(
            num_processes=self.LidarConfig.count,
            app_logger_factory=app_logger_factory,
        )

        self.print_disabled: bool = DefaultConfig.print_disabled
        self.QueueList: list[
            multiprocessing.Queue[tuple[NDArray[np.float32], int, float]]
        ] = []
        self.working_task_flags: multiprocessing.sharedctypes.SynchronizedArray[
            bool
        ] = multiprocessing.Array("b", self.LidarConfig.count)

        self._logger.info(
            f"** MultiCameraManager->__init__ ** {len(self.QueueList) = }, {len(self.working_task_flags)}, {self.DataCaptureConfig.Lidar.count = }",
        )

        self.sec: SharedExcepts = sec
        self.sac: SharedAppConfig = sac
        self._ser = shared_errors

        self.index_reset_flag: multiprocessing.sharedctypes.SynchronizedArray[bool] = (
            multiprocessing.Array("b", self.LidarConfig.count)
        )
        for ix in range(self.LidarConfig.count):
            self.index_reset_flag[ix] = True

        self.lidars: list[mid360] | None = None
        # 設定値読み込み
        if self.DefaultConfig.File_Input is True:
            self.portdictlist = [{"hoge": 000}]
        else:
            self.portdictlist: list[dict] = [
                {"pnt": 56304, "imu": 56404},
                {"pnt": 56303, "imu": 56403},
            ]

    def add_taskinfo(self, task_index: int, argsK: dict):
        self.QueueList.append(multiprocessing.Queue())
        self.working_task_flags[task_index] = True
        print(
            # task_index, argsK, " task start", self.LidarConfig.lidar_files[task_index]
            task_index,
            argsK,
            " task start",
            lidar_filepath_loader(sac=self.sac, lidarConf=self.DataCaptureConfig.Lidar)[
                task_index
            ],
        )

        argsK["DefaultConfig"] = self.DefaultConfig
        argsK["DataCaptureConfig"] = self.DataCaptureConfig
        argsK["infoQueue"] = self.QueueList[-1]
        argsK["working_task_flags"] = self.working_task_flags
        argsK["portdictlist"] = self.portdictlist
        argsK["loopEnable"] = self.loopEnable
        argsK["print_disabled"] = self.print_disabled
        argsK["index_reset_req"] = self.index_reset_flag
        argsK["sec"] = self.sec
        argsK["sac"] = self.sac
        argsK["shared_errors"] = self._ser
        return argsK

    def check_running(self, task_index):
        return (
            self.working_task_flags[task_index],
            self.QueueList[task_index].qsize(),
        )  # [0]:カメラデータ入力中 [1]:バッファデータあり

    def prepare_close(self, wait_for_worker: bool = False, wait_timeout: float = 0.2):
        self.loopEnable.value = False
        for ix, dq in enumerate(self.QueueList):
            self._logger.info(f"camera {ix}: closing")
            if wait_for_worker:
                deadline = time.time() + wait_timeout
                while self.working_task_flags[ix] and time.time() < deadline:
                    self._logger.info(f"camera {ix}: read end waiting")
                    time.sleep(0.1)
            # ★ キューは Empty まで捨て切る（センチネル含む）
            dropped = 0
            deadline = time.time() + wait_timeout
            while time.time() < deadline:
                try:
                    dq.get_nowait()
                    dropped += 1
                except Empty:
                    break
            self._logger.info(f"camera {ix}: queue emptied (dropped={dropped})")

    def terminate_all(self):
        # ★ prepare_close() は呼ばない（まず kill）
        super().terminate_all()
        try:
            del self.QueueList
        except Exception:
            pass
        self.QueueList = []

    def join_all(self, timeout=None):
        # ★ join は join だけ（prepare_close は呼ばない）
        return super().join_all(timeout)

    def is_end(
        self,
    ) -> tuple[bool, dict]:  # いずれかの点群が読み込み中 or キューが空でない場合はFalse
        endflag = True
        status_dict = {}
        for task_index, image_queue in enumerate(self.QueueList):
            lidar_workingflag: bool = self.working_task_flags[task_index]
            if lidar_workingflag is True:
                endflag = False
                status_dict[f"lidar{task_index}"] = "reading"

            elif image_queue.qsize() != 0:
                endflag = False
                status_dict[f"lidar{task_index}"] = "queue not empty"

            else:
                print(f"lidar{task_index} - end")

        return endflag, status_dict

    def is_end_any(
        self,
    ) -> tuple[bool, dict]:  # いずれかの点群が読み込み中 or キューが空でない場合はFalse
        endflag_any = False
        status_dict = {}
        for task_index, lidar_queue in enumerate(self.QueueList):
            lidar_workingflag: bool = self.working_task_flags[task_index]
            if lidar_workingflag is False:
                endflag_any = True
                status_dict[f"lidar{task_index}"] = "end"
                if not self.print_disabled:
                    self._logger.warning(f"lidar{task_index}: end")

        return endflag_any, status_dict

    def run(self, argsK_list: list[dict] | None = None, target=None) -> None:
        if argsK_list is None:
            argsK_list = [{"task_index": ix} for ix in range(self.LidarConfig.count)]
        else:
            for ix in range(len(argsK_list)):
                argsK_list[ix]["task_index"] = ix
        super().run(
            argsK_list=argsK_list,
            target=MultiLidarWorker.entrypoint,
            name_suffix="MultiLidarWorker",
        )

    def reset_all_index(self):
        for ix in range(len(self.index_reset_flag)):
            self.index_reset_flag[ix] = True


class MultiLidarWorker(MultiSensorWorkerBase):
    def __init__(
        self,
        loopEnable: multiprocessing.sharedctypes.Synchronized,
        DefaultConfig: DefaultConf,
        DataCaptureConfig: DataCaptureConf,
        infoQueue: multiprocessing.Queue[tuple[NDArray[np.float32], int, float]],
        working_task_flags: multiprocessing.sharedctypes.SynchronizedArray[bool],
        portdictlist: list[dict],
        print_disabled: bool,
        task_index: int,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        index_reset_req: multiprocessing.sharedctypes.SynchronizedArray[bool],
        app_logger_factory: AppLoggerFactory,
        shared_errors: SharedErrors | None = None,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.LidarConfig: DataCaptureConf.LidarConf = DataCaptureConfig.Lidar
        self.DefaultConfig: DefaultConf = DefaultConfig
        self.startframe: int = DataCaptureConfig.s_frame
        self.endframe: int = DataCaptureConfig.e_frame
        self.infoQueue: multiprocessing.Queue[
            tuple[NDArray[np.float32], int, float]
        ] = infoQueue
        self.loopEnable = loopEnable
        self.print_disabled = print_disabled
        self.working_task_flags: multiprocessing.sharedctypes.SynchronizedArray[
            bool
        ] = working_task_flags
        self.portdictlist: list[dict] = portdictlist
        self.task_index: int = task_index
        self.index_reset_req: multiprocessing.sharedctypes.SynchronizedArray[bool] = (
            index_reset_req
        )
        self.sec: SharedExcepts = sec
        self.sac: SharedAppConfig = sac
        self._ser = shared_errors
        super().__init__()

    def keep_loop_condition(self, task_index: int) -> bool:
        return (
            self.loopEnable.value != 0
            and self.working_task_flags[task_index] != 0
            and (not self.sec.CalMatGen_ex.IsFinished.value)
        )

    def _report_file_io_error(
        self, path: str, operation: str, error: Exception
    ) -> None:
        if self._ser is None:
            return
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        result = file_io_error.errors_diagnosis(True)
        file_io_error.log_output(
            *result,
            StateErrorDIndex.FILE_IO_ERROR,
            path,
            operation,
            f"{type(error).__name__}: {error}",
        )

    def _fileinput_task(
        self, task_index: int
    ) -> (
        None
    ):  # allow_lack=Trueで点群ファイル不在を許容 0スタートだとONにしないと落ちるかも
        try:
            buffersize_for_file = 10
            basetime = datetime.datetime.strptime(
                self.LidarConfig.lidarfile_basetime, "%Y-%m-%d_%H-%M-%S"
            )
            steptime = self.LidarConfig.lidarfile_steptime

            timestamp_ix: int = 0
            assert self.LidarConfig.count > task_index

            timestamp_ix = self.startframe

            self.working_task_flag = 1

            while timestamp_ix < self.endframe and self.keep_loop_condition(
                task_index=task_index
            ):
                if self.index_reset_req[task_index]:
                    timestamp_ix = 0
                    self.index_reset_req[task_index] = False
                    if not self.print_disabled:
                        self._logger.info(
                            f"_fileinput_task: reset index to {timestamp_ix} (lidar {task_index})",
                        )

                if not self.print_disabled:
                    self._logger.info(
                        f"_fileinput_task, {task_index = }, {self.infoQueue.qsize() = }",
                    )

                while (
                    self.infoQueue.qsize() >= buffersize_for_file
                    and self.keep_loop_condition(task_index=task_index)
                ):
                    time.sleep(
                        self.LidarConfig.capture_latency_ms / 1000.0 + 0.01
                    )  # 0.01s~ の程よいポーリング（キャプチャレイテンシ時間はあくまで目安として使用）

                if not self.keep_loop_condition(task_index=task_index):
                    break  # while文を抜けた理由が継続条件満たさなかった場合は終了

                frame = read_singleLidar_file(
                    timestamp_ix,
                    lidar_filepath_loader(sac=self.sac, lidarConf=self.LidarConfig)[
                        task_index
                    ],  # self.LidarConfig.lidar_files[task_index]
                    file_io_error_reporter=self._report_file_io_error,
                )
                if frame is None:
                    print(f"lidar{task_index} : cannot read")
                    if self.LidarConfig.allow_lack is True:
                        frame = np.zeros((0, 4))

                    else:
                        self._logger.info(
                            f"frame None:{self.working_task_flags[task_index] = }"
                        )
                        break

                # print(datetime.datetime.now(), args, kwargs)
                self.infoQueue.put(
                    (
                        frame,
                        timestamp_ix,
                        (
                            basetime
                            + datetime.timedelta(seconds=steptime * timestamp_ix)
                        ).timestamp(),
                    )
                )

                if self.LidarConfig.capture_latency_ms > 0:
                    time.sleep(self.LidarConfig.capture_latency_ms / 1000.0)
                timestamp_ix += 1

        finally:
            self._logger.info(f"lidar{task_index}: points end, close")
            # self.loopEnable.value = False
            # self.working_task_flags[task_index] = False
            # センチネルを先に入れてからフラグを折る
            self.infoQueue.put(SENTINEL)
            self.working_task_flags[task_index] = False

    def _sensorinput_task(self, task_index: int):
        print(
            f"lidar capture -> _sensorinput_task, starting... connection information: {self.portdictlist[task_index]}"
        )
        mid360_inst = mid360("192.168.1.1", self.portdictlist[task_index], False)

        timestamp_ix = self.startframe

        self.working_task_flags[task_index] = True

        try:
            while self.keep_loop_condition(task_index=task_index):
                accumtime_orig = 0.1  # TODO: config追加
                accumtime = accumtime_orig

                if self.index_reset_req[task_index]:
                    timestamp_ix = 0
                    self.index_reset_req[task_index] = False
                    if not self.print_disabled:
                        self._logger.info(
                            f"_sensorinput_task: reset index (lidar {task_index})"
                        )

                if not self.print_disabled:
                    self._logger.info(
                        f"_sensorinput_task, {task_index = }, {self.infoQueue.qsize() = }",
                    )

                while (
                    self.infoQueue.qsize() >= self.LidarConfig.data_buffersize
                    and self.keep_loop_condition(task_index=task_index)
                ):
                    time.sleep(
                        self.LidarConfig.capture_latency_ms / 1000.0 + 0.01
                    )  # 0.01s~ の程よいポーリング（キャプチャレイテンシ時間はあくまで目安として使用） バッファが空くまでの時間待ち この待ち方では点群取り逃すので修正が必要： TODO
                    accumtime = (
                        accumtime_orig
                        + self.LidarConfig.capture_latency_ms / 1000.0
                        + 0.01
                    )
                if not self.keep_loop_condition(task_index=task_index):
                    break  # while文を抜けた理由が継続条件満たさなかった場合は終了

                if (
                    self.infoQueue.qsize()
                    >= self.LidarConfig.framethinning_bufferlen_threshold
                ):  # バッファが閾値以上埋まっている場合はフレームレート半分に
                    self._logger.warning(
                        f"_sensorinput_task framerate 1/2, {task_index = }, {self.infoQueue.qsize() = }",
                    )
                    accumtime = accumtime_orig * 2

                frame: NDArray[np.float32] = mid360_inst.get_points(
                    max_accum_time=accumtime
                )
                frame = frame[frame[:, 3] != 0]  # 輝度ゼロの点群を弾く

                if (
                    self.infoQueue.qsize()
                    >= self.LidarConfig.framethinning_bufferlen_threshold
                ):
                    frame = frame[::2]  # 間引く

                if frame is None:
                    self._logger.info(f"lidar{task_index} : cannot read")
                    if self.LidarConfig.allow_lack is True:
                        frame: NDArray[np.float32] = np.zeros((0, 4))
                    else:
                        # self.working_task_flags[task_index] = False
                        self._logger.info(
                            f"frame:{frame},{self.working_task_flags[task_index]}"
                        )
                        break

                # print(datetime.datetime.now(), args, kwargs)
                timestamp_curtime = time.time()
                self.infoQueue.put((frame, timestamp_ix, timestamp_curtime))
                if not self.print_disabled:
                    self._logger.info(
                        f"lidar {task_index} read, ix {timestamp_ix}, time:{datetime.datetime.fromtimestamp(timestamp_curtime)}",
                    )

                timestamp_ix += 1
        except Exception as e:
            self._logger.error(
                f"lidar capture -> _sensorinput_task: exception! {e} - \n{traceback.format_exc()}",
            )

        finally:
            print("lidar capture -> _sensorinput_task end")
            try:
                mid360_inst.disconnect_mid360()
            except Exception as e:
                self._logger.error(f"mid360_inst (lidar {task_index}): Exception - {e}")

            # センチネルを先に入れてからフラグを折る
            self.infoQueue.put(SENTINEL)
            self.working_task_flags[task_index] = False

    def task(self):
        if self.DefaultConfig.File_Input is True:
            self._fileinput_task(self.task_index)
        else:
            self._sensorinput_task(self.task_index)
        self._logger.info(f"lidar {self.task_index} read task ended")
