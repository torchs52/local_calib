from __future__ import annotations

import datetime
import multiprocessing
import multiprocessing.sharedctypes
import time
import traceback
from queue import Empty

import cv2
import numpy as np
from numpy.typing import NDArray

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
    video_filepath_loader,
)
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_excepts import SharedExcepts

SENTINEL = (None, -1, -1.0)


class MultiCameraManager(MultiSensorManagerBase):
    def __init__(
        self,
        DefaultConfig: DefaultConf,
        DataCaptureConfig: DataCaptureConf,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
    ):
        # タスク管理関係
        self.DefaultConfig: DefaultConf = DefaultConfig
        self.DataCaptureConfig: DataCaptureConf = DataCaptureConfig
        self.is_preparing: bool = True  # プロセス開始や起動に時間が掛かるため起動中は終了フラグ対策が必要（メイン側）

        super().__init__(
            num_processes=DataCaptureConfig.Camera.count,
            app_logger_factory=app_logger_factory,
        )
        self.QueueList: list[
            multiprocessing.Queue[tuple[NDArray[np.uint8], int, float]]
        ] = []
        self.working_task_flags: multiprocessing.sharedctypes.SynchronizedArray[
            bool
        ] = multiprocessing.Array("b", self.DataCaptureConfig.Camera.count)

        self._logger.info(
            f"** MultiCameraManager->__init__ **{len(self.QueueList) = }, {len(self.working_task_flags)}, {self.DataCaptureConfig.Camera.count = }",
        )

        self.index_reset_flag: multiprocessing.sharedctypes.SynchronizedArray[bool] = (
            multiprocessing.Array("b", self.DataCaptureConfig.Camera.count)
        )
        for ix in range(self.DataCaptureConfig.Camera.count):
            self.index_reset_flag[ix] = True

        self.sec: SharedExcepts = sec
        self.sac: SharedAppConfig = sac

        self.print_disabled: bool = self.DefaultConfig.print_disabled
        # 設定値読み込み
        if self.DefaultConfig.File_Input is True:
            self.rtp_urls: list[str] = [""]
        else:
            self.rtp_urls = [
                "rtp://192.168.1.75:10750",
                "rtp://192.168.1.78:10780",
                "rtp://192.168.1.77:10770",
            ]

    def run(self, argsK_list: list[dict] | None = None, target=None) -> None:
        if argsK_list is None:
            argsK_list = [
                {"task_index": ix} for ix in range(self.DataCaptureConfig.Camera.count)
            ]
        super().run(
            argsK_list=argsK_list,
            target=MultiCameraWorker.entrypoint,
            name_suffix="MultiCameraWorker",
        )
        self.is_preparing = False

    def add_taskinfo(self, ix: int, argsK: dict) -> dict:
        self.QueueList.append(multiprocessing.Queue())
        self.working_task_flags[ix] = True
        # print(ix, argsK, " task start", self.DataCaptureConfig.Camera.video_files[ix])
        print(
            ix,
            argsK,
            " task start",
            video_filepath_loader(
                sac=self.sac, cameraConf=self.DataCaptureConfig.Camera
            )[ix],
        )

        argsK["DefaultConfig"] = self.DefaultConfig
        argsK["loopEnable"] = self.loopEnable
        argsK["DataCaptureConfig"] = self.DataCaptureConfig
        argsK["infoQueue"] = self.QueueList[-1]
        argsK["working_task_flags"] = self.working_task_flags
        argsK["rtp_urls"] = self.rtp_urls
        argsK["print_disabled"] = self.print_disabled
        argsK["index_reset_req"] = self.index_reset_flag
        argsK["sec"] = self.sec
        argsK["sac"] = self.sac
        return argsK

    def check_running(self, task_index: int) -> tuple[bool, int]:
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
    ) -> tuple[
        bool, dict
    ]:  # いずれかの動画が読み込み中 or キューが空でない場合はFalse。
        if self.is_preparing:
            return False, {}
        endflag = True
        status_dict = {}
        # print(f"camera - isend? {len(self.working_task_flags), len(self.QueueList)}")
        for task_index in range(self.DataCaptureConfig.Camera.count):
            (camera_workingflag, image_queue) = (
                self.working_task_flags[task_index],
                self.QueueList[task_index],
            )
            # print(f"check for {task_index}, {camera_workingflag}, {image_queue.qsize()}")
            if camera_workingflag != 0:
                endflag = False
                status_dict[f"camera{task_index}"] = "reading"

            elif image_queue.qsize() > 0:
                endflag = False
                status_dict[f"camera{task_index}"] = "queue not empty"

            # else:
            #    print(f"camera{task_index} - end")
        # print(f"is_end decision: {endflag, status_dict}")
        return endflag, status_dict

    def is_end_any(self) -> tuple[bool, dict]:  # いずれかの動画が読み込み終了ならTrue
        if self.is_preparing:
            return False, {}
        endflag_any = False
        status_dict = {}
        # print(f"camera - isend? {len(self.working_task_flags), len(self.QueueList)}")
        for task_index in range(self.DataCaptureConfig.Camera.count):
            (camera_workingflag, _image_queue) = (
                self.working_task_flags[task_index],
                self.QueueList[task_index],
            )
            # print(f"check for {task_index}, {camera_workingflag}, {image_queue.qsize()}")
            if camera_workingflag == 0:
                endflag_any = True
                status_dict[f"camera{task_index}"] = "end"

            # else:
            #    print(f"camera{task_index} - end")
        # print(f"is_end decision: {endflag, status_dict}")
        return endflag_any, status_dict

    def reset_all_index(self):
        for ix in range(len(self.index_reset_flag)):
            self.index_reset_flag[ix] = True


class MultiCameraWorker(MultiSensorWorkerBase):
    def __init__(
        self,
        loopEnable: multiprocessing.sharedctypes.Synchronized,
        DefaultConfig: DefaultConf,
        DataCaptureConfig: DataCaptureConf,
        infoQueue: multiprocessing.Queue[tuple[NDArray[np.uint8], int, float]],
        working_task_flags: multiprocessing.sharedctypes.SynchronizedArray[bool],
        rtp_urls: list[str],
        print_disabled: bool,
        task_index: int,
        sec: SharedExcepts,
        sac: SharedAppConfig,
        index_reset_req: multiprocessing.sharedctypes.SynchronizedArray[bool],
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.CameraConfig: DataCaptureConf.CameraConf = DataCaptureConfig.Camera
        self.DefaultConfig: DefaultConf = DefaultConfig
        self.DataCaptureConfig: DataCaptureConf = DataCaptureConfig
        self.startframe: int = DataCaptureConfig.s_frame
        self.endframe: int = DataCaptureConfig.e_frame
        self.infoQueue = infoQueue
        self.loopEnable = loopEnable
        self.print_disabled = print_disabled
        self.working_task_flags: multiprocessing.sharedctypes.SynchronizedArray[
            bool
        ] = working_task_flags
        self.rtp_urls: list[str] = rtp_urls
        self.task_index: int = task_index
        self.working_task_flags[task_index] = True
        self.sec: SharedExcepts = sec
        self.sac: SharedAppConfig = sac
        self.index_reset_req: multiprocessing.sharedctypes.SynchronizedArray[bool] = (
            index_reset_req
        )

    def keep_loop_condition(self, task_index: int) -> bool:
        return (
            self.loopEnable.value != 0
            and self.working_task_flags[task_index] != 0
            and (not self.sec.CalMatGen_ex.IsFinished.value)
        )

    def _fileinput_task(self, task_index: int):
        try:  # これを忘れるとCtrl+C等で返ってこなくなる
            buffersize_for_file = 10
            basetime = datetime.datetime.strptime(
                self.CameraConfig.videofile_basetime, "%Y-%m-%d_%H-%M-%S"
            )
            steptime = self.CameraConfig.videofile_steptime

            timestamp_ix: int = 0
            assert self.DataCaptureConfig.Camera.count > task_index
            # self._logger.ts_info(self, "video open:", self.DataCaptureConfig.Camera.video_files[task_index])
            self._logger.info(
                f"video open:{video_filepath_loader(sac=self.sac, cameraConf=self.DataCaptureConfig.Camera)[task_index]}"
            )
            videofile = video_filepath_loader(
                sac=self.sac, cameraConf=self.DataCaptureConfig.Camera
            )[task_index]
            VideoCapture_inst = cv2.VideoCapture(videofile)
            if not VideoCapture_inst.isOpened():
                self._logger.critical(f"Cannot load file: {videofile}")
                raise RuntimeError(f"Cannot load file: {videofile}")

            self._logger.info(
                f"video length:{VideoCapture_inst.get(cv2.CAP_PROP_FRAME_COUNT)}"
            )
            timestamp_ix = self.startframe
            timestamp_ix = max(timestamp_ix, 0)

            VideoCapture_inst.set(cv2.CAP_PROP_POS_FRAMES, int(timestamp_ix))
            self._logger.info(f"video open: {VideoCapture_inst.isOpened()}")

            while timestamp_ix < self.endframe and self.keep_loop_condition(
                task_index=task_index
            ):
                if self.index_reset_req[task_index]:
                    timestamp_ix = self.startframe
                    VideoCapture_inst.set(cv2.CAP_PROP_POS_FRAMES, int(timestamp_ix))
                    self.index_reset_req[task_index] = False
                    if not self.print_disabled:
                        self._logger.info(
                            f"_fileinput_task: reset index to {timestamp_ix} (camera {task_index})",
                        )

                if not self.print_disabled:
                    self._logger.info(
                        f"_fileinput_task, {task_index = }, {self.infoQueue.qsize() = }",
                    )

                while (
                    self.infoQueue.qsize() >= buffersize_for_file
                    and self.keep_loop_condition(task_index=task_index)
                ):  # バッファが空くまでの時間待ち
                    time.sleep(
                        self.DataCaptureConfig.Camera.capture_latency_ms / 1000.0
                    )  # 0.01s~ の程よいポーリング（キャプチャレイテンシ時間はあくまで目安として使用）

                if not self.keep_loop_condition(task_index=task_index):
                    break  # while文を抜けた理由が継続条件満たさなかった場合は終了

                result, frame = VideoCapture_inst.read()

                if result is False:
                    print(f"****  camera{task_index} : cannot read!!!!  ****")
                    time.sleep(
                        self.DataCaptureConfig.Camera.capture_latency_ms / 1000.0
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
                if self.DataCaptureConfig.Camera.capture_latency_ms > 0:
                    time.sleep(
                        self.DataCaptureConfig.Camera.capture_latency_ms / 1000.0
                    )
                timestamp_ix += 1

        finally:
            print("video end, close")
            VideoCapture_inst.release()
            # self.loopEnable.value = False

            # センチネルを先に入れてからフラグを折る
            self.infoQueue.put(SENTINEL)
            self.working_task_flags[task_index] = False

    def _sensorinput_task(self, task_index: int):
        rtp_url: str = self.rtp_urls[task_index]
        self._logger.info(
            f"camera capture -> _sensorinput_task, starting... URL: {rtp_url}"
        )
        self.capture = cv2.VideoCapture(rtp_url, cv2.CAP_FFMPEG)
        timestamp_ix: int = 0
        assert self.DataCaptureConfig.Camera.count > task_index
        timestamp_ix = self.startframe
        timestamp_ix = max(timestamp_ix, 0)

        self.working_task_flags[task_index] = True

        boottime_failedcount = 0

        try:
            failed_count = 0
            while self.keep_loop_condition(task_index=task_index):
                if self.index_reset_req[task_index]:
                    timestamp_ix = 0
                    self.index_reset_req[task_index] = False
                    if not self.print_disabled:
                        self._logger.info(
                            f"_sensorinput_task: reset index (camera {task_index}), time:{datetime.datetime.now()}",
                        )
                buffer_waitcount = 0

                if not self.print_disabled:
                    self._logger.info(
                        f"_sensorinput_task, {task_index = }, {self.infoQueue.qsize() = }",
                    )

                while (
                    self.infoQueue.qsize()
                    >= self.DataCaptureConfig.Camera.data_buffersize
                    and self.keep_loop_condition(task_index=task_index)
                ):
                    time.sleep(
                        self.DataCaptureConfig.Camera.capture_latency_ms / 1000.0
                    )  # 0.01s~ の程よいポーリング（キャプチャレイテンシ時間はあくまで目安として使用）
                    if buffer_waitcount > 100:
                        buffer_waitcount = 0
                        self._logger.warning(
                            f"camera{task_index} : buffer full, waiting..."
                        )

                if not self.keep_loop_condition(task_index=task_index):
                    break  # while文を抜けた理由が継続条件満たさなかった場合は終了

                divcount: int = self.DataCaptureConfig.Camera.framerate_div

                if (
                    self.infoQueue.qsize()
                    >= self.CameraConfig.framethinning_bufferlen_threshold
                ):  # バッファが最大値の半分以上埋まっている場合はフレームレート半分に  Todo:　何割バッファ埋まったら間引くかconfig追加
                    divcount = divcount * 2
                    self._logger.warning(
                        f"_sensorinput_task framerate 1/2, {task_index = }, {self.infoQueue.qsize() = }",
                    )

                retvals: NDArray[np.bool_] = np.zeros(divcount, dtype=np.bool_)
                for x in range(divcount):
                    retvals[x], frame = self.capture.read()

                # if frame is None:
                if not all(retvals):  # いずれかがFalseで実行
                    time.sleep(0.1)
                    if boottime_failedcount < 1000:
                        if boottime_failedcount % 10 == 0:
                            self._logger.warning(
                                f"camera{task_index} :  read failed, waiting... {boottime_failedcount}, buffiersize:{self.infoQueue.qsize()}",
                            )
                        boottime_failedcount += 1
                        continue

                    # self.working_task_flags[task_index] = False
                    self._logger.warning(f"camera{task_index} : cannot read")

                    if failed_count > 50:
                        self._logger.warning(
                            f"camera{task_index} : cannot read, camera reconnect"
                        )
                        failed_count = 0
                        self.capture.release()
                        time.sleep(1)
                        self.capture = cv2.VideoCapture(rtp_url, cv2.CAP_FFMPEG)
                    else:
                        failed_count += 1
                    continue
                    # break
                failed_count = 0

                # print(datetime.datetime.now(), args, kwargs)
                timestamp_curtime: float = time.time()
                self.infoQueue.put((frame, timestamp_ix, timestamp_curtime))
                if not self.print_disabled:
                    self._logger.info(
                        f"camera {task_index} read, ix {timestamp_ix}, time:{datetime.datetime.fromtimestamp(timestamp_curtime)}",
                    )

                if self.DataCaptureConfig.Camera.capture_latency_ms > 0:
                    time.sleep(
                        self.DataCaptureConfig.Camera.capture_latency_ms / 1000.0
                    )
                timestamp_ix += 1
        except Exception as e:
            self._logger.error(
                f"camera capture -> _sensorinput_task: exception! {e} - \n{traceback.format_exc()}",
            )

        finally:
            self._logger.info("camera capture -> _sensorinput_task end")
            # receiver.stop()
            self.capture.release()

            # センチネルを先に入れてからフラグを折る
            self.infoQueue.put(SENTINEL)
            self.working_task_flags[task_index] = False

    def task(self):
        if self.DefaultConfig.File_Input is True:
            self._fileinput_task(self.task_index)
        else:
            self._sensorinput_task(self.task_index)
        self._logger.info(f"camera {self.task_index} read task ended")
