from __future__ import annotations

import os

os.environ["OPENCV_FFMPEG_LOGLEVEL"] = "-8"  # Disable FFMPEG log messages
import cv2
from cv2.typing import MatLike

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import CameraConf


class Mcde7000File:
    def __init__(
        self,
        index: int,
        camera_file_path: str,
        start_frame: int,
        log_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = log_factory.register_from_type(self.__class__)

        self.start_frame: int = start_frame
        self.file: str = camera_file_path
        self.cap = cv2.VideoCapture()
        self._logger.info(f"video open:{self.file}")

    def init_capture(self) -> None:
        cap = cv2.VideoCapture(self.file)
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(self.start_frame + 1))
        self.cap: cv2.VideoCapture = cap
        if not self.cap.isOpened():
            self._logger.critical(f"Cannot load file: {self.file}")
            raise RuntimeError(f"Cannot load file: {self.file}")
        self._logger.info(f"video length:{self.cap.get(cv2.CAP_PROP_FRAME_COUNT)}")
        self._logger.info("Initialization: DONE")

    def get_image(self) -> tuple[bool, MatLike | None]:
        return self.cap.read()

    def release(self) -> None:
        self.cap.release()

    def __del__(self) -> None:
        self.release()

    def change_file_name_index(self, file_name: str, index: int) -> None:
        self.file = file_name
        self.start_frame = index
        self.cap.release()
        self.init_capture()


class Mcde7000Device:
    OPEN_TIMEOUT_MSEC = 1000
    READ_TIMEOUT_MSEC = 200

    def __init__(
        self,
        index: int,
        camera_conf: CameraConf,
        log_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = log_factory.register_from_type(self.__class__)

        self.ip: str = camera_conf.iptable[index]
        self.port: str = camera_conf.porttable[index]
        self.cap = cv2.VideoCapture()

    def init_capture(self) -> None:
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
        mcde_device: str = "rtp://" + self.ip + ":" + self.port + "/MCDE7000"
        timeout_options: tuple[int, ...] = (
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
            self.OPEN_TIMEOUT_MSEC,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC,
            self.READ_TIMEOUT_MSEC,
        )
        self.cap = cv2.VideoCapture()
        self.cap.open(mcde_device, cv2.CAP_FFMPEG, timeout_options)
        self._logger.info(f"Initialization: DONE, URL: {mcde_device}")

    def release(self) -> None:
        self.cap.release()

    def get_image(self) -> tuple[bool, MatLike | None]:
        return self.cap.read()

    def __del__(self) -> None:
        self.release()
