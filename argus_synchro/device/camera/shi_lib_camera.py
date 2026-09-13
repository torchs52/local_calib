from __future__ import annotations

import builtins
import contextlib
from typing import TYPE_CHECKING

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory

if TYPE_CHECKING:
    import pybind_shi_sensor_lib as shi
with contextlib.suppress(builtins.BaseException):
    import pybind_shi_sensor_lib as shi


class CameraDevReader:
    config = ""

    def __init__(
        self,
        index: int,
        config_file: str,
        log_factory: AppLoggerFactory,
    ) -> None:
        """
        インスタンス化 : shi.Camera()
        ※ shi.Camera.init()&run()をココですると、別プロセスにソケット情報や取得スレッド情報が渡せないので、
           shi.Camera.init()&run()は、shi.Camera.getSensorData()を実行するプロセスで実行すること。
        """
        self._logger: AppLogger = log_factory.register_from_type(self.__class__)

        self.camera_dev: shi.Camera = shi.Camera()
        self.handle: int = index
        self.config: str = config_file

        self.camera_img: shi.CameraImageData = shi.CameraImageData()
        self.timestamp_us = 1000000
        self.is_latest = True

    def init(self) -> None:
        """
        必要な情報の受け渡しがあればココで行う。
          - 共有メモリアドレス等
        """
        self._logger.info(f"init() : config({self.config})")

    def get_image(self) -> tuple[bool, shi.CameraImageData | None]:
        """
        カメラから画像を取得する
        """
        result: bool = True

        rts: shi.ApiStatus = self.camera_dev.getSensorData(
            self.handle,
            self.camera_img,
            self.timestamp_us,
            self.is_latest,
        )
        if rts == shi.ApiStatus.SUCCESS:
            self._logger.info(
                f"GET image ({self.handle}): width({self.camera_img.width}), height({self.camera_img.height})",
            )

        else:
            self._logger.info(f"get camera image error. status:{rts}")
            result = False

        return result, self.camera_img
