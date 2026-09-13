from __future__ import annotations

import time
from abc import ABC, abstractmethod
from enum import StrEnum
from typing import Final, final

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.common.error import NotStartedError
from argus_synchro.config.app_config_calibration import (
    DataCaptureConf,
)
from argus_synchro.device.camera.helper import CameraHelper
from argus_synchro.device.camera.mcde7000 import Mcde7000Device, Mcde7000File
from argus_synchro.device.camera.shi_lib_camera import CameraDevReader


class ImageProvider(ABC):
    @abstractmethod
    def get_image(self) -> NDArray[np.uint8] | None: ...

    @abstractmethod
    def handle_no_input(self) -> tuple[bool, NDArray[np.uint8] | None]:
        """返り値で停止が必要かどうかを返す。
        Trueなら停止が必要。
        子クラスによっては処理をする。
        """
        ...


@final
class NotStartedImageProvider(ImageProvider):
    def get_image(self) -> NDArray[np.uint8] | None:
        raise NotStartedError

    def handle_no_input(self) -> tuple[bool, NDArray[np.uint8] | None]:
        raise NotImplementedError


@final
class Mcde7000FileImageProvider(ImageProvider):
    def __init__(self, device: Mcde7000File, width: int, height: int) -> None:
        self._device: Mcde7000File = device

    def change_file_name_index(self, file_path: str, index: int) -> None:
        self._device.change_file_name_index(file_path, index)

    def get_image(self) -> NDArray[np.uint8] | None:
        _, image = self._device.get_image()
        if image is None:
            return None
        return np.asarray(image, dtype=np.uint8)

    def handle_no_input(self) -> tuple[bool, NDArray[np.uint8] | None]:
        """入力が無い場合の処理。Trueだと1回で停止するが、周辺監視モードでは停止させたくないのでFalseにして、再接続を試みる。"""
        return False, np.zeros((852, 480, 3), dtype=np.uint8)


@final
class CalibMcde7000FileImageProvider(ImageProvider):
    def __init__(
        self,
        index: int,
        device: Mcde7000File,
        camera_config: DataCaptureConf.CameraConf,
        width: int,
        height: int,
        logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = logger_factory.register_from_type(self.__class__)
        self._index: int = index
        self._device: Mcde7000File = device
        self._camera_config = camera_config
        self._size: Final[cv2.typing.Size] = (width, height)

    def get_image(self) -> NDArray[np.uint8] | None:
        image: cv2.typing.MatLike | None = None
        result, image = self._device.get_image()
        if result is False:
            return None
        if self._camera_config.capture_latency_ms > 0:
            time.sleep(self._camera_config.capture_latency_ms / 1000.0)

        return np.asarray(image, dtype=np.uint8)

    def handle_no_input(self) -> tuple[bool, NDArray[np.uint8] | None]:
        self._logger.error(f"****  camera{self._index} : cannot read!!!!  ****")
        time.sleep(self._camera_config.capture_latency_ms / 1000.0)
        return True, None

    def change_file_name_index(self, file_path: str, index: int) -> None:
        self._logger.info(f"change filepath to {file_path}")
        self._device.change_file_name_index(file_path, index)
        self._logger.info(f"change index to {index}")


@final
class Mcde7000DeviceImageProvider(ImageProvider):
    def __init__(
        self,
        index: int,
        device: Mcde7000Device,
        width: int,
        height: int,
        logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = logger_factory.register_from_type(self.__class__)
        self._index: int = index
        self._device: Mcde7000Device = device

    def get_image(self) -> NDArray[np.uint8] | None:
        result, image = self._device.get_image()
        if (not result) or (image is None):
            # NOTE: デバイスから映像の取得に失敗した場合、再接続を試みる
            self._logger.info(f"camera{self._index} : Trying to reconnect...")
            self._device.release()
            self._device.init_capture()
            return None
        return np.asarray(image, dtype=np.uint8)

    def handle_no_input(self) -> tuple[bool, NDArray[np.uint8] | None]:
        """入力が無い場合の処理。Trueだと1回で停止するが、周辺監視モードでは停止させたくないのでFalseにして、再接続を試みる。"""
        return False, np.zeros((852, 480, 3), dtype=np.uint8)


@final
class CalibMcde7000DeviceImageProvider(ImageProvider):
    def __init__(
        self,
        index: int,
        device: Mcde7000Device,
        camera_config: DataCaptureConf.CameraConf,
        width: int,
        height: int,
        logger_factory: AppLoggerFactory,
    ) -> None:
        self._failed_count = 0
        self._boottime_failedcount = 0
        self._index: int = index
        self._logger: AppLogger = logger_factory.register_from_type(self.__class__)
        self._device: Mcde7000Device = device
        self._size: Final[cv2.typing.Size] = (width, height)
        self._camera_config: DataCaptureConf.CameraConf = camera_config

    def get_image(self) -> NDArray[np.uint8] | None:
        divcount: int = self._camera_config.framerate_div
        retvals: NDArray[np.bool_] = np.zeros(divcount, dtype=np.bool_)
        frame: cv2.typing.MatLike | None = None
        for x in range(divcount):
            retvals[x], frame = self._device.get_image()

        if not all(retvals) or (frame is None):  # いずれかがFalseで実行
            return None

        if all(retvals):
            self._failed_count = 0
        if self._camera_config.capture_latency_ms > 0:
            time.sleep(self._camera_config.capture_latency_ms / 1000.0)

        return np.asarray(frame, dtype=np.uint8)

    def handle_no_input(self) -> tuple[bool, NDArray[np.uint8] | None]:
        # if frame is None:
        time.sleep(0.1)
        if self._boottime_failedcount < 1000:
            if self._boottime_failedcount % 10 == 0:
                self._logger.warning(
                    f"camera{self._index} :  read failed, waiting... {self._boottime_failedcount}"
                )
                # FIFOと接続していないため、buffersizeについてのログは削除
                # self._logger.warning(
                #     f"camera{self._index} :  read failed, waiting... {boottime_failedcount}, buffiersize:{self.infoQueue.qsize()}",
                # )
            self._boottime_failedcount += 1
            return False, None

        self._logger.warning(f"camera{self._index} : cannot read")

        if self._failed_count > 50:
            self._logger.warning(f"camera{self._index} : cannot read, camera reconnect")
            self._failed_count = 0
            self._device.release()
            time.sleep(1)
            self._device.init_capture()
        else:
            self._failed_count += 1
            # break
        return False, None


@final
class ShiLibCameraDeviceImageProvider(ImageProvider):
    def __init__(self, device: CameraDevReader) -> None:
        super().__init__()
        self._device = device

    def get_image(self) -> NDArray[np.uint8] | None:
        result, image = self._device.get_image()
        if (not result) or (image is None):
            return None
        return image.image

    def handle_no_input(self) -> tuple[bool, NDArray[np.uint8] | None]:
        """入力が無い場合の処理。Trueだと1回で停止するが、周辺監視モードでは停止させたくないのでFalseにして、再接続を試みる。"""
        return False, np.zeros((852, 480, 3), dtype=np.uint8)


class UndistortImageProvider(ABC):
    backend_name: str = "unknown"

    @abstractmethod
    def get_undistort_image(
        self,
        image: NDArray[np.uint8],
        dst: NDArray[np.uint8] | None = None,
    ) -> NDArray[np.uint8]: ...


@final
class NotStartedUndistortImageProvider(UndistortImageProvider):
    backend_name: str = "not_started"

    def get_undistort_image(
        self,
        image: NDArray[np.uint8],
        dst: NDArray[np.uint8] | None = None,
    ) -> NDArray[np.uint8]:
        raise NotImplementedError


@final
class Mcde7000UndistortImageProvider(UndistortImageProvider):
    backend_name: str = "cpu"

    def __init__(
        self, camera_intrinsics_path: str, sys_width: int, sys_height: int
    ) -> None:
        self.width: int = sys_width
        self.height: int = sys_height

        # 魚眼カメラ歪み補正データの読み込み
        (
            cm,
            dm,
            para_width,
            para_height,
            ncm1,
        ) = CameraHelper.read_fisheye_param(camera_intrinsics_path)

        cm, self.ncm1 = CameraHelper.apply_ratio(
            cm=cm,
            ncm1=ncm1,
            camera_size=(self.width, self.height),
            para_size=(para_width, para_height),
        )

        self.map1, self.map2 = CameraHelper.init_undistort_rectify_map(
            cm=cm,
            dm=dm,
            ncm1=ncm1,
            size=(self.width, self.height),
        )

    def get_undistort_image(
        self,
        image: NDArray[np.uint8],
        dst: NDArray[np.uint8] | None = None,
    ) -> NDArray[np.uint8]:
        result = CameraHelper.undistort_image(
            image,
            self.height,
            self.width,
            self.map1,
            self.map2,
            dst=dst,
        )
        if dst is not None:
            return dst
        return np.asarray(result, dtype=np.uint8)


@final
class CudaMcde7000UndistortImageProvider(UndistortImageProvider):
    backend_name: str = "cuda"

    def __init__(
        self, camera_intrinsics_path: str, sys_width: int, sys_height: int
    ) -> None:
        self.width: int = sys_width
        self.height: int = sys_height

        cm: NDArray[np.float32]
        dm: NDArray[np.float32]
        para_width: int
        para_height: int
        ncm1: NDArray[np.float32]
        (
            cm,
            dm,
            para_width,
            para_height,
            ncm1,
        ) = CameraHelper.read_fisheye_param(camera_intrinsics_path)

        cm, ncm1 = CameraHelper.apply_ratio(
            cm=cm,
            ncm1=ncm1,
            camera_size=(self.width, self.height),
            para_size=(para_width, para_height),
        )

        # Build CUDA remap maps directly in float format to avoid fixed-point
        # conversion ambiguity from CPU remap maps.
        self._cuda_xmap: cv2.typing.MatLike
        self._cuda_ymap: cv2.typing.MatLike
        self._cuda_xmap, self._cuda_ymap = cv2.fisheye.initUndistortRectifyMap(
            K=cm,
            D=dm,
            R=np.eye(3),
            P=ncm1,
            size=(self.width, self.height),
            m1type=cv2.CV_32FC1,
        )

        if self._cuda_xmap.dtype != np.float32 or self._cuda_ymap.dtype != np.float32:
            raise RuntimeError(
                "invalid cuda map dtype"
                f" x={self._cuda_xmap.dtype} y={self._cuda_ymap.dtype}"
            )
        if self._cuda_xmap.shape != self._cuda_ymap.shape:
            raise RuntimeError(
                "invalid cuda map shape"
                f" x={self._cuda_xmap.shape} y={self._cuda_ymap.shape}"
            )

        gpu_mat_ctor = getattr(getattr(cv2, "cuda", None), "GpuMat", None)
        if gpu_mat_ctor is None:
            gpu_mat_ctor = getattr(cv2, "cuda_GpuMat", None)
        if gpu_mat_ctor is None:
            raise RuntimeError("cv2.cuda.GpuMat/cv2.cuda_GpuMat is not available")

        self._src_gpu: cv2.cuda.GpuMat = gpu_mat_ctor()
        self._dst_gpu: cv2.cuda.GpuMat = gpu_mat_ctor()
        self._map1_gpu: cv2.cuda.GpuMat = gpu_mat_ctor()
        self._map2_gpu: cv2.cuda.GpuMat = gpu_mat_ctor()

        self._map1_gpu.upload(np.ascontiguousarray(self._cuda_xmap, dtype=np.float32))
        self._map2_gpu.upload(np.ascontiguousarray(self._cuda_ymap, dtype=np.float32))

        self._run_cuda_smoke_test()

    def _run_cuda_smoke_test(self) -> None:
        probe = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._src_gpu.upload(probe)
        self._dst_gpu = cv2.cuda.remap(  # pyright: ignore[reportAttributeAccessIssue, reportUnknownMemberType]
            src=self._src_gpu,
            xmap=self._map1_gpu,
            ymap=self._map2_gpu,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        probe_out: cv2.typing.MatLike = np.empty_like(probe)
        self._dst_gpu.download(probe_out)
        if probe_out.shape != probe.shape:
            raise RuntimeError(
                "invalid cuda smoke output shape"
                f" output={probe_out.shape} expected={probe.shape}"
            )

    def get_undistort_image(
        self,
        image: NDArray[np.uint8],
        dst: NDArray[np.uint8] | None = None,
    ) -> NDArray[np.uint8]:
        image_c: NDArray[np.uint8] = np.ascontiguousarray(image, dtype=np.uint8)
        self._src_gpu.upload(image_c)

        self._dst_gpu = cv2.cuda.remap(  # pyright: ignore[reportAttributeAccessIssue, reportUnknownMemberType]
            src=self._src_gpu,
            xmap=self._map1_gpu,
            ymap=self._map2_gpu,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

        result = np.empty_like(image_c)
        self._dst_gpu.download(result)

        if result.shape != image_c.shape:
            raise RuntimeError(
                "invalid cuda remap output shape"
                f" output={result.shape} expected={image_c.shape}"
            )

        if dst is not None:
            if dst.shape != result.shape or dst.dtype != np.uint8:
                raise ValueError(
                    "invalid dst buffer"
                    f" shape={dst.shape} dtype={dst.dtype}"
                    f" expected_shape={result.shape} expected_dtype=uint8"
                )
            np.copyto(dst, result)
            return dst
        return result


class UndistortBackend(StrEnum):
    AUTO = "auto"
    CPU = "cpu"
    CUDA = "cuda"


def _has_cv2_cuda_remap_support() -> bool:
    cuda_module = getattr(cv2, "cuda", None)
    if cuda_module is None:
        return False
    if getattr(cuda_module, "remap", None) is None:
        return False
    if getattr(cv2, "cuda_GpuMat", None) is None:
        return False

    get_device_count = getattr(cuda_module, "getCudaEnabledDeviceCount", None)
    if get_device_count is None:
        return False

    try:
        return int(get_device_count()) > 0
    except Exception:
        return False


def create_mcde7000_undistort_provider(
    camera_intrinsics_path: str,
    sys_width: int,
    sys_height: int,
    backend: UndistortBackend | str = UndistortBackend.AUTO,
) -> UndistortImageProvider:
    backend_name = backend.value if isinstance(backend, UndistortBackend) else backend
    requested: UndistortBackend = UndistortBackend(str(backend_name).lower())
    if requested == UndistortBackend.AUTO:
        requested = (
            UndistortBackend.CUDA
            if _has_cv2_cuda_remap_support()
            else UndistortBackend.CPU
        )

    if requested == UndistortBackend.CUDA and _has_cv2_cuda_remap_support():
        return CudaMcde7000UndistortImageProvider(
            camera_intrinsics_path=camera_intrinsics_path,
            sys_width=sys_width,
            sys_height=sys_height,
        )

    return Mcde7000UndistortImageProvider(
        camera_intrinsics_path=camera_intrinsics_path,
        sys_width=sys_width,
        sys_height=sys_height,
    )
