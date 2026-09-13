from collections.abc import Callable

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration


def bbox_to_corners(bboxmat: NDArray[np.int32]) -> NDArray[np.int32]:
    """
    bboxmat: (N, 2, 2) [[xmin,ymin],[xmax,ymax]]
    return : (N, 4, 2)
    """
    mins = bboxmat[:, 0]  # (N, 2)
    maxs = bboxmat[:, 1]  # (N, 2)

    corners = np.stack(
        [
            mins,  # (xmin, ymin)
            np.column_stack([maxs[:, 0], mins[:, 1]]),  # (xmax, ymin)
            maxs,  # (xmax, ymax)
            np.column_stack([mins[:, 0], maxs[:, 1]]),  # (xmin, ymax)
        ],
        axis=1,
    )

    return corners


def corners_zero_mask(img: np.ndarray, corners: np.ndarray) -> NDArray:
    """
    画像上の各頂点座標の画素が「ゼロかどうか」を bool で返す。
    Parameters
    ----------
    img : np.ndarray
        画像配列 (H, W, 3) ※ dtype=uint8 推奨（cv2.imread想定）
    corners : np.ndarray
        頂点座標 (N, 4, 2)  各点は (x, y) で画素座標系（左上(0,0)）
    all_channels : bool, default True
        True なら「全チャンネル==0」をゼロ判定。
        False なら「いずれかのチャンネル==0」をゼロ判定。
    Returns
    -------
    zero_mask : np.ndarray
        shape=(N, 4) の bool 配列。各頂点に対応して True=ゼロ画素 / False=非ゼロ。
    """
    H, W = img.shape[:2]

    if corners.ndim != 3 or corners.shape[-1] != 2 or corners.shape[1] != 4:
        raise ValueError(f"corners must be (N, 4, 2), got shape={corners.shape}")

    # (N,4,2) -> 整数画素座標（round or floor）
    # ここでは floor とし、負や範囲外はクリップ
    xs = np.floor(corners[..., 0]).astype(np.int64)
    ys = np.floor(corners[..., 1]).astype(np.int64)

    # 画像範囲内にクリップ
    xs_clipped = np.clip(xs, 0, W - 1)
    ys_clipped = np.clip(ys, 0, H - 1)

    # 範囲内に本来入っていたかを保持
    # in_bounds = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)

    # 画素値を一括取得
    # xs_clipped, ys_clipped: (N,4) -> img[ys_clipped, xs_clipped] -> (N,4,3)
    pixels = img[ys_clipped, xs_clipped]  # shape=(N,4,3)

    zero_mask = np.any(pixels == 0, axis=-1)  # (N,4)

    # もし「範囲外はFalse扱い」にしたいなら以下のようにマスクを掛ける
    # zero_mask = zero_mask & in_bounds

    return zero_mask


class bbox2Dmask_byimage:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        camera_index: int,
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        filterimgpath = app_config_calib.calib2d3d.Proc2d.camera_mask_images[
            camera_index
        ]
        self.enable_bboxfilter_byimg = (
            app_config_calib.calib2d3d.Proc2d.enable_bboxfilter_byimg
        )
        self.image_h = app_config_calib.dataCapture.Camera.sys_height
        self.image_w = app_config_calib.dataCapture.Camera.sys_width

        img = cv2.imread(filename=filterimgpath)
        if img is None:
            error = OSError(f"image mask could not be loaded: {filterimgpath}")
            if file_io_error_reporter is not None:
                file_io_error_reporter(
                    filterimgpath, "read 2D-3D bbox mask image", error
                )
            raise error
        self.img = cv2.resize(img, dsize=(self.image_w, self.image_h))

        self.verbose = not app_config_calib.default.print_disabled

    def filter_validbbox_byimg(
        self, yolo_results: list[NDArray[np.float64]] | None
    ) -> list[NDArray[np.float64]] | None:
        if yolo_results is None or not self.enable_bboxfilter_byimg:
            return yolo_results
        yolo_results_0 = yolo_results[0].reshape(
            -1, 4
        )  # YOLO座標出力が余計な3次元配列になる場合がある対策
        # YOLO形式→int xyxy形式
        # bbox_ymin = bboxmat[:,0] * self.image_h
        # bbox_ymax = bboxmat[:,2] * self.image_h
        # bbox_xmin = bboxmat[:,1] * self.image_w
        # bbox_xmax = bboxmat[:,3] * self.image_w
        origshapes = [x.shape for x in yolo_results]

        bboxmat_int_xyxy = (
            (
                yolo_results_0[:, [1, 0, 3, 2]]
                * np.array([self.image_w, self.image_h, self.image_w, self.image_h])
            )
            .reshape(-1, 2, 2)
            .astype(np.int32)
        )

        corners = bbox_to_corners(bboxmat=bboxmat_int_xyxy)
        points_isfiltered = corners_zero_mask(img=self.img, corners=corners)
        bbox_isfiltered = np.any(points_isfiltered, axis=1)

        if self.verbose:
            self._logger.info(
                f"{yolo_results[0].shape},{bboxmat_int_xyxy.shape},{bbox_isfiltered.shape=},{corners.shape=},{points_isfiltered.shape=},{yolo_results[1].shape=}",
            )

        yolo_results[1] = np.where(
            ~bbox_isfiltered, yolo_results[1].reshape(-1), 0
        ).reshape(origshapes[1])  # bbox削除はリスクがあるため信頼度を0に書き換え
        return yolo_results
