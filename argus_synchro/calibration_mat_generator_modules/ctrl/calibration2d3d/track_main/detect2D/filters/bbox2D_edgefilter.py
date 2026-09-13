import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config_calibration import AppConfigCalibration


def bbox_inside_thresholds(
    bboxes: np.ndarray,
    xlthresh: float,
    xhthresh: float,
    ylthresh: float,
    yhthresh: float,
) -> np.ndarray:
    """
    bboxes: shape (N, 4), each row [xmin, ymin, xmax, ymax] (float64 assumed)
    Returns: shape (N,), dtype=bool
      True  if xlthresh <= xmin,xmax <= xhthresh AND ylthresh <= ymin,ymax <= yhthresh
      False otherwise (NOT including equality to thresholds)
    """
    bboxes = np.asarray(bboxes)

    # --- basic validation ---
    if bboxes.ndim != 2 or bboxes.shape[1] != 4:
        raise ValueError(f"bboxes must have shape (N, 4). Got {bboxes.shape}")

    xmin = bboxes[:, 0]
    ymin = bboxes[:, 1]
    xmax = bboxes[:, 2]
    ymax = bboxes[:, 3]

    inside_x = (
        (xlthresh <= xmin)
        & (xmin <= xhthresh)
        & (xlthresh <= xmax)
        & (xmax <= xhthresh)
    )
    inside_y = (
        (ylthresh <= ymin)
        & (ymin <= yhthresh)
        & (ylthresh <= ymax)
        & (ymax <= yhthresh)
    )

    return inside_x & inside_y


class bbox2D_edgefilter:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        camera_index: int,
    ) -> None:
        self.enable_edgebboxfilter = (
            app_config_calib.calib2d3d.Proc2d.enable_edgebboxfilter
        )

        self.image_h = app_config_calib.dataCapture.Camera.sys_height
        self.image_w = app_config_calib.dataCapture.Camera.sys_width

        self.xlthresh = 10
        self.xhthresh = 1910
        self.ylthresh = 10
        self.yhthresh = 1070

        self.verbose = not app_config_calib.default.print_disabled

    def filter_bbox_inside(
        self, yolo_results: list[NDArray[np.float64]] | None
    ) -> list[NDArray[np.float64]] | None:
        if yolo_results is None or not self.enable_edgebboxfilter:
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
            .reshape(-1, 4)
            .astype(np.int32)
        )

        bbox_pass = bbox_inside_thresholds(
            bboxes=bboxmat_int_xyxy,
            xlthresh=self.xlthresh,
            xhthresh=self.xhthresh,
            ylthresh=self.ylthresh,
            yhthresh=self.yhthresh,
        )

        yolo_results[1] = np.where(bbox_pass, yolo_results[1].reshape(-1), 0).reshape(
            origshapes[1]
        )  # bbox削除はリスクがあるため信頼度を0に書き換え
        return yolo_results
