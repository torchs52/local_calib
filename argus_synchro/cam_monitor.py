# 標準系
from __future__ import annotations

from collections.abc import Callable

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.Camera import Camera
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.common.common import t_py_col_res
from argus_synchro.config.app_config import Detect2dConf
from argus_synchro.core import utils  # BB描画用\


# カメラベースの表示
class Monitoring:
    def __init__(
        self,
        show_cam: list[int],
        detect2d_conf: Detect2dConf,
        app_logger_factory: AppLoggerFactory,
        file_io_error_callback: Callable[[str, Exception], None] | None = None,
        file_io_recovery_callback: Callable[[], None] | None = None,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._file_io_error_callback = file_io_error_callback
        self._file_io_recovery_callback = file_io_recovery_callback
        # どのカメラ映像を表示するか
        self.update(show_cam, detect2d_conf)

    def update(self, show_cam: list[int], detect2d_conf: Detect2dConf) -> None:
        self.show_cam: list[int] = show_cam
        self.classes: dict[int, str] = utils.read_class_names(
            detect2d_conf.yolo_class,
            on_error=self._file_io_error_callback,
        )
        if self._file_io_recovery_callback is not None:
            self._file_io_recovery_callback()

    def draw_collision(
        self,
        frame: cv2.typing.MatLike,
        camera: Camera,
        collision_clusters: t_py_col_res,
        line_color: tuple[int, int, int] = (30, 200, 30),
        circle_color: tuple[int, int, int] = (0, 0, 255),
        radius: int = 24,
    ) -> cv2.typing.MatLike:
        """衝突判定結果をframeに追加する
        - 引数:
            - frame: imshowする画像の配列
            - camera: カメラの補正関連を行うインスタンス
            - collision_clusters: クラスタ毎の衝突判定結果を持つ辞書

        - 戻り値:
            - frame: 衝突部位に線が引かれた画像行列
        """
        if collision_clusters is None or len(collision_clusters) == 0:
            return frame

        for _, _, w_coord_from, w_coord_to, _, _ in collision_clusters.values():
            # w_coord1を始点, w_coord2を終点にして線を作る

            w_2d_coord_from, w_2d_coord_to = (
                cv2.projectPoints(
                    objectPoints=np.vstack([w_coord_from, w_coord_to]),
                    rvec=camera.rvec,
                    tvec=camera.tvec,
                    cameraMatrix=camera.ncm1,
                    distCoeffs=np.zeros((1, 5)),
                )[0]
                .squeeze(1)
                .round()
                .astype(np.int32)
            )

            # 3D座標がカメラ手前にあるか否かのテスト　外部パラメータ行列を同次３次元座標に掛けると[2,:]がカメラ座標上のZ軸に対応するのでそれで判別
            points3d: NDArray[np.float64] = np.vstack([w_coord_from, w_coord_to])
            homogeneous_points = np.hstack(
                [points3d, np.ones((points3d.shape[0], 1))]
            ).T
            camera_coordinate_pts = camera.extrmat @ homogeneous_points
            camera_coordin_z = camera_coordinate_pts[2]
            if (camera_coordin_z < 0).any():
                continue

            cv2.line(
                frame,
                pt1=(w_2d_coord_from[0], w_2d_coord_from[1]),
                pt2=(w_2d_coord_to[0], w_2d_coord_to[1]),
                color=line_color,
                thickness=10,
            )

            cv2.circle(
                frame,
                center=(w_2d_coord_from[0], w_2d_coord_from[1]),
                radius=radius,
                color=circle_color,
                thickness=-1,
            )

            cv2.circle(
                frame,
                center=(w_2d_coord_to[0], w_2d_coord_to[1]),
                radius=radius,
                color=circle_color,
                thickness=-1,
            )
        return frame
