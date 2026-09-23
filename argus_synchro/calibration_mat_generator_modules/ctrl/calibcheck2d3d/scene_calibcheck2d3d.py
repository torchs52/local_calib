from collections.abc import Callable

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.SceneDesc import (
    Scene,
)
from argus_synchro.config.app_config import SceneDescriptionConf
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.device.camera.helper import CameraHelper


class Scene_CalibCheck2d3d(Scene):
    def __init__(
        self,
        scene_conf: SceneDescriptionConf,
        app_config_calib: AppConfigCalibration,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        super().__init__(scene_conf)
        self.app_config_calib: AppConfigCalibration = app_config_calib
        self._image_width = app_config_calib.calibCheck2d3d.image_w
        self._image_height = app_config_calib.calibCheck2d3d.image_h

        # 魚眼カメラ歪み補正データの読み込み
        camera_intrinsics_path = (
            self.app_config_calib.calibCheck2d3d.camera_intrinsics_path
        )
        try:
            (
                _,
                _,
                _,
                _,
                self._ncm1,
            ) = CameraHelper.read_fisheye_param(camera_intrinsics_path)
        except (OSError, UnicodeError, ValueError, KeyError, TypeError) as error:
            if file_io_error_reporter is not None:
                file_io_error_reporter(
                    camera_intrinsics_path,
                    "read calibcheck2d3d camera intrinsics JSON",
                    error,
                )
            raise

    @classmethod
    def create_for_evaluation(
        cls,
        scene_conf: SceneDescriptionConf,
        camera_intrinsics: NDArray[np.float32],
        image_width: int,
        image_height: int,
    ) -> "Scene_CalibCheck2d3d":
        instance = cls.__new__(cls)
        Scene.__init__(instance, scene_conf)
        instance._ncm1 = camera_intrinsics
        instance._image_width = image_width
        instance._image_height = image_height
        return instance

    def integrate2d3d_calibcheck(
        self,
        rvec: NDArray[np.float64],
        tvec: NDArray[np.float64],
        boxpoints: NDArray[np.float64],  # 3次元bb座標, 8点*3*(valid_detects[0]個)が入っている オリジナルはLS_pcd_det.boxes
        minmax3ds: NDArray[np.float64],  # 3次元bbのminmax座標, (valid_detects[0],6)が入っている オリジナルはLS_pcd_det.minmax
        bbox2d: NDArray[np.float32],  # 2次元bb座標と言うことにしておく
        yolo_classes: NDArray[np.int32],  # yoloのクラスID
        n_clusters: int,  # クラスタ数
        bbox2d_detection_count: int,  # 2次元bb検出数
        method: str = "center",
    ) -> dict[int, str]:
        # 立体物と人検知の紐づけ処理
        """
        元の環境にて、人検知結果:
        LS_cam_det.boxes = copy.deepcopy(pred_bbox[0]) → box2ds
        LS_cam_det.scores = copy.deepcopy(pred_bbox[1])
        LS_cam_det.classes = copy.deepcopy(pred_bbox[2]) → yolo_classes
        LS_cam_det.valid_detects = copy.deepcopy(pred_bbox[3]) → bbox2d_detection_count
        点群バウンディングボックス作成
        LS_pcd_det.boxes = copy.deepcopy(multi_points) → box3ds
        LS_pcd_det.minmax = copy.deepcopy(multi_minmax) → minmax3ds
        LS_pcd_det.valid_detects = copy.deepcopy(valid_detect_num) → n_clusters

        """
        width: int = self._image_width
        height: int = self._image_height
        ncm1: NDArray[np.float32] = self._ncm1

        # integrated_retults_2d3d : 元は3カメラ共通の3DBBの属性リスト。今回はカメラごとに独立して結果を出したいので、ただの辞書で良い
        integrated_retults_2d3d: dict[int, str] = {}  # 3DBBの属性リスト

        box3ds_reproj: NDArray[np.float64] = np.zeros(
            (n_clusters * 8, 2), dtype=np.float64
        )  # 3次元bbの射影変換結果を格納する配列
        if n_clusters != 0:
            box3ds_reproj = cv2.projectPoints(
                np.array([boxpoints]),
                rvec,
                tvec,
                ncm1,
                np.zeros((1, 5)),
            )[0].squeeze(1)
            assert np.array([boxpoints]).shape[0] == 1

            extrinsic_matrix = np.hstack([cv2.Rodrigues(rvec)[0], tvec.reshape(3, 1)])
            homogeneous_points = np.hstack(
                [boxpoints, np.ones((boxpoints.shape[0], 1))]
            ).T
            camera_coordinate_pts = extrinsic_matrix @ homogeneous_points
            camera_coordin_z = camera_coordinate_pts[2]

            BBOX_VERTEX_POINTS = 8

            # box3ds_reproj: bbox8点分ずつ格納。前からn_clusters*8点分のみ有効（n_clusters*8以降は不定？）対応するz座標を8個ずつ見て1つでも<0なら8点全て除去する必要がある
            # 本当に除去してしまうとintegrated_retults_2d3d反映時のインデックスと整合が取れなくなるので-1e6に飛ばすことで対応。
            camera_coordin_z_bboxset = camera_coordin_z.reshape(-1, BBOX_VERTEX_POINTS)
            camera_coordin_z_bboxset_flag = np.all(camera_coordin_z_bboxset > 0, axis=1)
            box3ds_zfilter = np.repeat(
                camera_coordin_z_bboxset_flag, BBOX_VERTEX_POINTS
            )
            box3ds_reproj[box3ds_zfilter == 0] = -1e6

            # 改良版：人らしさの寸法ゲートを追加
            for i in range(int(bbox2d_detection_count)):  # 2dbbを順番にチェック
                if yolo_classes[i] == 0:  # 人である場合
                    box2d_single: NDArray[np.float32] = bbox2d[i]
                    index: int = self.get_human_3bb(
                        box2d_single,
                        int(width),
                        int(height),
                        box3ds_reproj,
                        n_clusters,
                        method,
                    )
                    if index >= 0:
                        # ---- 寸法ゲート（人らしさ）で最終確認：外れたら HUMAN を取り消す ----
                        if self.use_human_gate:
                            # 最後に人寸法ゲートで判断
                            if self.passes_human_size(minmax3ds[index]):
                                integrated_retults_2d3d[index] = "HUMAN"
                            else:
                                # 人寸法を外れるので、元の属性のまま（誤通知抑止）
                                # AppLogger.debug(f"Rejected HUMAN by size gate: idx={index}")
                                pass
                        else:
                            # 人寸法ゲートは使わず、そのまま追加
                            integrated_retults_2d3d[index] = "HUMAN"

        return integrated_retults_2d3d