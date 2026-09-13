# デバッグ用マルチプロセス系
from __future__ import annotations

# センサ取得ライブラリに必要なインポート
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from argus_synchro.config.app_config import CalibrationConf


class SyscamRes:
    def __init__(self, sys_width: int, sys_height: int) -> None:
        self.WIDTH: int = sys_width
        self.HEIGHT: int = sys_height


# カメラ映像の取得と広角変換のみの役割に分離
class Camera:
    def __init__(
        self,
        cam_index: int,
        calibration_conf: CalibrationConf,
        syscam_res: SyscamRes,
    ) -> None:
        self.update(cam_index, calibration_conf, syscam_res)

    def update(
        self,
        cam_index: int,
        calibration_conf: CalibrationConf,
        syscam_res: SyscamRes,
    ) -> None:
        import cv2

        from argus_synchro import calibration

        self.index: int = cam_index
        self.calibration_conf: CalibrationConf = calibration_conf
        self.width: int = syscam_res.WIDTH
        self.height: int = syscam_res.HEIGHT

        # ファイル入力:歪み補正パラメータ、2d3d校正の回転並進ベクトルを読み込み
        _, _, p_width, p_height, self.ncm1, self.rvec, self.tvec, _ = (
            calibration.get_lidar_camera_calib_para(
                cam_index,
                calibration_conf,
            )
        )

        # 3D座標がカメラ手前にあるか否かのテスト用　外部パラメータ行列を同次３次元座標に掛けると[2,:]がカメラ座標上のZ軸に対応するのでそれで判別
        extrmat: NDArray[np.float64] = np.zeros((4, 4))
        extrmat[0:3, 0:3] = cv2.Rodrigues(self.rvec)[0]
        extrmat[0:3, 3] = self.tvec.T
        extrmat[3, 3] = 1
        self.extrmat = extrmat

        # カメラ内部パラメータを計算
        # if abs(W / H - self.syscam_res.WIDTH / self.syscam_res.HEIGHT) > 1e-4:
        #     AppLogger.auto_info(
        #         self,
        #         "WARNING: Aspect ratio between intrinsics and syscam_width/height did not match",
        #     )

        intrinsics_coeff_x: float = self.width / p_width
        intrinsics_coeff_y: float = self.height / p_height

        # 1280x720対応 カメラ内部パラメータ(z軸1以外)にW,Hの比率を適用
        # self.cm[0, :] = self.cm[0, :] * intrinsics_coeff_x
        # self.cm[1, :] = self.cm[1, :] * intrinsics_coeff_y

        self.ncm1[0, :] = self.ncm1[0, :] * intrinsics_coeff_x
        self.ncm1[1, :] = self.ncm1[1, :] * intrinsics_coeff_y

        # 内部パラメータはSYSCAM_RES.WIDTH & HEIGHTと合わせてあるのでself.width/heightを内部パラメータ取り扱い時のサイズとして良い
        # self.width: int = self.syscam_res.WIDTH
        # self.height: int = self.syscam_res.HEIGHT

        # 魚眼補正用変換行列を作成し保持
        # AppLogger.info(
        #     "calculate UndistortRectifyMap,cm:",
        #     self.cm,
        #     " dm:",
        #     self.dm,
        #     " ncm1:",
        #     self.ncm1,
        #     " size:",
        #     (self.width, self.height),
        # )

        # self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
        #     K=self.cm,
        #     D=self.dm,
        #     R=np.eye(3),
        #     P=self.ncm1,
        #     size=(self.width, self.height),
        #     m1type=cv2.CV_16SC2,
        # )
