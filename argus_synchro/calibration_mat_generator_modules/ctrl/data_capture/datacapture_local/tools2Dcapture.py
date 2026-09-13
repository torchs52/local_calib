# 画像対応点抽出
import warnings  # 移行期間の非推奨コード警告用 後でimport削除しエラーになったコードもまとめて削除を。

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.provider.image import Mcde7000UndistortImageProvider
from argus_synchro.shared_app_config import SharedAppConfig


class capture2d:
    def __init__(
        self, app_config_calib: AppConfigCalibration, sac: SharedAppConfig
    ) -> None:
        # dataConverter2D3DConf: DataConverter2D3DConf, dataCaptureConf: DataCaptureConf, camerasel: int
        self.sac = sac
        self.app_config_calib = app_config_calib

        self.dataCaptureConf = app_config_calib.dataCapture
        self.dataConverter2D3DConf = app_config_calib.dataConverter2D3D
        camerasel = sac.read().CalibMode.cameraID
        verbose = not app_config_calib.default.print_disabled
        self.undistort_work = Mcde7000UndistortImageProvider(
            camera_intrinsics_path=self.dataConverter2D3DConf.Camera.intrinsics_path,
            sys_width=self.dataCaptureConf.Camera.sys_width,  # 変換後画像縦横
            sys_height=self.dataCaptureConf.Camera.sys_height,
        )

        self.cam_num: int = camerasel

    def release(self):
        warnings.warn(
            "capture2d: 別機構にて動画を読むことに変更。このクラスのreleaseは機能していません",
            DeprecationWarning,
        )
        # self.cap.release()

    def isOpened(self):
        # return self.cap.isOpened()
        return True

    def read(self, data_cameras: list, cam_select: int | None = None):
        # if self.e_frame is not None and self.cap.get(cv2.CAP_PROP_POS_FRAMES) >= self.e_frame:
        #    return False, None

        if cam_select is None:
            cam_select = self.cam_num

        if data_cameras[self.cam_num] is None:
            ret = False
            frame = None
            timestamps = None
        else:
            ret, frame, timestamps = (
                True,
                data_cameras[cam_select][0],
                data_cameras[cam_select][1:],
            )
        if ret and self.dataConverter2D3DConf.Camera.undistort_enable:
            frame: NDArray[np.uint8] = self.undistort_work.get_undistort_image(frame)

        debug_char = False  # TODO: config化　ここTrueで読み込み画像にフレーム番号・タイムスタンプ付ける
        if debug_char:
            text = f"{timestamps}"
            # フォント設定
            font = cv2.FONT_HERSHEY_SIMPLEX

            # 文字サイズを高さの1/20に合わせる
            font_scale: float = 1080 / 20 / 30  # 30はフォントの基準高さの目安
            thickness = 2

            # テキストの位置（左上）
            x, y = 10, 50  # yはフォントのベースラインを考慮

            cv2.putText(
                frame, text, (x, y), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA
            )

        return ret, frame, timestamps

    def get_cameramatrix(self):
        return self.undistort_work.ncm1
