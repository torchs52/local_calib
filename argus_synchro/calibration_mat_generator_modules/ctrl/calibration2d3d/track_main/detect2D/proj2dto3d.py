import json

import cv2
import numpy as np

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class proj2dto3d:
    def read_fisheye_param(self, path):
        # tslog.append(("read_fisheye_param entering:",datetime.datetime.now()))
        json_open = open(path)
        json_load = json.load(json_open)

        # fisheye関連はfloat32で渡す必要あり
        cm = np.array(json_load["camera_matrix"]["data"])
        cm = cm.reshape(
            [json_load["camera_matrix"]["rows"], json_load["camera_matrix"]["cols"]]
        )
        dm = np.array(
            [json_load["k1"], json_load["k2"], json_load["k3"], json_load["k4"]]
        )
        W = json_load["image_width"]
        H = json_load["image_height"]

        ncm1 = np.array(json_load["new_camera_matrix_alpha1"]["data"])
        ncm1 = ncm1.reshape(
            [
                json_load["new_camera_matrix_alpha1"]["rows"],
                json_load["new_camera_matrix_alpha1"]["cols"],
            ]
        )
        # tslog.append(("read_fisheye_param end:",datetime.datetime.now()))
        return cm, dm, W, H, ncm1

    def __init__(
        self,
        rvec,
        tvec,
        intrinsic_path,
        app_logger_factory: AppLoggerFactory,
        debug=False,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        # 内部/外部パラメータ、シンボル準備
        _, self.dm, _, _, self.ncm1 = self.read_fisheye_param(intrinsic_path)

        self.Xw = sympy.Symbol("X_w")
        self.Yw = sympy.Symbol("Y_w")
        self.Zw = sympy.Symbol("Z_w")

        self.rvec = rvec
        self.tvec = tvec
        self.sym_t = sympy.Symbol("t")

        self.extrmat = np.zeros((4, 4))
        self.extrmat[0:3, 0:3] = cv2.Rodrigues(self.rvec)[0]
        self.extrmat[0:3, 3] = self.tvec.T
        self.extrmat[3, 3] = 1

        if debug:
            self._logger.info(f"回転・並進行列{self.extrmat}")

        self.resmat = (
            self.ncm1
            * sympy.Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
            * self.extrmat
        )

        self.res = self.resmat * sympy.Matrix([[self.Xw, self.Yw, self.Zw, 1]]).T

        if debug:
            self._logger.info(f"3D→2D変換行列{self.res}")  # 同次形式の3D→2D投影式
        # ここまでで3D→2D変換の行列が出た。ここまでは一般的な計算

        # 焦点座標invtvecの算出
        self.invtvec = np.matmul(cv2.Rodrigues(-self.rvec)[0], -self.tvec)
        if debug:
            self._logger.info(
                f"並進・回転ベクトルから算出した焦点座標:{self.invtvec}これを同次2D形式へ投影すると(x,y,0)に映る{self.res.subs([(self.Xw, float(self.invtvec[0])), (self.Yw, float(self.invtvec[1])), (self.Zw, float(self.invtvec[2]))])}"
            )

    def project2Dto3D(self, p_2d: np.array, debug=False) -> np.array:
        if debug:
            self._logger.info(f"計算例：同次2D点{p_2d}から投影したときの距離")

        Xraw = np.matmul(np.linalg.pinv(np.array(self.resmat, dtype=np.float32)), p_2d)
        Xraw = Xraw / Xraw[3]
        if debug:
            self._logger.info(f"2D→3D投影結果（１点）:{Xraw}")

        nL = (np.array(Xraw[0:3]).T - self.invtvec.T).T
        # L = invtvec+self.t*nL
        if debug:
            self._logger.info(f"2D→3D直線方向ベクトル{nL}")

        return nL  # , sympy.Matrix(L)

    def distance_linetopoint(self, nL, p2, debug=False) -> float:
        dotres = None
        t = sympy.Symbol("t")
        L = sympy.Matrix(self.invtvec + t * nL)
        for i in range(3):
            if dotres is None:
                dotres = nL[i] * (L[i] - p2[i])
            else:
                dotres += nL[i] * (L[i] - p2[i])
        tval = sympy.solve(dotres, t)
        if debug:
            self._logger.info(tval)
        tval = float(tval[0])
        calc3dpoint = L.subs([(t, tval)])
        if debug:
            self._logger.info(calc3dpoint)

        dist = np.linalg.norm(np.array(calc3dpoint, dtype=np.float32).ravel() - p2)
        if debug:
            self._logger.info(f"結果距離:{dist}")
        return dist, np.array(calc3dpoint, dtype=np.float32).ravel()
