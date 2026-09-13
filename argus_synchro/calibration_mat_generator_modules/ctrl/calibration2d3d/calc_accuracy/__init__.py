import cv2
import numpy as np

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.calc_accuracy import (
    accuracy_tools as acctools,
)


class calc_accuracy_class:
    def __init__(self):
        self._read_settings()

    def _read_settings(self):
        self.divcount = 5
        self.ncm1 = None
        self.distCoeffs = np.zeros(5)

    def set_cameramatrix(self, ncm):
        self.ncm1 = ncm

    def LOOCV_bytime(self, p2d, p3d):
        if self.ncm1 is None:
            raise RuntimeError("no camera matrix")

        resultlist = []
        for n in range(self.divcount):
            labels = acctools.random_split_time_series(len(p2d), self.divcount)
            train2d = p2d[labels != n]
            train3d = p3d[labels != n]
            test2d = p2d[labels == n]
            test3d = p3d[labels == n]

            success, rvec, tvec = cv2.solvePnP(
                train3d.astype(np.float32).reshape(-1, 3),
                train2d.astype(np.float32).reshape(-1, 2),
                self.ncm1,
                self.distCoeffs,
            )
            acc = acctools.calc_accuracy_proj3dto2d(
                rvec, tvec, test2d, test3d, self.ncm1, self.distCoeffs
            )

            resultlist.append(acc)

        return np.array(resultlist).std()  # 標準偏差が大きいほど精度が悪い
