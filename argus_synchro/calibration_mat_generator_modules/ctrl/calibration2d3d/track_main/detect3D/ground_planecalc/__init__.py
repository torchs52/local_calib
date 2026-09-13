import datetime
import os
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D import (
    ground_removal as grem,
)
from argus_synchro.calibration_mat_generator_modules.utils.utils3d import (
    np_to_pcd,
    pcd_to_np,
    set_xyz_range,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration


class ground_planecalc:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.points_stack = np.zeros((0, 3), dtype=np.float32)
        self.data_required = True
        self.app_config_calib = app_config_calib
        self.ransac_thresh = (
            self.app_config_calib.calib2d3d.Proc3d.groundplane_ransac_coeff
        )
        self.points_threshold = (
            self.app_config_calib.calib2d3d.Proc3d.groundplane_requiredpoints
        )

        self.accumulate_length = (
            self.app_config_calib.dataConverter2D3D.Lidar.accumulate_length
        )
        self.avoid_accumpts_count = (
            self.accumulate_length
        )  # 初めから蓄積フレーム数分は飛ばさなければ蓄積中の粗い点群をとってしまう
        self.ransac_resultlist = []
        self.debug_verb = not self.app_config_calib.default.print_disabled

        self._logger.info("__init__ called")

    def datreq(self) -> bool:
        return self.data_required

    def stack(self, points) -> None:
        if not self.data_required:
            return

        self.avoid_accumpts_count -= 1
        if self.avoid_accumpts_count > 0:
            return

        self.avoid_accumpts_count = int(self.accumulate_length)
        self.points_stack = np.vstack([self.points_stack, points])

        assert len(self.points_stack.shape) == 2
        assert self.points_stack.shape[1] == 3

        if len(self.points_stack) > self.points_threshold:
            self.data_required = False

    def calc_single(
        self,
        savefile_suffixstr,
        range_xyz: tuple[float, float, float, float, float, float] | None,
        thinning_div,
    ) -> NDArray[np.float64]:
        # range_xyz: xmin,xmax,ymin,ymax,zmin,zmax

        calctimemeasure_prevtime = datetime.datetime.now()
        target_points = self.points_stack
        if range_xyz is not None:
            target_points = set_xyz_range(
                self.points_stack,
                *[
                    (range_xyz[0], range_xyz[1]),
                    (range_xyz[2], range_xyz[3]),
                    (-1e6, 1e6),
                ],
            )

        if thinning_div > 1:
            points_pcd = np_to_pcd(target_points)
            points_pcd = points_pcd.farthest_point_down_sample(
                int(len(target_points) / thinning_div)
            )
            target_points = pcd_to_np(points_pcd)

        self._logger.info(
            f"{target_points.shape = }, {range_xyz = }, {self.points_stack.shape = }",
        )
        _ground, _non_ground, best_eq, len_best_inliers = grem.ransac(
            pc_np=target_points, thresh=self.ransac_thresh
        )
        self._logger.info(
            f"地面計算時間:{(datetime.datetime.now() - calctimemeasure_prevtime)}"
        )
        if self.debug_verb:
            with open(
                os.path.join(
                    self.app_config_calib.default.outputdir_root,
                    "ransac_plane_coeff_calc" + savefile_suffixstr + ".txt",
                ),
                mode="w",
            ) as planecoefftxt:
                print(f"{best_eq=},{len_best_inliers=}", file=planecoefftxt)
            np.savetxt(
                os.path.join(
                    self.app_config_calib.default.outputdir_root,
                    "ransac_plane_coeff_calc" + savefile_suffixstr + ".csv",
                ),
                best_eq,
            )
            np.save(
                os.path.join(
                    self.app_config_calib.default.outputdir_root,
                    "ransac_plane_points_calc" + savefile_suffixstr + ".npy",
                ),
                target_points,
            )
        return np.array(best_eq)

    def calc(self) -> None:
        self.ransac_resultlist = []

        frame_prevtime = datetime.datetime.now()

        self.ransac_resultlist.append(
            grem.ransac(pc_np=self.points_stack, thresh=self.ransac_thresh)[2:]
        )
        self._logger.info("計算時間:", (datetime.datetime.now() - frame_prevtime))
        frame_prevtime = datetime.datetime.now()

        with open(
            os.path.join(
                self.app_config_calib.default.outputdir_root,
                f"{os.getpid()}_{datetime.datetime.now().strftime('%Y%m%d_%H')}"
                "_ransac_plane_coeff_calc.txt",
            ),
            mode="w",
        ) as planecoefftxt:
            for best_eq, inliner_count in self.ransac_resultlist:
                print(best_eq, inliner_count, file=planecoefftxt)

    def plane_coeff(self, index: int):
        return self.ransac_resultlist[index][0]
