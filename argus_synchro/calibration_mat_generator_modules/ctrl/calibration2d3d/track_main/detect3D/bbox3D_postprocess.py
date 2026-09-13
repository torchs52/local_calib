import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    dtypeBBox3D,
    dtypePreprocess3d,
    tupleBBox_to_dtypeBBox,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration


def tupleBBoxset_to_dtypePreprocess3d(
    integrated_bboxinfoset: tuple[
        tuple[NDArray[np.float64], NDArray[np.int32], NDArray[np.float64]],
        NDArray[np.float64],
    ],
) -> dtypePreprocess3d:
    return tupleBBox_to_dtypeBBox(integrated_bboxinfoset[0]), integrated_bboxinfoset[1]


class bbox_postprocess:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.app_config_calib = app_config_calib
        self.xlength_range = (0.05, 1)
        self.ylength_range = (0.05, 1)
        self.zlength_range = (0.1, 3)

        self.verbose = not app_config_calib.default.print_disabled

    def apply(self, BBoxinfoset: dtypePreprocess3d) -> dtypePreprocess3d:
        multi_minmax = BBoxinfoset[0].multi_minmax.reshape(
            -1, 6
        )  # 形状でのエラーを避ける為

        if multi_minmax.shape[0] == 0:
            return BBoxinfoset

        if self.verbose:
            self._logger.info(f"{multi_minmax.shape=}")
        # 明らかに人でないbboxを無効化
        valid_flag = np.ones(multi_minmax.shape[0], dtype=np.bool_)
        if self.verbose:
            self._logger.info(f"{valid_flag.shape=}")
        for ix, (xmin, xmax, ymin, ymax, zmin, zmax) in enumerate(multi_minmax):
            if abs(xmax - xmin) > self.xlength_range[1]:
                valid_flag[ix] = False
            elif (
                abs(xmax - xmin) < self.xlength_range[0]
            ):  # かすれを考慮 点群は表面にしか出ない
                valid_flag[ix] = False

            if (
                abs(ymax - ymin) > self.ylength_range[1]
                or abs(ymax - ymin) < self.ylength_range[0]
            ):
                valid_flag[ix] = False

            if (
                abs(zmax - zmin) > self.zlength_range[1]
                or abs(zmax - zmin) < self.zlength_range[0]
            ):
                valid_flag[ix] = False

        return dtypePreprocess3d(
            (
                dtypeBBox3D(
                    multi_points=BBoxinfoset[0].multi_points[
                        np.repeat(valid_flag, 8), :
                    ],  # 8 / bbox
                    multi_lines=BBoxinfoset[0].multi_lines[
                        np.repeat(valid_flag, 12), :
                    ],  # 12 / bbox
                    multi_minmax=BBoxinfoset[0].multi_minmax[
                        valid_flag, :
                    ],  # 1x6 / bbox
                ),
                BBoxinfoset[1],
            )
        )
