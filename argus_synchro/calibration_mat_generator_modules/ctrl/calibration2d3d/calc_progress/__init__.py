import json
from collections.abc import Callable
from typing import Any

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.calc_progress import (
    block_summarize as blksum,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.calc_progress import (
    total_summarize as totsum,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import (
    Calib2d3dConf,
)

dtype_integrated_progress_info = tuple[
    float,
    list[tuple[int, float, float, tuple[float, dict[str, Any]]]],
    dict[str, NDArray],
]


class calc_progress_class:
    def __init__(
        self,
        calib2d3d_CalcProgress: Calib2d3dConf.CalcProgressConf,
        camerasel: int,
        verbose: bool,
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ):
        self._app_logger_factory = app_logger_factory
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.verbose: bool = verbose
        self.file_io_error_reporter = file_io_error_reporter
        self.calib2d3d_CalcProgress: Calib2d3dConf.CalcProgressConf = (
            calib2d3d_CalcProgress
        )
        (
            setting_params_list,
            grid_x_info,
            grid_y_info,
            block_groupid,
            id_threshold,
            common_zparam,
            sb_gui_to_sbindex,
            subblock_amount,
        ) = self.read_settings(camerasel)
        self.block_summarize_instlist = self._make_grid(
            setting_params_list,
            grid_x_info,
            grid_y_info,
            common_zparam,
            subblock_amount,
        )
        self.total_summarize_inst = self._setting_totalsummarizer(
            block_groupid, id_threshold
        )
        self.block_summarize_result = []
        self.sb_gui_to_sbindex: list[int] = sb_gui_to_sbindex

    def read_settings(self, camerasel: int):
        calib2d3d_areadefinition_filepath: str = (
            self.calib2d3d_CalcProgress.areadefinition_filepathes[camerasel]
        )
        try:
            with open(calib2d3d_areadefinition_filepath) as rf_json:
                setting_json = json.load(rf_json)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            if self.file_io_error_reporter is not None:
                self.file_io_error_reporter(
                    calib2d3d_areadefinition_filepath,
                    "read 2D-3D area definition JSON",
                    error,
                )
            raise

        grid_y_info = setting_json["grid_y_info"]
        grid_x_info = setting_json["grid_x_info"]
        block_param = setting_json["block_param"]
        subblock_amount = setting_json["subblock_amount"]

        blank_block_param = setting_json["blank_block_param"]
        id_threshold = setting_json["id_threshold"]
        common_zparam = setting_json["common_zparam"]
        sb_gui_to_sbindex = setting_json["sb_gui_to_sbindex"]

        self._logger.info(
            f"grid_y_info:{grid_y_info}, grid_x_info:{grid_x_info}, block_param:{block_param}, id_threshold:{id_threshold}",
        )

        # setting_params = {"sb_param":{"weight":1,"coeff":5.} }
        # grid_y_info = {"start":-12, "stop":12, "step":3}
        # grid_x_info = {"start":1, "stop":1+12, "step":3}

        block_groupid = [-1 for _ in range(32)]
        for d in block_param:
            block_groupid[d["bid"]] = d["gid"]
            self._logger.info(f"block_groupid[{d['bid']}] = {d['gid']}")

        setting_params_list = [blank_block_param for _ in range(32)]
        for d in block_param:
            setting_params_list[d["bid"]] = d
            self._logger.info(f"setting_params_list[{d['bid']}] = {d}")

        return (
            setting_params_list,
            grid_x_info,
            grid_y_info,
            block_groupid,
            id_threshold,
            common_zparam,
            sb_gui_to_sbindex,
            subblock_amount,
        )

    def _make_grid(
        self,
        setting_params_list,
        grid_x_info,
        grid_y_info,
        common_zparam,
        subblock_amount,
    ) -> list[tuple[float, float, blksum.block_summarize]]:
        (x_size, y_size), (x_center, y_center), (min_x2c, min_y2c), (minx2i, miny2i) = (
            blksum.block_summarize(
                verbose=self.verbose, app_logger_factory=self._app_logger_factory
            ).calc_xyshape(grid_x_info, grid_y_info)
        )

        block_summarize_instlist = []
        bid = 0

        for grid_y in np.linspace(
            grid_y_info["start"],
            grid_y_info["stop"],
            num=int(
                (grid_y_info["stop"] - grid_y_info["start"])
                / float(grid_y_info["step"])
            ),
            endpoint=False,
        ):
            for grid_x in np.linspace(
                grid_x_info["start"],
                grid_x_info["stop"],
                num=int(
                    (grid_x_info["stop"] - grid_x_info["start"])
                    / float(grid_x_info["step"])
                ),
                endpoint=False,
            ):
                block_summarize_instlist.append(
                    (
                        grid_x,
                        grid_y,
                        blksum.block_summarize(
                            verbose=self.verbose,
                            app_logger_factory=self._app_logger_factory,
                        ),
                    )
                )
                block_summarize_instlist[-1][2].setup(
                    (grid_x, grid_x + grid_x_info["step"]),
                    (grid_y, grid_y + grid_y_info["step"]),
                    (common_zparam["bottom"], common_zparam["top"]),
                    setting_params_list[bid],
                    subblock_amount,
                )
                bid += 1
        self._logger.info(
            f"block_summarize_instlist : {[(i, x) for i, x in enumerate(block_summarize_instlist)]}",
        )

        return block_summarize_instlist

    def _setting_totalsummarizer(self, block_groupid, id_threshold):
        total_summarize_inst = totsum.total_summarize()
        total_summarize_inst.setup(block_groupid, id_threshold)
        return total_summarize_inst

    def _data_import_and_calc(
        self, data3d: NDArray, points_per_time: int
    ) -> list[tuple[int, float, float, tuple[float, dict[str, Any]]]]:
        dtemp = data3d.reshape((-1, points_per_time, 3))
        # deval = (dtemp[:, 0] + dtemp[:, 1]) / points_per_time
        deval = dtemp.mean(axis=1)

        for i, _ in enumerate(self.block_summarize_instlist):
            self.block_summarize_instlist[i][2].data_import(deval)
        block_summarize_result = []
        for i, (ax, ay, x) in enumerate(self.block_summarize_instlist):
            block_summarize_result.append((i, ax, ay, x.calc_progress()))
        return block_summarize_result

    def calc_progress(
        self, data3d: NDArray, points_per_time: int
    ) -> dtype_integrated_progress_info:
        block_summarize_result = self._data_import_and_calc(
            data3d=data3d, points_per_time=points_per_time
        )
        total_results = self.total_summarize_inst.calc_progress(
            [x[3][0] for x in block_summarize_result]
        )
        return total_results[0], block_summarize_result, total_results[1]
