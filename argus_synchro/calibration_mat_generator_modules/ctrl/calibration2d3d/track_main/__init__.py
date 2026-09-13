from collections.abc import Callable
from typing import Optional

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D import (
    detect2d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D import (
    detect3d_class,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.target_selector import (
    target_selector,
)
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_store,
)
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.shared_app_config import SharedAppConfig
from argus_synchro.shared_errors import SharedErrors


class track_main_class:
    def __init__(
        self,
        sac: SharedAppConfig,
        app_config_calib: AppConfigCalibration,
        Mc: NDArray[np.float64],
        camera_index: int,
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
        shared_errors: SharedErrors | None = None,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.app_config_calib = app_config_calib

        self.ini_savemiddata_dir = app_config_calib.default.outputdir_root

        self.detect2d = detect2d_class(
            Mc=Mc,
            app_config_calib=app_config_calib,
            camera_index=camera_index,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=file_io_error_reporter,
            shared_errors=shared_errors,
        )
        self.detect3d = detect3d_class(
            app_config_calib=app_config_calib,
            sac=sac,
            camera_index=camera_index,
            app_logger_factory=app_logger_factory,
        )
        self.target_selector = target_selector(
            app_config_calib=app_config_calib,
            app_logger_factory=app_logger_factory,
        )

        self.resetflag_delay = False

        self.lastread_dataprop = None

        self.monitor_data = {}

    def __delattr__(self, name):
        self.close()

    def close(self) -> None:
        pass

    # detect: 継続可能時Trueを返す 終了時False(現状画像を表示した際にqボタンを押されたときしかこの条件を達成しない)
    def detect(self, indata):
        continueflag = True

        self.detect2d.detect(indata[0])
        # self.monitor_data[f"detect2d_image{0}"] = indata[0][0]
        self.monitor_data[f"detect2d_image{0}"] = self.detect2d.make_monitorimage(
            indata[0]
        )

        monitor_data_3D = self.detect3d.detect(data3d=indata[1])

        self.monitor_data.update(monitor_data_3D)

        self.monitor_data["detect3d_points_raw"] = indata[1][0]
        self.monitor_data["detect3d_points_raw_ts"] = indata[1][1]
        multi_points, multi_lines = self.detect3d.extract_rawbboxes()
        self.monitor_data["detect3d_multipoints"] = multi_points
        self.monitor_data["detect3d_multi_lines"] = multi_lines

        return continueflag

    def get_last_singleyoloBB(self) -> list[NDArray[np.float64]] | None:
        self.detect2d.get_last_singleyoloBB()

    def get_monitor_data(self):  # -> dict[Any, Any]:
        return self.monitor_data

    def set_progress(self, progress: float) -> None:
        self.target_selector.set_progress(progress=progress)

    def extract_fromcenter(
        self, frame_ix: int, recalc_bbox_index: bool = True
    ) -> tuple[
        tuple[NDArray[np.float64], NDArray[np.int32]],
        tuple[NDArray[np.float64], NDArray[np.int32]],
    ]:
        tracking2d_results = self.detect2d.get_tracking_results()
        tracking2d_results = self.target_selector.trackerresult_filter2d(
            tracking2d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )
        tracking3d_results = self.detect3d.get_tracking_results()
        tracking3d_results = self.target_selector.trackerresult_filter3d(
            tracking3d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )

        tracking2d_results, tracking3d_results = self.target_selector.compare(
            tracker_result2d_interface=tracking2d_results,
            frame2d_index=frame_ix,
            tracker_result3d_interface=tracking3d_results,
            frame3d_index=frame_ix,
        )

        tracking2d_results = self.target_selector.targetbbox2d_correction(
            tracking2d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )
        tracking3d_results = self.target_selector.targetbbox3d_correction(
            tracking3d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )

        debug_store("tracking2d_results", tracking2d_results)
        debug_store("tracking3d_results", tracking3d_results)

        corner2d_set = self.detect2d.extract_fromyolobb(
            tracker_result2d_interface=tracking2d_results, frame_ix=frame_ix
        )
        corner3d_set = self.detect3d.extract_fromcenter(
            tracker_result_interface=tracking3d_results, frame_ix=frame_ix
        )

        return corner2d_set, corner3d_set

    def extract_withaxis(
        self, rvec, tvec, frame_ix: int, recalc_bbox_index: bool = True
    ) -> tuple[
        tuple[NDArray[np.float64], NDArray[np.int32]],
        tuple[NDArray[np.float64], NDArray[np.int32]],
    ]:
        tracking2d_results = self.detect2d.get_tracking_results()
        tracking2d_results = self.target_selector.trackerresult_filter2d(
            tracking2d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )
        tracking3d_results = self.detect3d.get_tracking_results()
        tracking3d_results = self.target_selector.trackerresult_filter3d(
            tracking3d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )

        tracking2d_results, tracking3d_results = self.target_selector.compare(
            tracker_result2d_interface=tracking2d_results,
            frame2d_index=frame_ix,
            tracker_result3d_interface=tracking3d_results,
            frame3d_index=frame_ix,
        )

        tracking2d_results = self.target_selector.targetbbox2d_correction(
            tracking2d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )
        tracking3d_results = self.target_selector.targetbbox3d_correction(
            tracking3d_results,
            app_config_calib=self.app_config_calib,
            frame_ix=frame_ix,
        )

        corner2d_set = self.detect2d.extract_withaxis(
            rvec=rvec,
            tvec=tvec,
            tracker_result2d_interface=tracking2d_results,
            frame_ix=frame_ix,
        )
        corner3d_set = self.detect3d.extract(
            tracker_result_interface=tracking3d_results, frame_ix=frame_ix
        )

        debug_store("tracking_results", tracking2d_results, index=frame_ix)

        return corner2d_set, corner3d_set

    def make_monitor_image(self, indata) -> NDArray[np.uint8]:
        return self.detect2d.make_monitorimage(data2d=indata[0])

    def save_debugdata_2dtracker(self, openflag: str = "wb"):
        self.detect2d.save_debugdata(openflag=openflag)
