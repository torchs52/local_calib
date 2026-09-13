import copy
from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.filters.bbox2D_edgefilter import (
    bbox2D_edgefilter,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.filters.bbox2D_shapefilter_revproj import (
    bbox2D_shapefilter,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.filters.bbox2Dmask_byimage import (
    bbox2Dmask_byimage,
)
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_store,
)
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration


class bbox2D_postprocess:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        Mc: NDArray[np.float64],
        camera_index: int,
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        self.bbsf = bbox2D_shapefilter(
            Mc=Mc, app_config_calib=app_config_calib, camera_index=camera_index
        )
        self.bbmf = bbox2Dmask_byimage(
            app_config_calib=app_config_calib,
            camera_index=camera_index,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=file_io_error_reporter,
        )
        self.bbef = bbox2D_edgefilter(
            app_config_calib=app_config_calib, camera_index=camera_index
        )

    def filter_bbox(
        self,
        allframe_bbox: list[NDArray[np.float64]] | None,
    ) -> list[NDArray[np.float64]] | None:
        result = self.bbef.filter_bbox_inside(yolo_results=allframe_bbox)
        debug_store("filter_bbox_inside", copy.copy(result))
        result = self.bbmf.filter_validbbox_byimg(yolo_results=result)
        debug_store("filter_validbbox_byimg", copy.copy(result))
        result = self.bbsf.filter_bbox(allframe_bbox=result)
        debug_store("filter_bbox", copy.copy(result))

        return result
