import numpy as np
from argus_synchro_lib.detect3d import bounding_box, dbscan
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    dtype3DArea_xxyyzz,
    dtypeTupleBBox,
)
from argus_synchro.calibration_mat_generator_modules.utils.utils3d import (
    set_xyz_range,
)


def make_BBox3D(
    pcd: NDArray[np.float64],
    datarange_xyz: dtype3DArea_xxyyzz,
    dbscan_eps: float,
    dbscan_min_samples: int,
) -> tuple[dtypeTupleBBox, NDArray[np.float64]]:
    data_array = set_xyz_range(pcd, *datarange_xyz)

    if data_array.shape[0] < 10:
        return (
            np.zeros((0, 3)),
            np.zeros((0, 3), dtype=np.int32),
            np.zeros((0, 3)),
        ), data_array
    labels = dbscan(data_array, eps=dbscan_eps, min_samples=dbscan_min_samples)

    return bounding_box(data_array, np.unique(labels), labels), data_array
