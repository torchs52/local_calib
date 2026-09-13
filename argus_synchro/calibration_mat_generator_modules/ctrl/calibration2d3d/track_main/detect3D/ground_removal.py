import numpy as np
import pyransac3d as pyrsc
from numpy.typing import NDArray


def ransac(
    pc_np, thresh=0.1
) -> tuple[NDArray[np.float64], NDArray[np.float64], list, int]:
    # ground removal
    plane = pyrsc.Plane()
    best_eq, best_inliers = plane.fit(pts=pc_np, thresh=thresh)
    ground = pc_np[best_inliers]

    non_ground_idx = np.ones(pc_np.shape[0], dtype=bool)
    non_ground_idx[best_inliers] = False
    non_ground = pc_np[non_ground_idx]
    return ground, non_ground, best_eq, len(best_inliers)
