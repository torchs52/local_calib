import numpy as np
from numpy.typing import NDArray


class calc_headpoint_z:
    def __init__(
        self,
        xrange: tuple[float, float],
        yrange: tuple[float, float],
        grid_axis: str = "x",
    ) -> None:
        if grid_axis not in ("x", "y"):
            raise ValueError("grid_axis must be 'x' or 'y'")
        self.clear(xrange=xrange, yrange=yrange)
        self.grid_axis = grid_axis

    def clear(self, xrange: tuple[float, float], yrange: tuple[float, float]) -> None:
        self.xrange = xrange
        self.yrange = yrange

    def apply(self, corrpoint3d_set: NDArray[np.float64]) -> float:
        ph = corrpoint3d_set[:, 0, :]
        mask = (
            (ph[:, 0] > self.xrange[0])
            & (ph[:, 0] < self.xrange[1])
            & (ph[:, 1] > self.yrange[0])
            & (ph[:, 1] < self.yrange[1])
        )
        return float(np.median(ph[mask, 2]))

    def apply_per_point(
        self, corrpoint3d_set: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Replace each head z with the median of its 1 m grid section."""
        head_points = corrpoint3d_set[:, 0, :]
        corrected_z = head_points[:, 2].copy()
        range_mask = (
            (head_points[:, 0] > self.xrange[0])
            & (head_points[:, 0] < self.xrange[1])
            & (head_points[:, 1] > self.yrange[0])
            & (head_points[:, 1] < self.yrange[1])
        )
        valid_mask = range_mask & np.isfinite(head_points[:, 2])
        if not np.any(valid_mask):
            return corrected_z

        axis_index = 0 if self.grid_axis == "x" else 1
        grid_indices = np.floor(head_points[:, axis_index]).astype(np.int64)
        fallback_z = float(np.median(head_points[valid_mask, 2]))
        for grid_index in np.unique(grid_indices[range_mask]):
            grid_mask = valid_mask & (grid_indices == grid_index)
            if np.any(grid_mask):
                corrected_z[grid_mask] = np.median(head_points[grid_mask, 2])
            else:
                corrected_z[range_mask & (grid_indices == grid_index)] = fallback_z

        return corrected_z
