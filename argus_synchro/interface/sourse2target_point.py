from abc import ABC, abstractmethod

import numpy as np
import open3d as o3d
from open3d.geometry import PointCloud


class PaintColorInterface(ABC):
    @abstractmethod
    def get_transformed_data(
        self,
        source_transformed: PointCloud,
        target: PointCloud,
    ) -> PointCloud:
        pass


class UnifromColor(PaintColorInterface):
    def get_transformed_data(
        self,
        source_transformed: PointCloud,
        target: PointCloud,
    ) -> PointCloud:
        source_transformed.paint_uniform_color(np.array([0, 0, 0]) / 255)
        target.paint_uniform_color(np.array([0, 255, 0]) / 255)
        o3d.visualization.draw_geometries(
            geometry_list=[source_transformed, target],
            window_name="Merged PCD",
        )

        return source_transformed


class NormalColor(PaintColorInterface):
    def get_transformed_data(
        self,
        source_transformed: PointCloud,
        target: PointCloud,
    ) -> PointCloud:
        return source_transformed
