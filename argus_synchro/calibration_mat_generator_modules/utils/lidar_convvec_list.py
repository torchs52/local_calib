from dataclasses import dataclass


# 全体
@dataclass(frozen=True)
class lidar_convvec_list:
    cam0: list[tuple[list[float], list[float]]] = [
        ([0.0, -2.5, 0.0], [180.0, 0.0, 0.0]),
        ([0.0, -2.5, 0.0], [180.0, 0.0, 0.0]),
    ]
    cam1: list[tuple[list[float], list[float]]] = [
        ([0.0, -4.3, 0.0], [180.0, 0.0, 90.0]),
        ([0.0, -4.3, 0.0], [180.0, 0.0, 90.0]),
    ]
    cam2: list[tuple[list[float], list[float]]] = [
        ([0.0, -2.5, 0.0], [180.0, 0.0, 180.0]),
        ([0.0, -2.5, 0.0], [180.0, 0.0, 180.0]),
    ]
