from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

import argus_synchro.AccumulatePoints as accumulate_points_module
from argus_synchro.AccumulatePoints import accumulate_point, trim_accumulated_frames
from argus_synchro.SubScrutinizer import initialize_accum_counter


def test_reduced_load_trim_keeps_newest_frames() -> None:
    frames = deque(
        (np.array([[frame]], dtype=np.float64) for frame in range(4)),
        maxlen=4,
    )

    trim_accumulated_frames(frames, 2)

    assert [frame.item() for frame in frames] == [2.0, 3.0]
    assert frames.maxlen == 4

    frames.append(np.array([[4]], dtype=np.float64))
    frames.append(np.array([[5]], dtype=np.float64))
    assert [frame.item() for frame in frames] == [2.0, 3.0, 4.0, 5.0]


def test_accumulation_buffers_use_larger_physical_limit() -> None:
    lidar_grid = SimpleNamespace(
        side_max=1.0,
        side_min=-1.0,
        fwd_max=1.0,
        fwd_min=-1.0,
        grid_size=1.0,
    )
    accumulation = SimpleNamespace(
        max_accumulated_frames=4,
        max_accumulated_frames_reduced_load=2,
        max_accumulated_frames_ground=3,
        max_accumulated_frames_ground_reduced_load=5,
    )

    _, _, _, points, ground, _ = initialize_accum_counter(
        lidar_grid,
        accumulation,
    )

    assert points.maxlen == 4
    assert ground.maxlen == 5


def test_trim_rejects_non_positive_limit() -> None:
    frames: deque[np.ndarray] = deque()

    try:
        trim_accumulated_frames(frames, 0)
    except ValueError as error:
        assert str(error) == "max accumulated frames must be greater than zero"
    else:
        raise AssertionError("zero max frames must be rejected")


@pytest.mark.parametrize(
    ("is_reduced_load_mode", "points_limit", "ground_limit"),
    ((False, 3, 2), (True, 1, 1)),
)
def test_accumulate_point_keeps_mode_limit_after_append(
    monkeypatch: pytest.MonkeyPatch,
    is_reduced_load_mode: bool,
    points_limit: int,
    ground_limit: int,
) -> None:
    class FakePointCloud:
        def __init__(self, points: np.ndarray) -> None:
            self.points = points

        def voxel_down_sample(self, _: float) -> "FakePointCloud":
            return self

    monkeypatch.setattr(
        accumulate_points_module.utils,
        "np_to_pcd",
        lambda points: FakePointCloud(points),
    )
    monkeypatch.setattr(
        accumulate_points_module.utils,
        "pcd_to_np",
        lambda point_cloud: point_cloud.points,
    )

    lidar_grid = SimpleNamespace(
        side_max=1.0,
        side_min=-1.0,
        fwd_max=1.0,
        fwd_min=-1.0,
        grid_size=1.0,
    )
    accumulation = SimpleNamespace(
        max_accumulated_frames=3,
        max_accumulated_frames_reduced_load=1,
        max_accumulated_frames_ground=2,
        max_accumulated_frames_ground_reduced_load=1,
        voxel_size_for_accumulated_points=0.1,
        voxel_size_for_accumulated_points_reduced_load=0.2,
        voxel_size_for_accumulated_ground_points=0.1,
        voxel_size_for_accumulated_ground_points_reduced_load=0.2,
        increment_accum_counter=1,
        accum_counter_max_cap=10,
        decrement_speed_threshold=5,
        decrement_accum_counter=1,
        decrement_accum_counter_slow=1,
        prob_present_threshold=1,
        accum_counter_lower_reset_threshold=-10,
    )
    general_conf = SimpleNamespace(ground_height=0.0, ground_height_margin=0.0)
    counter = np.zeros((2, 2), dtype=np.int32)
    points: deque[np.ndarray] = deque(maxlen=3)
    ground: deque[np.ndarray] = deque(maxlen=2)
    accum_counter = -1

    for frame in range(4):
        xyz = np.array(
            [[frame / 10, 0.0, 1.0], [frame / 10, 0.0, -1.0]],
            dtype=np.float64,
        )
        counter, _, _, points, ground, accum_counter = accumulate_point(
            xyz,
            counter,
            points,
            ground,
            lidar_grid,
            accumulation,
            general_conf,
            is_edge_detection_applied=True,
            accum_counter=accum_counter,
            is_reduced_load_mode=is_reduced_load_mode,
        )
        assert len(points) <= points_limit
        assert len(ground) <= ground_limit

    np.testing.assert_array_equal(points[-1], xyz[[0]])
    np.testing.assert_array_equal(ground[-1], xyz[[1]])