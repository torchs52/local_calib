# pyright: reportAttributeAccessIssue=false, reportPrivateUsage=false

from types import SimpleNamespace

from argus_synchro.process.can_process import CanDataProviderProcess
from argus_synchro.process.get_data_process import GetDataProcess
from argus_synchro.process.image_process import CameraProviderProcess
from argus_synchro.process.points_process import PointsProviderProcess
from argus_synchro.provider.can_data import CanFileProvider
from argus_synchro.provider.image import Mcde7000FileImageProvider
from argus_synchro.provider.point_cloud import Mid360FilePointCloudProvider
from argus_synchro.shared_errors import StateErrorDIndex


class _FileDevice:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def change_file_name_index(self, *args: object) -> None:
        self.calls.append(args)


def _app_config(
    operation_mode: int = 0, file_input_loop: bool = True
) -> SimpleNamespace:
    scrutinizer = SimpleNamespace(
        s_frame=3,
        e_frame=5,
        file_input_loop=file_input_loop,
        v0_file="camera0.mp4",
        v1_file="camera1.mp4",
        v2_file="camera2.mp4",
    )
    lidar = SimpleNamespace(
        lidar0_file="lidar0_",
        lidar1_file="lidar1_",
        lidar2_file="lidar2_",
        lidar3_file="lidar3_",
        lidar4_file="lidar4_",
        lidar5_file="lidar5_",
    )
    return SimpleNamespace(
        General=SimpleNamespace(operation_mode=operation_mode),
        DEFAULT=SimpleNamespace(File_Input=True),
        Scrutinizer=scrutinizer,
        Lidar=lidar,
        CAN=SimpleNamespace(c_file="can.csv"),
    )


def test_surround_file_inputs_restart_at_s_frame_after_e_frame() -> None:
    config = _app_config()

    camera_device = _FileDevice()
    camera = object.__new__(CameraProviderProcess)
    camera._app_config = config
    camera._scrutinizer_conf = config.Scrutinizer
    camera._file_input = True
    camera._frame = 6
    camera._end_frame = 5
    camera._index = 1
    camera._provider = Mcde7000FileImageProvider(camera_device, 1, 1)

    points_device = _FileDevice()
    points = object.__new__(PointsProviderProcess)
    points._app_config = config
    points._file_input = True
    points._frame = 6
    points._end_frame = 5
    points._index = 1
    points._provider = Mid360FilePointCloudProvider(points_device, 6)

    can_device = _FileDevice()
    can = object.__new__(CanDataProviderProcess)
    can._app_config = config
    can._frame = 6
    can._end_frame = 5
    can._provider = CanFileProvider(can_device, 6)
    can_diagnosis_calls: list[tuple[bool]] = []
    can._ser = SimpleNamespace(
        state_errors_D={
            StateErrorDIndex.FILE_IO_ERROR: SimpleNamespace(
                errors_diagnosis=lambda *args: can_diagnosis_calls.append(args)
            )
        }
    )

    camera._restart_surround_file_input_if_needed()
    points._restart_surround_file_input_if_needed()
    can._restart_surround_file_input_if_needed()

    assert camera._frame == 3
    assert camera_device.calls == [("camera1.mp4", 3)]
    assert points._frame == 3
    assert points._provider._ref_t == 3
    assert points_device.calls == [("lidar1_",)]
    assert can._frame == 3
    assert can._provider._ref_t == 3
    assert can_device.calls == [("can.csv",)]
    assert can_diagnosis_calls == [(False,)]


def test_file_input_loop_does_not_change_calibration_or_disabled_loop() -> None:
    for config in (
        _app_config(operation_mode=1),
        _app_config(file_input_loop=False),
    ):
        camera_device = _FileDevice()
        camera = object.__new__(CameraProviderProcess)
        camera._app_config = config
        camera._scrutinizer_conf = config.Scrutinizer
        camera._file_input = True
        camera._frame = 6
        camera._end_frame = 5
        camera._index = 0
        camera._provider = Mcde7000FileImageProvider(camera_device, 1, 1)

        camera._restart_surround_file_input_if_needed()

        assert camera._frame == 6
        assert camera_device.calls == []


def test_get_data_loops_only_for_surround_file_input() -> None:
    for file_input, file_input_loop, expected_frame, unsubscribe_count in (
        (True, True, 3, 0),
        (True, False, 6, 1),
        (False, True, 6, 1),
    ):
        process = object.__new__(GetDataProcess)
        config = _app_config(file_input_loop=file_input_loop)
        config.DEFAULT.File_Input = file_input
        process._app_config = config
        process._ref_t = 5
        process._end_frame = 5
        unsubscribed: list[bool] = []
        process._unsubscribe = lambda: unsubscribed.append(True)

        process._advance_frame()

        assert process._ref_t == expected_frame
        assert len(unsubscribed) == unsubscribe_count