# ruff: noqa: ARG005, DTZ005, N802, PLR2004, SLF001
from __future__ import annotations

import ast
import datetime
import inspect
import json
from configparser import ConfigParser, ExtendedInterpolation
from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace
from typing import get_args

import numpy as np

from argus_synchro.calibration_mat_generator_modules.ctrl.wait_app import wait_app
from argus_synchro.calibration_mat_generator_modules.facade import CalibrationUIGodot
from argus_synchro.calibration_mat_generator_modules.facade.calib_godot_interface import (
    CalibGodotInterface,
)
from argus_synchro.config.app_config import CalibrationModeSwitchConf, GeneralConf
from argus_synchro.message.calib_fifo_message import FIFOData
from argus_synchro.message.input_message import CameraData, CanData, PointCloudData
from argus_synchro.process.calib_fifo_process import CalibFIFOProcess
from argus_synchro.process.message import MessageFlow
from argus_synchro.process.operation_mode import OPERATION_MODE, CalibMode

MMAP_ASSIGN_PATH = Path("config/calibration_mat_generator_modules/mmap_assign.json")
SETTINGS_PATH = Path("config/settings.ini")
MAIN_PATH = Path("argus_synchro/__main__.py")


class _SharedConfigStub:
    def read(self) -> SimpleNamespace:
        return SimpleNamespace(
            CalibUI_IF=SimpleNamespace(
                show_image2d3d=True,
                show_trajectory=False,
            )
        )


class _MmapWriterRecorder:
    def __init__(self) -> None:
        self.events: list[tuple[str, object]] = []

    def preprocess_info(self) -> None:
        self.events.append(("preprocess_info", None))

    def error_info(self, **kwargs: object) -> None:
        self.events.append(("error_info", kwargs["errorcode_pre"]))

    def WriteUInt8(self, value: int) -> None:
        self.events.append(("uint8", value))

    def WriteUInt32(self, value: int) -> None:
        self.events.append(("uint32", value))

    def WriteFloat32(self, value: float) -> None:
        self.events.append(("float32", value))

    def WriteFloat32Batch(self, values: np.ndarray) -> None:
        self.events.append(("float32_batch", values.copy()))

    def camera_img(self, frame: np.ndarray, yolo_result: np.ndarray) -> None:
        self.events.append(("camera_img", (frame, yolo_result)))

    def postprocess_info(self, ref_t: int | None, **kwargs: object) -> None:
        self.events.append(("postprocess_info", (ref_t, kwargs)))


class _FlagMmap:
    def __init__(self) -> None:
        self.values: dict[int, int] = {0: 0, 1: 0}

    def ReadInt8(self, address: int) -> int:
        return self.values[address]

    def ReadInt64(self, _address: int) -> int:
        return 0

    def WriteInt8(self, address: int, value: int) -> None:
        self.values[address] = value

    def WriteInt32(self, address: int, value: int) -> None:
        self.values[address] = value

    def WriteInt64(self, _address: int, _value: int) -> None:
        pass


def test_calibration_mode_setting_schema_and_values_are_stable() -> None:
    assert [field.name for field in fields(CalibrationModeSwitchConf)] == [
        "isRunning3D3Dcalib",
        "isRunning2D3Dcalib",
        "cameraID",
        "start2D3DCalibCalc",
        "isRunning2D3Dcheck",
        "start2D3DCheckCalc",
        "isRunningInterfaceDebug",
    ]
    assert GeneralConf.__annotations__["operation_mode"] is int
    assert OPERATION_MODE.SCRUT == 0
    assert OPERATION_MODE.CALIB == 1
    assert CalibMode.IsRunning3D3Dcalib == 1
    assert CalibMode.IsRunning2D3Dcalib == 2
    assert CalibMode.IsRunning2D3Dcheck == 3
    assert CalibMode.wait_app == 4

    settings = ConfigParser(interpolation=ExtendedInterpolation())
    assert settings.read(SETTINGS_PATH, encoding="utf-8") == [str(SETTINGS_PATH)]
    assert settings.getint("General", "operation_mode") in (0, 1)
    assert 0 <= settings.getint("CalibMode", "cameraID") <= 3
    for option in (
        "isRunning3D3Dcalib",
        "isRunning2D3Dcalib",
        "start2D3DCalibCalc",
        "isRunning2D3Dcheck",
        "start2D3DCheckCalc",
        "isRunningInterfaceDebug",
    ):
        assert isinstance(settings.getboolean("CalibMode", option), bool)


def test_facade_keeps_calibration_error_overwrite_hook() -> None:
    parameter = inspect.signature(CalibrationUIGodot.set_dummydata).parameters[
        "overwrite_caliberror"
    ]

    assert parameter.annotation == "bool"
    assert parameter.default is True


def test_wait_app_transmits_shi_dummy_data() -> None:
    controller = object.__new__(wait_app)
    controller.debug_index = 7
    controller.verbose = False
    controller.input_data_diagnosis = lambda *args: False
    dummy_calls: list[dict[str, object]] = []
    transmit_calls: list[dict[str, object]] = []
    monitor = SimpleNamespace(
        set_dummydata=lambda **kwargs: dummy_calls.append(kwargs),
        transmit_setdata=lambda **kwargs: transmit_calls.append(kwargs),
    )
    sec = SimpleNamespace()

    assert controller.dataproc(
        readresult_pop=([], [], (0, 0.0), 0),
        monitor=monitor,
        sec=sec,
    )

    assert dummy_calls == [
        {
            "enable_systemerrorflag": True,
            "enable_errorflag": True,
            "enable_yawangle": True,
            "overwrite_checkresult": True,
            "overwrite_calibresult": True,
        }
    ]
    assert transmit_calls == [{"sec": sec, "ref_t": 7}]
    assert controller.debug_index == 8


def test_calibration_fifo_data_has_four_ordered_elements() -> None:
    assert len(get_args(FIFOData)) == 4
    camera_data: list[tuple[np.ndarray, int, float]] = []
    lidar_data: list[tuple[np.ndarray, int, float]] = []
    can_data = (15, 123.5)
    ref_t = 42

    fifo_data = (camera_data, lidar_data, can_data, ref_t)

    assert fifo_data[0] is camera_data
    assert fifo_data[1] is lidar_data
    assert fifo_data[2] == (15, 123.5)
    assert fifo_data[3] == 42


def test_calibration_fifo_process_returns_synchronized_data_in_contract_order() -> None:
    process = object.__new__(CalibFIFOProcess)
    process._fps_prof = SimpleNamespace(enter=lambda: None, prof=lambda **kwargs: None)
    process._begin_time = datetime.datetime.now()
    process._loop_count = 0
    process._final_tsfilter = []
    process._camera_tsfilter_diff = np.ones(1, dtype=np.int64)
    process._lidar_tsfilter_diff = np.ones(1, dtype=np.int64)
    process._camera_datalist = [None]
    process._lidar_datalist = [None]
    process.sensor_sync_filter_inst = SimpleNamespace(
        datasync_filtering=lambda data, verbose: (True, [1, 1], 0)
    )
    process._app_config_calib = SimpleNamespace(
        default=SimpleNamespace(print_disabled=True)
    )
    process._last_print_time = datetime.datetime.now()
    process._ref_t = 42
    process._end_frame = 100
    process._logger = SimpleNamespace(warning=lambda message: None)
    camera_image = np.full((2, 3, 3), 7, dtype=np.uint8)
    lidar_points = np.array([[1.0, 2.0, 3.0]], dtype=np.float64)

    fifo_data = process._update(
        pcd_input_data=(PointCloudData(frame=20, time=2.5, point_cloud=lidar_points),),
        can_input_data=CanData(
            yaw_angle_deg=15.5,
            lever_pressure=np.zeros(2, dtype=np.float16),
            frame=30,
            time=3.5,
        ),
        camera_input_data=(
            CameraData(index=0, frame=10, time=1.5, image=camera_image),
        ),
    )

    assert fifo_data[0] == [(camera_image, 0, 1.5)]
    assert fifo_data[1] == [(lidar_points, 20, 2.5)]
    assert fifo_data[2] == (15.5, 3.5)
    assert fifo_data[3] == 42


def test_message_flow_allows_only_one_consumer() -> None:
    source = inspect.getsource(MessageFlow.create_consumer)

    assert "if self._is_created_consumer.value:" in source
    assert "raise RuntimeError" in source


def test_calibration_and_surround_pipelines_keep_separate_input_flows() -> None:
    module = ast.parse(MAIN_PATH.read_text(encoding="utf-8"))
    functions = {
        node.name: node
        for node in module.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {"start_calib_pipeline", "start_scrut_pipeline"}
    }

    for function_name in ("start_calib_pipeline", "start_scrut_pipeline"):
        calls = [
            node
            for node in ast.walk(functions[function_name])
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "create_input_message"
        ]
        assert len(calls) == 1


def test_calibration_mmap_header_layout_is_stable() -> None:
    layout = json.loads(MMAP_ASSIGN_PATH.read_text(encoding="utf-8"))

    assert layout == {
        "Start_ADDR": 0,
        "IsWriting": {"ADDR": 0, "WIDTH": 1},
        "IsReading": {"ADDR": 1, "WIDTH": 1},
        "UNIX_TIME": {"ADDR": 2, "WIDTH": 8},
        "ERROR": {"ADDR": 10, "WIDTH": 4},
        "CamImg": {"ADDR": 14, "WIDTH": -1},
    }


def test_calibration_mmap_legacy_error_field_is_always_zero() -> None:
    buffer = _FlagMmap()
    interface = object.__new__(CalibGodotInterface)
    interface.output_log = False
    interface.datPathList = []
    interface.damp_out = False
    interface.clsMMap = buffer
    interface.writtenAdr = 10

    interface.error_info(
        sec=SimpleNamespace(Scruti_ex=SimpleNamespace(IsSlow=SimpleNamespace(value=2))),
        errorcode_pre=0xFFFFFFFF,
    )

    assert buffer.values[10] == 0
    assert interface.writtenAdr == 14


def test_calibration_mmap_rotation_immediately_reserves_next_buffer() -> None:
    buffers = [_FlagMmap(), _FlagMmap()]
    interface = object.__new__(CalibGodotInterface)
    interface.output_log = False
    interface.s_frame = 1
    interface.ref_t_count = None
    interface.mapIndex = 0
    interface.datPathList = []
    interface.damp_out = False
    interface.clsMMap_list = buffers
    interface.clsMMap = buffers[0]
    interface.IsReading_ADR = 1
    interface.IsWriting_ADR = 0
    interface.UNIX_TIME_ADDR = 2
    interface.Start_ADR = 0
    interface.writtenAdr = 10
    interface.sTime = 0.0
    interface.preProcessTime = 0.0

    interface.postprocess_info(1, is_firstframe=True)

    assert interface.mapIndex == 1
    assert buffers[0].values[interface.IsWriting_ADR] == 0
    assert buffers[1].values[interface.IsWriting_ADR] == 1
    assert interface.writtenAdr == interface.Start_ADR


def test_calibration_common_fields_are_written_in_ui_contract_order() -> None:
    monitor = object.__new__(CalibrationUIGodot)
    writer = _MmapWriterRecorder()
    monitor.calibGodotInterfaceInst = writer
    monitor.output_log = False
    monitor.camera_num = 0
    monitor.sac = _SharedConfigStub()
    empty_points = np.empty((0, 3), dtype=np.float32)

    monitor.transmit(
        sec=object(),
        ref_t=99,
        is_end_calmode=1,
        status_calibcommon=2,
        errors_calibcommon=0x10203040,
        currentmode=3,
        currentcamera=2,
        frames=[],
        YOLOresults=[],
        yaw=4.5,
        points=empty_points,
        corner3d=empty_points,
        progress_summary=0.75,
        is_calib_available=True,
        calibcheck_status=[],
        calib_status=[],
        mblock_progress_status=[],
        sblock_progress_status=[],
    )

    assert writer.events[:7] == [
        ("preprocess_info", None),
        ("error_info", 0),
        ("uint8", 1),
        ("uint8", 2),
        ("uint8", 3),
        ("uint8", 2),
        ("uint32", 0x10203040),
    ]
