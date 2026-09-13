from __future__ import annotations

import argparse
import enum
import json
import multiprocessing as mp
import time
import traceback
from datetime import datetime
from multiprocessing import Process
from pathlib import Path
from typing import TYPE_CHECKING, cast

from argus_synchro import machine_profile
from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.core.closable import CompositeClosable
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import (
    ResultDiagnosis,
    StateErrorDiagnosisD,
)
from argus_synchro.diagnosis.state_d_errors import (
    LogCompressionFailure,
    LogTimeReversal,
)
from argus_synchro.edge_det.base import EdgeDetectionResult
from argus_synchro.process import MessageFlow, SyncType
from argus_synchro.process.error_monitor_process import ErrorMonitorProcess
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.process.operation_mode import CalibMode
from argus_synchro.process.process import ProcessBase, ProcessManager
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.shared_app_config import SharedAppConfig, SharedAppConfigCalibration
from argus_synchro.shared_errors import (
    ActionErrorIndex,
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
    diagnose_startup_calibration_data,
)
from argus_synchro.shared_excepts import SharedExcepts
from argus_synchro.SystemMonitor import MonitorArgus
from argus_synchro.SystemMonitor.status_mmap import (
    StatusCode,
    StatusMMAP,
    setup_signal_handlers,
)

if TYPE_CHECKING:
    from argus_synchro.edge_det.base import EdgeDetectionResult
    from argus_synchro.message.calib_fifo_message import FIFOData
    from argus_synchro.message.input_message import (
        CameraData,
        CanData,
        ImuData,
        PointCloudData,
    )
    from argus_synchro.message.scrutinizer_message import (
        AccumPointsData,
        CameraDetectionsData,
        CanAngleData,
        CanLeverData,
    )


def _get_calibration_mat_generator_module():
    # spawn 子プロセスで不要な TensorFlow import を避けるため遅延 import する
    from argus_synchro import calibration_mat_generator

    return calibration_mat_generator


class MessageIndex(enum.IntEnum):
    CAMERA = 0
    PCD = 1
    IMU = 2
    CAN = 3

    CAN_ANGLE = 2
    CAN_LEVER = 3

    FRAME = 0
    CAM_DET = 1

    REMOVE_POINT = 0
    DELTA_YAW = 1

    ACCUM = 0
    CLIFF = 1


def create_input_message(
    closables: CompositeClosable,
    app_config: AppConfig,
    process_activator: ProcessActivator,
    sync_type: SyncType,
) -> tuple[
    tuple[MessageFlow[CameraData], ...],
    tuple[MessageFlow[PointCloudData], ...],
    tuple[MessageFlow[ImuData], ...],
    MessageFlow[CanData],
]:
    """
    入力プロセスで使用するメッセージフローを作成する
    """
    from argus_synchro.message.input_message import (
        CameraMessage,
        CanDataMessage,
        ImuMessage,
        PCDDataMessage,
    )

    # カメラ画像のメッセージ
    camera_flows: tuple[MessageFlow[CameraData], ...] = tuple(
        MessageFlow(
            message=CameraMessage().add_to(closables),
            activator=process_activator,
            sync_type=sync_type,
        ).add_to(closables)
        for _ in range(app_config.camera.count)
    )

    # LiDAR(点群)のメッセージ
    lidar_flows: tuple[MessageFlow[PointCloudData], ...] = tuple(
        MessageFlow(
            message=PCDDataMessage().add_to(closables),
            activator=process_activator,
            sync_type=sync_type,
        ).add_to(closables)
        for _ in range(app_config.Lidar.count)
    )

    # LiDAR(IMU)のメッセージ
    imu_flows: tuple[MessageFlow[ImuData], ...] = tuple(
        MessageFlow(
            message=ImuMessage().add_to(closables),
            activator=process_activator,
            sync_type=SyncType.LATEST,
        ).add_to(closables)
        for _ in range(app_config.Lidar.count)
    )

    # CanDataのメッセージ
    candata_flow: MessageFlow[CanData] = MessageFlow(
        CanDataMessage().add_to(closables),
        activator=process_activator,
        sync_type=sync_type,
    ).add_to(closables)

    return camera_flows, lidar_flows, imu_flows, candata_flow


def create_calib_fifo_message(
    closables: CompositeClosable,
    app_config: AppConfig,
    process_activator: ProcessActivator,
    sync_type: SyncType,
) -> MessageFlow[FIFOData]:
    """
    Calibration用のFIFOプロセスで使用するメッセージフローを作成する
    """
    from argus_synchro.message.calib_fifo_message import CalibFIFOMessage

    # FIFOのメッセージ
    calib_fifo_flow: MessageFlow[FIFOData] = MessageFlow(
        message=CalibFIFOMessage().add_to(closables),
        activator=process_activator,
        sync_type=sync_type,
    ).add_to(closables)

    return calib_fifo_flow


def create_scrutinizer_message(
    closables: CompositeClosable,
    app_config: AppConfig,
    process_activator: ProcessActivator,
    sync_type: SyncType,
) -> tuple[
    tuple[MessageFlow[CameraData], ...],
    tuple[MessageFlow[PointCloudData], ...],
    MessageFlow[CanAngleData],
    MessageFlow[CanLeverData],
]:
    """
    Scrutinizerで使用するメッセージフローを作成する
    """
    from argus_synchro.message.input_message import CameraMessage, PCDDataMessage
    from argus_synchro.message.scrutinizer_message import (
        CanAngleMessage,
        CanLeverMessage,
    )

    # カメラ画像のメッセージ
    camera_flows: tuple[MessageFlow[CameraData], ...] = tuple(
        MessageFlow(
            message=CameraMessage().add_to(closables),
            activator=process_activator,
            sync_type=sync_type,
        ).add_to(closables)
        for _ in range(app_config.camera.count)
    )

    # LiDAR(点群)のメッセージ
    lidar_flows: tuple[MessageFlow[PointCloudData], ...] = tuple(
        MessageFlow(
            message=PCDDataMessage().add_to(closables),
            activator=process_activator,
            sync_type=sync_type,
        ).add_to(closables)
        for _ in range(app_config.Lidar.count)
    )

    # CanDataのメッセージ
    canangle_flow: MessageFlow[CanAngleData] = MessageFlow(
        message=CanAngleMessage().add_to(closables),
        activator=process_activator,
        sync_type=sync_type,
    ).add_to(closables)

    # CanDataのメッセージ
    canlever_flow: MessageFlow[CanLeverData] = MessageFlow(
        CanLeverMessage().add_to(closables),
        activator=process_activator,
        sync_type=sync_type,
    ).add_to(closables)

    return (
        camera_flows,
        lidar_flows,
        canangle_flow,
        canlever_flow,
    )


def create_undisimage_message(
    closables: CompositeClosable,
    app_config: AppConfig,
    process_activator: ProcessActivator,
    sync_type: SyncType,
) -> MessageFlow[CameraDetectionsData]:
    from argus_synchro.Camera import SyscamRes
    from argus_synchro.message.scrutinizer_message import CameraDetectionsDataMessage

    bb_box_flows: MessageFlow[CameraDetectionsData] = MessageFlow(
        message=CameraDetectionsDataMessage(
            app_config.camera.count,
            SyscamRes(
                app_config.camera.sys_width,
                app_config.camera.sys_height,
            ),
        ).add_to(closables),
        activator=process_activator,
        sync_type=sync_type,
    ).add_to(closables)

    return bb_box_flows


def create_points_refine_message(
    closables: CompositeClosable,
    app_config: AppConfig,
    process_activator: ProcessActivator,
    sync_type: SyncType,
) -> tuple[MessageFlow[AccumPointsData], MessageFlow[EdgeDetectionResult]]:
    from argus_synchro.message.scrutinizer_message import (
        AccumPointsDataMessage,
        CreateEdgeDetectionResultMessage,
    )

    accum_flow: MessageFlow[AccumPointsData] = MessageFlow(
        message=AccumPointsDataMessage(app_config).add_to(closables),
        activator=process_activator,
        sync_type=sync_type,
    ).add_to(closables)

    edge_detection_flow: MessageFlow[EdgeDetectionResult] = MessageFlow(
        message=CreateEdgeDetectionResultMessage().add_to(closables),
        activator=process_activator,
        sync_type=sync_type,
    ).add_to(closables)

    return accum_flow, edge_detection_flow


def create_input_prosess(
    processes: ProcessManager,
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        tuple[MessageFlow[ImuData], ...],
        MessageFlow[CanData],
    ],
    activator: ProcessActivator,
    sac_calib: SharedAppConfigCalibration,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
) -> None:
    """
    入力プロセスを作成する(カメラ、LiDAR(点群)、LiDAR(IMU)、CAN)
    """
    from argus_synchro.process.can_process import CanDataProviderProcess
    from argus_synchro.process.image_process import CameraProviderProcess
    from argus_synchro.process.imu_process import ImuProviderProcess
    from argus_synchro.process.points_process import PointsProviderProcess

    camera_flows: tuple[MessageFlow[CameraData], ...] = input_message_flows[
        MessageIndex.CAMERA.value
    ]
    for i in range(len(camera_flows)):
        CameraProviderProcess(
            i,
            sec.CAM_ex[i],
            sac_calib,
            sac,
            ser,
            camera_flows[i],
            activator,
            f"CameraProviderProcess[{i}]",
        ).add_to(processes)

    lidar_flows: tuple[MessageFlow[PointCloudData], ...] = input_message_flows[
        MessageIndex.PCD.value
    ]
    for i in range(len(lidar_flows)):
        PointsProviderProcess(
            i,
            sec.LiDAR_ex[i],
            sac_calib,
            sac,
            ser,
            lidar_flows[i],
            activator,
            f"PointsProviderProcess[{i}]",
        ).add_to(processes)

    imu_flows: tuple[MessageFlow[ImuData], ...] = input_message_flows[
        MessageIndex.IMU.value
    ]
    for i in range(len(lidar_flows)):
        ImuProviderProcess(
            i,
            sec.IMU_ex[i],
            sac,
            ser,
            imu_flows[i],
            activator,
            f"ImuProviderProcess[{i}]",
        ).add_to(processes)

    candata_flow: MessageFlow[CanData] = input_message_flows[MessageIndex.CAN.value]
    CanDataProviderProcess(
        0,
        sec.CAN_ex,
        sac_calib,
        sac,
        ser,
        candata_flow,
        activator,
        "CanDataProviderProcess",
    ).add_to(processes)


def create_calib_input_prosess(
    processes: ProcessManager,
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        tuple[MessageFlow[ImuData], ...],
        MessageFlow[CanData],
    ],
    activator: ProcessActivator,
    sac_calib: SharedAppConfigCalibration,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
) -> None:
    """
    入力プロセスを作成する(カメラ、LiDAR(点群)、LiDAR(IMU)、CAN)
    """
    from argus_synchro.process.can_process import CanDataProviderProcess
    from argus_synchro.process.image_process import CameraProviderProcess
    from argus_synchro.process.points_process import PointsProviderProcess

    camera_flows: tuple[MessageFlow[CameraData], ...] = input_message_flows[
        MessageIndex.CAMERA.value
    ]
    for i in range(len(camera_flows)):
        CameraProviderProcess(
            i,
            sec.CAM_ex[i],
            sac_calib,
            sac,
            ser,
            camera_flows[i],
            activator,
            f"CameraProviderProcess[{i}]",
        ).add_to(processes)

    lidar_flows: tuple[MessageFlow[PointCloudData], ...] = input_message_flows[
        MessageIndex.PCD.value
    ]
    for i in range(len(lidar_flows)):
        PointsProviderProcess(
            i,
            sec.LiDAR_ex[i],
            sac_calib,
            sac,
            ser,
            lidar_flows[i],
            activator,
            f"PointsProviderProcess[{i}]",
        ).add_to(processes)

    candata_flow: MessageFlow[CanData] = input_message_flows[MessageIndex.CAN.value]
    CanDataProviderProcess(
        0,
        sec.CAN_ex,
        sac_calib,
        sac,
        ser,
        candata_flow,
        activator,
        "CanDataProviderProcess",
    ).add_to(processes)


def create_lidar_shift_monitor_prosess(
    processes: ProcessManager,
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        tuple[MessageFlow[ImuData], ...],
        MessageFlow[CanData],
    ],
    activator: ProcessActivator,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
) -> None:
    """
    LidarShiftMonitorのプロセスを作成する
    """
    from argus_synchro.process.lidar_shift_monitor_process import (
        LidarShiftMonitorProcess,
    )

    LidarShiftMonitorProcess(
        sec.Lidar_SM_ex,
        sac,
        ser,
        input_message_flows[MessageIndex.IMU.value],
        activator,
    ).add_to(processes)


def create_calib_fifo_prosess(
    processes: ProcessManager,
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        tuple[MessageFlow[ImuData], ...],
        MessageFlow[CanData],
    ],
    output_message_flow: MessageFlow[FIFOData],
    activator: ProcessActivator,
    sac_calib: SharedAppConfigCalibration,
    sac: SharedAppConfig,
    sec: SharedExcepts,
) -> None:
    from argus_synchro.process.calib_fifo_process import CalibFIFOProcess

    """
    calibration用のFIFOプロセスを作成する
    """
    CalibFIFOProcess(
        sec.getData_ex,
        sac,
        sac_calib,
        input_message_flows[MessageIndex.CAMERA.value],
        input_message_flows[MessageIndex.PCD.value],
        input_message_flows[MessageIndex.CAN.value],
        output_message_flow,
        activator,
    ).add_to(processes)


def create_scrutinizer_prosess(
    processes: ProcessManager,
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        tuple[MessageFlow[ImuData], ...],
        MessageFlow[CanData],
    ],
    output_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        MessageFlow[CanAngleData],
        MessageFlow[CanLeverData],
    ],
    activator: ProcessActivator,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
) -> None:
    """
    Scrutinizerのプロセスを作成する
    """
    from argus_synchro.process.get_data_process import GetDataProcess

    GetDataProcess(
        sec.getData_ex,
        sac,
        ser,
        input_message_flows[MessageIndex.CAMERA.value],
        input_message_flows[MessageIndex.PCD.value],
        input_message_flows[MessageIndex.CAN.value],
        output_message_flows[MessageIndex.CAMERA.value],
        output_message_flows[MessageIndex.PCD.value],
        output_message_flows[MessageIndex.CAN_ANGLE.value],
        output_message_flows[MessageIndex.CAN_LEVER.value],
        activator,
    ).add_to(processes)


def create_undisimage_process(
    processes: ProcessManager,
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        MessageFlow[CanAngleData],
        MessageFlow[CanLeverData],
    ],
    output_message_flows: MessageFlow[CameraDetectionsData],
    activator: ProcessActivator,
    sec: SharedExcepts,
    sac: SharedAppConfig,
    ser: SharedErrors,
) -> None:
    from argus_synchro.process.object_detect_process import ObjectDetectProcess

    ObjectDetectProcess(
        0,
        sac,
        sec,
        ser,
        sec.ObjDet_ex,
        input_message_flows[MessageIndex.CAMERA.value],
        output_message_flows,
        activator,
    ).add_to(processes)


def create_points_refine_process(
    processes: ProcessManager,
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        MessageFlow[CanAngleData],
        MessageFlow[CanLeverData],
    ],
    output_message_flows: tuple[
        MessageFlow[AccumPointsData],
        MessageFlow[EdgeDetectionResult],
    ],
    activator: ProcessActivator,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
) -> None:
    from argus_synchro.process.points_refine_process import PointsRefineProcess

    PointsRefineProcess(
        sac,
        sec.PointsRefine_ex,
        ser,
        input_message_flows[MessageIndex.PCD.value],
        input_message_flows[MessageIndex.CAN_ANGLE.value],
        input_message_flows[MessageIndex.CAN_LEVER.value],
        output_message_flows[MessageIndex.ACCUM.value],
        output_message_flows[MessageIndex.CLIFF.value],
        activator,
    ).add_to(processes)


def create_calib_process(
    processes: ProcessManager,
    input_message_flow: MessageFlow[FIFOData],
    activator: ProcessActivator,
    calib_settings_ini_path: str,
    sac_calib: SharedAppConfigCalibration,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
) -> None:
    from argus_synchro.process.calib_process import CalibProcess

    CalibProcess(
        sac_calib,
        sac,
        sec,
        ser,
        sec.CalMatGen_ex,
        input_message_flow,
        activator,
        inifilepath=calib_settings_ini_path,
    ).add_to(processes)


def create_visual_process(
    processes: ProcessManager,
    input_message1_flows: tuple[
        MessageFlow[AccumPointsData], MessageFlow[EdgeDetectionResult]
    ],
    input_message3_flow: MessageFlow[CameraDetectionsData],
    activator: ProcessActivator,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
) -> None:
    from argus_synchro.process.visual_process import VisualProcess

    VisualProcess(
        sac,
        sec,
        ser,
        sec.Visu_ex,
        input_message1_flows[MessageIndex.ACCUM.value],
        input_message1_flows[MessageIndex.CLIFF.value],
        input_message3_flow,
        activator,
    ).add_to(processes)


# =========================================================
# 2段階+kill 停止（並列版）
# =========================================================
def _alive(p: Process | None) -> bool:
    return bool(p and p.is_alive())


def _phase_wait_until(deadline: float, procs: list[Process]) -> None:
    """deadline まで 100ms 間隔で生存確認をポーリング"""
    while time.monotonic() < deadline:
        if all(not _alive(p) for p in procs):
            return
        time.sleep(0.1)


def graceful_stop_parallel(
    groups: list[tuple[Process | None, str]],
    _calib_procstop_logger: AppLogger,
    *,
    t_grace: float = 3.0,
    t_term: float = 3.0,
    t_kill: float = 2.0,
) -> None:
    """
    複数プロセスを “並列段階停止” する。
      1) 優雅停止待ち（親は既に終了フラグを出している前提）
      2) 一斉 terminate -> 待機
      3) 一斉 kill -> 待機
    """
    # None を除外して操作対象を確定
    procs: list[Process] = [p for (p, _) in groups if p is not None]

    # 初期ログ
    for p, name in groups:
        if p is None:
            continue
        _calib_procstop_logger.info(f"[{name}] state: alive={p.is_alive()} pid={p.pid}")

    # --- Phase 1: 優雅停止（自発終了待ち） ---
    _calib_procstop_logger.info(f"phase=graceful wait {t_grace}s (parallel)")
    deadline = time.monotonic() + t_grace
    _phase_wait_until(deadline, procs)

    # --- Phase 2: terminate 同時送信 ---
    still_alive_term: list[Process] = [p for p in procs if _alive(p)]
    if still_alive_term:
        _calib_procstop_logger.info(f"phase=terminate {len(still_alive_term)} procs")
        for p, name in groups:
            if _alive(p):
                _calib_procstop_logger.info(f"[{name}] sending terminate (pid={p.pid})")
                try:
                    p.terminate()
                except Exception as e:
                    _calib_procstop_logger.error(f"[{name}] terminate error: {e!r}")
        deadline = time.monotonic() + t_term
        _phase_wait_until(deadline, still_alive_term)

    # --- Phase 3: kill 同時送信 ---
    still_alive_kill: list[Process] = [p for p in procs if _alive(p)]
    if still_alive_kill:
        _calib_procstop_logger.info(f"phase=kill {len(still_alive_kill)} procs")
        for p, name in groups:
            if _alive(p):
                _calib_procstop_logger.info(f"[{name}] sending kill (pid={p.pid})")
                try:
                    p.kill()
                except Exception as e:
                    _calib_procstop_logger.error(f"[{name}] kill error: {e!r}")
        deadline = time.monotonic() + t_kill
        _phase_wait_until(deadline, still_alive_kill)

    # 終了コードログ
    for p, name in groups:
        if p is None:
            continue
        try:
            _calib_procstop_logger.info(f"[{name}] exitcode={p.exitcode}")
        except Exception:
            pass


# 単体 逐次停止
def graceful_stop(
    proc: Process | None,
    name: str,
    _calib_procstop_logger: AppLogger,
    timeout: float = 3.0,
) -> None:
    if not proc:
        return
    try:
        if not proc.is_alive():
            _calib_procstop_logger.info(f"[{name}] already not alive (pid={proc.pid})")
            return
        _calib_procstop_logger.info(f"[{name}] stop request (pid={proc.pid})")
        # 自発終了待ち
        start = time.monotonic()
        while proc.is_alive() and (time.monotonic() - start) < timeout:
            time.sleep(0.1)
        if not proc.is_alive():
            _calib_procstop_logger.info(f"[{name}] exited gracefully (pid={proc.pid})")
            return
        # まだ動いている -> terminate
        _calib_procstop_logger.info(f"[{name}] sending terminate (pid={proc.pid})")
        proc.terminate()
        start = time.monotonic()
        while proc.is_alive() and (time.monotonic() - start) < timeout:
            time.sleep(0.1)
        if not proc.is_alive():
            _calib_procstop_logger.info(f"[{name}] terminated (pid={proc.pid})")
            return
        # それでも動いている -> kill
        _calib_procstop_logger.info(f"[{name}] sending kill (pid={proc.pid})")
        proc.kill()
        start = time.monotonic()
        while proc.is_alive() and (time.monotonic() - start) < 2.0:
            time.sleep(0.1)
        if not proc.is_alive():
            _calib_procstop_logger.info(f"[{name}] killed (pid={proc.pid})")
        else:
            _calib_procstop_logger.info(
                f"[{name}] still alive after kill!? (pid={proc.pid})"
            )
    except Exception as e:
        _calib_procstop_logger.error(f"[{name}] stop error: {e!r}")
    finally:
        try:
            _calib_procstop_logger.info(f"[{name}] exitcode={proc.exitcode}")
        except Exception:
            pass


# =========================================================
# 起動/停止: 校正実行モード
# =========================================================
def start_calib_pipeline(
    sac_calib: SharedAppConfigCalibration,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
    closables: CompositeClosable,
    process_activator: ProcessActivator,
    datasource_sync_type: SyncType,
    scrutinizer_sync_type: SyncType,
    calib_settings_ini_path: str,
    app_logger: AppLogger,
) -> ProcessManager:
    """
    校正モードのプロセスを作成する。
    """
    processes: ProcessManager = ProcessManager(process_activator, app_logger).add_to(
        closables
    )
    process_activator.enable()
    app_config: AppConfig = sac.read()

    # 入力データメッセージ/プロセス作成
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        tuple[MessageFlow[ImuData], ...],
        MessageFlow[CanData],
    ] = create_input_message(
        closables, app_config, process_activator, datasource_sync_type
    )
    create_calib_input_prosess(
        processes, input_message_flows, process_activator, sac_calib, sac, sec, ser
    )

    # calib_fifoメッセージ/プロセス作成
    calib_fifo_message_flow: MessageFlow[FIFOData] = create_calib_fifo_message(
        closables, app_config, process_activator, scrutinizer_sync_type
    )
    create_calib_fifo_prosess(
        processes,
        input_message_flows,
        calib_fifo_message_flow,
        process_activator,
        sac_calib,
        sac,
        sec,
    )

    # FIFOから校正プロセス
    create_calib_process(
        processes,
        calib_fifo_message_flow,
        process_activator,
        calib_settings_ini_path,
        sac_calib,
        sac,
        sec,
        ser,
    )
    return processes


def stop_calib_pipeline(
    closables: CompositeClosable,
    processes: ProcessManager,
    ser: SharedErrors,
) -> tuple[CompositeClosable, ProcessActivator]:
    """
    校正モードのプロセス、Activatorを停止
    """
    result: tuple[list[ProcessBase], list[ProcessBase]] = processes.graceful_stop_all(
        t_grace=10.0, t_term=2.0, t_kill=1.0
    )
    diag: StateErrorDiagnosisD = ser.state_errors_D[
        StateErrorDIndex.PROCESS_FORCED_TERMINATION
    ]
    diag_result: tuple[ResultDiagnosis, ResultDiagnosis] = diag.errors_diagnosis(
        *result
    )
    diag.log_output(*diag_result, StateErrorDIndex.PROCESS_FORCED_TERMINATION, *result)

    closables.close()
    new_process_activator = ProcessActivator()
    new_process_activator.disable()
    return CompositeClosable(), new_process_activator


def is_automated_calibration_run(app_config: AppConfig, mode: str) -> bool:
    """ファイル入力による校正シミュレーションの自動反復かを判定する。

    校正パラメータや入力ファイルを少しずつ変更しながら、本アプリを外部
    スクリプトから繰り返し起動し、校正結果を無人で比較する用途を想定する。

    この用途は実センサを停止する必要がないファイル入力のCALIBに限定する。
    実機入力またはSCRUTでは、量産運転に必要な確実なプロセス停止と
    強制終了診断を維持するため、自動反復用の軽量終了を選択しない。
    """
    return app_config.DEFAULT.File_Input and mode == "CALIB"


def stop_automated_calibration_pipeline(
    closables: CompositeClosable,
    process_activator: ProcessActivator,
) -> tuple[CompositeClosable, ProcessActivator]:
    """校正シミュレーションの1試行を終了し、外部スクリプトへ制御を返す。

    ``graceful_stop_all()`` は量産時に子プロセスを確実に停止し、必要なら
    terminate/killを実行して、その発生を強制終了診断へ通知するためにある。
    一方、自動反復では、あるパラメータや入力ファイルで共有例外状態が残った
    試行も評価結果の一つとして終了し、次の条件でアプリを再実行する必要がある。
    この経路で量産向けの強制終了診断や待機を行うと、実害のない試行結果でも
    エラー処理へ入り、無人バッチが次の実行へ進めなくなることがある。

    そのため、この関数はまずActivatorを停止して通信資源を解放する。
    子プロセスの有限時間停止は呼出し側の``wait_for_pipeline_shutdown()``へ
    委ね、そこでterminate/killが必要になっても量産向け強制終了診断には
    接続しない。実機CALIBとSCRUTでは使用しない。
    """
    process_activator.disable()
    closables.close()
    new_process_activator = ProcessActivator()
    new_process_activator.disable()
    return CompositeClosable(), new_process_activator


def stop_completed_calibration_pipeline(
    app_config: AppConfig,
    closables: CompositeClosable,
    process_activator: ProcessActivator,
) -> tuple[CompositeClosable, ProcessActivator]:
    """完了したファイル入力CALIBを、後続の終了待ちより先に停止する。

    ``File_Input=True``かつCALIBの場合だけ自動反復用の停止を選ぶ。
    実機CALIBでは量産向け停止処理を維持するため、受け取った資源を変更しない。
    """
    if not is_automated_calibration_run(app_config, "CALIB"):
        return closables, process_activator
    return stop_automated_calibration_pipeline(closables, process_activator)


def wait_for_pipeline_shutdown(
    processes: ProcessManager,
    system_processes: ProcessManager,
    automated_calibration_completed: bool,
) -> None:
    """自動校正完了時だけ、無期限joinを避けて有限時間で停止する。

    外部scriptが次のparameter setへ必ず進めるよう、grace期間後も残る
    processはterminate、killの順で停止する。このfallbackは試験運用上の
    終了保証であり、量産向け``PROCESS_FORCED_TERMINATION``診断は行わない。
    通常運転では従来のjoin契約を変更しない。
    """
    if automated_calibration_completed:
        processes.graceful_stop_all(t_grace=10.0, t_term=2.0, t_kill=1.0)
        system_processes.graceful_stop_all(t_grace=3.0, t_term=2.0, t_kill=1.0)
        return
    processes.join()
    system_processes.join()


def start_scrut_pipeline(
    sac_calib: SharedAppConfigCalibration,
    sac: SharedAppConfig,
    sec: SharedExcepts,
    ser: SharedErrors,
    closables: CompositeClosable,
    process_activator: ProcessActivator,
    datasource_sync_type: SyncType,
    scrutinizer_sync_type: SyncType,
    app_logger: AppLogger,
) -> ProcessManager:
    """
    周辺監視モードのプロセスを作成する。
    """
    processes: ProcessManager = ProcessManager(process_activator, app_logger).add_to(
        closables
    )
    process_activator.enable()
    app_config: AppConfig = sac.read()
    # 入力データメッセージ/プロセス作成
    input_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        tuple[MessageFlow[ImuData], ...],
        MessageFlow[CanData],
    ] = create_input_message(
        closables, app_config, process_activator, datasource_sync_type
    )
    create_input_prosess(
        processes, input_message_flows, process_activator, sac_calib, sac, sec, ser
    )

    create_lidar_shift_monitor_prosess(
        processes, input_message_flows, process_activator, sac, sec, ser
    )
    # Scrutinizerメッセージ/プロセス作成
    scrutinizer_message_flows: tuple[
        tuple[MessageFlow[CameraData], ...],
        tuple[MessageFlow[PointCloudData], ...],
        MessageFlow[CanAngleData],
        MessageFlow[CanLeverData],
    ] = create_scrutinizer_message(
        closables, app_config, process_activator, scrutinizer_sync_type
    )
    create_scrutinizer_prosess(
        processes,
        input_message_flows,
        scrutinizer_message_flows,
        process_activator,
        sac,
        sec,
        ser,
    )

    # 歪み補正
    undistort_message_flows: MessageFlow[CameraDetectionsData] = (
        create_undisimage_message(
            closables, app_config, process_activator, scrutinizer_sync_type
        )
    )
    create_undisimage_process(
        processes,
        scrutinizer_message_flows,
        undistort_message_flows,
        process_activator,
        sec,
        sac,
        ser,
    )

    points_refine_message_flows: tuple[
        MessageFlow[AccumPointsData], MessageFlow[EdgeDetectionResult]
    ] = create_points_refine_message(
        closables, app_config, process_activator, scrutinizer_sync_type
    )
    create_points_refine_process(
        processes,
        scrutinizer_message_flows,
        points_refine_message_flows,
        process_activator,
        sac,
        sec,
        ser,
    )

    # 立体物検出から描画プロセス
    create_visual_process(
        processes,
        points_refine_message_flows,
        undistort_message_flows,
        process_activator,
        sac,
        sec,
        ser,
    )
    return processes


def stop_scrut_pipeline(
    closables: CompositeClosable,
    processes: ProcessManager,
    ser: SharedErrors,
) -> tuple[CompositeClosable, ProcessActivator]:
    """
    周辺監視モードのプロセス、Activatorを停止
    #"""
    result: tuple[list[ProcessBase], list[ProcessBase]] = processes.graceful_stop_all(
        t_grace=3.0, t_term=2.0, t_kill=1.0
    )
    diag: StateErrorDiagnosisD = ser.state_errors_D[
        StateErrorDIndex.PROCESS_FORCED_TERMINATION
    ]
    diag_result: tuple[ResultDiagnosis, ResultDiagnosis] = diag.errors_diagnosis(
        *result
    )
    diag.log_output(*diag_result, StateErrorDIndex.PROCESS_FORCED_TERMINATION, *result)

    closables.close()
    new_process_activator = ProcessActivator()
    new_process_activator.disable()
    return CompositeClosable(), new_process_activator


def stop_system_pipeline(
    system_closables: CompositeClosable,
    scrut_closables: CompositeClosable,
    system_processes: ProcessManager,
    scrut_processes: ProcessManager,
    ser: SharedErrors,
) -> None:
    """
    エラー監視プロセス以外の全てのプロセス、Activatorを停止
    """
    stop_scrut_pipeline(system_closables, system_processes, ser)
    system_closables.close()
    stop_scrut_pipeline(scrut_closables, scrut_processes, ser)
    scrut_closables.close()
    # TODO リファクタリング後stop_calib_pipeline追加 (NSW)


def stop_error_monitor(
    error_closables: CompositeClosable,
    error_processes: ProcessManager,
) -> None:
    error_processes.graceful_stop_all(t_grace=3.0, t_term=2.0, t_kill=1.0)
    error_closables.close()


def get_current_calibmode(sac: SharedAppConfig) -> CalibMode:
    if sac.read().CalibMode.isRunning3D3Dcalib:
        return CalibMode.IsRunning3D3Dcalib
    if sac.read().CalibMode.isRunning2D3Dcalib:
        return CalibMode.IsRunning2D3Dcalib
    if sac.read().CalibMode.isRunning2D3Dcheck:
        return CalibMode.IsRunning2D3Dcheck
    return CalibMode.wait_app


def load_config(
    ser: SharedErrors,
    _logger: AppLogger,
    directory_config: paths.DirectoryConfig,
) -> tuple[SharedAppConfig, SharedExcepts, AppConfig, SharedAppConfigCalibration]:
    app_logger_factory = AppLoggerFactory()
    while True:
        try:
            machine_profile.MachineProfileHandler.log_register(app_logger_factory)
            mprof_handler = machine_profile.MachineProfileHandler(
                app_logger_factory, directory_config
            )
            mprof_handler.apply_model_specific_config()
            mprof_handler.apply_model_specific_calib_config()

            sac = SharedAppConfig(directory_config)
            app_config: AppConfig = sac.read()
            if app_config.DEFAULT.use_shi_lib:
                with open(app_config.camera.config_file, encoding="utf-8") as file:
                    json.load(file)
            # in_factoryはメンテナンスモードを表す既存設定キー。
            ser.diagnosis_runtime_policy.update_in_factory(
                app_config.General.in_factory
            )
            sec = SharedExcepts(app_config=app_config, app_manager_ex=ser.AppMan_ex)

            calib_settings_path = str(
                paths.normalize_path("calib_settings.ini", directory_config.config_dir)
            )
            sac_calib = SharedAppConfigCalibration(
                arglist=[],
                inifilepath=calib_settings_path,
                directory_config=directory_config,
            )

            diagnose_startup_calibration_data(
                ser,
                app_config.calibration,
                sac_calib.read().Calib3d3d_CalibParams.lidars_calib_path,
                app_config.camera.count,
                ser.shared_err_conf.read(),
            )

            return sac, sec, app_config, sac_calib
        except Exception as e:
            # クリティカルエラー時は一旦再起動して復帰を試みる
            diagnosis = ser.action_errors_A_C[ActionErrorIndex.CONFIG_FILE_MISSING]
            is_config_error = diagnosis.excepts_diagnosis(e)
            if is_config_error:
                diagnosis.log_output(
                    True,
                    False,
                    ActionErrorIndex.CONFIG_FILE_MISSING,
                    e,
                )
            else:
                _logger.error(
                    f"fatal error: {e!r}, traceback: {traceback.format_exc()}"
                )


def load_err_config(ser: SharedErrors) -> None:
    err_config: ErrorConfig = ser.shared_err_conf.read()
    ser.state_errors_D[StateErrorDIndex.PROCESS_FORCED_TERMINATION].update(err_config)
    ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR].update(err_config)
    # AppManager起動前にlogger callbackが発火するため、先に初期化する。
    ser.state_errors_D[StateErrorDIndex.LOG_COMPRESSION_FAILURE].update(err_config)
    ser.state_errors_D[StateErrorDIndex.LOG_TIME_REVERSAL].update(err_config)
    ser.action_errors_A_C[ActionErrorIndex.OPERATION_MODE_TRANSITION_ERROR].update(
        err_config
    )
    ser.action_errors_A_C[ActionErrorIndex.MMAP_READ_WRITE_ERROR].update(err_config)
    ser.action_errors_A_C[ActionErrorIndex.REBOOT_LOOP_DETECTED].update(err_config)
    ser.action_errors_A_C[ActionErrorIndex.CONFIG_FILE_MISSING].update(err_config)
    ser.action_errors_A_C[ActionErrorIndex.AI_MODEL_LOAD_FAILED].update(err_config)
    ser.action_errors_A_C[ActionErrorIndex.LOG_FILE_IO_ERROR].update(err_config)
    ser.action_errors_A_C[ActionErrorIndex.PROCESS_STARTUP_ERROR].update(err_config)
    ser.module_errors[ModuleErrorIndex.MAIN_MODULE_ERROR].update(err_config)


def _detect_reboot_loop(ser: SharedErrors) -> None:
    diagnosis = ser.action_errors_A_C[ActionErrorIndex.REBOOT_LOOP_DETECTED]
    file_io_error = ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
    if not diagnosis.is_enabled:
        return

    state_path = Path(diagnosis.param.uptime_state_path)
    try:
        with state_path.open(encoding="utf-8") as state_file:
            state = json.load(state_file)
        last_boots = state["last_boots"]
        if not isinstance(last_boots, list):
            raise ValueError("last_boots must be a list")
        boot_times: list[float] = []
        for boot in last_boots:
            if not isinstance(boot, dict) or not isinstance(
                boot.get("boot_time_iso"), str
            ):
                raise ValueError("last_boots contains an invalid boot_time_iso")
            boot_time = datetime.fromisoformat(boot["boot_time_iso"])
            if boot_time.tzinfo is None:
                raise ValueError("boot_time_iso must include a timezone")
            boot_times.append(boot_time.timestamp())
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        result = file_io_error.errors_diagnosis(True)
        file_io_error.log_output(
            *result,
            StateErrorDIndex.FILE_IO_ERROR,
            str(state_path),
            "read uptime state for CE012",
            f"{type(error).__name__}: {error}",
        )
        return

    file_io_error.errors_diagnosis(False)
    result = diagnosis.errors_diagnosis(boot_times)
    if result[0]:
        diagnosis.increment_counter()
    diagnosis.log_output(*result[:2], ActionErrorIndex.REBOOT_LOOP_DETECTED, boot_times)


def _write_status_safe(
    status: StatusMMAP,
    code: StatusCode,
    ser: SharedErrors,
) -> None:
    try:
        status.write_status(code)
    except (OSError, ValueError, BufferError, RuntimeError) as error:
        diagnosis = ser.action_errors_A_C[ActionErrorIndex.MMAP_READ_WRITE_ERROR]
        is_target = diagnosis.excepts_diagnosis(error)
        diagnosis.log_output(
            is_target,
            False,
            ActionErrorIndex.MMAP_READ_WRITE_ERROR,
            f"status.mmap write failed ({code.name})",
            error,
        )


# read中やCALIB再起動中に発生した次の設定更新を取りこぼさないため、
# 検出した更新時刻をread前に固定し、後発の更新は次回の再読込対象として残す。
def _reload_app_config_if_updated(
    sac: SharedAppConfig,
    app_config: AppConfig,
    last_updated: int,
    logger: AppLogger,
    context: str | None = None,
) -> tuple[AppConfig, int, bool]:
    detected_last_updated = sac.last_updated
    if detected_last_updated <= last_updated:
        return app_config, last_updated, False

    app_config = sac.read()
    last_updated = detected_last_updated
    log_context = f" {context}" if context else ""
    logger.info(
        "config reloaded%s (last_updated=%s, operation_mode=%s)",
        log_context,
        last_updated,
        app_config.General.operation_mode,
    )
    return app_config, last_updated, True


def main() -> None:
    # TODO PyInstaller利用時にmultiprocessingを使用するときに必要 (NSW)
    mp.freeze_support()
    directory_config: paths.DirectoryConfig = paths.parse_directory_config()
    calib_settings_ini_path: str = str(
        paths.normalize_path("calib_settings.ini", directory_config.config_dir)
    )

    _logger: AppLogger = AppLoggerFactory.from_name("main")
    _calib_start_logger: AppLogger = AppLoggerFactory.from_name("CalibStart")
    _calib_stop_logger: AppLogger = AppLoggerFactory.from_name("CalibStop")
    _calib_procstop_logger: AppLogger = AppLoggerFactory.from_name("ProcStop")
    _scrutinizer_logger: AppLogger = AppLoggerFactory.from_name("ScrutinizerModeStart")
    _process_manager_logger: AppLogger = AppLoggerFactory.from_name("ProcessManager")

    # status.mmap（MonitorArgus側でcreateする運用）
    status: StatusMMAP = StatusMMAP(
        _logger,
        create=False,
        directory_config=directory_config,
    )

    # ErrorMonitor プロセス
    error_closables = CompositeClosable()
    error_activator: ProcessActivator = ProcessActivator()
    error_activator.enable()
    p_error: ProcessManager = ProcessManager(
        error_activator, _process_manager_logger
    ).add_to(error_closables)

    ser = SharedErrors(
        paths.normalize_path("error_config.json", directory_config.config_dir)
    )
    load_err_config(ser)
    ErrorMonitorProcess(
        ser,
        error_activator,
        directory_config,
        1 / 5,
        "ErrorMonitor",
    ).add_to(p_error)
    p_error.start("", directory_config)
    setup_signal_handlers(
        status_obj=status,
        logger=_logger,
        name="Main",
        shutdown_callback=lambda: stop_error_monitor(error_closables, p_error),
    )
    is_restart = True
    while is_restart:
        is_restart = False

        # 設定パラメータ
        sac: SharedAppConfig
        sec: SharedExcepts
        app_config: AppConfig
        sac_calib: SharedAppConfigCalibration
        sac, sec, app_config, sac_calib = load_config(ser, _logger, directory_config)
        log_dir = Path(app_config.DEFAULT.debug_log)
        app_logger_factory: AppLoggerFactory = AppLoggerFactory(to_file=log_dir)
        app_logger_factory.append_logger(_logger)
        app_logger_factory.append_logger(_calib_start_logger)
        app_logger_factory.append_logger(_calib_stop_logger)
        app_logger_factory.append_logger(_calib_procstop_logger)
        app_logger_factory.append_logger(_scrutinizer_logger)
        app_logger_factory.append_logger(_process_manager_logger)
        MonitorArgus.log_register(app_logger_factory)
        ser.log_register(app_logger_factory)
        app_logger_factory.update()
        _detect_reboot_loop(ser)
        ce015_diag = ser.action_errors_A_C[ActionErrorIndex.LOG_FILE_IO_ERROR]
        app_logger_factory.set_io_error_callback(ce015_diag.excepts_diagnosis)
        log_compression_failure = cast(
            LogCompressionFailure,
            ser.state_errors_D[StateErrorDIndex.LOG_COMPRESSION_FAILURE],
        )
        app_logger_factory.set_compression_error_callback(
            log_compression_failure.report_event
        )
        log_time_reversal = cast(
            LogTimeReversal,
            ser.state_errors_D[StateErrorDIndex.LOG_TIME_REVERSAL],
        )
        app_logger_factory.set_time_reversal_callback(
            log_time_reversal.report_record_time
        )

        config_dir = paths.get_config_dir(directory_config)
        mmap_dir: Path = paths.get_mmap_dir(directory_config)
        _logger.info(f"現在のPath設定:{log_dir=},{config_dir=},{mmap_dir=}")

        # ステータスをMonitorArgusへ通知
        _logger.info("REBOOT_CODE 設定")
        _write_status_safe(status, StatusCode.REBOOT, ser)
        # Godotプロセス終了を待機.
        # ここで待機しないと、状態遷移を検出できず、残存Godotが終了しない事がある
        time.sleep(2.0)
        _logger.info("BOOTING_CODE 設定")
        _write_status_safe(status, StatusCode.BOOTING, ser)

        system_activator: ProcessActivator = ProcessActivator()
        process_activator: ProcessActivator = ProcessActivator()
        system_activator.enable()
        process_activator.enable()

        system_closables = CompositeClosable()
        closables = CompositeClosable()

        p0: ProcessManager = ProcessManager(
            system_activator, _process_manager_logger
        ).add_to(system_closables)
        processes: ProcessManager = ProcessManager(
            process_activator, _process_manager_logger
        ).add_to(closables)
        # 校正パイプライン
        # p_calib: Process | None = None
        try:
            # 機種プロファイルの反映
            from argus_synchro.process.app_manager_process import AppManagerProcess

            # 最終更新との比較が適切にできていないので、暫定的に下記をコメントアウト.
            last_updated = sac.last_updated
            _logger.info(f"config reloaded (last_updated={last_updated})")

            # 同期タイプを設定(ファイル読み込み時は同期、実機は非同期)
            datasource_sync_type: SyncType = (
                SyncType.SYNC_FRAME
                if app_config.DEFAULT.File_Input
                else SyncType.LATEST
            )
            # NOTE: 通常はフレームが飛ぶため、1フレームごと表示する場合はSYNCにする
            scrutinizer_sync_type: SyncType = SyncType.LATEST

            # 初期モード決定
            if app_config.General.operation_mode == OPM.SCRUT:
                current_mode = "SCRUT"
            else:
                current_mode = "CALIB"
            _logger.info(f"initial mode = {current_mode}")

            p_error.update_cpu_affinity(app_config)

            try:
                if current_mode == "SCRUT":
                    processes = start_scrut_pipeline(
                        sac_calib,
                        sac,
                        sec,
                        ser,
                        closables,
                        process_activator,
                        datasource_sync_type,
                        scrutinizer_sync_type,
                        _process_manager_logger,
                    )
                else:
                    processes = start_calib_pipeline(
                        sac_calib,
                        sac,
                        sec,
                        ser,
                        closables,
                        process_activator,
                        datasource_sync_type,
                        scrutinizer_sync_type,
                        calib_settings_ini_path,
                        _process_manager_logger,
                    )
                processes.start(
                    app_config.DEFAULT.debug_log, directory_config, app_config
                )
            except Exception as e:
                startup_error = ser.action_errors_A_C[
                    ActionErrorIndex.PROCESS_STARTUP_ERROR
                ]
                is_error = startup_error.excepts_diagnosis(e)
                startup_error.log_output(
                    is_error,
                    recover=False,
                    err_idx=ActionErrorIndex.PROCESS_STARTUP_ERROR,
                )
                if is_automated_calibration_run(app_config, current_mode):
                    # 起動失敗も校正評価の1試行として終了し、次回実行へ進める。
                    closables, process_activator = stop_automated_calibration_pipeline(
                        closables, process_activator
                    )
                elif current_mode == "SCRUT":
                    stop_scrut_pipeline(closables, processes, ser)
                else:
                    stop_calib_pipeline(closables, processes, ser)

                if is_error:
                    is_restart = True
                    system_activator.disable()
                else:
                    raise Exception("プロセス起動で予期しない例外が発生しました") from e

            else:
                current_calibmode: CalibMode = get_current_calibmode(sac)
                # AppManager プロセス
                AppManagerProcess(
                    sec, sac, ser, system_activator, process_activator, "AppManager"
                ).add_to(p0)
                # TODO AppManagerの起動に時間がかかっているが、プロセスが使用するCPUを固定することで、解消見込み (NSW)
                p0.start(app_config.DEFAULT.debug_log, directory_config, app_config)

            automated_calibration_completed = False
            while system_activator.value:
                # 設定更新の検知
                app_config, last_updated, config_reloaded = (
                    _reload_app_config_if_updated(
                        sac, app_config, last_updated, _logger
                    )
                )
                if config_reloaded:
                    if current_mode == "CALIB":
                        new_calibmode: CalibMode = get_current_calibmode(sac)
                        if current_calibmode != new_calibmode:
                            # 共有メモリ更新
                            sac_calib.write(sec)
                            processes.restart()
                            current_calibmode = new_calibmode
                            app_config, last_updated, _ = (
                                _reload_app_config_if_updated(
                                    sac,
                                    app_config,
                                    last_updated,
                                    _logger,
                                    "after CALIB restart",
                                )
                            )
                # モード遷移判定(現在モードと設定されたモードが食い違う時)
                set_calib: bool = bool(app_config.General.operation_mode == OPM.CALIB)
                if current_mode == "SCRUT" and set_calib:
                    _logger.info("switch SCRUT -> CALIB")
                    # モード切替中は診断を停止する
                    processes.stop_diagnosis()
                    if is_automated_calibration_run(app_config, "CALIB"):
                        # 自動校正評価への遷移では強制終了診断を行わない。
                        closables, process_activator = (
                            stop_automated_calibration_pipeline(
                                closables, process_activator
                            )
                        )
                    else:
                        closables, process_activator = stop_scrut_pipeline(
                            closables, processes, ser
                        )
                    processes.join()
                    process_activator.enable()
                    sec.reset_operation_mode_calib_ex()
                    sac_calib.write(sec)
                    current_calibmode = get_current_calibmode(sac)
                    try:
                        processes = start_calib_pipeline(
                            sac_calib,
                            sac,
                            sec,
                            ser,
                            closables,
                            process_activator,
                            datasource_sync_type,
                            scrutinizer_sync_type,
                            calib_settings_ini_path,
                            _process_manager_logger,
                        )
                        processes.start(
                            app_config.DEFAULT.debug_log, directory_config, app_config
                        )
                    except Exception as e:
                        translate_error = ser.action_errors_A_C[
                            ActionErrorIndex.OPERATION_MODE_TRANSITION_ERROR
                        ]
                        is_error = translate_error.excepts_diagnosis(e)
                        translate_error.log_output(
                            is_error,
                            recover=False,
                            err_idx=ActionErrorIndex.OPERATION_MODE_TRANSITION_ERROR,
                        )
                        stop_scrut_pipeline(closables, processes, ser)
                        if is_error:
                            is_restart = True
                            system_activator.disable()
                        else:
                            raise Exception(
                                "モード遷移で予期しない例外が発生しました"
                            ) from e

                    current_mode = "CALIB"

                elif current_mode == "CALIB" and not set_calib:
                    # モード切替中は診断を停止する
                    processes.stop_diagnosis()
                    if is_automated_calibration_run(app_config, current_mode):
                        # 自動校正評価の試行結果を強制終了診断へ昇格させない。
                        closables, process_activator = (
                            stop_automated_calibration_pipeline(
                                closables, process_activator
                            )
                        )
                    else:
                        closables, process_activator = stop_calib_pipeline(
                            closables, processes, ser
                        )
                    processes.join()
                    process_activator.enable()
                    sec.reset_operation_mode_scrut_ex()
                    try:
                        processes = start_scrut_pipeline(
                            sac_calib,
                            sac,
                            sec,
                            ser,
                            closables,
                            process_activator,
                            datasource_sync_type,
                            scrutinizer_sync_type,
                            _process_manager_logger,
                        )
                        processes.start(
                            app_config.DEFAULT.debug_log, directory_config, app_config
                        )
                    except Exception as e:
                        translate_error = ser.action_errors_A_C[
                            ActionErrorIndex.OPERATION_MODE_TRANSITION_ERROR
                        ]
                        is_error = translate_error.excepts_diagnosis(e)
                        translate_error.log_output(
                            is_error,
                            recover=False,
                            err_idx=ActionErrorIndex.OPERATION_MODE_TRANSITION_ERROR,
                        )
                        stop_calib_pipeline(closables, processes, ser)
                        if is_error:
                            is_restart = True
                            system_activator.disable()
                        else:
                            raise Exception(
                                "モード遷移で予期しない例外が発生しました"
                            ) from e

                    current_mode = "SCRUT"

                if current_mode == "SCRUT" and sec.check_scrut_mode_is_finished():
                    system_activator.disable()
                if current_mode == "CALIB" and sec.CalMatGen_ex.IsFinished.value:
                    # CalibProcessが共有完了フラグを立てたらmain loopを抜ける。
                    # ファイル入力の自動反復では、先に入力processと通信資源を
                    # 停止する。これを終了待ちより後へ移すと、入力processが
                    # 自律終了せず外部scriptが次の試行へ進めない場合がある。
                    closables, process_activator = stop_completed_calibration_pipeline(
                        app_config,
                        closables,
                        process_activator,
                    )
                    automated_calibration_completed = is_automated_calibration_run(
                        app_config,
                        current_mode,
                    )
                    system_activator.disable()
                time.sleep(0.2)

                is_restart |= sac.is_restart_required.value
                if is_restart:
                    _logger.info("is_restart_required detected -> restart")
                    break

            # 自動校正だけ有限時間で収束させる。実機CALIBとSCRUTは従来どおり
            # 各processの自律終了をjoinで待ち、量産時の停止契約を維持する。
            wait_for_pipeline_shutdown(
                processes,
                p0,
                automated_calibration_completed,
            )

        except Exception as e:
            if not ser.is_state_error_d_exception(e, _logger):
                if ser.module_errors[
                    ModuleErrorIndex.MAIN_MODULE_ERROR
                ].excepts_diagnosis(e):
                    ser.module_errors[ModuleErrorIndex.MAIN_MODULE_ERROR].log_output(
                        ResultDiagnosis.DETECTION,
                        ResultDiagnosis.DETECTION,
                        ModuleErrorIndex.MAIN_MODULE_ERROR,
                        e,
                    )
                else:
                    raise e
            is_restart = True

        finally:
            stop_system_pipeline(
                system_closables=system_closables,
                scrut_closables=closables,
                system_processes=p0,
                scrut_processes=processes,
                ser=ser,
            )

            del sac
            del sec
            del process_activator
            calibration_mat_generator = _get_calibration_mat_generator_module()
            if calibration_mat_generator.Calib_Mat_Generator.allow_exit(
                str(calib_settings_ini_path),
                directory_config,
            ):
                is_restart = False

    stop_error_monitor(error_closables, p_error)
    del ser
    _logger.info("exited top-level loop")


if __name__ == "__main__":
    # もともとはTensowflowでspawnに設定する必要があったが、ONNXRuntime前提になったため不要となった。
    # しかし、Open3Dのマルチプロセス動作でも同様にforkだとハングアップする処理があるため、spawnに設定する必要がある。
    # Open3D.PointCloud.ClusterDBSCAN
    # Open3D.PointCloud.estimate_normals
    mp.set_start_method("spawn", force=True)
    # forkで動かす必要がある場合はOMP_NUM_THREADS=1の指定が必要となる。
    # mp.set_start_method("fork", force=True)
    main()
