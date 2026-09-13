"""
LiDARの点群取得プロセス

プロセスの概要

* カメラ画像取得プロセス
    * プロセス1: LiDAR1から点群を取得する (ファイル入力モード)
    * プロセス2: LiDAR2から点群を取得する (ファイル入力モード)
"""

from __future__ import annotations

import datetime
import json
import time
from typing import TYPE_CHECKING, Final, final

from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.app_config_calibration import (
    AppConfigCalibration,
    DataCaptureConf,
    DefaultConf,
)
from argus_synchro.config.fileinput_pathselector import (
    lidar_filepath_loader,
)
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.message.input_message import PointCloudData
from argus_synchro.process import InputProcess, MessageFlow
from argus_synchro.process.message import Producer
from argus_synchro.process.operation_mode import OPERATION_MODE as OPM
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import log_main, log_target
from argus_synchro.profiler.prof_mode import ProfCategory
from argus_synchro.provider.clock import (
    DummyClockProvider,
    PerfCounterClockProvider,
    TimeClockProvider,
)
from argus_synchro.provider.point_cloud import (
    CalibMid360FilePointCloudProvider,
    Mid360FilePointCloudProvider,
)
from argus_synchro.shared_app_config import SharedAppConfig, SharedAppConfigCalibration
from argus_synchro.shared_errors import (
    ModuleErrorIndex,
    SharedErrors,
    StateErrorDIndex,
    StateErrorIndex,
)
from argus_synchro.shared_excepts import INVALID_TIMESTAMP, SharedLIDExcept

if TYPE_CHECKING:
    from argus_synchro.provider.point_cloud import (
        CalibMid360FilePointCloudProvider,
        PointCloudProvider,
    )


@final
class PointsProviderProcess(InputProcess[PointCloudData]):
    __slots__ = (
        "_app_config",
        "_clockPprovider",
        "_end_frame",
        "_err_config",
        "_file_input",
        "_frame",
        "_heartbeat_interval",
        "_index",
        "_last_heartbeat",
        "_last_updated",
        "_lidar_config",
        "_lidar_config_index_map",
        "_print_disabled",
        "_provider",
        "_sac",
        "_sac_calib",
        "_sec_lid",
        "_ser",
        "_unique_lidar_name",
    )

    def __init__(
        self,
        index: int,
        sec_lid: SharedLIDExcept,
        sac_calib: SharedAppConfigCalibration,
        sac: SharedAppConfig,
        ser: SharedErrors,
        producer_flow: MessageFlow[PointCloudData],
        activator: ProcessActivator,
        name: str | None = None,
    ) -> None:
        super().__init__(producer_flow, sec_lid, activator, name)
        self._heartbeat_interval: Final[float] = 3.0
        self._index: int = index
        self._sac_calib: SharedAppConfigCalibration = sac_calib
        self._sac: SharedAppConfig = sac
        self._provider: PointCloudProvider
        self._lidar_config: dict[str, dict[str, str | int]] = {}
        self._lidar_config_index_map: dict[int, str] = {}
        self._unique_lidar_name: str = ""
        self._sec_lid: SharedLIDExcept = sec_lid
        self._last_heartbeat: float = time.perf_counter()
        self._ser: SharedErrors = ser

        # _startupで初期化
        self._err_config: ErrorConfig

    def _config_load(self) -> None:
        self._app_config: AppConfig = self._sac.read()
        self._app_config_calib: AppConfigCalibration = self._sac_calib.read()
        self._last_updated: int = self._sac.last_updated
        self.DefaultConfig: DefaultConf = self._app_config_calib.default
        self._LidarConfig: DataCaptureConf.LidarConf = (
            self._app_config_calib.dataCapture.Lidar
        )
        self._print_disabled: bool = self._app_config_calib.default.print_disabled
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            self._end_frame = self._app_config_calib.dataCapture.e_frame
            self._file_input: bool = self._app_config_calib.default.File_Input
        else:
            self._end_frame: int = self._app_config.Scrutinizer.e_frame
            self._file_input = self._app_config.DEFAULT.File_Input

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()
        self._ser.module_errors[ModuleErrorIndex.LIDAR_MODULE_ERROR].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR].update(
            self._err_config
        )
        self._ser.state_errors_D[StateErrorDIndex.LIDAR_DATA_MISSING].update(
            self._err_config
        )

        # NOTE: このプロセスで実施する全ての診断クラスのupdateをここに追加していく

    def _change_file_name_index(self) -> None:
        """
        校正モードかつFileInputのときにFileを変更する。
        """
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode and self._file_input:
            lidar_file_path: str = lidar_filepath_loader(self._sac, self._LidarConfig)[
                self._index
            ]
            assert isinstance(self._provider, CalibMid360FilePointCloudProvider)
            # 校正モードでは、reset後のtimestamp_ixを0としていたが、
            # フレームを指定した再起動に対応するため、s_frameに変更。
            self._provider.change_file_name_index(lidar_file_path, self._frame)

    def _log_register(self) -> None:
        super()._log_register()
        self._sac.log_register(self._app_logger_factory)
        self._sac_calib.log_register(self._app_logger_factory)
        self._ser.log_register(self._app_logger_factory)

    def _apply_parameters(self) -> None:
        pass

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            self._frame = self._app_config_calib.dataCapture.s_frame
        else:
            self._frame: int = self._app_config.Scrutinizer.s_frame
        self._read_lidar_config()
        self._change_device()
        self._change_clock_info()
        self.create_producer_and_consumer()

    def _start_restart(self) -> None:
        # TODO """必要に応じて実際にプロセスを落とさないで再起動で実行する処理を記載""" (NSW)
        self._config_load()
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            self._frame = self._app_config_calib.dataCapture.s_frame
        else:
            self._frame: int = self._app_config.Scrutinizer.s_frame
        self._change_file_name_index()
        self._clockPprovider.reset_time()
        self.producer.require_restart()
        del self.producer

    def _restart_surround_file_input_if_needed(self) -> None:
        calib_mode = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if (
            calib_mode
            or not self._file_input
            or not self._app_config.Scrutinizer.file_input_loop
            or self._frame <= self._end_frame
        ):
            return

        self._frame = self._app_config.Scrutinizer.s_frame
        lidar_file_path = (
            self._app_config.Lidar.lidar0_file,
            self._app_config.Lidar.lidar1_file,
            self._app_config.Lidar.lidar2_file,
            self._app_config.Lidar.lidar3_file,
            self._app_config.Lidar.lidar4_file,
            self._app_config.Lidar.lidar5_file,
        )[self._index]
        assert isinstance(self._provider, Mid360FilePointCloudProvider)
        self._provider.change_file_name_index(lidar_file_path, self._frame)

    def _change_clock_info(self) -> None:
        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if calib_mode:
            if self._file_input:
                basetime = datetime.datetime.strptime(
                    self._LidarConfig.lidarfile_basetime, "%Y-%m-%d_%H-%M-%S"
                )
                steptime = self._LidarConfig.lidarfile_steptime
                self._clockPprovider = DummyClockProvider(
                    basetime, steptime, self._frame, self._end_frame
                )

            else:
                self._clockPprovider = TimeClockProvider()
        else:
            self._clockPprovider = PerfCounterClockProvider()

    def _read_lidar_config(self) -> None:
        config_path = self._app_config.Lidar.path
        file_io_error = self._ser.state_errors_D[StateErrorDIndex.FILE_IO_ERROR]
        try:
            with open(config_path, encoding="utf-8") as f:
                self._lidar_config = json.loads(f.read())
        except (OSError, json.JSONDecodeError, UnicodeError, TypeError, ValueError) as error:
            result = file_io_error.errors_diagnosis(True)
            file_io_error.log_output(
                *result,
                StateErrorDIndex.FILE_IO_ERROR,
                config_path,
                "read config_lidars.json",
                f"{type(error).__name__}: {error}",
            )
            raise
        file_io_error.errors_diagnosis(False)
        # 挿入順でキー一覧を取得
        keys = list(
            self._lidar_config.keys(),
        )  # ["MID3601", "MID3602", ...]
        # index→key マップ(これで_lidar_config_index_map[0] → "MID3601"が得られる)
        self._lidar_config_index_map = dict(enumerate(keys))

    def _shutdown(self) -> None:
        pass

    @log_target("Lidar入力I/F", ProfCategory.Process)
    def _update(self) -> PointCloudData | None:
        try:
            pcd = self._provider.get_accum_points()
        except (OSError, ValueError, TypeError) as error:
            if self._file_input:
                lidar_file_paths = (
                    self._app_config.Lidar.lidar0_file,
                    self._app_config.Lidar.lidar1_file,
                    self._app_config.Lidar.lidar2_file,
                    self._app_config.Lidar.lidar3_file,
                    self._app_config.Lidar.lidar4_file,
                    self._app_config.Lidar.lidar5_file,
                )
                file_io_error = self._ser.state_errors_D[
                    StateErrorDIndex.FILE_IO_ERROR
                ]
                result = file_io_error.errors_diagnosis(True)
                file_io_error.log_output(
                    *result,
                    StateErrorDIndex.FILE_IO_ERROR,
                    lidar_file_paths[self._index],
                    "read file-input LiDAR point cloud",
                    f"{type(error).__name__}: {error}",
                )
            raise
        if self._file_input:
            self._ser.state_errors_D[
                StateErrorDIndex.FILE_IO_ERROR
            ].errors_diagnosis(False)
        self._ser.set_lidar_connected(self._index, pcd is not None)

        if pcd is None:
            return None

        if hasattr(self._provider, "last_quality_degraded"):
            self._sec_lid.last_quality_degraded.value = max(
                self._sec_lid.last_quality_degraded.value,
                self._provider.last_quality_degraded,  # type: ignore[attr-defined]
            )
        if hasattr(self._provider, "last_invalid_ratio"):
            self._sec_lid.invalid_data_ratio.value = (
                self._provider.last_invalid_ratio  # type: ignore[attr-defined]
            )

        is_connection_error = bool(
            self._ser.state_errors_A_C[
                StateErrorIndex.LIDAR0_CONNECTION_ERROR + self._index
            ].is_error.value
        )
        lidar_data_missing = self._ser.state_errors_D[
            StateErrorDIndex.LIDAR_DATA_MISSING
        ]
        result = lidar_data_missing.errors_diagnosis(
            len(pcd), is_connection_error
        )
        lidar_data_missing.log_output(
            *result,
            StateErrorDIndex.LIDAR_DATA_MISSING,
            self._index,
        )

        # for healthy check
        now: float = time.perf_counter()
        if now - self._last_heartbeat > self._heartbeat_interval:
            self._sec_lid.last_heartbeat.value = now
            self._last_heartbeat = now
        t: float = self._clockPprovider.get_time()
        if self._clockPprovider.isframeexceeded():
            self._unsubscribe()
        return PointCloudData(self._frame, t, pcd)

    def create_producer_and_consumer(self) -> None:
        self.producer: Producer[PointCloudData] = self._producer_flow.create_producer()

    def restart_completed(self) -> None:
        self.producer.restart_completed()

    def start_diagnosis(self) -> None:
        self._sec_lid.last_heartbeat.value = INVALID_TIMESTAMP
        self._sec_lid.is_heartbeat_enabled.value = True

    def stop_diagnosis(self) -> None:
        self._sec_lid.is_heartbeat_enabled.value = False

    @log_main()
    def _loop(self) -> None:
        self.start_diagnosis()
        while self.enable:
            if self._sac.last_updated > self._last_updated:
                self._config_load()
                self._apply_parameters()
            try:
                if not self.producer.wait():
                    continue

                self._restart_surround_file_input_if_needed()

                output_data = self._update()
                if output_data is None:
                    isunsubscribe, frame = self._provider.handle_no_input()
                    if isunsubscribe:
                        self._unsubscribe()
                        continue
                    # 入力がなかったとき用のframeで差し替え
                    t: float = self._clockPprovider.get_time()
                    # assert frame    # output_dataが無いときの代替が毎回無いのでAssertionErrorになる
                    output_data = PointCloudData(self._frame, t, frame)

                if not self._print_disabled:
                    self._logger.info(
                        f"points {self._index} read, ix {self._frame}, time:{datetime.datetime.fromtimestamp(time.time())}",
                    )
                self._frame += 1
                self.producer.produce(output_data)
            except Exception as e:
                is_state_error_d_exception = self._ser.is_state_error_d_exception(
                    e, self._logger
                )
                if not is_state_error_d_exception:
                    if self._ser.module_errors[
                        ModuleErrorIndex.LIDAR_MODULE_ERROR
                    ].excepts_diagnosis(e):
                        self._ser.module_errors[
                            ModuleErrorIndex.LIDAR_MODULE_ERROR
                        ].log_output(
                            ResultDiagnosis.DETECTION,
                            ResultDiagnosis.DETECTION,
                            ModuleErrorIndex.LIDAR_MODULE_ERROR,
                            e,
                            self._index,
                        )
                    else:
                        raise e

    def _change_device(self) -> None:
        use_shi_lib: bool = self._app_config.DEFAULT.use_shi_lib
        dot_num_low_threshold = (
            self._err_config.lidar_n_comm_quality_degraded.dot_num_low_threshold
        )

        calib_mode: bool = bool(self._app_config.General.operation_mode == OPM.CALIB)
        if self._file_input:
            from argus_synchro.device.lidar.mid360_points import MID360PointsFile

            # 課題:ファイル入力時のファイルデバイスの切り替え方法
            # 　　　現状のファイル形式はMID360のみなので決め打ちする
            if calib_mode:
                lidar_file_path: str = lidar_filepath_loader(
                    self._sac, self._LidarConfig
                )[self._index]
            else:
                lidar_file_path: str = [
                    self._app_config.Lidar.lidar0_file,
                    self._app_config.Lidar.lidar1_file,
                    self._app_config.Lidar.lidar2_file,
                    self._app_config.Lidar.lidar3_file,
                    self._app_config.Lidar.lidar4_file,
                    self._app_config.Lidar.lidar5_file,
                ][self._index]
            device = MID360PointsFile(
                self._index,
                lidar_file_path,
                self._app_config.Lidar,
                self._frame,
                self._app_logger_factory,
            )
            if calib_mode:
                from argus_synchro.provider.point_cloud import (
                    CalibMid360FilePointCloudProvider,
                )

                self._provider = CalibMid360FilePointCloudProvider(
                    self._index,
                    device,
                    self._LidarConfig,
                    self._frame,
                    self._app_logger_factory,
                )
            else:
                from argus_synchro.provider.point_cloud import (
                    Mid360FilePointCloudProvider,
                )

                self._provider = Mid360FilePointCloudProvider(
                    device,
                    self._frame,
                )
        elif use_shi_lib:
            from argus_synchro.device.lidar.shi_lib_points import ShiLibPoints
            from argus_synchro.provider.point_cloud import ShiLibPointCloudProvider

            device = ShiLibPoints(
                self._index,
                self._app_config.Lidar,
                self._app_logger_factory,
            )
            self._provider = ShiLibPointCloudProvider(device)
        else:
            # 実機

            # LiDARコンフィグのlidar_nameを読み取り、デバイス/プロバイダーを切り替える
            self._unique_lidar_name = self._lidar_config_index_map[self._index]
            lidar_config: dict[str, str | int] = self._lidar_config[
                self._unique_lidar_name
            ]
            if calib_mode:
                from argus_synchro.device.lidar.mid360_points import MID360Points
                from argus_synchro.provider.point_cloud import (
                    CalibMid360PointCloudProvider,
                )

                # 校正モードは現状MID360のみ
                device = MID360Points(
                    self._index,
                    lidar_config,
                    self._app_logger_factory,
                    dot_num_low_threshold,
                )
                self._provider = CalibMid360PointCloudProvider(
                    self._index,
                    device,
                    self._LidarConfig,
                    self._app_logger_factory,
                )
                return

            lidar_name = str(lidar_config["lidar_name"])

            if lidar_name == "MID360":
                from argus_synchro.device.lidar.mid360_points import MID360Points
                from argus_synchro.provider.point_cloud import Mid360PointCloudProvider

                device = MID360Points(
                    self._index,
                    lidar_config,
                    self._app_logger_factory,
                    dot_num_low_threshold,
                )
                self._provider = Mid360PointCloudProvider(device)
            elif lidar_name == "OS0128":
                from argus_synchro.device.lidar.os0128_points import OS0128Points
                from argus_synchro.provider.point_cloud import Os0128PointCloudProvider

                device = OS0128Points(
                    self._index,
                    lidar_config,
                    self._app_logger_factory,
                )
                self._provider = Os0128PointCloudProvider(device)
            elif lidar_name == "AIRY96":
                from argus_synchro.device.lidar.airy96_points import AIRY96Points
                from argus_synchro.provider.point_cloud import Airy96PointCloudProvider

                device = AIRY96Points(self._index, self._app_config)
                self._provider = Airy96PointCloudProvider(device)
            elif lidar_name == "AIRY192":
                from argus_synchro.device.lidar.airy192_points import AIRY192Points
                from argus_synchro.provider.point_cloud import Airy192PointCloudProvider

                device = AIRY192Points(self._index, self._app_config)
                self._provider = Airy192PointCloudProvider(device)
            else:
                err_msg: Final[str] = (
                    f"Unsupported LiDAR name: {lidar_config['lidar_name']}"
                )
                raise ValueError(err_msg)
