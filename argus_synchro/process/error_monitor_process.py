from __future__ import annotations

import time
from multiprocessing.sharedctypes import Synchronized
from pathlib import Path
from typing import Literal, final

from argus_synchro_lib.error_mmap_writer import ErrorMMapWriter

from argus_synchro.common import paths
from argus_synchro.common.paths import normalize_path
from argus_synchro.diagnosis.error_config import ErrorConfig
from argus_synchro.process import ProcessBase
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.shared_errors import ActionErrorIndex, SharedErrors, StateErrorIndex


@final
class ErrorMonitorProcess(ProcessBase):
    """
    エラー監視プロセス
    """

    __slots__ = (
        "_cycle",
        "_err_config",
        "_is_last_app_manager_diag_enabled",
        "_last_action_err_values",
        "_last_state_err_value",
        "_paths",
        "_ser",
        "_system_activator",
    )

    def __init__(
        self,
        ser: SharedErrors,
        system_activator: ProcessActivator,
        directory_config: paths.DirectoryConfig,
        cycle: float = 1 / 5,
        name: str | None = None,
    ) -> None:
        super().__init__(ser.ErrMoni_ex, system_activator, name=name)
        self._directory_config = directory_config
        self._ser: SharedErrors = ser
        self._system_activator: ProcessActivator = system_activator
        self._cycle: float = cycle  # fpsの逆数(秒)
        self._err_config: ErrorConfig
        self._is_last_app_manager_diag_enabled = False
        self._last_state_err_value: int | None = None
        self._last_action_err_values: tuple[int, ...] | None = None
        log_dir: Path = paths.get_mmap_dir(self._directory_config)
        err0_path: Path = normalize_path("./err0.dat", log_dir)
        err1_path: Path = normalize_path("./err1.dat", log_dir)
        self._paths: list[str] = [str(err0_path), str(err1_path)]
        # startupで宣言
        self._mmap: ErrorMMapWriter
        self.affinity_cores = [0]

    def _config_load(self) -> None:
        pass

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()
        self._ser.state_errors_A_C[
            StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING
        ].update(self._err_config)
        self._ser.action_errors_A_C[
            ActionErrorIndex.MMAP_READ_WRITE_ERROR
        ].update(self._err_config)

    def create_producer_and_consumer(self) -> None:
        pass

    def restart_completed(self) -> None:
        pass

    def _startup(self) -> None:
        self._config_load()
        self._err_config_load()
        self._mmap = ErrorMMapWriter(
            self._paths, lambda level, msg: self._logger.log(int(level), msg)
        )
        self._mmap.init()

    def _log_register(self) -> None:
        super()._log_register()
        self._ser.log_register(self._app_logger_factory)

    def _shutdown(self) -> None:
        self._mmap.close()

    def _loop(self) -> None:
        while self._system_activator.value:
            self._update()
            time.sleep(self._cycle)

    def _update(self) -> None:
        is_enabled = bool(self._ser.AppMan_ex.is_heartbeat_enabled.value)
        if is_enabled:
            diagnosis = self._ser.state_errors_A_C[
                StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING
            ]
            if self._is_last_app_manager_diag_enabled is False:
                diagnosis.clear()
            try:
                result = diagnosis.errors_diagnosis(
                    time.monotonic(), self._ser.AppMan_ex.last_heartbeat.value
                )
                diagnosis.log_output(
                    *result, StateErrorIndex.APPLICATION_MANAGER_NOT_RESPONDING
                )
            except RuntimeError as e:
                self._logger.error(
                    "APPLICATION_MANAGER_NOT_RESPONDING diagnosis failed: %s: %s",
                    type(e).__name__,
                    e,
                )
        self._is_last_app_manager_diag_enabled = is_enabled

        try:
            self._mmap.start_write()
            state_err: bytes = self._make_state_error_bits(self._ser.state_errors)
            self._debug_log_state_errors(state_err)
            self._mmap.write_state_error(state_err)
            action_err: bytes = self._make_action_error_bits(self._ser.action_errors)
            self._debug_log_action_errors(action_err)
            self._mmap.write_action_error(action_err)

            camera_connected: tuple[bool, ...] = self._ser.get_cameras_connected()
            lidar_connected: tuple[bool, ...] = self._ser.get_lidars_connected()
            status: bytes = self._make_status_bits(
                self._ser,
                camera_connected=camera_connected,
                lidar_connected=lidar_connected,
            )
            self._mmap.write_status(status)
            self._mmap.rotate_if_busy()
        except (OSError, ValueError, BufferError, RuntimeError) as error:
            diagnosis = self._ser.action_errors_A_C[
                ActionErrorIndex.MMAP_READ_WRITE_ERROR
            ]
            is_target = diagnosis.excepts_diagnosis(error)
            diagnosis.log_output(
                is_target,
                False,
                ActionErrorIndex.MMAP_READ_WRITE_ERROR,
                "ErrorMMapWriter transaction failed",
                error,
            )

    def _start_restart(self) -> None:
        pass

    def _make_state_error_bits(
        self,
        error_list: tuple[Synchronized[bool], ...],
        byteorder: Literal["little", "big"] = "little",
        num_bytes: int = 16,
    ) -> bytes:
        """
        状態エラーリストからエラーフラグのビット列を作成する
        """
        err_bits: int = sum((int(err.value) << i) for i, err in enumerate(error_list))
        # 念のため範囲外のbitをマスク
        err_bits &= (1 << (num_bytes * 8)) - 1
        return err_bits.to_bytes(num_bytes, byteorder)

    def _make_action_error_bits(
        self,
        error_list: tuple[Synchronized[int], ...],
        byteorder: Literal["little", "big"] = "little",
        num_bytes: int = 32,
    ) -> bytes:
        """
        動作エラーリストからエラーカウンタのビット列を作成する
        """
        err_bits: int = sum((err.value << (i * 8)) for i, err in enumerate(error_list))
        err_bits &= (1 << (num_bytes * 8)) - 1
        return err_bits.to_bytes(num_bytes, byteorder)

    def _debug_log_state_errors(self, state_err: bytes) -> None:
        state_err_value = int.from_bytes(state_err, byteorder="little")
        self._logger.debug("state_error: hex=0x%032X", state_err_value)

        previous_value = getattr(self, "_last_state_err_value", None)
        if previous_value is None:
            changed_bits = state_err_value
        elif state_err_value == previous_value:
            return
        else:
            changed_bits = state_err_value ^ previous_value

        for error_index in StateErrorIndex:
            if error_index is StateErrorIndex.INDEX_MAX or error_index.name.startswith(
                "RESERVED_"
            ):
                continue
            bit_no = int(error_index)
            if ((changed_bits >> bit_no) & 1) == 0:
                continue
            is_on = (state_err_value >> bit_no) & 1
            self._logger.error(
                "state_error %s: bit%d=%s",
                "ON " if is_on else "OFF",
                bit_no,
                error_index.name,
            )

        self._last_state_err_value = state_err_value

    def _debug_log_action_errors(self, action_err: bytes) -> None:
        current_values = tuple(
            action_err[index] for index in range(len(self._ser.action_errors))
        )
        action_err_value = int.from_bytes(action_err, byteorder="little")
        self._logger.debug("action_error: hex=0x%064X", action_err_value)

        previous_values = getattr(self, "_last_action_err_values", None)
        changed_errors: list[str] = []
        for error_index in ActionErrorIndex:
            if error_index is ActionErrorIndex.INDEX_MAX or error_index.name.startswith(
                "RESERVED_"
            ):
                continue
            index = int(error_index)
            if index >= len(current_values):
                continue
            current_value = current_values[index]
            if previous_values is None:
                changed_errors.append(
                    f"index{index}={error_index.name}:{current_value}"
                )
            elif current_value != previous_values[index]:
                changed_errors.append(
                    f"index{index}={error_index.name}:"
                    f"{previous_values[index]}->{current_value}"
                )

        if changed_errors:
            self._logger.error(
                "action_error counters: %s",
                ", ".join(changed_errors),
            )
        self._last_action_err_values = current_values

    def _make_status_bits(
        self,
        ser: SharedErrors,
        camera_connected: tuple[bool, ...],
        lidar_connected: tuple[bool, ...],
        byteorder: Literal["little", "big"] = "little",
        num_bytes: int = 3,
    ) -> bytes:
        """
        各状態からステータスのビット列を作成する
        """
        reduced_load_mode_bits: int = int(ser.reduced_load_mode.enabled)
        camera_connected_bits: int = sum(
            (int(connected) << i) for i, connected in enumerate(camera_connected)
        )
        lidar_connected_bits: int = sum(
            (int(connected) << i) for i, connected in enumerate(lidar_connected)
        )

        self._logger.debug(
            f"Reduced Load Mode: {reduced_load_mode_bits}, "
            f"Camera Connected: {camera_connected_bits:08b}, "
            f"Lidar Connected: {lidar_connected_bits:08b}"
        )

        statuses: tuple[int, ...] = (
            reduced_load_mode_bits,
            camera_connected_bits,
            lidar_connected_bits,
        )
        return b"".join(statuses[i].to_bytes(1, byteorder) for i in range(num_bytes))
