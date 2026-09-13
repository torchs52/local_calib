from __future__ import annotations

from abc import ABC, abstractmethod
from multiprocessing.sharedctypes import Synchronized
from pathlib import Path

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.shared_data import create_shared_single_data

INVALID_TIMESTAMP: float = -1.0  # タイムスタンプが未更新の間の初期値。Noneの代替


class SharedProcessExcept(ABC):
    def __init__(self) -> None:
        self.IsFinished: Synchronized[bool] = create_shared_single_data(False)

    @abstractmethod
    def close(self) -> None:
        err_msg: str = f"class: {self.__class__.__name__}, method: close()"
        raise NotImplementedError(err_msg)


# 例外処理:共有メモリクラス(Getdata)
class SharedGetDataExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理:共有メモリクラス(Lidar)
class SharedLIDExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(0.0)
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)
        self.last_quality_degraded: Synchronized[float] = create_shared_single_data(0.0)
        self.invalid_data_ratio: Synchronized[float] = create_shared_single_data(0.0)
        self.IsDead: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理:共有メモリクラス(IMU)
class SharedIMUExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理:共有メモリクラス(CAN)
class SharedCANExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        # プロセスの死活診断用
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)
        # CAN受信の診断用
        self.last_received: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_received_enabled: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理:共有メモリクラス(カメラ)
class SharedCAMExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        # プロセスの死活診断用
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)
        # カメラ受信の診断用
        self.last_received: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_received_enabled: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理:共有メモリクラス(描画)
class SharedVisualizeExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理:共有メモリクラス(AppManager)
class SharedAppManagerExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        self.is_started: Synchronized[bool] = create_shared_single_data(False)
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理:共有メモリクラス(Scrutinizer)
class SharedScrutinizerExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # エラーフラグをここに足していく.(Is... or Has...)
        self.IsSlow: Synchronized[int] = create_shared_single_data(0)
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)

    def close(self) -> None:
        pass


# 例外処理：共有メモリクラス(LidarShiftMonitor).
class SharedLidarShiftMonitorExcept(SharedProcessExcept):
    def __init__(self, path: Path) -> None:
        # エラーフラグをここに足していく.(Is... or Has...)
        super().__init__()
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(
            INVALID_TIMESTAMP
        )
        self.is_heartbeat_enabled: Synchronized[bool] = create_shared_single_data(False)
        self.is_shifted_fast: Synchronized[bool] = create_shared_single_data(False)
        self.is_shifted_slow: Synchronized[bool] = create_shared_single_data(False)

        self.has_not_calibrated_path: Path = path
        self._has_not_calibrated: Synchronized[bool] = create_shared_single_data(False)
        """校正未実施フラグ"""
        self.load_has_not_calibrated()

    @property
    def has_not_calibrated(self) -> bool:
        return self._has_not_calibrated.value

    def write_has_not_calibrated(self, value: bool) -> None:
        self._has_not_calibrated.value = value
        try:
            self.has_not_calibrated_path.write_text(str(value).lower())
        except Exception as e:
            raise RuntimeError(
                f"Failed to write {self.has_not_calibrated_path}: {e}"
            ) from e

    def load_has_not_calibrated(self) -> None:
        if not self.has_not_calibrated_path.exists():
            self._has_not_calibrated.value = False
        try:
            self._has_not_calibrated.value = (
                self.has_not_calibrated_path.read_text().strip().lower() == "true"
            )
        except FileNotFoundError:
            self._has_not_calibrated.value = False
        except Exception as e:
            raise RuntimeError(
                f"Failed to read {self.has_not_calibrated_path}: {e}"
            ) from e

    def close(self) -> None:
        pass


# 例外処理：共有メモリクラス(calibration_mat_generator).
class SharedCalMatGeneratorExcept(SharedProcessExcept):
    def __init__(self) -> None:
        # エラーフラグをここに足していく.(Is... or Has...)
        super().__init__()

    def close(self) -> None:
        pass


# 例外処理:共有メモリ変数の定義.(総合)
class SharedExcepts:
    def __init__(
        self,
        app_config: AppConfig,
        app_manager_ex: SharedAppManagerExcept | None = None,
    ) -> None:
        # このクラスは各プロセスの生成前にインスタンス生成され、各プロセスの生成時にインスタンスを渡す。
        # プロセスは受け取ったインスタンスを自分のAppLoggerFactoryインスタンスに登録する。
        # そのため、この時点ではregisterはせずAppLoggerのインスタンスのみ生成する。
        self._logger: AppLogger = AppLoggerFactory.from_type(self.__class__)
        self.app_config: AppConfig = app_config
        lidar_ex: list[SharedLIDExcept] = []
        imu_ex: list[SharedIMUExcept] = []
        for _ in range(self.app_config.Lidar.count):
            _lid_ex = SharedLIDExcept()
            lidar_ex.append(_lid_ex)
            _imu_ex = SharedIMUExcept()
            imu_ex.append(_imu_ex)

        cam_ex: list[SharedCAMExcept] = []
        for _ in range(self.app_config.camera.count):
            _cam_ex = SharedCAMExcept()  # カメラの共有メモリをインスタンス化
            cam_ex.append(_cam_ex)

        # クラスメンバの可読性を踏まえ、最後にまとめてselfに設定
        self.LiDAR_ex: list[SharedLIDExcept] = lidar_ex  # LID_NUM 分の要素数を持つ配列
        self.IMU_ex: list[SharedIMUExcept] = imu_ex  # IMU_NUM 分の要素数を持つ配列
        self.CAM_ex: list[SharedCAMExcept] = cam_ex  # CAM_NUM 分の要素数を持つ配列
        self.CAN_ex = SharedCANExcept()  # CAN例外処理フラグ
        self.getData_ex = SharedGetDataExcept()  # データ取得例外処理フラグ
        self.Visu_ex = SharedVisualizeExcept()  # UI関連例外処理フラグ
        self.AppMan_ex = (
            SharedAppManagerExcept() if app_manager_ex is None else app_manager_ex
        )  # アプリマネジャー例外処理フラグ・heartbeat
        self.Scruti_ex = SharedScrutinizerExcept()  # Scrutinizer例外処理フラグ
        self.ObjDet_ex = SharedScrutinizerExcept()  # ObjectDetect例外処理フラグ
        self.PointsRefine_ex = SharedScrutinizerExcept()  # PointsRefine例外処理フラグ
        self.Lidar_SM_ex = SharedLidarShiftMonitorExcept(
            Path(app_config.LiDARShiftMonitor.has_not_calibrated_path)
        )  # Lidarズレ検出例外処理フラグ
        self.CalMatGen_ex = (
            SharedCalMatGeneratorExcept()
        )  # 校正マトリクス生成 例外処理フラグ
        # 共有用フレーム番号
        self.frame_number: Synchronized[int] = create_shared_single_data(0)

    def check_scrut_mode_is_finished(self) -> bool:
        res_lid = False
        for i in range(self.app_config.Lidar.count):
            res_lid = res_lid or self.LiDAR_ex[i].IsFinished.value
        res_cam = False
        for i in range(self.app_config.camera.count):
            res_cam = res_cam or self.CAM_ex[i].IsFinished.value

        total_res = (
            res_lid
            or res_cam
            or self.Visu_ex.IsFinished.value
            or self.Scruti_ex.IsFinished.value
            or self.ObjDet_ex.IsFinished.value
            or self.PointsRefine_ex.IsFinished.value
        )
        return total_res

    # 周辺監視モード時の終了フラグのリセット
    def reset_operation_mode_scrut_ex(self) -> None:
        for lidar_ex in self.LiDAR_ex:
            lidar_ex.IsFinished.value = False
        for cam_ex in self.CAM_ex:
            cam_ex.IsFinished.value = False
        self.Visu_ex.IsFinished.value = False
        self.Scruti_ex.IsFinished.value = False
        self.ObjDet_ex.IsFinished.value = False
        self.PointsRefine_ex.IsFinished.value = False
        self.Lidar_SM_ex.IsFinished.value = False

    # 校正モード時の終了フラグのリセット
    def reset_operation_mode_calib_ex(self) -> None:
        self.CalMatGen_ex.IsFinished.value = False

    def show_present_calib_ex(self) -> None:
        self._logger.info(f"{self.CalMatGen_ex.IsFinished.value = }")

    def show_present_scrut_ex(self) -> None:
        for i in range(self.app_config.Lidar.count):
            self._logger.info(f"{i = }, {self.LiDAR_ex[i].IsFinished.value = }")
        for i in range(self.app_config.Lidar.count):
            self._logger.info(f"{i = }, {self.IMU_ex[i].IsFinished.value = }")
        for i in range(self.app_config.camera.count):
            self._logger.info(f"{i = }, {self.CAM_ex[i].IsFinished.value = }")
        self._logger.info(f"{self.Scruti_ex.IsFinished.value = }")
        self._logger.info(f"{self.ObjDet_ex.IsFinished.value = }")
        self._logger.info(f"{self.PointsRefine_ex.IsFinished.value = }")
        self._logger.info(f"{self.Visu_ex.IsFinished.value = }")
        self._logger.info(f"{self.AppMan_ex.IsFinished.value = }")

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        app_logger_factory.append_logger(self._logger)

    def __del__(self) -> None:
        for lidar_ex in self.LiDAR_ex:
            lidar_ex.close()
        for cam_ex in self.CAM_ex:
            cam_ex.close()
        self.Visu_ex.close()
        self.AppMan_ex.close()
        self.Scruti_ex.close()
        self.ObjDet_ex.close()
        self.PointsRefine_ex.close()
