from contextlib import suppress
from enum import IntEnum, auto
from itertools import chain
from multiprocessing.sharedctypes import Synchronized
from pathlib import Path
from typing import TYPE_CHECKING

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.diagnosis.action_errors import (
    AiModelLoadFailed,  # AI_MODEL_LOAD_FAILED  # AIモデルロード失敗/破損
    CameraXCalibDataInvalidDiagnosis,  # CAMERA_N_CALIB_DATA_INVALID  # カメラN校正データ不正
    ConfigFileMissingDiagnosis,  # CONFIG_FILE_MISSING  # 設定ファイル欠損/破損
    CraneModelFileMissingDiagnosis,  # CRANE_MODEL_FILE_MISSING  # 機体モデルファイル欠損/破損
    LidarPositionMisalignmentDetectedDiagnosis,  # LIDAR_POSITION_MISALIGNMENT_DETECTED  # LiDAR位置ズレ検出
    LogFileIoErrorDiagnosis,  # LOG_FILE_IO_ERROR  # ログファイルI/Oエラー
    MmapReadWriteErrorDiagnosis,  # MMAP_READ_WRITE_ERROR  # MMAP read/writeエラー
    ModelInfoMismatchDiagnosis,  # MODEL_INFO_MISMATCH  # 機種情報不一致
    OperationModeTransitionErrorDiagnosis,  # OPERATION_MODE_TRANSITION_ERROR  # 動作モード遷移エラー
    ProcessStartupErrorDiagnosis,  # PROCESS_STARTUP_ERROR  # プロセス起動エラー
    RebootLoopDetectedDiagnosis,  # REBOOT_LOOP_DETECTED  # 再起動ループ検出
    SensorCalibDataInvalidDiagnosis,  # SENSOR_CALIB_DATA_INVALID  # センサ校正データ不正
    SensorCalibrationRequiredDiagnosis,  # SENSOR_CALIBRATION_REQUIRED  # 要センサ校正
)
from argus_synchro.diagnosis.error_diagnosis import (
    ActionErrorDiagnosisA,
    ActionErrorDiagnosisB,
    ActionErrorDiagnosisC,
    DiagnosisRuntimePolicy,
    ResultDiagnosis,
    StateErrorDiagnosisA,
    StateErrorDiagnosisD,
)
from argus_synchro.diagnosis.reduced_load_mode import ReducedLoadMode
from argus_synchro.diagnosis.state_d_errors import (
    AccumulationModuleError,
    AiInferenceResultError,
    AppManagerModuleError,
    ArrayShapeError,
    CalibrationModuleError,
    CameraDataMissing,
    CameraHumanDetectionModuleError,
    CameraModuleError,
    CanModuleError,
    CollisionJudgmentModuleError,
    FileIoError,
    GetDataModuleError,
    ImuModuleError,
    Integrate2d3dModuleError,
    InvalidDataInput,
    LidarDataMissing,
    LidarModuleError,
    LidarShiftMonitorModuleError,
    LogCompressionFailure,
    LogTimeReversal,
    MainModuleError,
    MemoryLeakDetected,
    NumericAnomalyException,
    Object3DDetectionModuleError,
    OtherHardwareError,
    PointsRefineModuleError,
    ProcessForcedTermination,
    VisualModuleError,
)
from argus_synchro.diagnosis.state_errors import (
    ApplicationManagerNotRespondingDiagnosis,  # APPLICATION_MANAGER_NOT_RESPONDING  # アプリケーションマネージャー未応答
    CameraNCommQualityDegradedDiagnosis,  # CAMERA0_COMM_QUALITY_DEGRADED  # カメラ0通信品質低下
    CameraNCommQualityErrorDiagnosis,  # CAMERA0_COMM_QUALITY_ERROR  # カメラ0通信品質エラー
    CameraNConnectionErrorDiagnosis,  # CAMERA0_CONNECTION_ERROR  # カメラ0接続エラー
    CameraNInvalidDataDiagnosis,  # CAMERA0_INVALID_DATA  # カメラ0データ不正
    CanCommQualityDegradedDiagnosis,  # CAN_COMM_QUALITY_DEGRADED  # CAN通信品質低下
    CanCommQualityErrorDiagnosis,  # CAN_COMM_QUALITY_ERROR  # CAN通信品質エラー
    CanConnectionErrorDiagnosis,  # CAN_CONNECTION_ERROR  # CAN接続エラー
    CanInvalidDataDiagnosis,  # CAN_INVALID_DATA_DIAGNOSIS  # CANデータ不正
    GpuPerformanceDegradedDiagnosis,  # GPU_PERFORMANCE_DEGRADED  # GPU性能低下
    ImuNConnectionErrorDiagnosis,  # IMU_N_CONNECTION_ERROR  # imuN接続エラー
    InternalTemperatureRiseDiagnosis,  # INTERNAL_TEMPERATURE_RISE  # 内部温度上昇
    LidarNCommQualityDegradedDiagnosis,  # LIDAR0_COMM_QUALITY_DEGRADED  # Lidar0通信品質低下
    LidarNCommQualityErrorDiagnosis,  # LIDAR0_COMM_QUALITY_ERROR  # Lidar0通信品質エラー
    LidarNConnectionErrorDiagnosis,  # LIDAR0_CONNECTION_ERROR  # Lidar0接続エラー
    LidarNInvalidDataDiagnosis,  # LIDAR0_INVALID_DATA  # Lidar0データ不正
    LidarPositionMisalignmentNotRespondingDiagnosis,  # LIDAR_POSITION_MISALIGNMENT_NOT_RESPONDING  # Lidar位置ズレ検出 未応答
    LogOutputStoppedDiagnosis,  # LOG_OUTPUT_STOPPED  # ログ出力停止
    MonitorProcessNotRespondingDiagnosis,  # MONITOR_PROCESS_NOT_RESPONDING  # Monitorプロセス未応答
    OutOfMemoryDiagnosis,  # OUT_OF_MEMORY  # メモリ不足
    ProcessingSpeedDegradationTrendDiagnosis,  # PROCESSING_SPEED_DEGRADATION_TREND  # 処理速度低下トレンド
    ProcessingSpeedDegradedDiagnosis,  # PROCESSING_SPEED_DEGRADED  # 処理速度低下
    StatusInfoNotUpdatedDiagnosis,  # STATUS_INFO_NOT_UPDATED  # ステータス情報　未更新
    StorageSpaceLowDiagnosis,  # STORAGE_SPACE_LOW  # ストレージ残容量低下
    SurroundMonitorModuleNotRespondingDiagnosis,  # SURROUND_MONITOR_MODULE_NOT_RESPONDING  # 周辺監視モジュール 未応答
    TemperatureRiseTrendContinuesDiagnosis,  # TEMPERATURE_RISE_TREND_CONTINUES  # 温度上昇傾向の継続
    TemperatureSensorAbnormalDiagnosis,  # TEMPERATURE_SENSOR_ABNORMAL  # 温度センサ異常
    YawAngleInfoErrorDiagnosis,  # YAW_ANGLE_INFO_ERROR  # 旋回角情報エラー
)
from argus_synchro.shared_data import create_shared_single_data
from argus_synchro.shared_err_config import SharedErrorConfig
from argus_synchro.shared_excepts import SharedAppManagerExcept, SharedProcessExcept

if TYPE_CHECKING:
    from argus_synchro.config.app_config import CalibrationConf
    from argus_synchro.diagnosis.error_config import ErrorConfig
    from argus_synchro.diagnosis.error_diagnosis import (
        StateErrorDiagnosisA,
        StateErrorDiagnosisB,
        StateErrorDiagnosisC,
        StateErrorDiagnosisD,
    )


class ActionErrorIndex(IntEnum):
    """
    動作エラーインデックス
    """

    LIDAR_POSITION_MISALIGNMENT_DETECTED = 0  # LiDAR位置ズレ検出
    SENSOR_CALIBRATION_REQUIRED = auto()  # 要センサ校正
    MODEL_INFO_MISMATCH = auto()  # 機種情報不一致
    CRANE_MODEL_FILE_MISSING = auto()  # 機体モデルファイル欠損/破損
    CONFIG_FILE_MISSING = auto()  # 設定ファイル欠損/破損

    SENSOR_CALIB_DATA_INVALID = auto()  # センサ校正データ不正
    CAMERA0_CALIB_DATA_INVALID = auto()  # カメラ0校正データ不正
    CAMERA1_CALIB_DATA_INVALID = auto()  # カメラ1校正データ不正
    CAMERA2_CALIB_DATA_INVALID = auto()  # カメラ2校正データ不正
    CAMERA3_CALIB_DATA_INVALID = auto()  # カメラ3校正データ不正

    MMAP_READ_WRITE_ERROR = auto()  # MMAP read/writeエラー
    REBOOT_LOOP_DETECTED = auto()  # 再起動ループ検出
    AI_MODEL_LOAD_FAILED = auto()  # AIモデルロード失敗/破損

    OPERATION_MODE_TRANSITION_ERROR = auto()  # 動作モード遷移エラー
    LOG_FILE_IO_ERROR = auto()  # ログファイルI/Oエラー

    PROCESS_STARTUP_ERROR = auto()  # プロセス起動エラー

    RESERVED_16 = auto()  # 予約
    RESERVED_17 = auto()  # 予約
    RESERVED_18 = auto()  # 予約
    RESERVED_19 = auto()  # 予約
    RESERVED_20 = auto()  # 予約
    RESERVED_21 = auto()  # 予約
    RESERVED_22 = auto()  # 予約
    RESERVED_23 = auto()  # 予約
    RESERVED_24 = auto()  # 予約
    RESERVED_25 = auto()  # 予約
    RESERVED_26 = auto()  # 予約
    RESERVED_27 = auto()  # 予約
    RESERVED_28 = auto()  # 予約
    RESERVED_29 = auto()  # 予約
    RESERVED_30 = auto()  # 予約
    RESERVED_31 = auto()  # 予約
    INDEX_MAX = auto()  # 最大値


class StateErrorIndex(IntEnum):
    """
    状態エラーインデックス
    """

    LIDAR0_CONNECTION_ERROR = 0  # Lidar0接続エラー
    LIDAR1_CONNECTION_ERROR = auto()  # Lidar1接続エラー
    CAMERA0_CONNECTION_ERROR = auto()  # カメラ0接続エラー
    CAMERA1_CONNECTION_ERROR = auto()  # カメラ1接続エラー
    CAMERA2_CONNECTION_ERROR = auto()  # カメラ2接続エラー
    CAMERA3_CONNECTION_ERROR = auto()  # カメラ3接続エラー
    CAN_CONNECTION_ERROR = auto()  # CAN接続エラー

    LIDAR0_COMM_QUALITY_DEGRADED = auto()  # Lidar0通信品質低下
    LIDAR1_COMM_QUALITY_DEGRADED = auto()  # Lidar1通信品質低下
    CAMERA0_COMM_QUALITY_DEGRADED = auto()  # カメラ0通信品質低下
    CAMERA1_COMM_QUALITY_DEGRADED = auto()  # カメラ1通信品質低下
    CAMERA2_COMM_QUALITY_DEGRADED = auto()  # カメラ2通信品質低下
    CAMERA3_COMM_QUALITY_DEGRADED = auto()  # カメラ3通信品質低下

    LIDAR0_COMM_QUALITY_ERROR = auto()  # Lidar0通信品質エラー
    LIDAR1_COMM_QUALITY_ERROR = auto()  # Lidar1通信品質エラー
    CAMERA0_COMM_QUALITY_ERROR = auto()  # カメラ0通信品質エラー
    CAMERA1_COMM_QUALITY_ERROR = auto()  # カメラ1通信品質エラー
    CAMERA2_COMM_QUALITY_ERROR = auto()  # カメラ2通信品質エラー
    CAMERA3_COMM_QUALITY_ERROR = auto()  # カメラ3通信品質エラー

    LIDAR0_INVALID_DATA = auto()  # Lidar0データ不正
    LIDAR1_INVALID_DATA = auto()  # Lidar1データ不正
    CAMERA0_INVALID_DATA = auto()  # カメラ0データ不正
    CAMERA1_INVALID_DATA = auto()  # カメラ1データ不正
    CAMERA2_INVALID_DATA = auto()  # カメラ2データ不正
    CAMERA3_INVALID_DATA = auto()  # カメラ3データ不正

    YAW_ANGLE_INFO_ERROR = auto()  # 旋回角情報エラー
    CAN_COMM_QUALITY_DEGRADED = auto()  # CAN通信品質低下
    CAN_COMM_QUALITY_ERROR = auto()  # CAN通信品質エラー
    CAN_INVALID_DATA_DIAGNOSIS = auto()  # CANデータ不正

    STORAGE_SPACE_LOW = auto()  # ストレージ残容量低下
    PROCESSING_SPEED_DEGRADED = auto()  # 処理速度低下
    PROCESSING_SPEED_DEGRADATION_TREND = auto()  # 処理速度低下トレンド
    OUT_OF_MEMORY = auto()  # メモリ不足
    GPU_PERFORMANCE_DEGRADED = auto()  # GPU性能低下
    MONITOR_PROCESS_NOT_RESPONDING = auto()  # Monitorプロセス未応答
    STATUS_INFO_NOT_UPDATED = auto()  # ステータス情報　未更新
    SURROUND_MONITOR_MODULE_NOT_RESPONDING = auto()  # 周辺監視モジュール 未応答

    LIDAR_POSITION_MISALIGNMENT_NOT_RESPONDING = auto()  # Lidar位置ズレ検出 未応答
    APPLICATION_MANAGER_NOT_RESPONDING = auto()  # アプリケーションマネージャー未応答
    IMU0_CONNECTION_ERROR = auto()  # imu0接続エラー
    IMU1_CONNECTION_ERROR = auto()  # imu1接続エラー

    LOG_OUTPUT_STOPPED = auto()  # ログ出力停止
    INTERNAL_TEMPERATURE_RISE = auto()  # 内部温度上昇
    TEMPERATURE_SENSOR_ABNORMAL = auto()  # 温度センサ異常
    TEMPERATURE_RISE_TREND_CONTINUES = auto()  # 温度上昇傾向の継続

    RESERVED_45 = auto()  # 予約
    RESERVED_46 = auto()  # 予約
    RESERVED_47 = auto()  # 予約
    RESERVED_48 = auto()  # 予約
    RESERVED_49 = auto()  # 予約
    RESERVED_50 = auto()  # 予約
    RESERVED_51 = auto()  # 予約
    RESERVED_52 = auto()  # 予約
    RESERVED_53 = auto()  # 予約
    RESERVED_54 = auto()  # 予約
    RESERVED_55 = auto()  # 予約
    RESERVED_56 = auto()  # 予約
    RESERVED_57 = auto()  # 予約
    RESERVED_58 = auto()  # 予約
    RESERVED_59 = auto()  # 予約
    RESERVED_60 = auto()  # 予約
    RESERVED_61 = auto()  # 予約
    RESERVED_62 = auto()  # 予約
    RESERVED_63 = auto()  # 予約
    RESERVED_64 = auto()  # 予約
    RESERVED_65 = auto()  # 予約
    RESERVED_66 = auto()  # 予約
    RESERVED_67 = auto()  # 予約
    RESERVED_68 = auto()  # 予約
    RESERVED_69 = auto()  # 予約
    RESERVED_70 = auto()  # 予約
    RESERVED_71 = auto()  # 予約
    RESERVED_72 = auto()  # 予約
    RESERVED_73 = auto()  # 予約
    RESERVED_74 = auto()  # 予約
    RESERVED_75 = auto()  # 予約
    RESERVED_76 = auto()  # 予約
    RESERVED_77 = auto()  # 予約
    RESERVED_78 = auto()  # 予約
    RESERVED_79 = auto()  # 予約
    RESERVED_80 = auto()  # 予約
    RESERVED_81 = auto()  # 予約
    RESERVED_82 = auto()  # 予約
    RESERVED_83 = auto()  # 予約
    RESERVED_84 = auto()  # 予約
    RESERVED_85 = auto()  # 予約
    RESERVED_86 = auto()  # 予約
    RESERVED_87 = auto()  # 予約
    RESERVED_88 = auto()  # 予約
    RESERVED_89 = auto()  # 予約
    RESERVED_90 = auto()  # 予約
    RESERVED_91 = auto()  # 予約
    RESERVED_92 = auto()  # 予約
    RESERVED_93 = auto()  # 予約
    RESERVED_94 = auto()  # 予約
    RESERVED_95 = auto()  # 予約
    RESERVED_96 = auto()  # 予約
    RESERVED_97 = auto()  # 予約
    RESERVED_98 = auto()  # 予約
    RESERVED_99 = auto()  # 予約
    RESERVED_100 = auto()  # 予約
    RESERVED_101 = auto()  # 予約
    RESERVED_102 = auto()  # 予約
    RESERVED_103 = auto()  # 予約
    RESERVED_104 = auto()  # 予約
    RESERVED_105 = auto()  # 予約
    RESERVED_106 = auto()  # 予約
    RESERVED_107 = auto()  # 予約
    RESERVED_108 = auto()  # 予約
    RESERVED_109 = auto()  # 予約
    RESERVED_110 = auto()  # 予約
    RESERVED_111 = auto()  # 予約
    RESERVED_112 = auto()  # 予約
    RESERVED_113 = auto()  # 予約
    RESERVED_114 = auto()  # 予約
    RESERVED_115 = auto()  # 予約
    RESERVED_116 = auto()  # 予約
    RESERVED_117 = auto()  # 予約
    RESERVED_118 = auto()  # 予約
    RESERVED_119 = auto()  # 予約
    RESERVED_120 = auto()  # 予約
    RESERVED_121 = auto()  # 予約
    RESERVED_122 = auto()  # 予約
    RESERVED_123 = auto()  # 予約
    RESERVED_124 = auto()  # 予約
    RESERVED_125 = auto()  # 予約
    RESERVED_126 = auto()  # 予約
    RESERVED_127 = auto()  # 予約
    INDEX_MAX = auto()  # 最大値


class StateErrorDIndex(IntEnum):
    """
    状態エラーDインデックス
    """

    CAMERA_DATA_MISSING = 0  # カメラデータ欠落
    MEMORY_LEAK_DETECTED = auto()  # メモリリーク検出
    OTHER_HARDWARE_ERROR = auto()  # その他のエラー（ハードウェア）
    INVALID_DATA_INPUT = auto()  # 不正データ入力
    NUMERIC_ANOMALY_EXCEPTION = auto()  # NaN・0除算
    ARRAY_SHAPE_ERROR = auto()  # 配列形状エラー
    PROCESS_FORCED_TERMINATION = auto()  # プロセスの強制終了を実施
    FILE_IO_ERROR = auto()  # ファイルI/Oエラー
    LOG_COMPRESSION_FAILURE = auto()  # ログ圧縮失敗
    LOG_TIME_REVERSAL = auto()  # ログ時刻逆転
    LIDAR_DATA_MISSING = auto()  # Lidarデータ欠落
    AI_INFERENCE_RESULT_ERROR = auto()  # AI推論結果異常


class ModuleErrorIndex(IntEnum):
    """
    モジュールエラーインデックス
    """

    LIDAR_MODULE_ERROR = 0  # LiDARモジュールエラー
    CAMERA_MODULE_ERROR = auto()  # カメラモジュールエラー
    ACCUMULATION_MODULE_ERROR = auto()  # 蓄積モジュールエラー
    CAN_MODULE_ERROR = auto()  # CANモジュールエラー
    INTEGRATE_2D3D_MODULE_ERROR = auto()  # 2D-3D紐づけモジュールエラー
    OBJECT_3D_DETECTION_MODULE_ERROR = auto()  # 3D物体検知モジュールエラー
    CAMERA_HUMAN_DETECTION_MODULE_ERROR = auto()  # カメラ人検知モジュールエラー
    COLLISION_JUDGMENT_MODULE_ERROR = auto()  # 衝突判定モジュールエラー
    CALIBRATION_MODULE_ERROR = auto()  # 校正モジュールエラー
    IMU_MODULE_ERROR = auto()  # IMUモジュールエラー
    APP_MANAGER_MODULE_ERROR = auto()  # アプリケーションマネージャーモジュールエラー
    MAIN_MODULE_ERROR = auto()  # Mainモジュールエラー
    POINTS_REFINE_MODULE_ERROR = auto()  # PointsRefineモジュールエラー
    VISUAL_MODULE_ERROR = auto()  # VisualProcessモジュールエラー
    LIDAR_SHIFT_MONITOR_MODULE_ERROR = auto()  # LiDARシフトモニタモジュールエラー
    GET_DATA_MODULE_ERROR = auto()  # データ取得モジュールエラー


# 例外処理：共有メモリクラス(ErrorMonitor).
class SharedErrorMonitorExcept(SharedProcessExcept):
    def __init__(self) -> None:
        # エラーフラグをここに足していく.(Is... or Has...)
        super().__init__()

    def close(self) -> None:
        pass


class SharedErrors:
    def __init__(self, err_conf_path: Path) -> None:
        self.ErrMoni_ex = SharedErrorMonitorExcept()  # エラー監視機能例外処理フラグ
        self.AppMan_ex = SharedAppManagerExcept()  # AppManager診断用heartbeat
        self.reduced_load_mode: ReducedLoadMode = ReducedLoadMode()  # 負荷低減モード
        # 全processで同じメンテナンスモード状態を参照する。
        self.diagnosis_runtime_policy = DiagnosisRuntimePolicy()
        self.shared_err_conf = SharedErrorConfig(
            err_conf_path
        )  # エラー設定ファイルの共有メモリ
        self._lidars_connected: list[Synchronized[bool]] = [
            create_shared_single_data(False) for _ in range(8)
        ]  # LiDAR接続状態の保持

        self._cameras_connected: list[Synchronized[bool]] = [
            create_shared_single_data(False) for _ in range(8)
        ]  # カメラ接続状態の保持
        # 診断系はいったんここに追加する
        self.state_errors_A_C: tuple[
            StateErrorDiagnosisA | StateErrorDiagnosisB | StateErrorDiagnosisC, ...
        ] = (
            LidarNConnectionErrorDiagnosis(self.diagnosis_runtime_policy),  # LIDAR0_CONNECTION_ERROR  # Lidar0接続エラー
            LidarNConnectionErrorDiagnosis(self.diagnosis_runtime_policy),  # LIDAR1_CONNECTION_ERROR  # Lidar1接続エラー
            CameraNConnectionErrorDiagnosis(),  # CAMERA0_CONNECTION_ERROR  # カメラ0接続エラー
            CameraNConnectionErrorDiagnosis(),  # CAMERA1_CONNECTION_ERROR  # カメラ1接続エラー
            CameraNConnectionErrorDiagnosis(),  # CAMERA2_CONNECTION_ERROR  # カメラ2接続エラー
            CameraNConnectionErrorDiagnosis(),  # CAMERA3_CONNECTION_ERROR  # カメラ3接続エラー
            CanConnectionErrorDiagnosis(self.diagnosis_runtime_policy),  # CAN_CONNECTION_ERROR  # CAN接続エラー
            LidarNCommQualityDegradedDiagnosis(),  # LIDAR0_COMM_QUALITY_DEGRADED  # Lidar0通信品質低下
            LidarNCommQualityDegradedDiagnosis(),  # LIDAR1_COMM_QUALITY_DEGRADED  # Lidar1通信品質低下
            CameraNCommQualityDegradedDiagnosis(),  # CAMERA0_COMM_QUALITY_DEGRADED  # カメラ0通信品質低下
            CameraNCommQualityDegradedDiagnosis(),  # CAMERA1_COMM_QUALITY_DEGRADED  # カメラ1通信品質低下
            CameraNCommQualityDegradedDiagnosis(),  # CAMERA2_COMM_QUALITY_DEGRADED  # カメラ2通信品質低下
            CameraNCommQualityDegradedDiagnosis(),  # CAMERA3_COMM_QUALITY_DEGRADED  # カメラ3通信品質低下
            LidarNCommQualityErrorDiagnosis(),  # LIDAR0_COMM_QUALITY_ERROR  # Lidar0通信品質エラー
            LidarNCommQualityErrorDiagnosis(),  # LIDAR1_COMM_QUALITY_ERROR  # Lidar1通信品質エラー
            CameraNCommQualityErrorDiagnosis(),  # CAMERA0_COMM_QUALITY_ERROR  # カメラ0通信品質エラー
            CameraNCommQualityErrorDiagnosis(),  # CAMERA1_COMM_QUALITY_ERROR  # カメラ1通信品質エラー
            CameraNCommQualityErrorDiagnosis(),  # CAMERA2_COMM_QUALITY_ERROR  # カメラ2通信品質エラー
            CameraNCommQualityErrorDiagnosis(),  # CAMERA3_COMM_QUALITY_ERROR  # カメラ3通信品質エラー
            LidarNInvalidDataDiagnosis(),  # LIDAR0_INVALID_DATA  # Lidar0データ不正
            LidarNInvalidDataDiagnosis(),  # LIDAR1_INVALID_DATA  # Lidar1データ不正
            CameraNInvalidDataDiagnosis(),  # CAMERA0_INVALID_DATA  # カメラ0データ不正
            CameraNInvalidDataDiagnosis(),  # CAMERA1_INVALID_DATA  # カメラ1データ不正
            CameraNInvalidDataDiagnosis(),  # CAMERA2_INVALID_DATA  # カメラ2データ不正
            CameraNInvalidDataDiagnosis(),  # CAMERA3_INVALID_DATA  # カメラ3データ不正
            YawAngleInfoErrorDiagnosis(self.diagnosis_runtime_policy),  # YAW_ANGLE_INFO_ERROR  # 旋回角情報エラー
            CanCommQualityDegradedDiagnosis(),  # CAN_COMM_QUALITY_DEGRADED  # CAN通信品質低下
            CanCommQualityErrorDiagnosis(),  # CAN_COMM_QUALITY_ERROR  # CAN通信品質エラー
            CanInvalidDataDiagnosis(),  # CAN_INVALID_DATA_DIAGNOSIS  # CANデータ不正
            StorageSpaceLowDiagnosis(),  # STORAGE_SPACE_LOW  # ストレージ残容量低下
            ProcessingSpeedDegradedDiagnosis(),  # PROCESSING_SPEED_DEGRADED  # 処理速度低下
            ProcessingSpeedDegradationTrendDiagnosis(),  # PROCESSING_SPEED_DEGRADATION_TREND  # 処理速度低下トレンド
            OutOfMemoryDiagnosis(),  # OUT_OF_MEMORY  # メモリ不足
            GpuPerformanceDegradedDiagnosis(),  # GPU_PERFORMANCE_DEGRADED  # GPU性能低下
            MonitorProcessNotRespondingDiagnosis(self.diagnosis_runtime_policy),  # MONITOR_PROCESS_NOT_RESPONDING  # Monitorプロセス未応答
            StatusInfoNotUpdatedDiagnosis(),  # STATUS_INFO_NOT_UPDATED  # ステータス情報　未更新
            SurroundMonitorModuleNotRespondingDiagnosis(self.diagnosis_runtime_policy),  # SURROUND_MONITOR_MODULE_NOT_RESPONDING  # 周辺監視モジュール 未応答
            LidarPositionMisalignmentNotRespondingDiagnosis(),  # LIDAR_POSITION_MISALIGNMENT_NOT_RESPONDING  # Lidar位置ズレ検出 未応答
            ApplicationManagerNotRespondingDiagnosis(self.diagnosis_runtime_policy),  # APPLICATION_MANAGER_NOT_RESPONDING  # アプリケーションマネージャー未応答
            ImuNConnectionErrorDiagnosis(),  # IMU0_CONNECTION_ERROR  # imu0接続エラー
            ImuNConnectionErrorDiagnosis(),  # IMU1_CONNECTION_ERROR  # imu1接続エラー
            LogOutputStoppedDiagnosis(),  # LOG_OUTPUT_STOPPED  # ログ出力停止
            InternalTemperatureRiseDiagnosis(),  # INTERNAL_TEMPERATURE_RISE  # 内部温度上昇
            TemperatureSensorAbnormalDiagnosis(),  # TEMPERATURE_SENSOR_ABNORMAL  # 温度センサ異常
            TemperatureRiseTrendContinuesDiagnosis(),  # TEMPERATURE_RISE_TREND_CONTINUES  # 温度上昇傾向の継続
        )
        """
        状態エラー(重要度A~C)検出クラスリスト
        """

        self.state_errors: tuple[Synchronized[bool], ...] = tuple(
            diag.is_error for diag in self.state_errors_A_C
        )
        """
        状態エラーリスト
        """

        self.state_fail_safes: tuple[Synchronized[bool], ...] = tuple(
            diag.is_fail_safe for diag in self.state_errors_A_C
        )
        """
        状態エラー時動作リスト
        """
        self.state_errors_D: tuple[StateErrorDiagnosisD, ...] = (
            CameraDataMissing(),
            MemoryLeakDetected(),
            OtherHardwareError(),
            InvalidDataInput(),
            NumericAnomalyException(),
            ArrayShapeError(),
            ProcessForcedTermination(),
            FileIoError(),
            LogCompressionFailure(),
            LogTimeReversal(),
            LidarDataMissing(),
            AiInferenceResultError(),
        )
        """
        重要度D検出クラスリスト
        """
        self.state_errors_D_ex: tuple[StateErrorDiagnosisD, ...] = (
            self.state_errors_D[StateErrorDIndex.NUMERIC_ANOMALY_EXCEPTION],
            self.state_errors_D[StateErrorDIndex.AI_INFERENCE_RESULT_ERROR],
        )
        """
        重要度D例外検出クラスリスト
        """

        self.module_errors: tuple[StateErrorDiagnosisD, ...] = (
            LidarModuleError(),  # LIDAR_MODULE_ERROR  # LiDARモジュールエラー
            CameraModuleError(),  # CAMERA_MODULE_ERROR  # カメラモジュールエラー
            AccumulationModuleError(),  # ACCUMULATION_MODULE_ERROR  # 蓄積モジュールエラー
            CanModuleError(),  # CAN_MODULE_ERROR  # CANモジュールエラー
            Integrate2d3dModuleError(),  # LINKAGE_2D3D_MODULE_ERROR  # 2D-3D紐づけモジュールエラー
            Object3DDetectionModuleError(),  # OBJECT_3D_DETECTION_MODULE_ERROR  # 3D物体検知モジュールエラー
            CameraHumanDetectionModuleError(),  # CAMERA_HUMAN_DETECTION_MODULE_ERROR  # カメラ人検知モジュールエラー
            CollisionJudgmentModuleError(),  # COLLISION_JUDGMENT_MODULE_ERROR  # 衝突判定モジュールエラー
            CalibrationModuleError(),  # CALIBRATION_MODULE_ERROR  # 校正モジュールエラー
            ImuModuleError(),  # IMU_MODULE_ERROR  # IMUモジュール
            AppManagerModuleError(),  # APP_MANAGER_MODULE_ERROR  # アプリケーションマネージャーモジュールエラー
            MainModuleError(),  # MAIN_MODULE_ERROR  # Mainモジュールエラー
            PointsRefineModuleError(),  # POINTS_REFINE_MODULE_ERROR  # PointsRefineモジュールエラー
            VisualModuleError(),  # VISUAL_MODULE_ERROR  # VisualProcessモジュールエラー
            LidarShiftMonitorModuleError(),  # LIDAR_SHIFT_MONITOR_MODULE_ERROR  # LiDAR Shift Monitorモジュールエラー
            GetDataModuleError(),  # GET_DATA_MODULE_ERROR  # データ取得モジュールエラー
        )
        """
        モジュールエラーリスト
        """

        self.action_errors_A_C: tuple[
            ActionErrorDiagnosisA | ActionErrorDiagnosisB | ActionErrorDiagnosisC, ...
        ] = (
            LidarPositionMisalignmentDetectedDiagnosis(self.diagnosis_runtime_policy),  # LIDAR_POSITION_MISALIGNMENT_DETECTED  # LiDAR位置ズレ検出
            SensorCalibrationRequiredDiagnosis(self.diagnosis_runtime_policy),  # SENSOR_CALIBRATION_REQUIRED  # 要センサ校正
            ModelInfoMismatchDiagnosis(),  # MODEL_INFO_MISMATCH  # 機種情報不一致
            CraneModelFileMissingDiagnosis(),  # CRANE_MODEL_FILE_MISSING  # 機体モデルファイル欠損/破損
            ConfigFileMissingDiagnosis(),  # CONFIG_FILE_MISSING  # 設定ファイル欠損/破損
            SensorCalibDataInvalidDiagnosis(),  # SENSOR_CALIB_DATA_INVALID  # センサ校正データ不正
            CameraXCalibDataInvalidDiagnosis(),  # CAMERA0_CALIB_DATA_INVALID  # カメラ0校正データ不正
            CameraXCalibDataInvalidDiagnosis(),  # CAMERA1_CALIB_DATA_INVALID  # カメラ1校正データ不正
            CameraXCalibDataInvalidDiagnosis(),  # CAMERA2_CALIB_DATA_INVALID  # カメラ2校正データ不正
            CameraXCalibDataInvalidDiagnosis(),  # CAMERA3_CALIB_DATA_INVALID  # カメラ3校正データ不正
            MmapReadWriteErrorDiagnosis(),  # MMAP_READ_WRITE_ERROR  # MMAP read/writeエラー
            RebootLoopDetectedDiagnosis(self.diagnosis_runtime_policy),  # REBOOT_LOOP_DETECTED  # 再起動ループ検出
            AiModelLoadFailed(),  # AI_MODEL_LOAD_FAILED  # AIモデルロード失敗/破損
            OperationModeTransitionErrorDiagnosis(),  # OPERATION_MODE_TRANSITION_ERROR  # 動作モード遷移エラー
            LogFileIoErrorDiagnosis(),  # LOG_FILE_IO_ERROR  # ログファイルI/Oエラー
            ProcessStartupErrorDiagnosis(),  # PROCESS_STARTUP_ERROR  # プロセス起動エラー
        )
        """
        動作エラー(重要度A~C)検出クラスリスト
        """

        self.action_errors: tuple[Synchronized[int], ...] = tuple(
            diag.err_cnt for diag in self.action_errors_A_C
        )
        """
        動作エラーリスト
        """

        self.action_fail_safes: tuple[Synchronized[bool], ...] = tuple(
            diag.is_fail_safe for diag in self.action_errors_A_C
        )
        """
        動作エラー時動作リスト
        """

        self.state_idles: tuple[Synchronized[bool], ...] = tuple(
            diag.is_idle
            if isinstance(diag, StateErrorDiagnosisA)
            else create_shared_single_data(False)
            for diag in self.state_errors_A_C
        )

        self.action_idles: tuple[Synchronized[bool], ...] = tuple(
            diag.is_idle
            if isinstance(diag, ActionErrorDiagnosisA)
            else create_shared_single_data(False)
            for diag in self.action_errors_A_C
        )

    @property
    def is_idle_mode(self) -> bool:
        return any(tuple(chain(self.state_idles, self.action_idles)))

    def set_lidar_connected(self, lidar_index: int, is_connected: bool) -> None:
        self._lidars_connected[lidar_index].value = is_connected

    def set_camera_connected(self, camera_index: int, is_connected: bool) -> None:
        self._cameras_connected[camera_index].value = is_connected

    def get_lidars_connected(self) -> tuple[bool, ...]:
        return tuple(connected.value for connected in self._lidars_connected)

    def get_cameras_connected(self) -> tuple[bool, ...]:
        return tuple(connected.value for connected in self._cameras_connected)

    def is_state_error_d_exception(self, e: Exception, logger: AppLogger) -> bool:
        d_list: tuple[StateErrorDiagnosisD, ...] = self.state_errors_D_ex
        for diag in d_list:
            with suppress(Exception):
                if diag.excepts_diagnosis(e):
                    with suppress(Exception):
                        diag.log_output(
                            ResultDiagnosis.DETECTION, ResultDiagnosis.DETECTION, 0, e
                        )
                    return True
        return False

    def log_register(self, app_logger_factory: AppLoggerFactory) -> None:
        for diag in self.state_errors_A_C:
            diag.log_register(app_logger_factory)
        for diag in self.action_errors_A_C:
            diag.log_register(app_logger_factory)
        for diag in self.state_errors_D:
            diag.log_register(app_logger_factory)
        for diag in self.module_errors:
            diag.log_register(app_logger_factory)
        self.reduced_load_mode.log_register(app_logger_factory)


def diagnose_startup_calibration_data(
    shared_errors: SharedErrors,
    calibration_conf: "CalibrationConf",
    lidar2crane_reference_paths: list[str],
    camera_count: int,
    error_config: "ErrorConfig",
) -> None:
    sensor_diagnosis = shared_errors.action_errors_A_C[
        ActionErrorIndex.SENSOR_CALIB_DATA_INVALID
    ]
    if not isinstance(sensor_diagnosis, SensorCalibDataInvalidDiagnosis):
        raise TypeError("SENSOR_CALIB_DATA_INVALID diagnosis type mismatch")
    sensor_diagnosis.update(error_config)
    try:
        sensor_diagnosis.diagnose_calibration_matrices(
            calibration_conf,
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
            lidar2crane_reference_paths=lidar2crane_reference_paths,
        )
    except Exception as error:
        if not sensor_diagnosis.excepts_diagnosis(error):
            raise
        sensor_diagnosis.log_output(
            True,
            False,
            ActionErrorIndex.SENSOR_CALIB_DATA_INVALID,
            error,
        )

    camera_error_indices = (
        ActionErrorIndex.CAMERA0_CALIB_DATA_INVALID,
        ActionErrorIndex.CAMERA1_CALIB_DATA_INVALID,
        ActionErrorIndex.CAMERA2_CALIB_DATA_INVALID,
        ActionErrorIndex.CAMERA3_CALIB_DATA_INVALID,
    )
    enabled_camera_count = max(0, min(camera_count, len(camera_error_indices)))
    for camera_index, error_index in enumerate(
        camera_error_indices[:enabled_camera_count]
    ):
        diagnosis = shared_errors.action_errors_A_C[error_index]
        if not isinstance(diagnosis, CameraXCalibDataInvalidDiagnosis):
            raise TypeError(f"{error_index.name} diagnosis type mismatch")
        diagnosis.update(error_config)
        try:
            diagnosis.diagnose_calibration_data(
                camera_index,
                calibration_conf,
                error_index,
            )
        except Exception as error:
            if not diagnosis.excepts_diagnosis(error):
                raise
            diagnosis.log_output(
                True,
                False,
                error_index,
                error,
            )
