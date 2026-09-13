import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Self, TypeAlias

from argus_synchro.common.app_logger import AppLoggerFactory

ParamDict: TypeAlias = dict[str, int | float | str | bool]
TopParamDict: TypeAlias = dict[str, int | float | str | bool | ParamDict]


# NOTE: 静的解析で辞書型の構造への指摘が入るためにチェック関数を作ったが、これが成功しても指摘は消えない
def check_dictionary_structure(
    data: dict[Any, Any], expected_structure: dict[str, type]
) -> bool:
    """
    Check if the structure of the given dictionary matches the expected structure.

    Args:
        data: The dictionary to check.
        expected_structure: A dictionary specifying the expected keys and their types.

    Returns:
        True if the dictionary matches the expected structure, False otherwise.
    """
    for key, expected_type in expected_structure.items():
        if key not in data:
            return False
        if not isinstance(data[key], expected_type):
            return False
    return True


@dataclass(frozen=False, slots=True)
class ErrorParameterBase:
    """
    エラー診断パラメータのベースクラス

    【基本ルール】
    - エラー診断パラメータは、必ずこのクラスを継承して作成する。
    - 継承して作成したエラー診断パラメータは、int, float, str, bool型の属性を持つことができる。
    - このクラスを継承したサブパラメータクラスの属性を持っても良いが、load_from_dictメソッドを個別にオーバーライドすること。
      - ただし、サブパラメータクラスがそれ以上深くサブパラメータインスタンスを持つことは想定していない。
        必要性を感じた場合は、成立性や可読性を検討して設計しなおすこと。
    """

    @classmethod
    def load_from_dict(cls, data: TopParamDict) -> Self:
        self: Self = cls()
        for cls_field in cls.__dataclass_fields__:
            if cls_field in data:
                setattr(self, cls_field, data[cls_field])
        return self

    @classmethod
    def load_from_json(cls, json_path: Path) -> Self:
        with open(json_path, "r") as f:
            json_data = json.load(f)
            return cls.load_from_dict(json_data)

    is_enabled: bool = True
    """エラー診断有効フラグ"""


# 状態エラー
@dataclass(frozen=False, slots=True)
class LidarNConnectionErrorParameters(ErrorParameterBase):
    """LidarN接続エラー用パラメータ"""

    error_threshold_sec: float = 5.0
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0


@dataclass(frozen=False, slots=True)
class CameraNConnectionErrorParameters(ErrorParameterBase):
    """カメラN接続エラー用パラメータ"""

    error_threshold_sec: float = 5.0
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0


@dataclass(frozen=False, slots=True)
class CanConnectionErrorParameters(ErrorParameterBase):
    """CAN接続エラー用パラメータ"""

    error_threshold_sec: float = 5.0
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0


@dataclass(frozen=False, slots=True)
class LidarCommQualityDegradedParameters(ErrorParameterBase):
    """LidarN通信品質低下用パラメータ"""

    # udp_cnt欠番・ dot_num低下・ safety_info異常のうちどれか発生した場合のイベント番号
    dot_num_low_threshold: int = 50
    # この秒以内に品質低下イベントがあれば「現在低下中」と判定する
    recent_event_threshold_sec: float = 1.0
    # 低下中がこの秒継続で DETECTION (点滅防止)
    error_confirm_duration_sec: float = 3.0
    # 正常中がこの秒継続で RECOVERY
    recovery_confirm_duration_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class CameraCommQualityDegradedParameters(ErrorParameterBase):
    """カメラN通信品質低下用パラメータ"""

    error_threshold_count: int = 5
    error_threshold_sec: float = 1.0
    error_recovery_receive_interval_sec: float = 1.0
    fail_safe_recovery_receive_interval_sec: float = 1.0
    error_recovery_threshold_count: int = 5
    fail_safe_recovery_threshold_count: int = 5


@dataclass(frozen=False, slots=True)
class LidarCommQualityErrorParameters(ErrorParameterBase):
    """LidarN通信品質エラー用パラメータ"""

    # SE008 ON がこの秒継続で DETECTION
    error_confirm_duration_sec: float = 30.0
    # SE008 OFF がこの秒継続で RECOVERY
    recovery_confirm_duration_sec: float = 30.0
    failsafe_recovery_confirm_duration_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class CameraCommQualityErrorParameters(ErrorParameterBase):
    """カメラN通信品質エラー用パラメータ"""

    read_error_threshold_sec: float = 3.0
    read_error_threshold_count: int = 10
    read_error_rate_window_sec: float = 10.0
    read_error_rate_threshold: float = 0.3
    recovery_receive_interval_sec: float = 1.0
    fail_safe_recovery_receive_interval_sec: float = 1.0
    read_error_recovery_confirm_duration_sec: float = 3.0
    fail_safe_recovery_confirm_duration_sec: float = 3.0
    read_error_recovery_threshold_count: int = 10
    fail_safe_recovery_threshold_count: int = 10
    error_rate_recovery_confirm_duration_sec: float = 10.0
    fail_safe_rate_recovery_confirm_duration_sec: float = 10.0


@dataclass(frozen=False, slots=True)
class LidarInvalidDataParameters(ErrorParameterBase):
    """LidarNデータ不正用パラメータ"""

    # 原点点割合がこれ以上でエラー判定開始 (0.0〜1.0)
    invalid_ratio_threshold: float = 0.7
    # 上記割合がこの秒継続で DETECTION
    error_confirm_duration_sec: float = 3.0
    # 原点点割合がこれ未満で復帰判定開始 (0.0〜1.0)
    recovery_ratio_threshold: float = 0.3
    # 復帰条件継続確認時間 [s]
    recovery_confirm_duration_sec: float = 3.0
    failsafe_recovery_confirm_duration_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class CameraInvalidDataParameters(ErrorParameterBase):
    """カメラNデータ不正用パラメータ"""

    error_threshold_frames: int = 5
    error_recovery_frame_threshold: int = 5
    fail_safe_recovery_frame_threshold: int = 5


@dataclass(frozen=False, slots=True)
class YawAngleInfoErrorParameters(ErrorParameterBase):
    """旋回角情報エラー用パラメータ"""

    error_duration_sec: float = 5.0
    error_recovery_duration_sec: float = 5.0
    fail_safe_recovery_duration_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0
    fail_safe_recovery_receive_interval_sec: float = 1.0


@dataclass(frozen=False, slots=True)
class CanCommQualityDegradedParameters(ErrorParameterBase):
    """CAN通信品質低下用パラメータ"""

    angle_can_id: str = "18FFD1D1"
    lever_can_id: str = "18FC4401"
    error_threshold_count: dict[str, int] = field(
        default_factory=lambda: {"18FFD1D1": 5, "18FC4401": 5}
    )
    error_threshold_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 1.0, "18FC4401": 1.0}
    )
    recovery_receive_interval_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 1.0, "18FC4401": 1.0}
    )
    fail_safe_recovery_receive_interval_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 1.0, "18FC4401": 1.0}
    )
    recovery_threshold_count: dict[str, int] = field(
        default_factory=lambda: {"18FFD1D1": 5, "18FC4401": 5}
    )
    fail_safe_recovery_threshold_count: dict[str, int] = field(
        default_factory=lambda: {"18FFD1D1": 5, "18FC4401": 5}
    )


@dataclass(frozen=False, slots=True)
class CanCommQualityErrorParameters(ErrorParameterBase):
    """CAN通信品質エラー用パラメータ"""

    angle_can_id: str = "18FFD1D1"
    lever_can_id: str = "18FC4401"
    read_error_threshold_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 3.0, "18FC4401": 3.0}
    )
    read_error_threshold_count: dict[str, int] = field(
        default_factory=lambda: {"18FFD1D1": 10, "18FC4401": 10}
    )
    read_error_rate_window_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 10.0, "18FC4401": 10.0}
    )
    read_error_rate_threshold: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 0.3, "18FC4401": 0.3}
    )
    recovery_receive_interval_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 1.0, "18FC4401": 1.0}
    )
    fail_safe_recovery_receive_interval_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 1.0, "18FC4401": 1.0}
    )
    read_error_recovery_confirm_duration_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 3.0, "18FC4401": 3.0}
    )
    fail_safe_recovery_confirm_duration_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 3.0, "18FC4401": 3.0}
    )
    read_error_recovery_threshold_count: dict[str, int] = field(
        default_factory=lambda: {"18FFD1D1": 10, "18FC4401": 10}
    )
    fail_safe_recovery_threshold_count: dict[str, int] = field(
        default_factory=lambda: {"18FFD1D1": 10, "18FC4401": 10}
    )
    error_rate_recovery_confirm_duration_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 10.0, "18FC4401": 10.0}
    )
    fail_safe_rate_recovery_confirm_duration_sec: dict[str, float] = field(
        default_factory=lambda: {"18FFD1D1": 10.0, "18FC4401": 10.0}
    )


@dataclass(frozen=False, slots=True)
class CanInvalidDataParametersByCanId(ErrorParameterBase):
    """CANデータ不正のCANIDごとのパラメータ"""

    required_length: int = 4
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class CanInvalidDataParameters(ErrorParameterBase):
    """CANデータ不正用パラメータ"""

    # NOTE: 辞書のキーにCANIDを指定することで、CANIDが増減した場合も対応可能
    params_by_canid: dict[str, CanInvalidDataParametersByCanId] = field(
        default_factory=lambda: {
            "18FFD1D1": CanInvalidDataParametersByCanId(
                is_enabled=True,
                required_length=4,
                error_recovery_confirm_duration_sec=5.0,
                failsafe_recovery_confirm_duration_sec=5.0,
            ),
            "18FC4401": CanInvalidDataParametersByCanId(
                is_enabled=True,
                required_length=18,
                error_recovery_confirm_duration_sec=5.0,
                failsafe_recovery_confirm_duration_sec=5.0,
            ),
        }
    )

    @classmethod
    def load_from_dict(cls, data: TopParamDict) -> Self:
        self: Self = cls()
        for cls_field in cls.__dataclass_fields__:
            if cls_field in data:
                param_dict = data[cls_field]
                if cls_field == "params_by_canid" and isinstance(param_dict, dict):
                    # If the field is params_by_canid, load each CanInvalidDataParametersByCanId from the dictionary
                    setattr(self, cls_field, self.__build_params_by_canid(param_dict))
                else:
                    setattr(self, cls_field, data[cls_field])

        return self

    @classmethod
    def __build_params_by_canid(
        cls, param_dict: ParamDict
    ) -> dict[str, CanInvalidDataParametersByCanId]:
        params_by_canid: dict[str, CanInvalidDataParametersByCanId] = {}
        for canid, params in param_dict.items():
            if isinstance(params, dict):
                params_by_canid[canid] = CanInvalidDataParametersByCanId.load_from_dict(
                    params
                )
        return params_by_canid


@dataclass(frozen=False, slots=True)
class StorageSpaceLowParameters(ErrorParameterBase):
    """ストレージ残容量低下用パラメータ"""

    error_threshold_gb: float = 10.0
    error_recovery_threshold_gb: float = 10.0
    fail_safe_recovery_threshold_gb: float = 10.0
    error_duration_sec: float = 5.0
    error_recovery_duration_sec: float = 5.0
    fail_safe_recovery_duration_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class ProcessingSpeedDegradedParameters(ErrorParameterBase):
    """処理速度低下用パラメータ"""


@dataclass(frozen=False, slots=True)
class ProcessingSpeedDegradationTrendParameters(ErrorParameterBase):
    """処理速度低下傾向用パラメータ"""


@dataclass(frozen=False, slots=True)
class OutOfMemoryParameters(ErrorParameterBase):
    """メモリ不足用パラメータ"""

    threshold_mb: float = 10.0 * 1024
    error_duration_sec: float = 3.0
    error_recovery_duration_sec: float = 3.0
    fail_safe_recovery_duration_sec: float = 3.0


@dataclass(frozen=False, slots=True)
class GpuPerformanceDegradedParameters(ErrorParameterBase):
    """GPU性能低下用パラメータ"""

    error_duration_sec: float = 5.0
    error_recovery_duration_sec: float = 5.0
    fail_safe_recovery_duration_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class MonitorProcessNotRespondingParameters(ErrorParameterBase):
    """モニタープロセス未応答用パラメータ"""

    error_threshold_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class StatusInfoNotUpdatedParameters(ErrorParameterBase):
    """ステータス情報 未更新用パラメータ"""


@dataclass(frozen=False, slots=True)
class SurroundMonitorModuleNotRespondingParameters(ErrorParameterBase):
    """周辺監視モジュール 未応答用パラメータ"""

    error_threshold_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class LidarPositionMisalignmentNotRespondingParameters(ErrorParameterBase):
    """Lidar位置ズレ検出 未応答用パラメータ"""

    error_threshold_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class ApplicationManagerNotRespondingParameters(ErrorParameterBase):
    """アプリケーションマネージャ 未応答用パラメータ"""

    error_threshold_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class ImuNConnectionErrorParameters(ErrorParameterBase):
    """IMUN接続エラー用パラメータ"""

    error_threshold_sec: float = 5.0
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0


@dataclass(frozen=False, slots=True)
class LogOutputStoppedParameters(ErrorParameterBase):
    """ログ出力停止用パラメータ"""

    error_threshold_sec: float = 5.0
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0


@dataclass(frozen=False, slots=True)
class InternalTemperatureRiseParameters(ErrorParameterBase):
    """内部温度上昇用パラメータ"""

    least_sample_count: int = 5
    moving_avg_window_sec: float = 5.0
    error_threshold_degree: float = 60.0


@dataclass(frozen=False, slots=True)
class TemperatureSensorAbnormalParameters(ErrorParameterBase):
    """温度センサー異常用パラメータ"""

    error_threshold_sec: float = 5.0
    error_recovery_duration_sec: float = 5.0
    fail_safe_recovery_duration_sec: float = 5.0


@dataclass(frozen=False, slots=True)
class TemperatureRiseTrendContinuesParameters(ErrorParameterBase):
    """温度上昇傾向の継続用パラメータ"""

    least_sample_count: int = 5
    moving_avg_window_sec: float = 5.0
    error_threshold_degree: float = 70.0


@dataclass(frozen=False, slots=True)
class CalibHumanDetectionFailureParameters(ErrorParameterBase):
    """Skeleton for CalibHumanDetectionFailureDiagnosis."""


@dataclass(frozen=False, slots=True)
class CalibHumanTrackingFailureParameters(ErrorParameterBase):
    """Skeleton for CalibHumanTrackingFailureDiagnosis."""


# 動作エラー
@dataclass(frozen=False, slots=True)
class LidarPositionMisalignmentDetectedParameters(ErrorParameterBase):
    """LIDAR位置ずれ検出用パラメータ"""


@dataclass(frozen=False, slots=True)
class SensorCalibrationRequiredParameters(ErrorParameterBase):
    """要センサ校正用パラメータ"""


@dataclass(frozen=False, slots=True)
class ModelInfoMismatchParameters(ErrorParameterBase):
    """機種情報不一致用パラメータ"""


@dataclass(frozen=False, slots=True)
class CraneModelFileMissingParameters(ErrorParameterBase):
    """機体モデルファイル欠損/破損用パラメータ"""


@dataclass(frozen=False, slots=True)
class ConfigFileMissingParameters(ErrorParameterBase):
    """設定ファイル欠損/破損用パラメータ"""


@dataclass(frozen=False, slots=True)
class SensorCalibDataInvalidParameters(ErrorParameterBase):
    """センサ校正データ不正用パラメータ"""

    # 対象マトリクス
    # LiDAR間の統合行列を検査する。
    check_lidar2lidar: bool = False
    # LiDAR-機体座標の行列を検査する。
    check_lidar2crane: bool = True

    # 基本健全性チェック (枠組み段階で有効)
    # 行列形状を固定値で検査する。
    enforce_shape_4x4: bool = True
    # NaN/inf を不正値として扱う。
    finite_value_only: bool = True

    # 標準データとの差分判定を有効化する。
    enable_reference_diff_check: bool = True

    # 参照行列との差分判定パラメータ
    # 並進誤差ノルム[m]の許容上限。
    # 機能としては実装したが大きな値に設定して、実質的に無効化している。
    # translation_threshold_m: float = 0.5
    translation_threshold_m: float = 99.9

    # 回転誤差[deg]の許容上限。
    # 機能としては実装したが大きな値に設定して、実質的に無効化している。
    # rotation_threshold_deg: float = 10.0
    rotation_threshold_deg: float = 99.9

    # 評価グリッド上の最大XY変位[m]の許容上限。
    # translationとrotationズレを無効化したが、ここのxy方向ズレ量で判定する。
    # xyズレと並進ズレの両方がここのxyズレに乗るので、実質的にこの値で判定するば良いことになる。
    # max_xy_displacement_threshold_m: float = 1.0
    max_xy_displacement_threshold_m: float = 1.5
    # XY評価グリッド半径[m]。
    grid_radius_m: float = 5.0
    # XY評価グリッド間隔[m]。
    grid_spacing_m: float = 1.0
    # 閾値比較時の絶対許容差。
    comparison_atol: float = 1e-12


@dataclass(frozen=False, slots=True)
class CameraNCalibDataInvalidParameters(ErrorParameterBase):
    """カメラN校正データ不正用パラメータ"""

    # 外部パラメータ行列の形状を検査する。
    enforce_shape_4x4: bool = True
    finite_value_only: bool = True
    # fisheye内部パラメータの必須項目を検査する。
    check_fisheye_intrinsics: bool = True

    # 標準データ差分判定 (アルゴリズム実装時に利用)
    # 標準データとの差分判定を有効化する。
    enable_reference_diff_check: bool = False
    # 比較対象の標準データパス。
    reference_data_path: str = ""
    # 差分判定のしきい値。
    diff_threshold: float = 0.0


@dataclass(frozen=False, slots=True)
class MmapReadWriteErrorParameters(ErrorParameterBase):
    """MMAP read/writeエラー用パラメータ"""


@dataclass(frozen=False, slots=True)
class RebootLoopDetectedParameters(ErrorParameterBase):
    """再起動ループ検出用パラメータ"""

    uptime_state_path: str = "/var/lib/argus3d/uptime/uptime_state.json"
    required_boot_count: int = 5
    window_sec: float = 600.0


@dataclass(frozen=False, slots=True)
class AiModelLoadFailedParameters(ErrorParameterBase):
    """AIモデルロード失敗/破損用パラメータ"""


@dataclass(frozen=False, slots=True)
class OperationModeTransitionErrorParameters(ErrorParameterBase):
    """動作モード遷移エラー用パラメータ"""


@dataclass(frozen=False, slots=True)
class LogFileIoErrorParameters(ErrorParameterBase):
    """ログファイルI/Oエラー用パラメータ"""


# 重要度Dエラー
@dataclass(frozen=False, slots=True)
class MonitorConnectionErrorParameters(ErrorParameterBase):
    """モニタ接続エラー用パラメータ"""


@dataclass(frozen=False, slots=True)
class LidarDataMissingParameters(ErrorParameterBase):
    """Lidarデータ欠落用パラメータ"""

    min_point_count: int = 100


@dataclass(frozen=False, slots=True)
class CameraDataMissingParameters(ErrorParameterBase):
    """カメラデータ欠落用パラメータ"""

    black_frame_rate_window_sec: float = 3.0
    black_frame_rate_threshold: float = 0.2


@dataclass(frozen=False, slots=True)
class MemoryLeakDetectedParameters(ErrorParameterBase):
    """メモリリーク検出用パラメータ"""

    window_sec: float = 300.0  # 診断間隔
    min_samples: int = 300  # 診断に必要な最小サンプル数
    leak_ratio_threshold: float = 1.5  # リーク判定の増加率の閾値


@dataclass(frozen=False, slots=True)
class OtherHardwareErrorParameters(ErrorParameterBase):
    """その他のエラー(ハードウェア)用パラメータ"""

    """
    以下から診断対象にするもののみhardware_typesに追加する。
        "TOUCHSCREEN": "ID_INPUT_TOUCHSCREEN=1",
        "KEYBOARD": "ID_INPUT_KEYBOARD=1",
        "MOUSE": "ID_INPUT_MOUSE=1",
        "SPATIAL_CONTROLLER": "SPATIAL_CONTROLLER_IS_NO_ID",
    """

    hardware_types: dict[str, str] = field(
        default_factory=lambda: {
            "TOUCHSCREEN": "ID_INPUT_TOUCHSCREEN=1",
            "KEYBOARD": "ID_INPUT_KEYBOARD=1",
            "MOUSE": "ID_INPUT_MOUSE=1",
            "SPATIAL_CONTROLLER": "SPATIAL_CONTROLLER_IS_NO_ID",
        },
    )


@dataclass(frozen=False, slots=True)
class InvalidDataInputParameters(ErrorParameterBase):
    """不正データ入力(空データなど)用パラメータ"""


@dataclass(frozen=False, slots=True)
class NumericAnomalyExceptionParameters(ErrorParameterBase):
    """不正演算(NaN/ゼロ除算等)用パラメータ"""


@dataclass(frozen=False, slots=True)
class ArrayShapeErrorParameters(ErrorParameterBase):
    """配列形状エラー用パラメータ"""

    expected_shape_by_array_name: dict[str, tuple[int, ...]] = field(
        default_factory=lambda: {
            # -1 は任意の長さを許容する
            "pcds_point_cloud": (-1, 3),
            "lever_pressure": (4,),
            "images": (-1, -1, 3),
            "frames_buf": (-1, -1, -1, 3),
            "imu_values": (-1, 6),
            "removed_pcds_point_cloud": (-1, 3),
            "accum_ground_pcd_point_cloud": (-1, 3),
            "accum_pcds_point_cloud": (-1, 3),
            "edge_points": (-1, 3),
            "edge_lines": (-1, 2),
            "edge_length": (-1,),
            "camera_detections_boxes": (3, -1, 4),
            "camera_detections_scores": (3, -1),
            "camera_detections_classes": (3, -1),
            "camera_detections_valid_detects": (3,),
            "camera_detections_image": (-1, -1, -1, 3),
            "visual_d3_boxes": (-1, 3),
            "visual_2d3d_boxes": (-1, 3),
            "minmax": (-1, 6),
            "valid_detects": (1,),
        }
    )


@dataclass(frozen=False, slots=True)
class FileIoErrorParameters(ErrorParameterBase):
    """ファイルI/Oエラー用パラメータ"""


@dataclass(frozen=False, slots=True)
class AiInferenceResultErrorParameters(ErrorParameterBase):
    """AI推論結果異常用パラメータ"""

    score_min: float = 0.0
    score_max: float = 1.0


@dataclass(frozen=False, slots=True)
class DetectionTargetErrorParameters(ErrorParameterBase):
    """検知対象エラー用パラメータ"""


@dataclass(frozen=False, slots=True)
class ConsecutiveRetryLimitExceededParameters(ErrorParameterBase):
    """連続リトライ上限超過用パラメータ"""


@dataclass(frozen=False, slots=True)
class ProcessForcedTerminationParameters(ErrorParameterBase):
    """プロセス強制終了用パラメータ"""


@dataclass(frozen=False, slots=True)
class LogCompressionFailureParameters(ErrorParameterBase):
    """ログ圧縮失敗用パラメータ"""


@dataclass(frozen=False, slots=True)
class LogTimeReversalParameters(ErrorParameterBase):
    """ログ時刻逆転用パラメータ"""

    allowed_backward_sec: float = 0.0


@dataclass(frozen=False, slots=True)
class LidarModuleErrorParameters(ErrorParameterBase):
    """LiDARモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class CameraModuleErrorParameters(ErrorParameterBase):
    """カメラモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class StorageModuleErrorParameters(ErrorParameterBase):
    """蓄積モジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class CanModuleErrorParameters(ErrorParameterBase):
    """CANモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class Linkage2D3DModuleErrorParameters(ErrorParameterBase):
    """2D-3D紐づけモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class Object3DDetectionModuleErrorParameters(ErrorParameterBase):
    """3D物体検知モジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class CameraHumanDetectionModuleErrorParameters(ErrorParameterBase):
    """カメラ人検知モジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class CollisionJudgmentModuleErrorParameters(ErrorParameterBase):
    """衝突判定モジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class CalibrationModuleErrorParameters(ErrorParameterBase):
    """校正モジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class ImuModuleErrorParameters(ErrorParameterBase):
    """IMUモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class AppManagerModuleErrorParameters(ErrorParameterBase):
    """アプリケーションマネージャーモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class MainModuleErrorParameters(ErrorParameterBase):
    """Mainモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class PointsRefineModuleErrorParameters(ErrorParameterBase):
    """PointsRefineモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class VisualModuleErrorParameters(ErrorParameterBase):
    """VisualProcessモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class LidarShiftMonitorModuleErrorParameters(ErrorParameterBase):
    """LiDARシフトモニタモジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


@dataclass(frozen=False, slots=True)
class GetDataModuleErrorParameters(ErrorParameterBase):
    """データ取得モジュールエラー用パラメータ"""

    ongoing_log_interval_sec: float = 60.0


class ErrorConfig:
    _logger = AppLoggerFactory.from_name("ErrorConfig")

    def __init__(self) -> None:
        # 状態エラー
        self.lidar_n_connection_error = LidarNConnectionErrorParameters()
        """LidarN接続エラー用パラメータ"""
        self.camera_n_connection_error = CameraNConnectionErrorParameters()
        """CameraN接続エラー用パラメータ"""
        self.can_connection_error = CanConnectionErrorParameters()
        """CAN接続エラー用パラメータ"""
        self.lidar_n_comm_quality_degraded = LidarCommQualityDegradedParameters()
        """LidarN通信品質低下用パラメータ"""
        self.camera_n_comm_quality_degraded = CameraCommQualityDegradedParameters()
        """CameraN通信品質低下用パラメータ"""
        self.lidar_n_comm_quality_error = LidarCommQualityErrorParameters()
        """LidarN通信品質エラー用パラメータ"""
        self.camera_n_comm_quality_error = CameraCommQualityErrorParameters()
        """CameraN通信品質エラー用パラメータ"""
        self.lidar_n_invalid_data = LidarInvalidDataParameters()
        """LidarNデータ不正用パラメータ"""
        self.camera_n_invalid_data = CameraInvalidDataParameters()
        """CameraNデータ不正用パラメータ"""
        self.yaw_angle_info_error = YawAngleInfoErrorParameters()
        """旋回角情報エラー用パラメータ"""
        self.can_comm_quality_degraded = CanCommQualityDegradedParameters()
        """CAN通信品質低下用パラメータ"""
        self.can_comm_quality_error = CanCommQualityErrorParameters()
        """CAN通信品質エラー用パラメータ"""
        self.can_invalid_data = CanInvalidDataParameters()
        """CANデータ不正用パラメータ"""
        self.storage_space_low = StorageSpaceLowParameters()
        """ストレージ容量低下用パラメータ"""
        self.processing_speed_degraded = ProcessingSpeedDegradedParameters()
        """処理速度低下用パラメータ"""
        self.processing_speed_degradation_trend = (
            ProcessingSpeedDegradationTrendParameters()
        )
        """処理速度低下傾向用パラメータ"""
        self.out_of_memory = OutOfMemoryParameters()
        """メモリ不足用パラメータ"""
        self.gpu_performance_degraded = GpuPerformanceDegradedParameters()
        """GPU性能低下用パラメータ"""
        self.monitor_process_not_responding = MonitorProcessNotRespondingParameters()
        """モニタープロセス未応答用パラメータ"""
        self.status_info_not_updated = StatusInfoNotUpdatedParameters()
        """ステータス情報未更新用パラメータ"""
        self.surround_monitor_module_not_responding = (
            SurroundMonitorModuleNotRespondingParameters()
        )
        """周辺監視モジュール 未応答用パラメータ"""
        self.lidar_position_misalignment_not_responding = (
            LidarPositionMisalignmentNotRespondingParameters()
        )
        """Lidar位置ズレ検出 未応答用パラメータ"""
        self.application_manager_not_responding = (
            ApplicationManagerNotRespondingParameters()
        )
        """アプリケーションマネージャ 未応答用パラメータ"""
        self.imu_n_connection_error = ImuNConnectionErrorParameters()
        """IMUN接続エラー用パラメータ"""
        self.log_output_stopped = LogOutputStoppedParameters()
        """ログ出力停止用パラメータ"""
        self.internal_temperature_rise = InternalTemperatureRiseParameters()
        """内部温度上昇用パラメータ"""
        self.temperature_sensor_abnormal = TemperatureSensorAbnormalParameters()
        """温度センサー異常用パラメータ"""
        self.temperature_rise_trend_continues = (
            TemperatureRiseTrendContinuesParameters()
        )
        """温度上昇傾向の継続用パラメータ"""

        # 動作エラー
        self.lidar_position_misalignment_detected = (
            LidarPositionMisalignmentDetectedParameters()
        )
        """LIDAR位置ずれ検出用パラメータ"""
        self.sensor_calibration_required = SensorCalibrationRequiredParameters()
        """要センサ校正用パラメータ"""
        self.model_info_mismatch = ModelInfoMismatchParameters()
        """機種情報不一致用パラメータ"""
        self.crane_model_file_missing = CraneModelFileMissingParameters()
        """機体モデルファイル欠損/破損用パラメータ"""
        self.config_file_missing = ConfigFileMissingParameters()
        """設定ファイル欠損/破損用パラメータ"""
        self.sensor_calib_data_invalid = SensorCalibDataInvalidParameters()
        """センサ校正データ不正用パラメータ"""
        self.camera_n_calib_data_invalid = CameraNCalibDataInvalidParameters()
        """カメラN校正データ不正用パラメータ"""
        self.mmap_read_write_error = MmapReadWriteErrorParameters()
        """MMAP read/writeエラー用パラメータ"""
        self.reboot_loop_detected = RebootLoopDetectedParameters()
        """再起動ループ検出用パラメータ"""
        self.ai_model_load_failed = AiModelLoadFailedParameters()
        """AIモデルロード失敗/破損用パラメータ"""
        self.operation_mode_transition_error = OperationModeTransitionErrorParameters()
        """動作モード遷移エラー用パラメータ"""
        self.log_file_io_error = LogFileIoErrorParameters()
        """ログファイルI/Oエラー用パラメータ"""

        # 重要度Dエラー
        self.monitor_connection_error = MonitorConnectionErrorParameters()
        """モニタ接続エラー用パラメータ"""
        self.lidar_data_missing = LidarDataMissingParameters()
        """LiDARデータ欠落用パラメータ"""
        self.camera_data_missing = CameraDataMissingParameters()
        """カメラデータ欠落用パラメータ"""
        self.memory_leak_detected = MemoryLeakDetectedParameters()
        """メモリリーク検出用パラメータ"""
        self.other_hardware_error = OtherHardwareErrorParameters()
        """その他のエラー(ハードウェア)用パラメータ"""
        self.invalid_data_input = InvalidDataInputParameters()
        """不正データ入力(空データなど)用パラメータ"""
        self.numeric_anomaly_exception = NumericAnomalyExceptionParameters()
        """不正演算(NaN/ゼロ除算等)用パラメータ"""
        self.array_shape_error = ArrayShapeErrorParameters()
        """配列形状エラー用パラメータ"""
        self.file_io_error = FileIoErrorParameters()
        """ファイルI/Oエラー用パラメータ"""
        self.ai_inference_result_error = AiInferenceResultErrorParameters()
        """AI推論結果異常用パラメータ"""
        self.detection_target_error = DetectionTargetErrorParameters()
        """検出対象エラー用パラメータ"""
        self.consecutive_retry_limit_exceeded = (
            ConsecutiveRetryLimitExceededParameters()
        )
        """連続リトライ上限超過用パラメータ"""
        self.process_forced_termination = ProcessForcedTerminationParameters()
        """プロセス強制終了用パラメータ"""
        self.log_compression_failure = LogCompressionFailureParameters()
        """ログ圧縮失敗用パラメータ"""
        self.log_time_reversal = LogTimeReversalParameters()
        """ログ時刻逆転用パラメータ"""
        self.lidar_module_error = LidarModuleErrorParameters()
        """LiDARモジュールエラー用パラメータ"""
        self.camera_module_error = CameraModuleErrorParameters()
        """カメラモジュールエラー用パラメータ"""
        self.storage_module_error = StorageModuleErrorParameters()
        """ストレージモジュールエラー用パラメータ"""
        self.can_module_error = CanModuleErrorParameters()
        """CANモジュールエラー用パラメータ"""
        self.linkage_2d3d_module_error = Linkage2D3DModuleErrorParameters()
        """2D-3D紐づけモジュールエラー用パラメータ"""
        self.object3_d_detection_module_error = Object3DDetectionModuleErrorParameters()
        """3D物体検知モジュールエラー用パラメータ"""
        self.camera_human_detection_module_error = (
            CameraHumanDetectionModuleErrorParameters()
        )
        """カメラ人検知モジュールエラー用パラメータ"""
        self.collision_judgment_module_error = CollisionJudgmentModuleErrorParameters()
        """衝突判定モジュールエラー用パラメータ"""
        self.calibration_module_error = CalibrationModuleErrorParameters()
        """校正モジュールエラー用パラメータ"""
        self.imu_module_error = ImuModuleErrorParameters()
        """IMUモジュールエラー用パラメータ"""
        self.app_manager_module_error = AppManagerModuleErrorParameters()
        """アプリケーションマネージャーモジュールエラー用パラメータ"""
        self.main_module_error = MainModuleErrorParameters()
        """Mainモジュールエラー用パラメータ"""
        self.points_refine_module_error = PointsRefineModuleErrorParameters()
        """PointsRefineモジュールエラー用パラメータ"""
        self.visual_module_error = VisualModuleErrorParameters()
        """VisualProcessモジュールエラー用パラメータ"""
        self.lidar_shift_monitor_module_error = (
            LidarShiftMonitorModuleErrorParameters()
        )
        """LiDARシフトモニタモジュールエラー用パラメータ"""
        self.get_data_module_error = GetDataModuleErrorParameters()
        """データ取得モジュールエラー用パラメータ"""

    def load_from_dict(self, data: TopParamDict) -> Self:
        """
        辞書からErrorConfigの属性をロードするメソッド。
        基本的に、辞書はjson.load()で読み込んだ形式を想定している。

        【基本ルール】
        - エラーとインスタンス変数は1対1で対応する。
          例えばカメラ0とカメラ1で同じエラーがある場合、インスタンス変数はcamera0_connection_errorとcamera1_connection_errorのように分ける。
        - 辞書のキーがErrorConfigのインスタンス変数名と一致する場合、その属性を更新する。
        - インスタンス変数を追加する場合は、ErrorConfigクラスの__init__メソッドに新しい属性を追加する。
        - インスタンス変数は基本的にそのエラーに関するパラメータを直接持つ。CANIDなどの可変長のパラメータは、辞書型でまとめて持つ。
            - 設定が固定長になる場合は、配列ではなく個々に属性名を付与する
            - 設定が可変長になる場合、リスト型やタプル型は使わず辞書型を使用する。
        - 現時点では属性名が持つ辞書型は、さらに辞書型を持たない。持たせる必要がある場合はクラス設計を要検討。
        Args:
            data (dict): 辞書形式のエラー設定データ。
        Returns:
            Self: 更新されたErrorConfigインスタンス。
        """
        for dict_key, dict_value in data.items():
            if hasattr(self, dict_key):
                attr = getattr(self, dict_key)
                if isinstance(attr, ErrorParameterBase) and isinstance(
                    dict_value, dict
                ):
                    # If the attribute is an instance of ErrorParameterBase and the value is a dictionary,
                    # load the parameters from the dictionary.
                    self._logger.debug(
                        f"Loading parameters for {dict_key} from dict: {dict_value}"
                    )
                    loaded_attr = attr.load_from_dict(dict_value)
                    setattr(self, dict_key, loaded_attr)
                else:
                    # Otherwise, set the attribute directly.
                    setattr(self, dict_key, dict_value)
            else:
                self._logger.warning(f"Unknown parameter key: {dict_key}")
        return self

    def load_from_json(self, json_path: Path) -> Self:
        with open(json_path, "r") as f:
            json_data = json.load(f)
            return self.load_from_dict(json_data)


if __name__ == "__main__":
    # Example usage
    _logger = AppLoggerFactory.from_name("TestErrorConfig", level=20)
    test_json = {
        "error_code": 1,
        "error_message": "Camera connection lost",
        "camera_n_connection_error": {
            "error_threshold_sec": 10.0,
            "error_recovery_confirm_duration_sec": 15.0,
            "failsafe_recovery_confirm_duration_sec": 20.0,
            # "recovery_receive_interval_sec": 2.0,
            "いらない項目": 123,
            "is_enabled": False,
        },
        "camera1_connection_error": {
            "error_threshold_sec": 10.0,
            "error_recovery_confirm_duration_sec": 15.0,
            "failsafe_recovery_confirm_duration_sec": 20.0,
            "recovery_receive_interval_sec": 2.0,
            "is_enabled": True,
        },
        "can_invalid_data": {
            "is_enabled": False,
            "params_by_canid": {
                "18FFD1D1": {
                    "required_length": 40,
                    "error_recovery_confirm_duration_sec": 50.0,
                    "failsafe_recovery_confirm_duration_sec": 50.0,
                },
                "18FC4401": {
                    "required_length": 180,
                    "error_recovery_confirm_duration_sec": 55.0,
                    "failsafe_recovery_confirm_duration_sec": 55.0,
                },
                "1": {
                    "required_lengt": 1,
                    "error_recovery_confirm_duration_sec!!!": 1.0,
                    "failsafe_recovery_confirm_duration_sec": 1.0,
                },
            },
        },
    }
    json.dump(test_json, open("test_err_param.json", "w"), indent=4, ensure_ascii=False)

    error_params = ErrorConfig()
    # _logger.info(f"{error_params.camera_n_connection_error.is_enabled=}")
    # error_params.load_from_dict(test_json)
    error_params.load_from_json(Path("./config/error_config.json"))
    _logger.info(f"{error_params=}")
    camera_n_params = error_params.camera_n_connection_error
    _logger.info(f"{camera_n_params=}")
    _logger.info(f"{error_params.can_invalid_data.is_enabled=}")
    params_by_canid = error_params.can_invalid_data.params_by_canid
    _logger.info(f"{params_by_canid['18FFD1D1']=}")
    _logger.info(f"{params_by_canid['18FC4401'].required_length=}")
    _logger.info(f"{error_params.array_shape_error.expected_shape_by_array_name=}")
    try:
        _logger.info(f"{params_by_canid['2']=}")
    except KeyError as e:
        _logger.error(f"KeyError: params_by_canid[{e}]")
