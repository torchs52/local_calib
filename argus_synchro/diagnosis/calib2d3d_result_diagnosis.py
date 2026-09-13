"""2D-3D校正モード用のerrors_calibcommon定数と診断スタブ。

このモジュールで扱うエラーはargus_synchro/diagnosis以下の重要度A~Dシステムエラーとは別枠。
校正UI(errors_calibcommon)へ状況を伝えるための専用コードであり、致命的なシステムエラーではない。
"""

from __future__ import annotations

from enum import IntEnum


class Calib2d3dErrorCommon(IntEnum):
    """2D-3D校正のerrors_calibcommonに設定する値。必ずいずれか1つを設定する。"""
    # 0-15は警告メッセージは出すが、校正処理の継続は可能.
    DEFAULT = 0  # デフォルト
    CALIBRATION_SUCCEEDED = 1  # 校正成功
    WALKING_RANGE_INVALID = 2  # 歩行範囲が不適切
    WALKING_PERSON_COUNT_INVALID = 3  # 歩行人数が不適切
    POOR_PERSON_DETECTION = 4  # 人検知不良
    POOR_TRACKING_3D = 5  # 3D側の追跡不具合
    POOR_TRACKING_2D = 6  # 2D側の追跡不具合
    UNSUITABLE_CONDITION = 7  # 撮影条件が校正に不適(周囲が暗いなど)
    RESERVED_8 = 8
    RESERVED_9 = 9
    RESERVED_10 = 10
    RESERVED_11 = 11
    RESERVED_12 = 12
    RESERVED_13 = 13
    RESERVED_14 = 14
    RESERVED_15 = 15
    # 16-31は校正の継続が不可能な状態. 一旦UI側で校正のエラー完了状態に導く.
    LONG_DURATION_CALIBRATION = 16  # 長時間の校正作業
    PERSON_DETECTION_IMPOSSIBLE = 17  # 人検知不能
    TRACKING_IMPOSSIBLE = 18  # 追跡不能
    RESERVED_19 = 19
    RESERVED_20 = 20
    RESERVED_21 = 21
    RESERVED_22 = 22
    RESERVED_23 = 23
    RESERVED_24 = 24
    RESERVED_25 = 25
    RESERVED_26 = 26
    RESERVED_27 = 27
    RESERVED_28 = 28
    RESERVED_29 = 29
    RESERVED_30 = 30
    RESERVED_31 = 31


class Calib2d3dResultDiagnosis:
    """
    2D-3D校正の状況診断(枠組み先行、現状は常にDEFAULTを返す)。
    """

    def diagnose(self, *args: object) -> Calib2d3dErrorCommon:
        # TODO: 歩行範囲/人数/検知/追跡/撮影条件の各判定を実装する。
        return Calib2d3dErrorCommon.DEFAULT


class CameraCalibrationStatus(IntEnum):
    """2D-3D校正のset_camera_calibration_statusに設定する値。必ずいずれか1つを設定する。"""

    DEFAULT = 0  # デフォルト(計算結果が出る前)
    CALIBRATION_SUCCEEDED = 1  # 校正成功
    POOR_SENSOR_CORRESPONDENCE = 2  # 複数センサの対応付け不良
    WALKING_PERSON_COUNT_INVALID = 3  # 歩行人数が不適切
    POOR_TRACKING_3D = 4  # 3D側の追跡不具合
    POOR_TRACKING_2D = 5  # 2D側の追跡不具合
    CALIBRATION_MATRIX_INVALID = 6  # 校正マトリクスデータ不正
    RESERVED_7 = 7


class CameraCalibrationStatusDiagnosis:
    """
    カメラ単位の校正結果診断(枠組み先行、現状は常にDEFAULTを返す)。

    校正マトリクスデータ不正の判定は、将来的にaction_errors.pyの
    Camera0CalibDataInvalidDiagnosis.validate_calibration_dataと同様の検証を組み込む予定。
    """

    def diagnose(self, *args: object) -> CameraCalibrationStatus:
        # TODO: センサ対応付け/歩行人数/2D・3D追跡/校正マトリクス妥当性の各判定を実装する。
        # 一カ所で一気に判定するというよりは、処理の途中の何カ所かで適切な診断メソッドを呼ぶ実装かも.
        return CameraCalibrationStatus.DEFAULT
