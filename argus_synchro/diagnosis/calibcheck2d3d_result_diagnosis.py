"""2D-3D校正要否チェックのUI通知用ステータスと診断。"""

from __future__ import annotations

from enum import IntEnum
from typing import ClassVar


class CameraCalibCheckStatus(IntEnum):
    """set_camera_calibcheck_statusに設定する値。"""

    CALIBRATION_NOT_REQUIRED = 0
    FORBIDDEN = 1
    CALIBRATION_REQUIRED = 2
    UNKNOWN_INSUFFICIENT_DATA = 3
    UNKNOWN_PERSON_NOT_DETECTED = 4
    UNKNOWN_POOR_PERSON_DETECTION = 5
    RESERVED_6 = 6
    RESERVED_7 = 7


_WRITABLE_STATUSES = frozenset(
    {
        CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED,
        CameraCalibCheckStatus.CALIBRATION_REQUIRED,
        CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA,
        CameraCalibCheckStatus.UNKNOWN_PERSON_NOT_DETECTED,
        CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION,
    }
)


def validate_camera_calibcheck_status(
    value: int | CameraCalibCheckStatus,
) -> CameraCalibCheckStatus:
    """MMAPへ書き込み可能なcalibcheckステータスへ変換する。"""
    try:
        status = CameraCalibCheckStatus(value)
    except ValueError as error:
        raise ValueError(f"invalid camera calibcheck status: {value}") from error
    if status not in _WRITABLE_STATUSES:
        raise ValueError(f"camera calibcheck status {int(status)} is not writable")
    return status


class CameraCalibCheckStatusDiagnosis:
    """校正要否判定と判定不能理由からUI通知ステータスを決定する。"""

    _REASON_STATUS_MAP: ClassVar[dict[int, CameraCalibCheckStatus]] = {
        1: CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA,
        2: CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA,
        3: CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA,
        4: CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION,
        5: CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION,
        6: CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION,
        7: CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION,
        8: CameraCalibCheckStatus.UNKNOWN_POOR_PERSON_DETECTION,
        9: CameraCalibCheckStatus.UNKNOWN_PERSON_NOT_DETECTED,
        10: CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA,
        11: CameraCalibCheckStatus.UNKNOWN_PERSON_NOT_DETECTED,
    }

    def diagnose(
        self, *, reason_code: int, calibration_is_acceptable: bool
    ) -> CameraCalibCheckStatus:
        if reason_code != 0:
            return self._REASON_STATUS_MAP.get(
                reason_code, CameraCalibCheckStatus.UNKNOWN_INSUFFICIENT_DATA
            )
        if calibration_is_acceptable:
            return CameraCalibCheckStatus.CALIBRATION_NOT_REQUIRED
        return CameraCalibCheckStatus.CALIBRATION_REQUIRED
