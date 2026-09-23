from collections.abc import Callable, Sequence
from pathlib import Path

from argus_synchro.diagnosis.calibcheck2d3d_result_diagnosis import (
    CameraCalibCheckDiagnosisResult,
    CameraCalibCheckStatus,
    calibcheck_reason_to_status,
    calibcheck_reason_to_string,
    normalize_calibcheck_reason,
)


def error_reason_to_string(reason: int) -> str:
    """判定不能を含む評価 reason code を表示用の文言へ変換する。"""
    return calibcheck_reason_to_string(reason)


def error_reason_to_ui_errornum(reason: int) -> int:
    """評価 reason code を UI 共通エラー番号へ変換する。"""
    try:
        normalized_reason = normalize_calibcheck_reason(reason)
    except ValueError:
        return -1
    return int(
        calibcheck_reason_to_status(
            normalized_reason,
            calibration_is_acceptable=None,
        )
    )


def publish_camera_calibcheck_statuses(
    results: Sequence[CameraCalibCheckDiagnosisResult],
    *,
    set_status: Callable[[int, CameraCalibCheckStatus], None],
) -> None:
    """診断が確定したカメラ別statusをUIへ設定する。"""
    for result in results:
        set_status(result.camera_index, result.status)


def publish_calibcheck_lifecycle_status(
    monitor: object,
    *,
    sec: object,
    status: object,
) -> None:
    """校正診断の終了状態を既存の dummy 値と共に MMAP へ公開する。"""
    monitor.set_status_calibcommon(status)
    monitor.set_dummydata(
        enable_systemerrorflag=True,
        enable_errorflag=True,
        overwrite_checkresult=True,
        enable_yawangle=True,
    )
    monitor.transmit_setdata(sec=sec, ref_t=None)


def write_calibcheck_result_files(
    resultfiles: Sequence[str],
    results: Sequence[CameraCalibCheckDiagnosisResult],
    *,
    logger: object,
) -> None:
    """カメラ別の校正診断結果を OK、NG、Unknown として出力する。"""
    for result in results:
        result_string = (
            "Unknown"
            if result.calibration_is_acceptable is None
            else ("OK" if result.calibration_is_acceptable else "NG")
        )
        with open(resultfiles[result.camera_index], "w") as output_file:
            print(result_string, file=output_file)
        logger.info(
            f"Camera{result.camera_index} result: {result_string}, "
            f"reason:{int(result.primary_reason)}",
        )


def write_evaluation_point_debug_file(
    outputdir_root: str,
    *,
    checked_points3d: list[object],
    checked_points3d_score: list[object],
    checked_points2d: list[object],
    checked_points2d_score: list[object],
) -> None:
    """評価で使った点群と score を、既存のデバッグファイル形式で出力する。"""
    result_path = Path(outputdir_root, "calibcheck2d3d_results.txt")
    with open(result_path, "w") as output_file:
        print("checked_points3d", file=output_file)
        for value in checked_points3d:
            print(value, file=output_file)
        print("checked_points3d_score", file=output_file)
        for value in checked_points3d_score:
            print(value, file=output_file)
        print("checked_points2d", file=output_file)
        for value in checked_points2d:
            print(value, file=output_file)
        print("checked_points2d_score", file=output_file)
        for value in checked_points2d_score:
            print(value, file=output_file)
