import glob
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from argus_synchro.config.app_config import CalibrationConf
from argus_synchro.diagnosis.error_config import (
    CameraNCalibDataInvalidParameters,
    SensorCalibDataInvalidParameters,
)


@dataclass(frozen=True, slots=True)
class LidarCalibValidationIssue:
    matrix_kind: str
    matrix_path: str
    detail: str


class LidarCalibValidator:
    def __init__(self, param: SensorCalibDataInvalidParameters) -> None:
        self._param = param
        self._xy_grid_points = self._build_xy_grid_points()

    def validate(
        self,
        calibration_conf: CalibrationConf,
        lidar2crane_reference_paths: list[str] | None = None,
    ) -> tuple[LidarCalibValidationIssue, ...]:
        targets: list[tuple[str, str]] = []
        if self._param.check_lidar2lidar:
            targets.append(("lidar2lidar", calibration_conf.BothLidars))
        if self._param.check_lidar2crane:
            targets.extend(
                ("lidar2crane", path)
                for path in calibration_conf.Lidar_calib_files
            )

        issues: list[LidarCalibValidationIssue] = []
        if (
            lidar2crane_reference_paths is not None
            and len(lidar2crane_reference_paths)
            != len(calibration_conf.Lidar_calib_files)
        ):
            raise ValueError(
                "lidar2crane_reference_paths must match Lidar_calib_files length"
            )

        lidar2crane_index = 0
        for matrix_kind, matrix_path in targets:
            path = Path(matrix_path)
            try:
                matrix = np.loadtxt(path, delimiter=",", dtype=np.float64)
            except (OSError, UnicodeError, ValueError) as error:
                issues.append(
                    LidarCalibValidationIssue(
                        matrix_kind,
                        str(path),
                        f"failed to load CSV: {type(error).__name__}: {error}",
                    )
                )
                continue

            matrix_issues = self._validate_matrix(matrix_kind, str(path), matrix)
            issues.extend(matrix_issues)
            if matrix_issues or matrix_kind != "lidar2crane":
                continue
            if (
                self._param.enable_reference_diff_check
                and lidar2crane_reference_paths is not None
            ):
                reference_path = Path(
                    lidar2crane_reference_paths[lidar2crane_index]
                )
                issues.extend(
                    self._validate_reference_file(
                        matrix_kind,
                        str(path),
                        matrix,
                        reference_path,
                    )
                )
            lidar2crane_index += 1

        return tuple(issues)

    def validate_matrices(
        self,
        matrices: list[NDArray[np.float64]] | NDArray[np.float64],
        matrix_paths: list[str] | None = None,
        reference_matrices: list[NDArray[np.float64]] | None = None,
        reference_matrix_paths: list[str] | None = None,
        matrix_kind: str = "lidar2crane",
    ) -> list[LidarCalibValidationIssue]:
        array = np.asarray(matrices, dtype=np.float64)
        matrix_dimension = 2
        matrix_batch_dimension = 3
        if array.ndim == matrix_dimension:
            matrix_list = [array]
        elif array.ndim == matrix_batch_dimension:
            matrix_list = [array[index] for index in range(array.shape[0])]
        else:
            raise ValueError("matrices must have shape (4, 4) or (N, 4, 4)")

        if matrix_paths is None:
            paths = [f"<generated:{index}>" for index in range(len(matrix_list))]
        else:
            paths = matrix_paths
        if len(paths) != len(matrix_list):
            raise ValueError("matrix_paths must have the same length as matrices")
        if reference_matrices is not None and len(reference_matrices) != len(
            matrix_list
        ):
            raise ValueError(
                "reference_matrices must have the same length as matrices"
            )
        if reference_matrix_paths is not None and len(reference_matrix_paths) != len(
            matrix_list
        ):
            raise ValueError(
                "reference_matrix_paths must have the same length as matrices"
            )
        if reference_matrices is not None and reference_matrix_paths is not None:
            raise ValueError(
                "provide either reference_matrices or reference_matrix_paths, not both"
            )
        issues: list[LidarCalibValidationIssue] = []
        for index, matrix in enumerate(matrix_list):
            matrix_issues = self._validate_matrix(matrix_kind, paths[index], matrix)
            issues.extend(matrix_issues)
            if (
                not matrix_issues
                and self._param.enable_reference_diff_check
            ):
                if reference_matrices is not None:
                    issues.extend(
                        self._validate_reference_diff(
                            matrix_kind,
                            paths[index],
                            matrix,
                            reference_matrices[index],
                        )
                    )
                elif reference_matrix_paths is not None:
                    issues.extend(
                        self._validate_reference_file(
                            matrix_kind,
                            paths[index],
                            matrix,
                            Path(reference_matrix_paths[index]),
                        )
                    )
        return issues


    def _validate_reference_file(
        self,
        matrix_kind: str,
        matrix_path: str,
        matrix: NDArray[np.float64],
        reference_path: Path,
    ) -> list[LidarCalibValidationIssue]:
        try:
            reference_matrix = np.loadtxt(
                reference_path, delimiter=",", dtype=np.float64
            )
        except (OSError, UnicodeError, ValueError) as error:
            return [
                LidarCalibValidationIssue(
                    matrix_kind,
                    str(reference_path),
                    "failed to load reference CSV: "
                    f"{type(error).__name__}: {error}",
                )
            ]
        return self._validate_reference_diff(
            matrix_kind,
            matrix_path,
            matrix,
            reference_matrix,
        )

    def _build_xy_grid_points(self) -> NDArray[np.float64]:
        if self._param.grid_radius_m < 0 or self._param.grid_spacing_m <= 0:
            raise ValueError("grid radius must be non-negative and spacing positive")
        axis = np.arange(
            -self._param.grid_radius_m,
            self._param.grid_radius_m + self._param.grid_spacing_m * 0.5,
            self._param.grid_spacing_m,
            dtype=np.float64,
        )
        xx, yy = np.meshgrid(axis, axis, indexing="xy")
        inside = xx * xx + yy * yy <= self._param.grid_radius_m**2 + 1e-12
        return np.column_stack(
            (xx[inside], yy[inside], np.zeros(np.count_nonzero(inside)))
        )

    def _exceeds_threshold(self, value: float, threshold: float) -> bool:
        return value > threshold and not math.isclose(
            value,
            threshold,
            rel_tol=0.0,
            abs_tol=self._param.comparison_atol,
        )

    def _validate_reference_diff(
        self,
        matrix_kind: str,
        matrix_path: str,
        matrix: NDArray[np.float64],
        reference_matrix: NDArray[np.float64],
    ) -> list[LidarCalibValidationIssue]:
        reference = np.asarray(reference_matrix, dtype=np.float64)
        reference_issues = self._validate_matrix(
            f"{matrix_kind}_reference", "<reference>", reference
        )
        if reference_issues:
            return reference_issues

        try:
            error_transform = np.linalg.solve(reference, matrix)
        except (ValueError, np.linalg.LinAlgError) as error:
            return [
                LidarCalibValidationIssue(
                    matrix_kind,
                    matrix_path,
                    "reference diff validation failed: "
                    f"{type(error).__name__}: {error}",
                )
            ]

        rotation = error_transform[:3, :3]
        translation = error_transform[:3, 3]
        translation_error_m = float(np.linalg.norm(translation))
        cosine = (float(np.trace(rotation)) - 1.0) / 2.0
        rotation_error_deg = math.degrees(
            math.acos(float(np.clip(cosine, -1.0, 1.0)))
        )
        transformed = self._xy_grid_points @ rotation.T + translation
        max_xy_displacement_m = float(
            np.max(
                np.linalg.norm(
                    transformed[:, :2] - self._xy_grid_points[:, :2], axis=1
                )
            )
        )

        issues: list[LidarCalibValidationIssue] = []
        metrics = (
            (
                "translation error",
                translation_error_m,
                self._param.translation_threshold_m,
                "m",
            ),
            (
                "rotation error",
                rotation_error_deg,
                self._param.rotation_threshold_deg,
                "deg",
            ),
            (
                "max XY displacement",
                max_xy_displacement_m,
                self._param.max_xy_displacement_threshold_m,
                "m",
            ),
        )
        for name, value, threshold, unit in metrics:
            if self._exceeds_threshold(value, threshold):
                issues.append(
                    LidarCalibValidationIssue(
                        matrix_kind,
                        matrix_path,
                        f"{name} exceeds threshold: "
                        f"{value:.6f}{unit} > {threshold}{unit}",
                    )
                )
        return issues

    def _validate_matrix(
        self,
        matrix_kind: str,
        matrix_path: str,
        matrix: NDArray[np.float64],
    ) -> list[LidarCalibValidationIssue]:
        issues: list[LidarCalibValidationIssue] = []
        if self._param.enforce_shape_4x4 and matrix.shape != (4, 4):
            issues.append(
                LidarCalibValidationIssue(
                    matrix_kind,
                    matrix_path,
                    f"matrix shape must be (4, 4), actual={matrix.shape}",
                )
            )
        if self._param.finite_value_only and not bool(np.isfinite(matrix).all()):
            issues.append(
                LidarCalibValidationIssue(
                    matrix_kind,
                    matrix_path,
                    "matrix contains non-finite values",
                )
            )
        return issues


@dataclass(frozen=True, slots=True)
class CameraCalibValidationIssue:
    camera_index: int
    path: str
    kind: str
    detail: str


class CameraCalibValidator:
    def __init__(self, param: CameraNCalibDataInvalidParameters) -> None:
        self._param = param

    def validate(
        self, camera_index: int, calibration_conf: CalibrationConf
    ) -> tuple[CameraCalibValidationIssue, ...]:
        issues = self._validate_fisheye_param(
            camera_index, calibration_conf.fisheye_param_file
        )
        issues.extend(
            self._validate_camera_lidar_matrix(camera_index, calibration_conf)
        )
        return tuple(issues)

    def _validate_fisheye_param(
        self, camera_index: int, fisheye_path: str
    ) -> list[CameraCalibValidationIssue]:
        try:
            with open(fisheye_path, encoding="utf-8") as file:
                payload = json.load(file)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            return [
                CameraCalibValidationIssue(
                    camera_index,
                    fisheye_path,
                    "fisheye_param",
                    f"failed to load: {type(error).__name__}: {error}",
                )
            ]
        if not self._param.check_fisheye_intrinsics:
            return []

        try:
            camera_matrix = self._json_matrix(payload, "camera_matrix")
            new_camera_matrix = self._json_matrix(
                payload, "new_camera_matrix_alpha1"
            )
            distortion = np.asarray(
                [payload[key] for key in ("k1", "k2", "k3", "k4")],
                dtype=np.float64,
            )
            int(payload["image_width"])
            int(payload["image_height"])
        except (KeyError, TypeError, ValueError) as error:
            return [
                CameraCalibValidationIssue(
                    camera_index,
                    fisheye_path,
                    "fisheye_param",
                    f"missing or invalid field: {type(error).__name__}: {error}",
                )
            ]

        issues: list[CameraCalibValidationIssue] = []
        for kind, matrix, expected_shape in (
            ("fisheye_camera_matrix", camera_matrix, (3, 3)),
            ("fisheye_new_camera_matrix", new_camera_matrix, (3, 3)),
            ("fisheye_dist_coeff", distortion, (4,)),
        ):
            if self._param.enforce_shape_4x4 and matrix.shape != expected_shape:
                issues.append(
                    CameraCalibValidationIssue(
                        camera_index,
                        fisheye_path,
                        kind,
                        f"invalid shape: {matrix.shape}, expected={expected_shape}",
                    )
                )
            if self._param.finite_value_only and not bool(np.isfinite(matrix).all()):
                issues.append(
                    CameraCalibValidationIssue(
                        camera_index,
                        fisheye_path,
                        kind,
                        "non-finite values",
                    )
                )
        return issues

    @staticmethod
    def _json_matrix(payload: object, key: str) -> NDArray[np.float64]:
        if not isinstance(payload, dict):
            raise TypeError("fisheye calibration root must be an object")
        value = payload[key]
        if not isinstance(value, dict):
            raise TypeError(f"{key} must be an object")
        rows = int(value["rows"])
        cols = int(value["cols"])
        return np.asarray(value["data"], dtype=np.float64).reshape(rows, cols)

    def _validate_camera_lidar_matrix(
        self,
        camera_index: int,
        calibration_conf: CalibrationConf,
    ) -> list[CameraCalibValidationIssue]:
        patterns = {
            0: calibration_conf.list_0_files,
            1: calibration_conf.list_1_files,
            2: calibration_conf.list_2_files,
        }
        pattern = patterns.get(camera_index)
        if pattern is None:
            return [
                CameraCalibValidationIssue(
                    camera_index,
                    "",
                    "camera_lidar_matrix",
                    f"unsupported camera index: {camera_index}",
                )
            ]
        matches = glob.glob(pattern)
        if not matches:
            return [
                CameraCalibValidationIssue(
                    camera_index,
                    pattern,
                    "camera_lidar_matrix",
                    "no matched files",
                )
            ]
        latest_file = max(matches, key=os.path.getmtime)
        try:
            matrix = np.loadtxt(latest_file, delimiter=",", dtype=np.float64)
        except (OSError, UnicodeError, ValueError) as error:
            return [
                CameraCalibValidationIssue(
                    camera_index,
                    latest_file,
                    "camera_lidar_matrix",
                    f"failed to load: {type(error).__name__}: {error}",
                )
            ]
        issues: list[CameraCalibValidationIssue] = []
        if self._param.enforce_shape_4x4 and matrix.shape != (4, 4):
            issues.append(
                CameraCalibValidationIssue(
                    camera_index,
                    latest_file,
                    "camera_lidar_matrix",
                    f"invalid shape: {matrix.shape}, expected=(4, 4)",
                )
            )
        if self._param.finite_value_only and not bool(np.isfinite(matrix).all()):
            issues.append(
                CameraCalibValidationIssue(
                    camera_index,
                    latest_file,
                    "camera_lidar_matrix",
                    "non-finite values",
                )
            )
        return issues
