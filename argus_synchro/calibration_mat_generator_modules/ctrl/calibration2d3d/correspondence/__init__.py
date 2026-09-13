import datetime
import json
import os
from abc import abstractmethod
from collections.abc import Callable
from typing import Any, Dict, Optional

import numpy as np
import pandas as pd
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.correspondence import (
    FusionSensorProcessor,
    pnp_estimation_opt,
)
from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_store,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration


class correspondence_class_base:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.additional_info = app_config_calib

        self.app_config_calib = app_config_calib
        self.file_io_error_reporter = file_io_error_reporter
        postprocess_mat_path = (
            app_config_calib.calib2d3d.CalcCorrespondence.postprocess_mat
        )
        try:
            self.postprocess_mat = np.array(
                pd.read_csv(postprocess_mat_path, header=None)
            )
        except (
            OSError,
            UnicodeError,
            pd.errors.EmptyDataError,
            pd.errors.ParserError,
        ) as error:
            if self.file_io_error_reporter is not None:
                self.file_io_error_reporter(
                    postprocess_mat_path,
                    "read 2D-3D postprocess matrix CSV",
                    error,
                )
            raise

    def close(self) -> None:
        pass

    def preproc(
        self,
        corner2d_set,
        corner3d_set,
        adjustfunc_coord_calib2normal,
        adjustfunc_coord_normal2argus,
        savename_suffix: str = "",
        points_per_time: int = 2,
    ):
        cornerset_index_points = 0

        self._logger.info(
            f"2d all point length: {len(corner2d_set[cornerset_index_points])}"
        )
        self._logger.info(
            f"3d all point length: {len(corner3d_set[cornerset_index_points])}"
        )

        debug_store(key="corner2d_set", value=corner2d_set)
        debug_store(key="corner3d_set", value=corner3d_set)

        corner2d, corner3d = self._match_ts(
            corner2d_set,
            corner3d_set,
            points_per_time=points_per_time,
        )

        corner2d = self._prepare2d(corner2d)
        corner3d = self._prepare3d(
            corner3d, adjustfunc_coord_calib2normal, adjustfunc_coord_normal2argus
        )

        return corner2d, corner3d

    def _prepare2d(self, corner2d):
        return corner2d

    def _prepare3d(
        self, corner3d, adjustfunc_coord_calib2normal, adjustfunc_coord_normal2argus
    ):
        # 校正用座標系から一度戻す
        corner3d = adjustfunc_coord_calib2normal(corner3d, 0)
        # argus用に再び変更
        corner3d = adjustfunc_coord_normal2argus(corner3d)

        return corner3d

    def _match_ts(
        self,
        corner2d_set,
        corner3d_set,
        points_per_time: int,
    ):
        (cornerlist2d, tslist2d) = corner2d_set
        (cornerlist3d, tslist3d) = corner3d_set

        if len(tslist2d) == 0 or len(tslist3d) == 0:
            return np.zeros((0, 2), dtype=np.float64), np.zeros(
                (0, 3), dtype=np.float64
            )

        tslist2d = np.array(tslist2d, dtype=np.int32)
        tslist3d = np.array(tslist3d, dtype=np.int32)
        cornerlist2d = np.array(cornerlist2d, dtype=np.float64)
        cornerlist3d = np.array(cornerlist3d, dtype=np.float64)

        self._logger.info(
            f"{cornerlist2d.shape=},{cornerlist3d.shape=},{tslist2d.shape=},{tslist3d.shape=},{len(tslist2d)=},{len(tslist3d)=}",
        )

        tslist_overlap = tslist2d[np.isin(tslist2d, tslist3d)]
        tslist2d_overlap_index = np.where(np.isin(tslist2d, tslist3d))[0]
        tslist3d_overlap_index = np.where(np.isin(tslist3d, tslist2d))[0]

        if (
            len(tslist_overlap) == 0
            or len(tslist2d_overlap_index) == 0
            or len(tslist3d_overlap_index) == 0
        ):
            return np.zeros((0, 2), dtype=np.float64), np.zeros(
                (0, 3), dtype=np.float64
            )

        debug_store(key="tslist_overlap", value=tslist_overlap)
        debug_store(key="tslist2d_overlap_index", value=tslist2d_overlap_index)
        debug_store(key="tslist3d_overlap_index", value=tslist3d_overlap_index)

        corner2d_l = list()
        for pts in cornerlist2d[tslist2d_overlap_index]:
            if points_per_time >= 2:
                assert pts.ravel().size % 2 == 0
                assert pts.shape[0] == points_per_time
                for pt in pts:
                    corner2d_l.append(pt)
            else:
                assert len(pts.ravel()) == 2
                corner2d_l.append(pts.ravel())

        corner2d = np.array(corner2d_l, dtype=np.float64)

        corner3d_l = list()
        for pts in cornerlist3d[tslist3d_overlap_index]:
            if points_per_time >= 2:
                assert pts.ravel().size % 2 == 0
                assert pts.shape[0] == points_per_time
                for pt in pts:
                    corner3d_l.append(pt)
            else:
                assert len(pts.ravel()) == 3
                corner3d_l.append(pts.ravel())
        corner3d = np.array(corner3d_l, dtype=np.float64)

        debug_store(key="corner2d_sync", value=corner2d)
        debug_store(key="corner3d_sync", value=corner3d)

        assert len(corner2d.shape) == 2, (
            f"{corner2d.shape=},{corner3d.shape=},{len(tslist2d)=},{len(tslist3d)=}"
        )
        assert len(corner3d.shape) == 2, (
            f"{corner2d.shape=},{corner3d.shape=},{len(tslist2d)=},{len(tslist3d)=}"
        )
        assert corner2d.shape[0] == corner3d.shape[0], (
            f"{corner2d.shape=},{corner3d.shape=},{len(tslist2d)=},{len(tslist3d)=}"
        )
        assert corner2d.shape[1] == 2, (
            f"{corner2d.shape=},{corner3d.shape=},{len(tslist2d)=},{len(tslist3d)=}"
        )
        assert corner3d.shape[1] == 3, (
            f"{corner2d.shape=},{corner3d.shape=},{len(tslist2d)=},{len(tslist3d)=}"
        )

        return corner2d, corner3d

    @abstractmethod
    def estimate(self, corner2d, corner3d, centerF_or_axisT: bool): ...

    @abstractmethod
    def get_rotation_mat(self): ...

    @abstractmethod
    def get_last_rtvec(self): ...

    @abstractmethod
    def save(self, file_savedir="", name_suffix=""): ...


class correspondence_class_oldmethod(correspondence_class_base):
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        cameramatrix: NDArray[np.float64],
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        super().__init__(
            app_config_calib=app_config_calib,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=file_io_error_reporter,
        )
        self._app_logger_factory = app_logger_factory
        self.reset(cameramatrix)

    def reset(self, cameramatrix: NDArray[np.float64]):
        self.proccorr_old = FusionSensorProcessor.FusionSensorProcessor(
            dist_coeffs=np.zeros((1, 5)),
            camera_matrix=cameramatrix,
            app_logger_factory=self._app_logger_factory,
            verbose=True,
        )

    def estimate(
        self,
        corner2d,
        corner3d,
        centerF_or_axisT: bool,
    ):
        self.fit_result = self.proccorr_old.fit(
            corners_3d=corner3d, corners_2d=corner2d
        )
        self._logger.info(f"result: {self.fit_result}")
        return self.fit_result

    def get_rotation_mat(self):
        return self.proccorr_old.get_rotation_mat_useallpoint() @ self.postprocess_mat

    def get_last_rtvec(self):
        return (
            self.proccorr_old.rotation_vector_useallpoint,
            self.proccorr_old.translation_vector_useallpoint,
        )

    def save(self, file_savedir="", name_suffix=""):
        timestr = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.proccorr_old.save(
            savedir=file_savedir,
            suffix="_" + timestr + "_" + name_suffix,
            postprocess_mat=self.postprocess_mat,
        )  # 校正計算結果を保存


class correspondence_class_optmethod(correspondence_class_base):
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        cameramatrix: NDArray[np.float64],
        camera_index: int,
        app_logger_factory: AppLoggerFactory,
        file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
    ) -> None:
        super().__init__(
            app_config_calib=app_config_calib,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=file_io_error_reporter,
        )
        self._app_logger_factory: AppLoggerFactory = app_logger_factory

        self.reset(
            app_config_calib=app_config_calib,
            cameramatrix=cameramatrix,
            camera_index=camera_index,
        )

    def reset(
        self,
        app_config_calib: AppConfigCalibration,
        cameramatrix: NDArray[np.float64],
        camera_index: int,
    ):
        self.proccorr_opt = pnp_estimation_opt.PnpEstimationCalculator(
            dist_coeffs=np.zeros((1, 5)),
            camera_matrix=cameramatrix,
            app_logger_factory=self._app_logger_factory,
            lambda_center_r=np.array(
                app_config_calib.calib2d3d.CalcCorrespondence.opt_lambda_center_r
            ),
            lambda_center_t=np.array(
                app_config_calib.calib2d3d.CalcCorrespondence.opt_lambda_center_t
            ),
            lambda_axis_r=np.array(
                app_config_calib.calib2d3d.CalcCorrespondence.opt_lambda_axis_r
            ),
            lambda_axis_t=np.array(
                app_config_calib.calib2d3d.CalcCorrespondence.opt_lambda_axis_t
            ),
            normalize_imagesize=np.array(
                [
                    self.app_config_calib.dataCapture.Camera.sys_width,
                    self.app_config_calib.dataCapture.Camera.sys_height,
                ]
            ),
            verbose=True,
            file_io_error_reporter=self.file_io_error_reporter,
        )

        initial_vectors_path = (
            app_config_calib.calib2d3d.CalcCorrespondence.optparam_initialvector
        )
        try:
            with open(initial_vectors_path, encoding="utf-8") as rtf:
                initial_vectors: list[dict[str, list[float]]] = json.load(rtf)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            if self.file_io_error_reporter is not None:
                self.file_io_error_reporter(
                    initial_vectors_path,
                    "read 2D-3D initial vector JSON",
                    error,
                )
            raise

        self.initial_rvec_rad: NDArray[np.float64] = np.array(
            initial_vectors[camera_index]["initial_rvec_rad"]
        )
        self.initial_tvec: NDArray[np.float64] = np.array(
            initial_vectors[camera_index]["initial_tvec"]
        )

    def estimate(
        self,
        corners_2d: NDArray[np.float64],
        corners_3d: NDArray[np.float64],
        centerF_or_axisT: bool,
        debug_dict: dict[str, Any] | None = None,  # デバッグ情報受け取り用辞書):
    ):
        self.fit_result = self.proccorr_opt.fit(
            corners_3d=corners_3d,
            corners_2d=corners_2d,
            initial_rvec_rad=self.initial_rvec_rad,
            initial_tvec=self.initial_tvec,
            centerF_or_axisT=centerF_or_axisT,
            debug_dict=debug_dict,
        )
        self._logger.info(f"result: {self.fit_result}")

    def get_rotation_mat(self):
        return self.proccorr_opt.get_rotation_mat() @ self.postprocess_mat

    def save(self, file_savedir="", name_suffix=""):
        timestr = f"{os.getpid()}_{datetime.datetime.now().strftime('%Y%m%d_%H')}"
        self.proccorr_opt.save(
            savedir=file_savedir,
            suffix="_" + timestr + "_" + name_suffix,
            postprocess_mat=self.postprocess_mat,
        )  # 校正計算結果を保存

    def get_last_rtvec(self):
        return (self.proccorr_opt.rotation_vector, self.proccorr_opt.translation_vector)


def correspondence_class_loader(
    app_config_calib: AppConfigCalibration,
    cameramatrix: NDArray[np.float64],
    camera_index: int,
    app_logger_factory: AppLoggerFactory,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
) -> correspondence_class_oldmethod | correspondence_class_optmethod:
    if app_config_calib.calib2d3d.CalcCorrespondence.calcmethod == "old":
        return correspondence_class_oldmethod(
            app_config_calib=app_config_calib,
            cameramatrix=cameramatrix,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=file_io_error_reporter,
        )
    if app_config_calib.calib2d3d.CalcCorrespondence.calcmethod == "opt":
        return correspondence_class_optmethod(
            app_config_calib=app_config_calib,
            cameramatrix=cameramatrix,
            camera_index=camera_index,
            app_logger_factory=app_logger_factory,
            file_io_error_reporter=file_io_error_reporter,
        )
    raise RuntimeError(
        f"undefined calcmethod: {app_config_calib.calib2d3d.CalcCorrespondence.calcmethod}"
    )
