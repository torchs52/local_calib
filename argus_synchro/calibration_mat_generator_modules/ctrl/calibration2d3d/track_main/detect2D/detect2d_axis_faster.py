import copy

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy.interpolate import griddata

from argus_synchro.calibration_mat_generator_modules.utils.debugdata_store import (
    debug_store,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory

_logger: AppLogger = AppLoggerFactory.from_name("detect2d_axis_faster")
debugdict_new = {}


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)


class Detect2dAxisFaster:
    def __init__(self, Mc, debugflag, app_logger_factory: AppLoggerFactory) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.Mc = Mc
        self.debugflag = debugflag

        self.accum_bbox = []
        self.accum_bbox_ts = []

    @staticmethod
    def _calc_slope_value(rvec, tvec, xrange: tuple, yrange: tuple, Mc):
        # 各軸の範囲と分割数を指定
        x_min, x_max, x_points = xrange
        y_min, y_max, y_points = yrange
        z_min, z_max, z_points = -0.05, 0.05, 2

        # 各軸の座標を生成
        x = np.linspace(x_min, x_max, x_points)
        x_offset = 4.0
        x = x * np.abs(x) + x_offset
        y = np.linspace(y_min, y_max, y_points)
        y = y * np.abs(y)
        z = np.linspace(z_min, z_max, z_points)

        # メッシュグリッドを作成（インデックス順を 'ij' にすると x, y, z の順になる）
        X, Y, Z = np.meshgrid(x, y, z, indexing="ij")

        # グリッド座標を1つの配列にまとめる（形状: (N, 3)）+ 2D空間へ投影
        imgpoints, _ = cv2.projectPoints(
            np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=-1),
            rvec,
            tvec,
            Mc,
            distCoeffs=np.zeros(5),
        )

        imgpoints = imgpoints.reshape(-1, 2, 2)
        refpts = (imgpoints[:, 0] + imgpoints[:, 1]) / 2
        slope = np.where(
            (imgpoints[:, 1] - imgpoints[:, 0])[:, 0],
            (imgpoints[:, 1] - imgpoints[:, 0])[:, 1]
            / (imgpoints[:, 1] - imgpoints[:, 0])[:, 0],
            np.inf,
        )
        head_over_foot = np.where(
            imgpoints[:, 1, 1] - imgpoints[:, 1, 0] < 0, 1, -1
        )  # 通常Zの低い方が足のはず。従うなら1、逆なら-1を立てておく。imgpointsは足・頭の順
        debugdict_new["imgpoints"] = copy.deepcopy(imgpoints)
        return refpts, slope, head_over_foot

    @staticmethod
    def calc_crosspoints(
        bbox,
        rvec,
        tvec,
        Mc,
        xrange,
        yrange,
        interpolate_firstmethod="linear",
        interpolate_secondmethod="nearest",
    ):
        if bbox is None:
            return None
        debugdict_new["bbox"] = copy.deepcopy(bbox)

        (x1, x2) = bbox[:, 0], bbox[:, 1]
        (y1, y2) = bbox[:, 2], bbox[:, 3]
        bbox_center = np.stack([(x1 + x2) / 2, (y1 + y2) / 2], axis=-1)
        bw = abs(x2 - x1)
        bh = abs(y2 - y1)

        debugdict_new["bbox_center"] = copy.deepcopy(bbox_center)
        debugdict_new["bw"] = copy.deepcopy(bw)
        debugdict_new["bh"] = copy.deepcopy(bh)

        refpts, slope, head_over_foot = Detect2dAxisFaster._calc_slope_value(
            rvec=rvec, tvec=tvec, xrange=xrange, yrange=yrange, Mc=Mc
        )
        debugdict_new["refpts"] = copy.deepcopy(refpts)
        debugdict_new["slope"] = copy.deepcopy(slope)

        interpolated_slope = griddata(
            points=refpts,
            values=slope,
            xi=bbox_center,
            method=interpolate_firstmethod,
        )
        debugdict_new["interpolated_slope"] = copy.deepcopy(interpolated_slope)
        _logger.info(
            f"[detect2d_axis_faster] after linear interpolation : interpolated_values:{interpolated_slope.shape}, nan:{np.count_nonzero(np.isnan(interpolated_slope))}",
        )
        nanval_filter = np.isnan(interpolated_slope)
        nan_coordinates = bbox_center[nanval_filter]
        interpolated_slope[nanval_filter] = griddata(
            points=refpts,
            values=slope,
            xi=nan_coordinates,
            method=interpolate_secondmethod,
        )
        _logger.info(
            f"[detect2d_axis_faster] after nearest interpolation : interpolated_values:{interpolated_slope.shape}, nan:{np.count_nonzero(np.isnan(interpolated_slope))}",
        )
        debugdict_new["interpolated_slope_a"] = copy.deepcopy(interpolated_slope)

        interpolated_head_over_foot = griddata(
            points=refpts, values=head_over_foot, xi=bbox_center, method="nearest"
        )
        _logger.info(
            f"[detect2d_axis_faster] after linear interpolation : interpolated_head_over_foot:{interpolated_head_over_foot.shape}, nan:{np.count_nonzero(np.isnan(interpolated_head_over_foot))}",
        )
        nanval_filter = np.isnan(interpolated_head_over_foot)
        nan_coordinates = bbox_center[nanval_filter]
        interpolated_head_over_foot[nanval_filter] = griddata(
            points=refpts, values=head_over_foot, xi=nan_coordinates, method="nearest"
        )
        _logger.info(
            f"[detect2d_axis_faster] after nearest interpolation : interpolated_head_over_foot:{interpolated_head_over_foot.shape}, nan:{np.count_nonzero(np.isnan(interpolated_head_over_foot))}",
        )

        slope_threshold = bh / bw
        debugdict_new["slope_threshold"] = copy.deepcopy(slope_threshold)

        # 頭(y小)
        p_ax = np.where(
            np.abs(interpolated_slope) < slope_threshold,
            bbox_center[:, 0] + bw / 2,
            bbox_center[:, 0] + bh / (2 * interpolated_slope),
        )
        p_ay = np.where(
            np.abs(interpolated_slope) < slope_threshold,
            bbox_center[:, 1] + interpolated_slope * bw / 2,
            bbox_center[:, 1] + bh / 2,
        )

        # 足(y大)
        p_bx = np.where(
            np.abs(interpolated_slope) < slope_threshold,
            bbox_center[:, 0] - bw / 2,
            bbox_center[:, 0] - bh / (2 * interpolated_slope),
        )
        p_by = np.where(
            np.abs(interpolated_slope) < slope_threshold,
            bbox_center[:, 1] - interpolated_slope * bw / 2,
            bbox_center[:, 1] - bh / 2,
        )

        # 頭y<足yは正常、逆の時入替
        ans = np.where(
            (p_ay <= p_by)[:, np.newaxis],
            np.stack([p_ax, p_ay, p_bx, p_by], axis=-1),
            np.stack([p_bx, p_by, p_ax, p_ay], axis=-1),
        ).reshape((-1, 2, 2))

        debugdict_new["ans"] = copy.deepcopy(ans)

        return ans

    def stack_bbox(self, bbox: NDArray[np.float64], ts: int):
        self.accum_bbox.append(bbox)
        self.accum_bbox_ts.append(ts)

    @staticmethod
    def is_inrange(x, begin, end, mode=0b10):
        if mode == 0b00:
            return (x > begin) & (x < end)
        if mode == 0b01:
            return (x > begin) & (x <= end)
        if mode == 0b10:
            return (x >= begin) & (x < end)
        return (x >= begin) & (x <= end)

    def extract_results(
        self,
        rvec,
        tvec,
        width,
        height,
        margin,
        xrange,
        yrange,
        interpolate_firstmethod,
        interpolate_secondmethod,
    ):
        return Detect2dAxisFaster.extract_results_core(
            rvec,
            tvec,
            width,
            height,
            margin,
            self.accum_bbox,
            self.accum_bbox_ts,
            self.Mc,
            xrange,
            yrange,
            interpolate_firstmethod,
            interpolate_secondmethod,
        )

    @staticmethod
    def extract_results_core(
        rvec,
        tvec,
        width,
        height,
        margin,
        accum_bbox,
        accum_bbox_ts,
        Mc,
        xrange,
        yrange,
        interpolate_firstmethod,
        interpolate_secondmethod,
    ):
        debug_store(key="accum_bbox", value=accum_bbox)
        new2dcorrpoints = Detect2dAxisFaster.calc_crosspoints(
            np.array(accum_bbox),
            rvec,
            tvec,
            Mc=Mc,
            xrange=xrange,
            yrange=yrange,
            interpolate_firstmethod=interpolate_firstmethod,
            interpolate_secondmethod=interpolate_secondmethod,
        )
        if new2dcorrpoints is None:
            return np.zeros((0, 2, 2)), np.zeros(0)

        accum_bbox_ts = np.array(accum_bbox_ts, dtype=np.float32)

        inrange_filter = Detect2dAxisFaster.is_inrange(
            new2dcorrpoints[:, :, 0], margin, width - margin, 0b11
        ) & Detect2dAxisFaster.is_inrange(
            new2dcorrpoints[:, :, 1], margin, height - margin, 0b11
        )
        inrange_filter = inrange_filter[:, 0] & inrange_filter[:, 1]

        return new2dcorrpoints[inrange_filter], np.array(
            accum_bbox_ts[inrange_filter], dtype=np.int32
        )
