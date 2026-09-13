# ブロック分割集計ファイル
from typing import Any

import numpy as np

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class block_summarize:
    def __init__(
        self,
        verbose: bool,
        app_logger_factory: AppLoggerFactory,
    ):
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.verbose = verbose

    def setup(self, xrange, yrange, zrange, sb_param, subblock_amount):
        # data_import関連
        self.xrange = xrange
        self.yrange = yrange
        self.zrange = zrange
        self.data = np.zeros((0, 3))
        # self.subblock_size = subblock_size
        self.subblock_amount = subblock_amount

        if self.verbose:
            self._logger.info(
                f"{self.xrange},\
                {self.yrange},\
                {self.zrange},\
                {self.data},\
                ",
            )

        # judge関連
        self.sb_param = sb_param
        if self.verbose:
            self._logger.info(f"setup - self.sb_param: {self.sb_param}")

    def data_import(self, data: np.ndarray):
        row_indices = np.arange(data.shape[0]).reshape(-1, 1)
        self.data = np.hstack((data, row_indices))[
            (self.xrange[0] <= data[:, 0])
            & (data[:, 0] < self.xrange[1])
            & (self.yrange[0] <= data[:, 1])
            & (data[:, 1] < self.yrange[1])
            & (self.zrange[0] <= data[:, 2])
            & (data[:, 2] < self.zrange[1])
        ]

    def _data_summarize(self):
        # 点数
        self.pcount = len(self.data)

        # 基本統計量
        self.stat = dict()

        for i, ax in enumerate(["x", "y", "z"]):
            self.stat[ax] = dict()
            # 平均・分散・最大値最小値
            if self.pcount > 0:
                self.stat[ax]["average"] = np.average(self.data[:, i])
                self.stat[ax]["std"] = np.std(self.data[:, i])
                self.stat[ax]["max"] = np.amax(self.data[:, i])
                self.stat[ax]["min"] = np.amin(self.data[:, i])
            else:
                self.stat[ax]["average"] = None
                self.stat[ax]["std"] = None
                self.stat[ax]["max"] = None
                self.stat[ax]["min"] = None
            # 第1～3 四分位数
            if self.pcount >= 3:
                quartile123 = np.percentile(self.data[:, i], [25, 50, 75])
                self.stat[ax]["quartile1"] = quartile123[0]
                self.stat[ax]["quartile2"] = quartile123[1]
                self.stat[ax]["quartile3"] = quartile123[2]
                self.stat[ax]["iqr"] = (
                    self.stat[ax]["quartile3"] - self.stat[ax]["quartile1"]
                )
                self.stat[ax]["ubound"] = self.stat[ax]["quartile3"] + (
                    self.stat[ax]["iqr"] * 1.5
                )
                self.stat[ax]["dbound"] = self.stat[ax]["quartile1"] - (
                    self.stat[ax]["iqr"] * 1.5
                )
            else:
                self.stat[ax]["quartile1"] = None
                self.stat[ax]["quartile2"] = None
                self.stat[ax]["quartile3"] = None
                self.stat[ax]["iqr"] = None
                self.stat[ax]["ubound"] = None
                self.stat[ax]["dbound"] = None

        # 移動距離
        if self.pcount >= 3:
            mvtimediff = self.data[1:, 3] - self.data[:-1, 3]
            mvlen_part = np.sqrt(
                np.sum((self.data[1:, :3] - self.data[:-1, :3]) ** 2, axis=1)
            )
            self.mvlen = np.sum(mvlen_part[mvtimediff <= 1])
        else:
            self.mvlen = 0

        self.subgrid_x = np.linspace(
            self.xrange[0], self.xrange[1], self.subblock_amount, endpoint=False
        )  # np.arange(self.xrange[0], self.xrange[1], self.subblock_size)
        subgrid_pitch = self.subgrid_x[1] - self.subgrid_x[0]
        self.subgrid_y = np.linspace(
            self.yrange[0], self.yrange[1], self.subblock_amount, endpoint=False
        )  # np.arange(self.yrange[0], self.yrange[1], self.subblock_size)
        self.subgrid_count = 0
        self.subgrid_counter_map = np.zeros((len(self.subgrid_x), len(self.subgrid_y)))
        self.debug_subgrid_satisfied_coordinates: list[
            tuple[float, float, float, float, int, int, bool, float]
        ] = []

        if self.verbose:
            self._logger.info(
                f"{self.subgrid_x = }, {self.subgrid_y = }, {self.subgrid_count = }, {self.subgrid_counter_map = }",
            )

        for ix, xmin in enumerate(self.subgrid_x):
            xmax = xmin + subgrid_pitch
            for iy, ymin in enumerate(self.subgrid_y):
                ymax = ymin + subgrid_pitch

                if any(
                    (xmin <= self.data[:, 0])
                    & (xmax > self.data[:, 0])
                    & (ymin <= self.data[:, 1])
                    & (ymax > self.data[:, 1])
                ):
                    self.subgrid_count += 1
                    self.subgrid_counter_map[ix, iy] += len(
                        self.data[
                            (xmin <= self.data[:, 0])
                            & (xmax > self.data[:, 0])
                            & (ymin <= self.data[:, 1])
                            & (ymax > self.data[:, 1])
                        ]
                    )
                    self.debug_subgrid_satisfied_coordinates.append(
                        (
                            xmin,
                            ymin,
                            xmax,
                            ymax,
                            ix,
                            iy,
                            True,
                            self.subgrid_counter_map[ix, iy],
                        )
                    )
                else:
                    self.debug_subgrid_satisfied_coordinates.append(
                        (
                            xmin,
                            ymin,
                            xmax,
                            ymax,
                            ix,
                            iy,
                            False,
                            self.subgrid_counter_map[ix, iy],
                        )
                    )

    def calc_progress(self) -> tuple[float, dict[str, Any]]:
        data_details: dict[str, Any] = {}
        self._data_summarize()
        score: float = 0.0
        if all([(self.stat[a]["iqr"] is not None) for a in ["x", "y", "z"]]):
            score += (
                max(0, min(1, self.subgrid_count / self.sb_param["sb_coeff"]))
                * self.sb_param["sb_weight"]
            )

        data_details["subgrid_count"] = self.subgrid_count
        data_details["sb_param_coeff"] = self.sb_param["sb_coeff"]
        data_details["sb_param_weight"] = self.sb_param["sb_weight"]
        data_details["subgrid_counter_map"] = self.subgrid_counter_map
        data_details["debug_subgrid_satisfied_coordinates"] = (
            self.debug_subgrid_satisfied_coordinates
        )

        return score, data_details

    # setting_params = { "std_param":{"weight":0.4,"thresh":3}, "iqr_param":{"weight":0.3,"thresh":1}, "mvl_param":{"weight":0.3,"thresh":1} }

    @staticmethod
    def calc_xyshape(xparams, yparams):
        """
        x軸とy軸の範囲に基づいて必要なサイズを計算 下記のような値を入れる（下記辞書はそのままnumpy.arange()で使用可能）
        grid_x_info = {"start":-9, "stop":9, "step":3}
        grid_y_info = {"start":0, "stop":9, "step":3}
        """
        xarr = np.linspace(
            xparams["start"],
            xparams["stop"] + xparams["step"],
            int(
                (xparams["stop"] + xparams["step"] - xparams["start"])
                / float(xparams["step"])
            ),
            endpoint=False,
        )  # np.arange(xparams["start"], xparams["stop"] + xparams["step"], xparams["step"])
        yarr = np.linspace(
            yparams["start"],
            yparams["stop"] + yparams["step"],
            int(
                (yparams["stop"] + yparams["step"] - yparams["start"])
                / float(yparams["step"])
            ),
            endpoint=False,
        )  # np.arange(yparams["start"], yparams["stop"] + yparams["step"], yparams["step"])
        x_size = len(xarr) - 1
        y_size = len(yarr) - 1

        x_center = (xarr[:-1] + xarr[1:]) / 2
        y_center = (yarr[:-1] + yarr[1:]) / 2

        min_x2c = {}
        minx2i = {}
        for i, xmin in enumerate(xarr[:-1]):
            min_x2c[xmin] = x_center[i]
            minx2i[xmin] = i
        min_y2c = {}
        miny2i = {}
        for i, ymin in enumerate(yarr[:-1]):
            min_y2c[ymin] = y_center[i]
            miny2i[ymin] = i

        return (
            (x_size, y_size),
            (x_center, y_center),
            (min_x2c, min_y2c),
            (minx2i, miny2i),
        )
        # 本体側で　(x_size, y_size), (x_center, y_center), (min_x2c, min_y2c), (minx2i, miny2i) = calc_xyshape(grid_x_info, grid_y_info)　のように受け取る
