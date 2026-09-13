from enum import Enum
from typing import Literal

import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class T_SyncType(Enum):
    NotDefined = 0
    ByIndex = 1
    ByTime_Nearby = 2


class sensor_sync_filter:
    def __init__(
        self,
        synctype_select: str,
        app_logger_factory: AppLoggerFactory,
    ):
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        # TODO: 時刻タイムスタンプの場合はずれの許容量を定義
        self.synctype = T_SyncType.NotDefined
        if synctype_select == "ByIndex":
            self.synctype = T_SyncType.ByIndex
        elif synctype_select == "ByTime_Nearby":
            self.synctype = T_SyncType.ByTime_Nearby
        else:
            raise NotImplementedError(
                f"class sensor_sync, __init__: synctype_select:{str} is not inplemented!"
            )

    def datasync_filtering(
        self, datalist: list, verbose: bool = False
    ) -> tuple[bool, NDArray[np.float64], float]:
        # datalist: (data,timestamp_ix, timestamp_time) の形を想定 返り値は(全マッチT/F, 各マッチ状況, 評価値)
        # 全マッチT/F: 各要素タイムスタンプが「マッチ」した時にTrue ずれが生じている場合はFalse
        # 各マッチ状況: どのデータが遅れているかint配列で示す
        #       インデックス同期： <0で遅い 数値は他との差分 最大で0（→遅いものを読み飛ばす）
        #       時刻同期（予定）：　>0で早い、<0で遅い。　遅いものを読み飛ばす
        # 評価値: 時刻の最大差分等？現状0固定・予約
        assert type(datalist) is list

        if self.synctype == T_SyncType.ByIndex:
            return self._datasync_filtering_byindex(
                datalist, indexval_pos=1, verbose=verbose
            )
        if self.synctype == T_SyncType.ByTime_Nearby:
            return self._datasync_filtering_bytime(
                datalist, indexval_pos=2, verbose=verbose
            )
        raise NotImplementedError("self.synctype != T_SyncType.ByIndex")

    def _datasync_filtering_byindex(
        self, datalist: list, indexval_pos: int, verbose: bool = False
    ) -> tuple[bool, NDArray[np.float64], Literal[0]]:
        dummyvalue = -10000
        eachdata_timestamp_ix: NDArray[np.int32] = np.array(
            [
                data[indexval_pos] if data is not None else dummyvalue
                for data in datalist
            ],
            dtype=np.int32,
        )

        if verbose:
            self._logger.info(
                f"datalist sync (by index), data: {[data[indexval_pos] if data is not None else 'None' for data in datalist]}",
            )

        values, counts = np.unique(eachdata_timestamp_ix, return_counts=True)
        mode = values[counts.argmax()]

        whole_equal = all(eachdata_timestamp_ix == mode)
        diff: NDArray[np.int32] = eachdata_timestamp_ix - eachdata_timestamp_ix.max()
        evaluate_value = 0

        return (whole_equal, diff.astype(np.float64), evaluate_value)

    def _datasync_filtering_bytime(
        self, datalist: list, indexval_pos: int = 2, verbose: bool = False
    ) -> tuple[bool, NDArray[np.float64], float]:
        np.set_printoptions(suppress=True)  # 指数表記を抑制
        np.set_printoptions(precision=6)  # 小数点以下6桁

        dummyvalue: np.float64 = -10000.0
        eachdata_timestamp_time: NDArray[np.float64] = np.array(
            [
                data[indexval_pos] if data is not None else dummyvalue
                for data in datalist
            ],
            dtype=np.float64,
        )

        # 有効なデータのみ抽出
        valid_times: NDArray[np.float64] = eachdata_timestamp_time[
            eachdata_timestamp_time != dummyvalue
        ]

        if len(valid_times) == 0:
            return (
                False,
                np.array([dummyvalue] * len(datalist), dtype=np.float64),
                float("inf"),
            )

        # 最も遅い（最大）時刻に合わせる
        max_time: np.float64 = valid_times.max()
        diff = eachdata_timestamp_time - max_time

        # 全ての差分が0に近ければ同期成功と判定（±0.2秒以内）
        tolerance = 0.2  # ±200ms以内なら同期とみなす (フレーム間引きを考慮)
        whole_equal: bool = np.all(np.abs(diff) <= tolerance)

        diff[np.abs(diff) <= tolerance] = 0  # 許容範囲内は差ゼロとする

        # 評価値として最大差分を返す
        evaluate_value = np.abs(diff).max()

        if verbose:
            self._logger.info(
                f"_datasync_filtering_bytime: {[str(x) for x in eachdata_timestamp_time]}, {max_time = }, {diff = }, {whole_equal = }, {evaluate_value = }",
            )

        return whole_equal, diff, evaluate_value
