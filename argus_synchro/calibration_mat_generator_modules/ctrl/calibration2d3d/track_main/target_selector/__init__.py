import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
    dtype_tracking2dIDbboxlog,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory

# 他モジュールインポート（クラス、関数、型定義等）
from argus_synchro.config.app_config_calibration import AppConfigCalibration


def reshape_bboxlist(
    target_bboxhistory: dtype_tracking2dIDbboxlog,
) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
    points_list: list[tuple[float, float, float, float]] = []
    ts_list: list[int] = []
    for k, vtl in target_bboxhistory.items():
        for ts, pt in vtl:
            points_list.append(pt)
            ts_list.append(ts)

    points = np.array(points_list)
    timestamps = np.array(ts_list)
    return points, timestamps


def overlap_time(a_start: float, a_end: float, b_start: float, b_end: float) -> float:
    """Return overlapping duration between interval A and B."""
    start = max(a_start, b_start)
    end = min(a_end, b_end)
    return max(0.0, end - start)


# TODO: frame_indexに現在のフレームインデックスを入れて同期が取れるようにしたい
def find_alive_trajectory_after_filtering(
    tracker_result_interface: Tracking2dDataInterface | Tracking3dDataInterface,
    frame_index: int,
) -> list[int]:
    """
    2Dまたは3D追跡履歴からis_aliveフラグが立っているIDを抽出、リストに積んで返す
    """

    alive_idlist: list[int] = []
    for key, data in tracker_result_interface.trackingIDmetadata.items():
        if data.is_alive:
            alive_idlist.append(key)
    return alive_idlist


# TODO: frame_indexに現在のフレームインデックスを入れて同期が取れるようにしたい
def find_target_trajectory(
    tracker_result_interface: Tracking2dDataInterface | Tracking3dDataInterface,
    frame_index: int,
) -> list[int]:
    """
    2Dまたは3D追跡履歴からis_alive&is_tracking_targetフラグが立っているIDを抽出、リストに積んで返す
    """
    alive_idlist: list[int] = []
    for key, data in tracker_result_interface.trackingIDmetadata.items():
        if data.is_alive and data.is_tracking_target:
            alive_idlist.append(key)

    return alive_idlist


# セレクタクラス定義 2D・3Dそれぞれについて加工されたデータを収集、compare()にて軌跡同士の対応などを取りselect2d/select3dでbbox作成のための情報を出力する
class target_selector:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.app_config_calib = app_config_calib
        self.progress: float = 0

    @staticmethod
    def trackerresult_filter2d(
        tracker_result_interface: Tracking2dDataInterface,
        app_config_calib: AppConfigCalibration,
        frame_ix: int,
        verbose=False,
    ) -> Tracking2dDataInterface:
        """
        Docstring for trackerresult_filter2d_main

        :param tracker_result_interface: 追跡idごとの追跡情報セット ※フラグ書き換えあり
        :type tracker_result_interface: Tracking2dDataInterface
        :param app_config_calib: 校正設定
        :type app_config_calib: AppConfigCalibration
        :return: Description
        :rtype: dtype_tracking2dIDmetadata

        tracker_result_metadataについて、条件に満たないbboxのis_aliveをFalseにする。(後で復活する可能性あり) dict型の書き換えのため元データへの反映を期待しており返り値は使わなくて良い想定
        """

        # 時間長さゲート適用、ID足切り
        for key, data in tracker_result_interface.trackingIDmetadata.items():
            time_length = data.frame_ix_max - data.frame_ix_min
            movelen = data.accum_track_length
            workarea_count = data.workarea_count

            tracker_result_interface.trackingIDmetadata[
                key
            ].last_filter_applied_frame = frame_ix
            if (
                (
                    time_length
                    > app_config_calib.calib2d3d.Proc2d.bbox_tracking_framelen_min
                )
                & (
                    movelen
                    > app_config_calib.calib2d3d.Proc2d.bbox_tracking_movelen_pixels
                )
                & (
                    workarea_count
                    > app_config_calib.calib2d3d.Proc2d.bbox_tracking_workarea2d_count
                )
            ):
                tracker_result_interface.trackingIDmetadata[key].is_alive = True
            else:
                tracker_result_interface.trackingIDmetadata[key].is_alive = False
                if verbose:
                    print(f"** skipped {key}: {data}")

        return tracker_result_interface

    @staticmethod
    def trackerresult_filter3d(
        tracker_result_interface: Tracking3dDataInterface,
        app_config_calib: AppConfigCalibration,
        frame_ix: int,
        verbose=False,
    ) -> Tracking3dDataInterface:
        """
        Docstring for trackerresult_filter2d_main

        :param tracker_result_interface: 追跡idごとの追跡情報セット ※フラグ書き換えあり
        :type tracker_result_interface: Tracking3dDataInterface
        :param app_config_calib: 校正設定
        :type app_config_calib: AppConfigCalibration
        :return: Description
        :rtype: dtype_tracking2dIDmetadata

        tracker_result_metadataについて、条件に満たないbboxのis_aliveをFalseにする。(後で復活する可能性あり) dict型の書き換えのため元データへの反映を期待しており返り値は使わなくて良い想定
        """

        # 時間長さゲート適用、ID足切り
        for key, data in tracker_result_interface.trackingIDmetadata.items():
            time_length = data.frame_ix_max - data.frame_ix_min
            movelen = data.accum_track_length
            workarea_count = data.workarea_count

            tracker_result_interface.trackingIDmetadata[
                key
            ].last_filter_applied_frame = frame_ix
            if (
                (
                    time_length
                    > app_config_calib.calib2d3d.Proc3d.bbox_tracking_framelen_min
                )
                & (
                    movelen
                    > app_config_calib.calib2d3d.Proc3d.bbox_tracking_movelen_meters
                )
                & (
                    workarea_count
                    > app_config_calib.calib2d3d.Proc3d.bbox_tracking_workarea3d_count
                )
            ):
                tracker_result_interface.trackingIDmetadata[key].is_alive = True
            else:
                tracker_result_interface.trackingIDmetadata[key].is_alive = False
                if verbose:
                    print(f"** skipped {key}: {data}")

        return tracker_result_interface

    def set_progress(self, progress: float) -> None:
        self.progress = progress

    # 校正作業者判定
    def compare(
        self,
        tracker_result2d_interface: Tracking2dDataInterface,
        frame2d_index: int,
        tracker_result3d_interface: Tracking3dDataInterface,
        frame3d_index: int,
    ) -> tuple[Tracking2dDataInterface, Tracking3dDataInterface]:
        tracker_result2d_interface = self._compare_trackresult2d(
            tracker_result2d_interface=tracker_result2d_interface,
            frame2d_index=frame2d_index,
        )
        tracker_result3d_interface = self._compare_trackresult3d(
            tracker_result3d_interface=tracker_result3d_interface,
            frame3d_index=frame3d_index,
        )
        return tracker_result2d_interface, tracker_result3d_interface

    @staticmethod
    def getfunc_evaluation2d_val(
        tracker_result2d_interface: Tracking2dDataInterface, key: int
    ):
        return tracker_result2d_interface.trackingIDmetadata[key].frame_evval_min

    def _compare_trackresult2d(
        self,
        tracker_result2d_interface: Tracking2dDataInterface,
        frame2d_index: int,
    ) -> Tracking2dDataInterface:
        """
        校正作業者判定アルゴリズム（今後差し替え予定）
        tracker_result2d_interface.trackingIDmetadataに記録された追跡履歴のうち
        is_aliveフラグの立っているものから
        一番近くを通った軌跡を抽出
        """
        # 出現期間の重複する追跡履歴について一番近いものを選択(→一番近い物以外をflag=False化で排除)
        tracker_result_metadata = (
            tracker_result2d_interface.trackingIDmetadata
        )  # 必要なデータ取出し

        scan_idlist = find_alive_trajectory_after_filtering(
            tracker_result2d_interface, frame_index=frame2d_index
        )

        # 出現期間の重複する追跡履歴について一番近いものを選択(→一番近い物以外をflag=False化で排除)
        target_id_flags: dict[int, bool] = dict.fromkeys(scan_idlist, True)

        for key in scan_idlist:
            if target_id_flags[key] is False:
                continue

            start = tracker_result_metadata[key].frame_ix_min
            end = tracker_result_metadata[key].frame_ix_max

            evaluaton_val_min = self.getfunc_evaluation2d_val(
                tracker_result2d_interface, key
            )
            evaluaton_val_min_key = key
            scanned_key_list = [key]

            for key_scan in scan_idlist:
                if target_id_flags[key_scan] is False or key == key_scan:
                    continue
                if (
                    overlap_time(
                        start,
                        end,
                        tracker_result_metadata[key_scan].frame_ix_min,
                        tracker_result_metadata[key_scan].frame_ix_max,
                    )
                    > 0
                ):
                    scanned_key_list.append(key_scan)
                    if evaluaton_val_min > self.getfunc_evaluation2d_val(
                        tracker_result2d_interface, key_scan
                    ):
                        evaluaton_val_min = self.getfunc_evaluation2d_val(
                            tracker_result2d_interface, key_scan
                        )
                        evaluaton_val_min_key = key_scan
            if len(scanned_key_list) == 1:
                continue

            for key_scan in scanned_key_list:
                if key_scan == evaluaton_val_min_key:
                    continue
                target_id_flags[key_scan] = False

        for key, flag in target_id_flags.items():
            tracker_result2d_interface.trackingIDmetadata[key].is_tracking_target = flag

        return tracker_result2d_interface

    @staticmethod
    def getfunc_evaluation3d_val(
        tracker_result3d_interface: Tracking3dDataInterface, key: int
    ):
        return tracker_result3d_interface.trackingIDmetadata[key].frame_evval_min

    def _compare_trackresult3d(
        self,
        tracker_result3d_interface: Tracking3dDataInterface,
        frame3d_index: int,
    ) -> Tracking3dDataInterface:
        """
        校正作業者判定アルゴリズム（今後差し替え予定）
        tracker_result2d_interface.trackingIDmetadataに記録された追跡履歴のうち
        is_aliveフラグの立っているものから
        一番近くを通った軌跡を抽出
        """
        # 出現期間の重複する追跡履歴について一番近いものを選択(→一番近い物以外をflag=False化で排除)
        tracker_result_metadata = (
            tracker_result3d_interface.trackingIDmetadata
        )  # 必要なデータ取出し

        scan_idlist = find_alive_trajectory_after_filtering(
            tracker_result3d_interface, frame_index=frame3d_index
        )

        # 出現期間の重複する追跡履歴について一番近いものを選択(→一番近い物以外をflag=False化で排除)
        target_id_flags: dict[int, bool] = dict.fromkeys(scan_idlist, True)

        for key in scan_idlist:
            if target_id_flags[key] is False:
                continue
            """data: data.accum_track_length,
                data.frame_ix_min,
                data.frame_ix_max,
                data.xymax,"""

            start = tracker_result_metadata[key].frame_ix_min
            end = tracker_result_metadata[key].frame_ix_max
            scanned_key_list = [key]
            evaluaton_val_min = self.getfunc_evaluation3d_val(
                tracker_result3d_interface, key
            )
            evaluaton_val_min_key = key

            for key_scan in scan_idlist:
                if target_id_flags[key_scan] is False or key == key_scan:
                    continue
                if (
                    overlap_time(
                        start,
                        end,
                        tracker_result_metadata[key_scan].frame_ix_min,
                        tracker_result_metadata[key_scan].frame_ix_max,
                    )
                    > 0
                ):
                    scanned_key_list.append(key_scan)
                    if evaluaton_val_min > self.getfunc_evaluation3d_val(
                        tracker_result3d_interface, key_scan
                    ):
                        evaluaton_val_min = self.getfunc_evaluation3d_val(
                            tracker_result3d_interface, key_scan
                        )
                        evaluaton_val_min_key = key_scan
            if len(scanned_key_list) == 1:
                continue

            for key_scan in scanned_key_list:
                if key_scan == evaluaton_val_min_key:
                    continue
                target_id_flags[key_scan] = False

        for key, flag in target_id_flags.items():
            self._logger.info(f"{key=},{flag=}")
            tracker_result3d_interface.trackingIDmetadata[key].is_tracking_target = flag

        return tracker_result3d_interface

    # 今後差し替え予定
    def targetbbox2d_correction(
        self,
        tracker_result2d_interface: Tracking2dDataInterface,
        app_config_calib: AppConfigCalibration,
        frame_ix: int,
        verbose=False,
    ) -> Tracking2dDataInterface:
        target_idlist = find_target_trajectory(tracker_result2d_interface, frame_ix)
        for target_id in target_idlist:
            tracker_result2d_interface.trackingIDbboxlog[target_id] = (
                tracker_result2d_interface.trackingIDbboxlog[target_id]
            )

        return tracker_result2d_interface

    # 今後差し替え予定
    def targetbbox3d_correction(
        self,
        tracker_result3d_interface: Tracking3dDataInterface,
        app_config_calib: AppConfigCalibration,
        frame_ix: int,
        verbose=False,
    ) -> Tracking3dDataInterface:
        target_idlist = find_target_trajectory(tracker_result3d_interface, frame_ix)
        for target_id in target_idlist:
            tracker_result3d_interface.trackingIDbboxlog[target_id] = (
                tracker_result3d_interface.trackingIDbboxlog[target_id]
            )

        return tracker_result3d_interface
