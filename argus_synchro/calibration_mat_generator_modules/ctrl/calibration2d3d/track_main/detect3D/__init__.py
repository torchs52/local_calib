# 点群対応点抽出

# 方向性として、このディレクトリ以外への依存性は極力減らす。
# privateメンバが使えないためmainから呼ばないものは名前空間で区切る。
import inspect  # debug

# 点群対応点抽出
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.bbox3D_postprocess import (
    bbox_postprocess,
    tupleBBoxset_to_dtypePreprocess3d,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.calc_headpoint_z import (
    calc_headpoint_z,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.ground_planecalc import (
    ground_planecalc,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.make_bbox3d import (
    make_BBox3D,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.person_tracker_SORT_3d import (
    proc3d_bboxtracker_recorder,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking3dDataInterface,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.target_selector import (
    find_target_trajectory,
    reshape_bboxlist,
)
from argus_synchro.calibration_mat_generator_modules.utils.filter_static_objects import (
    filter_static_objects,
)
from argus_synchro.calibration_mat_generator_modules.utils.utils3d import (
    Config_Proc3d_Datarange_loader,
    scale_transform,
)
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config_calibration import AppConfigCalibration
from argus_synchro.shared_app_config import SharedAppConfig


class detect3d_class:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        sac: SharedAppConfig,
        app_logger_factory: AppLoggerFactory,
        camera_index: int = 0,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        # 設定等
        self.app_config_calib = app_config_calib
        self.verbose = not self.app_config_calib.default.print_disabled
        self.datarange_xyz = Config_Proc3d_Datarange_loader(app_config_calib.calib2d3d)

        # 地面検出
        self.gplane_calc = ground_planecalc(
            app_config_calib=self.app_config_calib,
            app_logger_factory=app_logger_factory,
        )

        # 点群事前処理：静止点群除去
        self.static_point_filter = filter_static_objects(
            self.app_config_calib.calib2d3d.Proc3d.static_point_filter_boxelsize,
            *self.datarange_xyz,
        )

        # bbox追跡前事前処理
        self.bbox_preproc = bbox_postprocess(
            app_config_calib=app_config_calib,
            app_logger_factory=app_logger_factory,
        )

        # bbox追跡・記録
        self.tracking_recorder = proc3d_bboxtracker_recorder(
            sac=sac, app_config_calib=app_config_calib
        )

        # 点群抽出時のz座標補正
        grid_axis = app_config_calib.calib2d3d.Proc3d.calc_headpoint_grid_axis[
            camera_index
        ]
        self.calc_headpoint_z_inst = calc_headpoint_z(
            (
                app_config_calib.calib2d3d.Proc3d.calc_headpoint_pointrange_x_min,
                app_config_calib.calib2d3d.Proc3d.calc_headpoint_pointrange_x_max,
            ),
            (
                app_config_calib.calib2d3d.Proc3d.calc_headpoint_pointrange_y_min,
                app_config_calib.calib2d3d.Proc3d.calc_headpoint_pointrange_y_max,
            ),
            grid_axis=grid_axis,
        )

        self.reset()

    def reset(self):
        self.pointfilter_lastadd = -1
        self.tracking_recorder.reset()

        self.multi_points = None
        self.multi_lines = None
        self.multi_minmax = None

        self.ground_equation_coeff = None  # list()

    def _sub_detect_apply_static_point_filter(self, pcdframe, timestamp_pcd):
        if self.app_config_calib.calib2d3d.Proc3d.enable_static_point_filter:
            if (
                self.static_point_filter.filtersource_framecount
                <= self.app_config_calib.calib2d3d.Proc3d.static_point_filter_initlength
                or (timestamp_pcd - self.pointfilter_lastadd)
                > self.app_config_calib.calib2d3d.Proc3d.static_point_filter_refresh_period
            ):
                self.static_point_filter.add_single_voxel_map(frame=pcdframe)
                if (
                    self.static_point_filter.filtersource_framecount
                    >= self.app_config_calib.calib2d3d.Proc3d.static_point_filter_initlength
                ):
                    self.static_point_filter.apply_voxelfilter()
                if not self.app_config_calib.default.print_disabled:
                    self._logger.info(
                        f"pointfilter add, length: {self.static_point_filter.filtersource_framecount}",
                    )

            if (
                self.static_point_filter.filtersource_framecount
                >= self.app_config_calib.calib2d3d.Proc3d.static_point_filter_initlength
            ):
                pcdframe = self.static_point_filter.extract_moving_objects(pcdframe)
            else:
                self._logger.info("static_point_filter - stacking points")
        return pcdframe

    def detect(self, data3d):
        (pcdframe, timestamp_pcd) = data3d

        if self.gplane_calc.datreq():
            self.gplane_calc.stack(pcdframe[:, 0:3])

        # 点群事前処理
        pcdframe = self._sub_detect_apply_static_point_filter(
            pcdframe=pcdframe, timestamp_pcd=timestamp_pcd
        )

        # UI向け情報は辞書型にて受け渡し(表示内容からUIデータ用クラスで受け渡すべきではある)
        monitor_data = {}
        monitor_data["process3d_frame"] = {}

        if self.verbose:
            self._logger.info(f"{pcdframe.shape = }, {timestamp_pcd = }")

        if pcdframe.shape[0] < 10:
            return monitor_data  # 点群が薄いとbbox作れないので処理できないためearly return。2D画像等との同期はタイムスタンプで行うので問題ない

        # DBscan, BBox作成
        bbox_results = make_BBox3D(
            pcd=pcdframe,
            datarange_xyz=self.datarange_xyz,
            dbscan_eps=self.app_config_calib.calib2d3d.Proc3d.dbscan_eps,
            dbscan_min_samples=self.app_config_calib.calib2d3d.Proc3d.dbscan_min_samples,
        )

        # bbox追跡前事前処理
        bbox_results = self.bbox_preproc.apply(
            BBoxinfoset=tupleBBoxset_to_dtypePreprocess3d(bbox_results)
        )

        # UI向け情報を抽出
        self.multi_points, self.multi_lines, self.multi_minmax, pcd_limited = (
            bbox_results[0].multi_points,
            bbox_results[0].multi_lines,
            bbox_results[0].multi_minmax,
            bbox_results[1],
        )
        monitor_data["process3d_frame"]["limited_pts"] = pcd_limited
        monitor_data["process3d_frame"]["multi_points"] = self.multi_points
        monitor_data["process3d_frame"]["multi_lines"] = self.multi_lines
        monitor_data["process3d_frame"]["multi_minmax"] = self.multi_minmax

        # BBox追跡・記録
        bbox_results = self.tracking_recorder.update(
            bbox_multi_minmax=self.multi_minmax,
            frame_ix=timestamp_pcd,
            pcdframe=pcdframe,
        )

        return monitor_data

    def view_makedata(self, data3d, view_BB=True):
        (pcdframe, timestamp_pcd) = data3d
        pcddata_o3d = o3d.geometry.PointCloud()
        line_set = None
        if len(pcdframe) > 3:
            pcddata_o3d.points = o3d.utility.Vector3dVector(pcdframe[:, 0:3])
            if len(pcdframe[0]) == 4:
                cmap_plt = plt.get_cmap("bwr")
                pcdframe[:, 3] = np.where(
                    (pcdframe[:, 3] < 0), pcdframe[:, 3] + 256, pcdframe[:, 3]
                )
                color = cmap_plt(scale_transform(pcdframe[:, 3], val_min=0, val_max=1))
                pcddata_o3d.colors = o3d.utility.Vector3dVector(color[:, 0:3])

            if (
                view_BB
                and (self.multi_points is not None)
                and len(self.multi_points) > 0
            ):
                line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(self.multi_points),
                    lines=o3d.utility.Vector2iVector(self.multi_lines),
                )

        return pcddata_o3d, line_set

    def extract_rawbboxes(self, view_BB=True):
        if view_BB and (self.multi_points is not None) and len(self.multi_points) > 0:
            return self.multi_points, self.multi_lines
        return None, None

    def get_tracking_results(self) -> Tracking3dDataInterface:
        return self.tracking_recorder.get_rawresults()

    def get_target_bbox(
        self, tracker_result_interface: Tracking3dDataInterface, frame_ix: int
    ) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
        """
        使用bbox選択・取得
          追跡後フィルタ処理、校正作業者判定、bbox補正処理が終了した後のTracking2dDataInterface参照先データから
          bboxを選択、タイムスタンプと共に複数軌跡を統合して出力
        """
        idlist = find_target_trajectory(
            tracker_result_interface=tracker_result_interface, frame_index=frame_ix
        )

        return reshape_bboxlist(
            target_bboxhistory={
                key: data
                for key, data in tracker_result_interface.trackingIDbboxlog.items()
                if key in idlist
            }
        )

    def extract_fromcenter(
        self, tracker_result_interface: Tracking3dDataInterface, frame_ix: int
    ) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
        frame_corresp_point_ts_list = []
        timestamps = []

        for (
            framebb_points_list,
            ts_bbox,
            trackids_list,
            framebb_minmax,
        ) in self.tracking_recorder.get_data_array_fbb_point_history():
            framebb_index = -1
            for frame_ix, tracking_ix in enumerate(
                trackids_list
            ):  # この検索方法では初めにtrackids_listにあったtracking_ixのみがヒット。元々追跡対象のbboxは1フレームに1つしか存在しないためこれで良い
                if int(tracking_ix) in find_target_trajectory(
                    tracker_result_interface=tracker_result_interface,
                    frame_index=frame_ix,
                ):
                    framebb_index = frame_ix
            if framebb_index == -1:
                continue

            bbox = framebb_minmax[framebb_index]
            (x1L, x1H, y1L, y1H, z1L, z1H) = bbox
            bb_center = np.array([(x1L + x1H) / 2, (y1L + y1H) / 2, (z1L + z1H) / 2])

            pcd_np = framebb_points_list[framebb_index]
            if pcd_np.shape[0] == 0:
                continue

            # Vectorized filtering
            mask = (pcd_np[:, 1] < bb_center[1]) & (pcd_np[:, 2] < bb_center[2])
            pcdlimited = pcd_np[mask]

            if pcdlimited.shape[0] > 0:
                frame_corresp_point_ts_list.append(bb_center)
                timestamps.append(ts_bbox)

        (cornerlist3d, tslist3d) = frame_corresp_point_ts_list, timestamps
        if len(cornerlist3d) == 0:
            return (np.zeros(0), np.zeros(0, dtype=np.int32))
        cornerlist3d = np.array(cornerlist3d)
        tslist3d = np.array(tslist3d, dtype=np.int32)
        return (cornerlist3d, tslist3d)

    def extract(
        self, tracker_result_interface: Tracking3dDataInterface, frame_ix: int
    ) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
        range_xyz = None
        if self.app_config_calib.calib2d3d.Proc3d.gplane_detection_walkingarea_limit:
            range_xyz = self._internal_get_personwalking_area(
                tracker_result_interface=tracker_result_interface, frame_ix=frame_ix
            )

        if not self.app_config_calib.calib2d3d.Proc3d.is_footpoints_fixed:
            gplane_coeff = self.gplane_calc.calc_single(
                savefile_suffixstr="gplane_result",
                range_xyz=range_xyz,
                thinning_div=self.app_config_calib.calib2d3d.Proc3d.groundplane_ptthinning_div,
            )

        frame_corresp_point_ts_list = []
        timestamps = []

        for (
            framebb_points_list,
            ts_bbox,
            trackids_list,
            framebb_minmax,
        ) in self.tracking_recorder.get_data_array_fbb_point_history():
            framebb_index = -1
            for frame_ix, tracking_ix in enumerate(
                trackids_list
            ):  # この検索方法では初めにtrackids_listにあった
                if int(tracking_ix) in find_target_trajectory(
                    tracker_result_interface=tracker_result_interface,
                    frame_index=frame_ix,
                ):
                    framebb_index = frame_ix
            if framebb_index == -1:
                continue

            bbox = framebb_minmax[framebb_index]
            (x1L, x1H, y1L, y1H, z1L, z1H) = bbox
            bb_center = np.array([(x1L + x1H) / 2, (y1L + y1H) / 2, (z1L + z1H) / 2])

            pcd_np = framebb_points_list[framebb_index]
            if pcd_np.shape[0] == 0:
                continue

            # Vectorized filtering
            mask = (pcd_np[:, 1] < bb_center[1]) & (pcd_np[:, 2] < bb_center[2])
            pcdlimited = pcd_np[mask]

            if pcdlimited.shape[0] > 0:
                # Compute squared distances and argmin efficiently
                ref_point = np.array([bb_center[0], bbox[2], bbox[4]])
                diffs = pcdlimited - ref_point
                ix_foot = np.argmin(np.einsum("ij,ij->i", diffs, diffs))

                point_foot = pcdlimited[ix_foot]
                point_head = pcd_np[np.argmax(pcd_np[:, 2])]

                # Adjust foot point to ground plane
                pfx, pfy, pfz = point_foot

                if not self.app_config_calib.calib2d3d.Proc3d.is_footpoints_fixed:
                    pfz = (
                        -gplane_coeff[0] * pfx - gplane_coeff[1] * pfy - gplane_coeff[3]
                    ) / gplane_coeff[2]
                else:
                    pfz = self.app_config_calib.calib2d3d.Proc3d.footpoints_zval

                # frame_corresp_point_ts_list.append((point_head, np.array([pfx, pfy, pfz])))
                frame_corresp_point_ts_list.append(
                    (point_head, np.array([pfx, pfy, pfz]))
                )
                timestamps.append(ts_bbox)

        cornerlist3d = np.array(frame_corresp_point_ts_list)
        tslist3d = np.array(timestamps, dtype=np.int32)

        proc3d_conf = self.app_config_calib.calib2d3d.Proc3d
        if proc3d_conf.is_headpoint_overwrite and len(cornerlist3d) > 0:
            if proc3d_conf.is_headpoint_grid_overwrite:
                cornerlist3d[:, 0, 2] = self.calc_headpoint_z_inst.apply_per_point(
                    cornerlist3d
                )
            else:
                cornerlist3d[:, 0, 2] = self.calc_headpoint_z_inst.apply(
                    cornerlist3d
                )

        return (cornerlist3d, tslist3d)

    # =================  ここから旧tracker3d

    def _internal_get_personwalking_area(
        self, tracker_result_interface: Tracking3dDataInterface, frame_ix: int
    ) -> tuple[float, float, float, float, float, float] | None:
        x_min, x_max, y_min, y_max, z_min, z_max = None, -1, -1, -1, -1, -1
        # 該当トラックIDの全移動軌跡の点群から歩行履歴を取り出す
        for (
            framebb_points_list,
            ts,
            trackids_list,
            framebb_minmax,
        ) in self.tracking_recorder.get_data_array_fbb_point_history():
            for bb_points, trackid in zip(
                framebb_points_list, trackids_list, strict=False
            ):
                if trackid in find_target_trajectory(
                    tracker_result_interface=tracker_result_interface,
                    frame_index=frame_ix,
                ):
                    x_min_tmp, y_min_tmp, z_min_tmp = bb_points.min(axis=0)
                    x_max_tmp, y_max_tmp, z_max_tmp = bb_points.max(axis=0)

                    if x_min is None:
                        x_min, y_min, z_min = x_min_tmp, y_min_tmp, z_min_tmp
                        x_max, y_max, z_max = x_max_tmp, y_max_tmp, z_max_tmp
                    else:
                        x_min = min(x_min, x_min_tmp)
                        y_min = min(y_min, y_min_tmp)
                        z_min = min(z_min, z_min_tmp)
                        x_max = max(x_max, x_max_tmp)
                        y_max = max(y_max, y_max_tmp)
                        z_max = max(z_max, z_max_tmp)
        if x_min is not None:
            return (x_min, x_max, y_min, y_max, z_min, z_max)
        return None
