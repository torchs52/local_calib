import sys

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect2D.person_tracker_SORT_2d import (
    bbox2d_mot_tracker_wrapper,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.detect3D.person_tracker_SORT_3d import (
    bbox3d_mot_tracker_wrapper,
)
from argus_synchro.calibration_mat_generator_modules.ctrl.calibration2d3d.track_main.interface_definition import (
    Tracking2dDataInterface,
    Tracking3dDataInterface,
    dtype_tracking2dIDbboxlog,
    dtype_tracking2dIDmetadata,
    dtype_tracking3dIDbboxlog,
    dtype_tracking3dIDmetadata,
    tracking2d_dataclass,
    tracking3d_dataclass,
)
from argus_synchro.calibration_mat_generator_modules.utils.GrayImageLUT import (
    GrayImageLUT,
)
from argus_synchro.calibration_mat_generator_modules.utils.NumpyMatrixLUT import (
    NumpyMatrixLUT,
)
from argus_synchro.config.app_config_calibration import AppConfigCalibration


class calibcheck2d_bboxtracker_recorder:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        image_size_hw: tuple[int, int],
        camera_index: int,
    ) -> None:
        proc2d_conf = app_config_calib.calib2d3d.Proc2d
        self.mot_tracker = bbox2d_mot_tracker_wrapper(
            lost_track_buffer=int(proc2d_conf.lost_track_buffer),
            frame_rate=proc2d_conf.tracking_frame_rate,
            track_activation_threshold=proc2d_conf.track_activation_threshold,
            minimum_consecutive_frames=int(proc2d_conf.minimum_consecutive_frames),
            minimum_iou_threshold=proc2d_conf.minimum_iou_threshold,
        )
        self.image_size_hw = image_size_hw
        self.trackingID_data: dtype_tracking2dIDmetadata = {}
        self.trackingID_bboxlog: dtype_tracking2dIDbboxlog = {}
        self.last_tracker_result: NDArray | None = None
        self.lastframe_person_detected = False
        self.evLUT2D = NumpyMatrixLUT(
            A_X=proc2d_conf.cam_valmat_coord_A_X[camera_index],
            B_X=proc2d_conf.cam_valmat_coord_B_X[camera_index],
            A_Y=proc2d_conf.cam_valmat_coord_A_Y[camera_index],
            B_Y=proc2d_conf.cam_valmat_coord_B_Y[camera_index],
            ARRAY_PATH=proc2d_conf.cam_valmat_path[camera_index],
            DEFAULT_VALUE=proc2d_conf.cam_valmat_val_DEFAULT[camera_index],
        )
        self.evLUT2D_workarea = GrayImageLUT(
            A_X=proc2d_conf.cam_workareadef_img_coord_A_X[camera_index],
            B_X=proc2d_conf.cam_workareadef_img_coord_B_X[camera_index],
            A_Y=proc2d_conf.cam_workareadef_img_coord_A_Y[camera_index],
            B_Y=proc2d_conf.cam_workareadef_img_coord_B_Y[camera_index],
            IMAGE_PATH=proc2d_conf.cam_workareadef_img_path[camera_index],
            A_ETA=proc2d_conf.cam_workareadef_img_coord_A_ETA[camera_index],
            B_ETA=proc2d_conf.cam_workareadef_img_coord_B_ETA[camera_index],
            DEFAULT_VALUE=proc2d_conf.cam_workareadef_img_val_DEFAULT[camera_index],
        )

    def reset(self) -> None:
        self.mot_tracker.reset()
        self.last_tracker_result = None
        self.lastframe_person_detected = False

    @staticmethod
    def _calc_L2norm(a: tuple[float, float], b: tuple[float, float]) -> float:
        return float(np.sqrt(np.sum((np.array(a) - np.array(b)) ** 2)))

    def _update_trackinfo(self, frame_ix: int) -> None:
        if self.last_tracker_result is None:
            return

        for res in self.last_tracker_result:
            if len(res) == 6:
                x1, y1, x2, y2, _prob, tracker_id = res
            else:
                return

            xc = (x1 + x2) / 2
            yc = (y1 + y2) / 2
            xymin = min(x1, x2), min(y1, y2)
            xymax = max(x1, x2), max(y1, y2)
            if tracker_id >= 0:
                is_workarea = 1 if self.evLUT2D_workarea.evaluate(xc, yc) else 0
                frame_evval = self.evLUT2D.evaluate(xc, yc)
                if tracker_id not in self.trackingID_data:
                    self.trackingID_data[tracker_id] = tracking2d_dataclass(
                        accum_track_length=0,
                        final_xy=(xc, yc),
                        xymin=xymin,
                        xymax=xymax,
                        frame_ix_min=frame_ix,
                        frame_ix_max=frame_ix,
                        frame_ix_lastmove=frame_ix,
                        frame_evval_min=frame_evval,
                        frame_evval_max=frame_evval,
                        workarea_count=is_workarea,
                        is_alive=True,
                        is_tracking_target=True,
                    )
                    self.trackingID_bboxlog[tracker_id] = []
                else:
                    metadata = self.trackingID_data[tracker_id]
                    metadata.xymin = (
                        min(metadata.xymin[0], xymin[0]),
                        min(metadata.xymin[1], xymin[1]),
                    )
                    metadata.xymax = (
                        max(metadata.xymax[0], xymax[0]),
                        max(metadata.xymax[1], xymax[1]),
                    )
                    frame_movelen = self._calc_L2norm(metadata.final_xy, (xc, yc))
                    metadata.accum_track_length += frame_movelen
                    metadata.final_xy = (xc, yc)
                    metadata.frame_ix_max = frame_ix
                    if frame_movelen > 1e-6:
                        metadata.frame_ix_lastmove = frame_ix
                    metadata.frame_evval_min = min(
                        metadata.frame_evval_min, frame_evval
                    )
                    metadata.frame_evval_max = max(
                        metadata.frame_evval_max, frame_evval
                    )
                    metadata.workarea_count += is_workarea
                self.trackingID_bboxlog[tracker_id].append((frame_ix, (x1, y1, x2, y2)))

    def print_trackinfo(self, file=sys.stdout) -> None:
        print(f"追跡結果 len: {len(self.trackingID_data)}", file=file)
        for k, d in self.trackingID_data.items():
            print(
                f"key {k} : {d.__dict__}, points:{self.trackingID_bboxlog[k]}",
                file=file,
            )

    def get_rawresults(self) -> Tracking2dDataInterface:
        return Tracking2dDataInterface(self.trackingID_data, self.trackingID_bboxlog)

    def update(self, yoloresult_whole: list[NDArray], frame_ix: int) -> None:
        self.last_tracker_result, self.lastframe_person_detected = (
            self.mot_tracker.update(
                yoloresult_whole=yoloresult_whole,
                image_w=self.image_size_hw[1],
                image_h=self.image_size_hw[0],
                frame_ix=frame_ix,
            )
        )
        self._update_trackinfo(frame_ix=frame_ix)

    def is_person_detected(self) -> bool:
        return self.lastframe_person_detected

    @staticmethod
    def draw_bboxes(frame, YOLOsingleresult_conv):
        x1, x2, y1, y2, _cls_id, _ = np.array(YOLOsingleresult_conv, dtype=np.int32)
        prob = YOLOsingleresult_conv[5]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
        bbox_text = f"person: {prob:.1%}"
        text_size = cv2.getTextSize(bbox_text, 0, 1, 1)[0]
        cv2.rectangle(
            frame,
            (x1, y1),
            (x1 + text_size[0], y1 - text_size[1]),
            (255, 255, 0),
            -1,
        )
        cv2.putText(
            frame,
            bbox_text,
            (x1, y1),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            1,
            lineType=cv2.LINE_AA,
        )
        return frame

    @staticmethod
    def draw_detection(
        yoloresult_whole: list[NDArray],
        frame: NDArray[np.uint8],
        image_size_hw: tuple[int, int],
        file=sys.stdout,
    ):
        del file
        if yoloresult_whole is None:
            return None
        for result_ix in range(len(yoloresult_whole[0])):
            coor = yoloresult_whole[0][result_ix]
            prob = yoloresult_whole[1][result_ix]
            bbox_ymin = coor[0] * image_size_hw[0]
            bbox_ymax = coor[2] * image_size_hw[0]
            bbox_xmin = coor[1] * image_size_hw[1]
            bbox_xmax = coor[3] * image_size_hw[1]
            if prob > 0:
                calibcheck2d_bboxtracker_recorder.draw_bboxes(
                    frame=frame,
                    YOLOsingleresult_conv=np.array(
                        [
                            float(bbox_xmin),
                            float(bbox_xmax),
                            float(bbox_ymin),
                            float(bbox_ymax),
                            0,
                            float(prob),
                        ]
                    ),
                )
        return frame

    def draw_mot(self, frame_sortview: NDArray[np.uint8]) -> NDArray[np.uint8]:
        if self.last_tracker_result is None:
            return frame_sortview
        return self.mot_tracker.draw(
            frame_sortview=frame_sortview, tracks=self.last_tracker_result
        )


class calibcheck3d_bboxtracker_recorder:
    def __init__(
        self,
        app_config_calib: AppConfigCalibration,
        camera_index: int,
    ) -> None:
        proc3d_conf = app_config_calib.calib2d3d.Proc3d
        self.mot_tracker = bbox3d_mot_tracker_wrapper(
            lost_track_buffer=int(proc3d_conf.lost_track_buffer),
            frame_rate=proc3d_conf.tracking_frame_rate,
            track_activation_threshold=proc3d_conf.track_activation_threshold,
            minimum_consecutive_frames=int(proc3d_conf.minimum_consecutive_frames),
            minimum_iou_threshold=proc3d_conf.minimum_iou_threshold,
            enable_bbox3d_overlap_merge=proc3d_conf.enable_bbox3d_overlap_merge,
            bbox3d_overlap_merge_threshold=proc3d_conf.bbox3d_overlap_merge_threshold,
        )
        self.trackingID_data: dtype_tracking3dIDmetadata = {}
        self.trackingID_bboxlog: dtype_tracking3dIDbboxlog = {}
        self.last_tracker_result: NDArray | None = None
        self.lastframe_person_detected = False
        self.data_array_fbb_point_history: list[
            tuple[list[NDArray[np.float64]], int, list[float], NDArray[np.float64]]
        ] = []
        self.evLUT3D = NumpyMatrixLUT(
            A_X=proc3d_conf.lid_valmat_coord_A_X[camera_index],
            B_X=proc3d_conf.lid_valmat_coord_B_X[camera_index],
            A_Y=proc3d_conf.lid_valmat_coord_A_Y[camera_index],
            B_Y=proc3d_conf.lid_valmat_coord_B_Y[camera_index],
            ARRAY_PATH=proc3d_conf.lid_valmat_path[camera_index],
            DEFAULT_VALUE=proc3d_conf.lid_valmat_val_DEFAULT[camera_index],
        )
        self.evLUT3D_workarea = GrayImageLUT(
            A_X=proc3d_conf.lid_workareadef_img_coord_A_X[camera_index],
            B_X=proc3d_conf.lid_workareadef_img_coord_B_X[camera_index],
            A_Y=proc3d_conf.lid_workareadef_img_coord_A_Y[camera_index],
            B_Y=proc3d_conf.lid_workareadef_img_coord_B_Y[camera_index],
            IMAGE_PATH=proc3d_conf.lid_workareadef_img_path[camera_index],
            A_ETA=proc3d_conf.lid_workareadef_img_coord_A_ETA[camera_index],
            B_ETA=proc3d_conf.lid_workareadef_img_coord_B_ETA[camera_index],
            DEFAULT_VALUE=proc3d_conf.lid_workareadef_img_val_DEFAULT[camera_index],
        )

    def reset(self) -> None:
        self.mot_tracker.reset()
        self.last_tracker_result = None
        self.lastframe_person_detected = False
        self.data_array_fbb_point_history = []

    @staticmethod
    def _calc_L2norm(a: tuple[float, float], b: tuple[float, float]) -> float:
        return float(np.linalg.norm(np.array(a) - np.array(b)))

    def _update_trackinfo(self, frame_ix: int) -> None:
        if self.last_tracker_result is None:
            return
        for result in self.last_tracker_result:
            if len(result) != 6:
                return
            x1, y1, x2, y2, _prob, tracker_id = result
            if tracker_id < 0:
                continue
            xc = (x1 + x2) / 2
            yc = (y1 + y2) / 2
            distance = float(np.hypot(xc, yc))
            is_workarea = 1 if self.evLUT3D_workarea.evaluate(xc, yc) > 0 else 0
            frame_evval = self.evLUT3D.evaluate(xc, yc)
            if tracker_id not in self.trackingID_data:
                self.trackingID_data[tracker_id] = tracking3d_dataclass(
                    accum_track_length=0,
                    final_xy=(xc, yc),
                    dist_from_camera_min=distance,
                    dist_from_camera_max=distance,
                    frame_ix_min=frame_ix,
                    frame_ix_max=frame_ix,
                    frame_ix_lastmove=frame_ix,
                    frame_evval_min=frame_evval,
                    frame_evval_max=frame_evval,
                    workarea_count=is_workarea,
                    is_alive=True,
                    is_tracking_target=True,
                )
                self.trackingID_bboxlog[tracker_id] = []
            else:
                metadata = self.trackingID_data[tracker_id]
                frame_movelen = self._calc_L2norm(metadata.final_xy, (xc, yc))
                metadata.accum_track_length += frame_movelen
                metadata.final_xy = (xc, yc)
                metadata.frame_ix_max = frame_ix
                if frame_movelen > 1e-6:
                    metadata.frame_ix_lastmove = frame_ix
                metadata.workarea_count += is_workarea
                metadata.dist_from_camera_min = min(metadata.dist_from_camera_min, distance)
                metadata.dist_from_camera_max = max(metadata.dist_from_camera_max, distance)
                metadata.frame_evval_min = min(metadata.frame_evval_min, frame_evval)
                metadata.frame_evval_max = max(metadata.frame_evval_max, frame_evval)
            self.trackingID_bboxlog[tracker_id].append((frame_ix, (x1, y1, x2, y2)))

    def print_trackinfo(self, file=sys.stdout) -> None:
        print(f"追跡結果 len: {len(self.trackingID_data)}", file=file)
        for k, d in self.trackingID_data.items():
            print(
                f"key {k} : {d.__dict__}, points:{self.trackingID_bboxlog[k]}",
                file=file,
            )

    def get_rawresults(self) -> Tracking3dDataInterface:
        return Tracking3dDataInterface(
            trackingIDmetadata=self.trackingID_data,
            trackingIDbboxlog=self.trackingID_bboxlog,
        )

    def update(self, bbox_multi_minmax: NDArray, frame_ix: int) -> None:
        self.last_tracker_result = self.mot_tracker.update(
            bbox_multi_minmax=bbox_multi_minmax
        )
        merged_bbox = self.mot_tracker.last_bbox_multi_minmax
        assert merged_bbox is not None
        self._update_trackinfo(frame_ix=frame_ix)
        self.data_array_fbb_point_history.append(
            (
                [np.zeros((0, 3), dtype=np.float32)],
                frame_ix,
                list(self.last_tracker_result[:, 5]),
                merged_bbox.copy(),
            )
        )

    def is_person_detected(self) -> bool:
        return self.lastframe_person_detected

    def draw_mot(self, frame_sortview: NDArray[np.uint8]) -> NDArray[np.uint8]:
        if self.last_tracker_result is None:
            return frame_sortview
        return self.mot_tracker.draw(
            frame_sortview=frame_sortview, tracks=self.last_tracker_result
        )

    def get_data_array_fbb_point_history(
        self,
    ) -> list[tuple[list[NDArray[np.float64]], int, list[float], NDArray[np.float64]]]:
        return self.data_array_fbb_point_history