from configparser import ConfigParser
from dataclasses import dataclass, field
from pathlib import Path

# iniファイル読み込み時に引数リストを使って項目を置き換えるクラス
from argus_synchro.common import paths
from argus_synchro.common.calib_settings_validation import validate_calib_settings
from argus_synchro.common.paths import (
    get_path_list,
    normalize_path,
    resolve_ini_roots,
)
from argus_synchro.config.app_config import parse_float_list
from argus_synchro.config.ini_read_and_replace import ini_read_and_replace


def _path(value: str) -> str:
    return str(normalize_path(value, Path.cwd()))


def _path_list(text: str) -> list[str]:
    return get_path_list(text, Path.cwd())


def _optional_float_list(text: str) -> list[float]:
    return parse_float_list(text) if text.strip() else []


def _parse_xy_quadrilateral(text: str) -> tuple[tuple[float, float], ...]:
    """``[x0,y0,...,x3,y3]``を内部処理用の4点へ変換する。"""

    values = parse_float_list(text)
    expected_coordinate_count = 8
    if len(values) != expected_coordinate_count:
        raise ValueError(
            "walking area must contain eight values: "
            "[x0,y0,x1,y1,x2,y2,x3,y3]"
        )
    return tuple(
        (values[index], values[index + 1])
        for index in range(0, expected_coordinate_count, 2)
    )


# 全体
@dataclass(frozen=True)
class DefaultConf:
    home_dir: str
    File_Input: bool
    data_dir: str
    print_disabled: bool
    outputdir_root: str
    z_height: float
    z_height_withmargin: float


def DefaultConf_read(
    ini: ConfigParser,
) -> DefaultConf:
    return DefaultConf(
        home_dir=str(ini.get("DEFAULT", "home_dir")),
        File_Input=ini.getboolean("DEFAULT", "File_Input"),
        data_dir=str(ini.get("DEFAULT", "data_dir")),
        print_disabled=ini.getboolean("DEFAULT", "print_disabled"),
        outputdir_root=str(ini.get("DEFAULT", "outputdir_root")),
        z_height=ini.getfloat("DEFAULT", "z_height"),
        z_height_withmargin=ini.getfloat("DEFAULT", "z_height_withmargin"),
    )


# デバッグ関連フラグ※内容検討中
@dataclass(frozen=True)
class DebugConf:
    calib2d3d_fileend_autoexit: bool
    calib2d3d_fileend_autoexit_flagfile_path: str
    is_enable_profiler: bool
    save_cornerlist_pickle: bool
    enable_debugdata_store: bool


def DebugConf_read(
    ini: ConfigParser,
) -> DebugConf:
    return DebugConf(
        calib2d3d_fileend_autoexit=ini.getboolean(
            "debug", "calib2d3d_fileend_autoexit"
        ),
        calib2d3d_fileend_autoexit_flagfile_path=_path(
            ini.get("debug", "calib2d3d_fileend_autoexit_flagfile_path"),
        ),
        is_enable_profiler=ini.getboolean("debug", "is_enable_profiler"),
        save_cornerlist_pickle=ini.getboolean("debug", "save_cornerlist_pickle"),
        enable_debugdata_store=ini.getboolean("debug", "enable_debugdata_store"),
    )


@dataclass(frozen=True)
class Filepath_IOConf:
    Calib3d3dmat_lidars: list[str]
    Calib2d3dmat_cameras: list[str]


def Filepath_IOConf_read(
    ini: ConfigParser,
) -> Filepath_IOConf:
    return Filepath_IOConf(
        Calib3d3dmat_lidars=_path_list(ini.get("Filepath_IO", "Calib3d3dmat_lidars")),
        Calib2d3dmat_cameras=_path_list(ini.get("Filepath_IO", "Calib2d3dmat_cameras")),
    )


@dataclass(frozen=True)
class GeneralConf:
    initial_transform_file: str


def GeneralConf_read(ini: ConfigParser):
    return GeneralConf(
        initial_transform_file=_path(ini.get("General", "initial_transform_file")),
    )


@dataclass(frozen=True)
class DataCaptureConf:
    s_frame: int
    e_frame: int
    sync_type: str
    datawait_sec: float
    save_sensordata: bool
    save_sensordata_dir: str

    @dataclass(frozen=True)
    class CanConf:
        is_fixed_yaw: bool
        c_file: str
        fixed_yaw_deg: float

    Can: CanConf

    @dataclass(frozen=True)
    class LidarConf:
        count: int
        path: str
        config_file: str
        accum_time: float
        dev_str: str
        # lidar_files: list[str]
        lidar_files_for_cam0calib: list[str]
        lidar_files_for_cam1calib: list[str]
        lidar_files_for_cam2calib: list[str]
        lidar_files_for_othermode: list[str]
        lidarfile_basetime: str
        lidarfile_steptime: float
        data_buffersize: int
        framethinning_bufferlen_threshold: int
        allow_lack: bool
        capture_latency_ms: float
        capturerange_x_min: float
        capturerange_x_max: float
        capturerange_y_min: float
        capturerange_y_max: float
        capturerange_z_min: float
        capturerange_z_max: float

    Lidar: LidarConf

    @dataclass(frozen=True)
    class CameraConf:
        # both mode
        count: int
        config_file: str
        data_buffersize: int
        framethinning_bufferlen_threshold: int
        # カメラ解像度
        video_width: int
        video_height: int
        # 校正処理内解像度（Generalの方が適切かもしれない）
        sys_width: int
        sys_height: int

        # file input mode
        # video_files: list[str]
        video_files_for_cam0calib: list[str]
        video_files_for_cam1calib: list[str]
        video_files_for_cam2calib: list[str]
        video_files_for_othermode: list[str]
        capture_latency_ms: float
        videofile_basetime: str
        videofile_steptime: float

        # realtime
        MOTEC: bool
        porttable: list[str]
        iptable: list[str]
        framerate_div: int

    Camera: CameraConf


def DataCaptureConf_read(
    ini: ConfigParser,
):
    return DataCaptureConf(
        s_frame=ini.getint("DataCapture", "s_frame"),
        e_frame=ini.getint("DataCapture", "e_frame"),
        sync_type=ini.get("DataCapture", "sync_type"),
        datawait_sec=ini.getfloat("DataCapture", "datawait_sec"),
        save_sensordata=ini.getboolean("DataCapture", "save_sensordata"),
        save_sensordata_dir=_path(
            ini.get("DataCapture", "save_sensordata_dir"),
        ),
        Can=DataCaptureConf.CanConf(
            is_fixed_yaw=ini.getboolean("DataCapture_CAN", "is_fixed_yaw"),
            c_file=_path(ini.get("DataCapture_CAN", "c_file")),
            fixed_yaw_deg=ini.getfloat("DataCapture_CAN", "fixed_yaw_deg"),
        ),
        Lidar=DataCaptureConf.LidarConf(
            count=ini.getint("DataCapture_Lidar", "count"),
            path=_path(ini.get("DataCapture_Lidar", "path")),
            config_file=_path(ini.get("DataCapture_Lidar", "config_file")),
            accum_time=ini.getfloat("DataCapture_Lidar", "accum_time"),
            dev_str=ini.get("DataCapture_Lidar", "dev_str"),
            # dev_str = ini.get("DataCapture_Lidar", "dev_str"),
            # date_str = ini.get("DataCapture_Lidar", "date_str"),
            # lidar_files=parse_list(ini.get("DataCapture_Lidar", "lidar_files")),
            lidar_files_for_cam0calib=_path_list(
                ini.get("DataCapture_Lidar", "lidar_files_for_cam0calib")
            ),
            lidar_files_for_cam1calib=_path_list(
                ini.get("DataCapture_Lidar", "lidar_files_for_cam1calib")
            ),
            lidar_files_for_cam2calib=_path_list(
                ini.get("DataCapture_Lidar", "lidar_files_for_cam2calib")
            ),
            lidar_files_for_othermode=_path_list(
                ini.get("DataCapture_Lidar", "lidar_files_for_othermode")
            ),
            lidarfile_basetime=ini.get("DataCapture_Lidar", "lidarfile_basetime"),
            lidarfile_steptime=ini.getfloat("DataCapture_Lidar", "lidarfile_steptime"),
            data_buffersize=ini.getint("DataCapture_Lidar", "data_buffersize"),
            framethinning_bufferlen_threshold=ini.getint(
                "DataCapture_Lidar", "framethinning_bufferlen_threshold"
            ),
            allow_lack=ini.getboolean("DataCapture_Lidar", "allow_lack"),
            capture_latency_ms=ini.getfloat("DataCapture_Lidar", "capture_latency_ms"),
            capturerange_x_max=ini.getfloat("DataCapture_Lidar", "capturerange_x_max"),
            capturerange_x_min=ini.getfloat("DataCapture_Lidar", "capturerange_x_min"),
            capturerange_y_max=ini.getfloat("DataCapture_Lidar", "capturerange_y_max"),
            capturerange_y_min=ini.getfloat("DataCapture_Lidar", "capturerange_y_min"),
            capturerange_z_max=ini.getfloat("DataCapture_Lidar", "capturerange_z_max"),
            capturerange_z_min=ini.getfloat("DataCapture_Lidar", "capturerange_z_min"),
        ),
        Camera=DataCaptureConf.CameraConf(
            count=ini.getint("DataCapture_Camera", "count"),
            config_file=_path(ini.get("DataCapture_Camera", "config_file")),
            data_buffersize=ini.getint("DataCapture_Camera", "data_buffersize"),
            framethinning_bufferlen_threshold=ini.getint(
                "DataCapture_Camera", "framethinning_bufferlen_threshold"
            ),
            video_width=ini.getint("DataCapture_Camera", "video_width"),
            video_height=ini.getint("DataCapture_Camera", "video_height"),
            sys_width=ini.getint("DataCapture_Camera", "sys_width"),
            sys_height=ini.getint("DataCapture_Camera", "sys_height"),
            # file input mode
            # video_files=parse_list(ini.get("DataCapture_Camera", "video_files")),
            video_files_for_cam0calib=_path_list(
                ini.get("DataCapture_Camera", "video_files_for_cam0calib")
            ),
            video_files_for_cam1calib=_path_list(
                ini.get("DataCapture_Camera", "video_files_for_cam1calib")
            ),
            video_files_for_cam2calib=_path_list(
                ini.get("DataCapture_Camera", "video_files_for_cam2calib")
            ),
            video_files_for_othermode=_path_list(
                ini.get("DataCapture_Camera", "video_files_for_othermode")
            ),
            videofile_basetime=ini.get("DataCapture_Camera", "videofile_basetime"),
            videofile_steptime=ini.getfloat("DataCapture_Camera", "videofile_steptime"),
            capture_latency_ms=ini.getfloat("DataCapture_Camera", "capture_latency_ms"),
            # realtime
            MOTEC=ini.getboolean("DataCapture_Camera", "motec"),
            porttable=parse_list(ini.get("DataCapture_Camera", "porttable")),
            iptable=parse_list(ini.get("DataCapture_Camera", "iptable")),
            framerate_div=ini.getint("DataCapture_Camera", "framerate_div"),
        ),
    )


@dataclass(frozen=True)
class DataConverter2D3DConf:
    @dataclass(frozen=True)
    class CameraConf:
        undistort_enable: bool
        intrinsics_path: str

    Camera: CameraConf

    @dataclass(frozen=True)
    class LidarConf:
        calib_lidar2crane: bool
        bothlidars: str
        # lidar_calib_files: list[str]
        lidar_calib_files_for_cam0calib: list[str]
        lidar_calib_files_for_cam1calib: list[str]
        lidar_calib_files_for_cam2calib: list[str]
        lidar_calib_files_for_othermode: list[str]
        sensors: int
        voxel_down_sample_voxelsize: float
        voxel_down_sample_enable_afteraccum: bool
        accumulate_length: int
        calibration_coord_rtvec_jsonpath: str

    Lidar: LidarConf


def DataConverter2D3DConf_read(
    ini: ConfigParser,
):
    return DataConverter2D3DConf(
        Camera=DataConverter2D3DConf.CameraConf(
            undistort_enable=ini.getboolean(
                "DataConverter2D3D_Camera", "undistort_enable"
            ),
            intrinsics_path=_path(
                ini.get("DataConverter2D3D_Camera", "intrinsics_path")
            ),
        ),
        Lidar=DataConverter2D3DConf.LidarConf(
            calib_lidar2crane=ini.getboolean(
                "DataConverter2D3D_Lidar", "calib_lidar2crane"
            ),
            bothlidars=_path(ini.get("DataConverter2D3D_Lidar", "bothlidars")),
            # lidar_calib_files=parse_list(
            #    ini.get("DataConverter2D3D_Lidar", "lidar_calib_files")
            # ),
            sensors=ini.getint("DataConverter2D3D_Lidar", "sensors"),
            voxel_down_sample_voxelsize=ini.getfloat(
                "DataConverter2D3D_Lidar", "voxel_down_sample_voxelsize"
            ),
            voxel_down_sample_enable_afteraccum=ini.getboolean(
                "DataConverter2D3D_Lidar", "voxel_down_sample_enable_afteraccum"
            ),
            accumulate_length=ini.getint(
                "DataConverter2D3D_Lidar", "accumulate_length"
            ),
            calibration_coord_rtvec_jsonpath=_path(
                ini.get("DataConverter2D3D_Lidar", "calibration_coord_rtvec_jsonpath"),
            ),
            lidar_calib_files_for_cam0calib=_path_list(
                ini.get("DataConverter2D3D_Lidar", "lidar_calib_files_for_cam0calib"),
            ),
            lidar_calib_files_for_cam1calib=_path_list(
                ini.get("DataConverter2D3D_Lidar", "lidar_calib_files_for_cam1calib"),
            ),
            lidar_calib_files_for_cam2calib=_path_list(
                ini.get("DataConverter2D3D_Lidar", "lidar_calib_files_for_cam2calib"),
            ),
            lidar_calib_files_for_othermode=_path_list(
                ini.get("DataConverter2D3D_Lidar", "lidar_calib_files_for_othermode"),
            ),
        ),
    )


@dataclass(frozen=True)
class Calib2d3dConf:
    placeholder: str

    @dataclass(frozen=True)
    class Proc2dConf:
        intrinsics_path: str
        yolo_obj_countlimit: int
        yolo_type: str
        yolo_modelpath: str
        conf_thresh: float
        nms_thresh: float
        enable_bboxfilter_byimg: bool
        enable_bbox_shapefilter: bool
        enable_edgebboxfilter: bool
        enable_imgmask: bool
        enable_bbox2D_shapefilter: bool

        camera_mask_images: list[str]

        yolothreshold: float

        bbox_tracking_framelen_min: float
        bbox_tracking_movelen_pixels: float
        bbox_tracking_workarea2d_count: int

        lost_track_buffer: float
        tracking_frame_rate: float
        track_activation_threshold: float
        minimum_consecutive_frames: float
        minimum_iou_threshold: float
        trackresult_use_lastmove_ix: bool

        axis_gridpoints_xrange_min: list[float]
        axis_gridpoints_xrange_max: list[float]
        axis_gridpoints_yrange_min: list[float]
        axis_gridpoints_yrange_max: list[float]
        axis_gridpoints_interpolate_firstmethod: str
        axis_gridpoints_interpolate_secondmethod: str

        # 座標に対する評価値のパス
        cam_valmat_path: list[str]
        # 設定値：入力座標(x_i,y_i) -> 参照座標(x_r,y_r) への一次変換係数
        cam_valmat_coord_A_X: list[float]
        cam_valmat_coord_B_X: list[float]
        cam_valmat_coord_A_Y: list[float]
        cam_valmat_coord_B_Y: list[float]
        # 設定値4：デフォルト値（範囲外参照時に返す）
        cam_valmat_val_DEFAULT: list[float]

        # 座標に対する評価値のパス
        cam_workareadef_img_path: list[str]
        # 設定値：入力座標(x_i,y_i) -> 参照座標(x_r,y_r) への一次変換係数
        cam_workareadef_img_coord_A_X: list[float]
        cam_workareadef_img_coord_B_X: list[float]
        cam_workareadef_img_coord_A_Y: list[float]
        cam_workareadef_img_coord_B_Y: list[float]
        cam_workareadef_img_coord_A_ETA: list[float]
        cam_workareadef_img_coord_B_ETA: list[float]
        # 設定値4：デフォルト値（範囲外参照時に返す）
        cam_workareadef_img_val_DEFAULT: list[float]

    Proc2d: Proc2dConf

    @dataclass(frozen=True)
    class Proc3dConf:
        datarange_x_min: float
        datarange_x_max: float
        datarange_y_min: float
        datarange_y_max: float
        datarange_z_min: float
        datarange_z_max: float
        dbscan_eps: float
        dbscan_min_samples: int
        groundplane_ransac_coeff: float
        groundplane_ptthinning_div: int
        gplane_detection_walkingarea_limit: bool
        groundplane_requiredpoints: int
        save_debugdata: bool
        enable_static_point_filter: bool
        static_point_filter_boxelsize: float
        static_point_filter_initlength: int
        static_point_filter_refresh_period: float
        bbox_tracking_framelen_min: int
        bbox_tracking_movelen_meters: float
        bbox_tracking_workarea3d_count: int

        lost_track_buffer: float
        tracking_frame_rate: float
        track_activation_threshold: float
        minimum_consecutive_frames: float
        minimum_iou_threshold: float
        trackresult_use_lastmove_ix: bool

        is_headpoint_overwrite: bool
        calc_headpoint_pointrange_x_min: float
        calc_headpoint_pointrange_x_max: float
        calc_headpoint_pointrange_y_min: float
        calc_headpoint_pointrange_y_max: float

        is_footpoints_fixed: bool
        footpoints_zval: float

        # 座標に対する評価値のパス
        lid_valmat_path: list[str]
        # 設定値：入力座標(x_i,y_i) -> 参照座標(x_r,y_r) への一次変換係数
        lid_valmat_coord_A_X: list[float]
        lid_valmat_coord_B_X: list[float]
        lid_valmat_coord_A_Y: list[float]
        lid_valmat_coord_B_Y: list[float]
        # 設定値4：デフォルト値（範囲外参照時に返す）
        lid_valmat_val_DEFAULT: list[float]

        # 座標に対する評価値のパス
        lid_workareadef_img_path: list[str]
        # 設定値：入力座標(x_i,y_i) -> 参照座標(x_r,y_r) への一次変換係数
        lid_workareadef_img_coord_A_X: list[float]
        lid_workareadef_img_coord_B_X: list[float]
        lid_workareadef_img_coord_A_Y: list[float]
        lid_workareadef_img_coord_B_Y: list[float]
        lid_workareadef_img_coord_A_ETA: list[float]
        lid_workareadef_img_coord_B_ETA: list[float]
        # 設定値4：デフォルト値（範囲外参照時に返す）
        lid_workareadef_img_val_DEFAULT: list[float]
        enable_bbox3d_overlap_merge: bool = False
        keep_lowest_z_bbox_on_merge: bool = True
        bbox3d_overlap_merge_threshold: float = 0.7
        is_headpoint_grid_overwrite: bool = False
        calc_headpoint_grid_axis: list[str] = field(
            default_factory=lambda: ["x", "x", "x"]
        )

    Proc3d: Proc3dConf

    @dataclass(frozen=True)
    class CalcProgressConf:
        areadefinition_filepathes: list[str]
        progress_threshold: float
        subblock_overwrite_src: str

    CalcProgress: CalcProgressConf

    @dataclass(frozen=True)
    class CalcCorrespondenceConf:
        postprocess_mat: str
        calcmethod: str
        opt_lambda_center_r: list[float]
        opt_lambda_center_t: list[float]
        opt_lambda_axis_r: list[float]
        opt_lambda_axis_t: list[float]
        optparam_initialvector: str
        corrpoint_mode: str
        enable_recalc_center3d_z: bool
        bbox_center3d_z_ratio: float
        use_centerpoint_x_min: list[float]
        use_centerpoint_x_max: list[float]
        use_centerpoint_y_min: list[float]
        use_centerpoint_y_max: list[float]

        corner_rangefilter_mode: str
        corner_rangefilter_x_min: list[float]
        corner_rangefilter_x_max: list[float]
        corner_rangefilter_y_min: list[float]
        corner_rangefilter_y_max: list[float]

        bbox_center3d_z_ratio_area_xmin: list[float]
        bbox_center3d_z_ratio_area_xmax: list[float]
        bbox_center3d_z_ratio_area_ymin: list[float]
        bbox_center3d_z_ratio_area_ymax: list[float]
        center3d_zratio_coord_A_X: list[float] = field(default_factory=list)
        center3d_zratio_coord_B_X: list[float] = field(default_factory=list)
        center3d_zratio_coord_A_Y: list[float] = field(default_factory=list)
        center3d_zratio_coord_B_Y: list[float] = field(default_factory=list)
        center3d_zratio_path: list[str] = field(default_factory=list)
        center3d_zratio_val_DEFAULT: list[float] = field(default_factory=list)

    CalcCorrespondence: CalcCorrespondenceConf

    @dataclass(frozen=True)
    class CalcAccuracyConf:
        check_enable: bool
        accvalue: float

    CalcAccuracy: CalcAccuracyConf

    @dataclass(frozen=True)
    class DiagnosisConf:
        continuation_seconds: float
        long_duration_seconds: float
        detection_rate_window_seconds: float
        detection_rate_threshold: float
        expected_person_count: int
        brightness_threshold: float
        brightness_sample_stride: int
        walking_area_enabled: bool
        # 外側のlist添字が0始まりのcamera_id、内側が4つの(x, y)座標。
        walking_area_corners: list[tuple[tuple[float, float], ...]]

    Diagnosis: DiagnosisConf


def Calib2d3dConf_read(
    ini: ConfigParser,
):  # なるべく上記各クラスと離れさせたくないもののクラスメソッドにも出来ないためやむを得ずここにまとめて実装
    return Calib2d3dConf(
        placeholder=ini.get("Calib2d3d_General", "placeholder"),
        Proc2d=Calib2d3dConf.Proc2dConf(
            intrinsics_path=_path(ini.get("Calib2d3d_Proc2d", "intrinsics_path")),
            yolo_obj_countlimit=ini.getint("Calib2d3d_Proc2d", "yolo_obj_countlimit"),
            yolo_type=ini.get("Calib2d3d_Proc2d", "yolo_type"),
            yolo_modelpath=_path(ini.get("Calib2d3d_Proc2d", "yolo_modelpath")),
            conf_thresh=ini.getfloat("Calib2d3d_Proc2d", "conf_thresh"),
            nms_thresh=ini.getfloat("Calib2d3d_Proc2d", "nms_thresh"),
            camera_mask_images=_path_list(
                ini.get("Calib2d3d_Proc2d", "camera_mask_images")
            ),
            enable_bboxfilter_byimg=ini.getboolean(
                "Calib2d3d_Proc2d", "enable_bboxfilter_byimg"
            ),
            enable_bbox_shapefilter=ini.getboolean(
                "Calib2d3d_Proc2d", "enable_bbox_shapefilter"
            ),
            enable_edgebboxfilter=ini.getboolean(
                "Calib2d3d_Proc2d", "enable_edgebboxfilter"
            ),
            enable_imgmask=ini.getboolean("Calib2d3d_Proc2d", "enable_imgmask"),
            enable_bbox2D_shapefilter=ini.getboolean(
                "Calib2d3d_Proc2d", "enable_bbox2D_shapefilter"
            ),
            yolothreshold=ini.getfloat("Calib2d3d_Proc2d", "yolothreshold"),
            bbox_tracking_framelen_min=ini.getint(
                "Calib2d3d_Proc2d", "bbox_tracking_framelen_min"
            ),
            bbox_tracking_movelen_pixels=ini.getint(
                "Calib2d3d_Proc2d", "bbox_tracking_movelen_pixels"
            ),
            bbox_tracking_workarea2d_count=ini.getint(
                "Calib2d3d_Proc2d", "bbox_tracking_workarea2d_count"
            ),
            cam_valmat_path=_path_list(ini.get("Calib2d3d_Proc2d", "cam_valmat_path")),
            cam_valmat_coord_A_X=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_valmat_coord_A_X")
            ),
            cam_valmat_coord_B_X=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_valmat_coord_B_X")
            ),
            cam_valmat_coord_A_Y=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_valmat_coord_A_Y")
            ),
            cam_valmat_coord_B_Y=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_valmat_coord_B_Y")
            ),
            cam_valmat_val_DEFAULT=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_valmat_val_DEFAULT")
            ),
            cam_workareadef_img_path=_path_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_path")
            ),
            cam_workareadef_img_coord_A_X=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_coord_A_X")
            ),
            cam_workareadef_img_coord_B_X=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_coord_B_X")
            ),
            cam_workareadef_img_coord_A_Y=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_coord_A_Y")
            ),
            cam_workareadef_img_coord_B_Y=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_coord_B_Y")
            ),
            cam_workareadef_img_coord_A_ETA=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_coord_A_ETA")
            ),
            cam_workareadef_img_coord_B_ETA=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_coord_B_ETA")
            ),
            cam_workareadef_img_val_DEFAULT=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "cam_workareadef_img_val_DEFAULT")
            ),
            lost_track_buffer=ini.getfloat("Calib2d3d_Proc2d", "lost_track_buffer"),
            tracking_frame_rate=ini.getfloat("Calib2d3d_Proc2d", "tracking_frame_rate"),
            track_activation_threshold=ini.getfloat(
                "Calib2d3d_Proc2d", "track_activation_threshold"
            ),
            minimum_consecutive_frames=ini.getfloat(
                "Calib2d3d_Proc2d", "minimum_consecutive_frames"
            ),
            minimum_iou_threshold=ini.getfloat(
                "Calib2d3d_Proc2d", "minimum_iou_threshold"
            ),
            trackresult_use_lastmove_ix=ini.getboolean(
                "Calib2d3d_Proc2d", "trackresult_use_lastmove_ix"
            ),
            axis_gridpoints_xrange_min=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "axis_gridpoints_xrange_min")
            ),
            axis_gridpoints_xrange_max=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "axis_gridpoints_xrange_max")
            ),
            axis_gridpoints_yrange_min=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "axis_gridpoints_yrange_min")
            ),
            axis_gridpoints_yrange_max=parse_float_list(
                ini.get("Calib2d3d_Proc2d", "axis_gridpoints_yrange_max")
            ),
            axis_gridpoints_interpolate_firstmethod=ini.get(
                "Calib2d3d_Proc2d", "axis_gridpoints_interpolate_firstmethod"
            ),
            axis_gridpoints_interpolate_secondmethod=ini.get(
                "Calib2d3d_Proc2d", "axis_gridpoints_interpolate_secondmethod"
            ),
        ),
        Proc3d=Calib2d3dConf.Proc3dConf(
            datarange_x_min=ini.getfloat("Calib2d3d_Proc3d", "datarange_x_min"),
            datarange_x_max=ini.getfloat("Calib2d3d_Proc3d", "datarange_x_max"),
            datarange_y_min=ini.getfloat("Calib2d3d_Proc3d", "datarange_y_min"),
            datarange_y_max=ini.getfloat("Calib2d3d_Proc3d", "datarange_y_max"),
            datarange_z_min=ini.getfloat("Calib2d3d_Proc3d", "datarange_z_min"),
            datarange_z_max=ini.getfloat("Calib2d3d_Proc3d", "datarange_z_max"),
            dbscan_eps=ini.getfloat("Calib2d3d_Proc3d", "dbscan_eps"),
            dbscan_min_samples=ini.getint("Calib2d3d_Proc3d", "dbscan_min_samples"),
            groundplane_ransac_coeff=ini.getfloat(
                "Calib2d3d_Proc3d", "groundplane_ransac_coeff"
            ),
            groundplane_ptthinning_div=ini.getint(
                "Calib2d3d_Proc3d", "groundplane_ptthinning_div"
            ),
            gplane_detection_walkingarea_limit=ini.getboolean(
                "Calib2d3d_Proc3d", "gplane_detection_walkingarea_limit"
            ),
            groundplane_requiredpoints=ini.getint(
                "Calib2d3d_Proc3d", "groundplane_requiredpoints"
            ),
            save_debugdata=ini.getboolean("Calib2d3d_Proc3d", "save_debugdata"),
            enable_static_point_filter=ini.getboolean(
                "Calib2d3d_Proc3d", "enable_static_point_filter"
            ),
            static_point_filter_boxelsize=ini.getfloat(
                "Calib2d3d_Proc3d", "static_point_filter_boxelsize"
            ),
            static_point_filter_initlength=ini.getint(
                "Calib2d3d_Proc3d", "static_point_filter_initlength"
            ),
            static_point_filter_refresh_period=ini.getfloat(
                "Calib2d3d_Proc3d", "static_point_filter_refresh_period"
            ),
            bbox_tracking_framelen_min=ini.getint(
                "Calib2d3d_Proc3d", "bbox_tracking_framelen_min"
            ),
            bbox_tracking_movelen_meters=ini.getfloat(
                "Calib2d3d_Proc3d", "bbox_tracking_movelen_meters"
            ),
            bbox_tracking_workarea3d_count=ini.getint(
                "Calib2d3d_Proc3d", "bbox_tracking_workarea3d_count"
            ),
            lost_track_buffer=ini.getfloat("Calib2d3d_Proc3d", "lost_track_buffer"),
            tracking_frame_rate=ini.getfloat("Calib2d3d_Proc3d", "tracking_frame_rate"),
            track_activation_threshold=ini.getfloat(
                "Calib2d3d_Proc3d", "track_activation_threshold"
            ),
            minimum_consecutive_frames=ini.getfloat(
                "Calib2d3d_Proc3d", "minimum_consecutive_frames"
            ),
            minimum_iou_threshold=ini.getfloat(
                "Calib2d3d_Proc3d", "minimum_iou_threshold"
            ),
            trackresult_use_lastmove_ix=ini.getboolean(
                "Calib2d3d_Proc3d", "trackresult_use_lastmove_ix"
            ),
            is_headpoint_overwrite=ini.getboolean(
                "Calib2d3d_Proc3d", "is_headpoint_overwrite"
            ),
            calc_headpoint_pointrange_x_min=ini.getfloat(
                "Calib2d3d_Proc3d", "calc_headpoint_pointrange_x_min"
            ),
            calc_headpoint_pointrange_x_max=ini.getfloat(
                "Calib2d3d_Proc3d", "calc_headpoint_pointrange_x_max"
            ),
            calc_headpoint_pointrange_y_min=ini.getfloat(
                "Calib2d3d_Proc3d", "calc_headpoint_pointrange_y_min"
            ),
            calc_headpoint_pointrange_y_max=ini.getfloat(
                "Calib2d3d_Proc3d", "calc_headpoint_pointrange_y_max"
            ),
            is_footpoints_fixed=ini.getboolean(
                "Calib2d3d_Proc3d", "is_footpoints_fixed"
            ),
            footpoints_zval=ini.getfloat("Calib2d3d_Proc3d", "footpoints_zval"),
            lid_valmat_path=_path_list(ini.get("Calib2d3d_Proc3d", "lid_valmat_path")),
            lid_valmat_coord_A_X=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_valmat_coord_A_X")
            ),
            lid_valmat_coord_B_X=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_valmat_coord_B_X")
            ),
            lid_valmat_coord_A_Y=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_valmat_coord_A_Y")
            ),
            lid_valmat_coord_B_Y=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_valmat_coord_B_Y")
            ),
            lid_valmat_val_DEFAULT=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_valmat_val_DEFAULT")
            ),
            lid_workareadef_img_path=_path_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_path")
            ),
            lid_workareadef_img_coord_A_X=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_coord_A_X")
            ),
            lid_workareadef_img_coord_B_X=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_coord_B_X")
            ),
            lid_workareadef_img_coord_A_Y=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_coord_A_Y")
            ),
            lid_workareadef_img_coord_B_Y=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_coord_B_Y")
            ),
            lid_workareadef_img_coord_A_ETA=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_coord_A_ETA")
            ),
            lid_workareadef_img_coord_B_ETA=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_coord_B_ETA")
            ),
            lid_workareadef_img_val_DEFAULT=parse_float_list(
                ini.get("Calib2d3d_Proc3d", "lid_workareadef_img_val_DEFAULT")
            ),
            enable_bbox3d_overlap_merge=ini.getboolean(
                "Calib2d3d_Proc3d", "enable_bbox3d_overlap_merge", fallback=False
            ),
            bbox3d_overlap_merge_threshold=ini.getfloat(
                "Calib2d3d_Proc3d", "bbox3d_overlap_merge_threshold", fallback=0.7
            ),
            keep_lowest_z_bbox_on_merge=ini.getboolean(
                "Calib2d3d_Proc3d", "keep_lowest_z_bbox_on_merge"
            ),
            is_headpoint_grid_overwrite=ini.getboolean(
                "Calib2d3d_Proc3d", "is_headpoint_grid_overwrite", fallback=False
            ),
            calc_headpoint_grid_axis=parse_list(
                ini.get(
                    "Calib2d3d_Proc3d",
                    "calc_headpoint_grid_axis",
                    fallback="x,x,x",
                )
            ),
        ),
        CalcProgress=Calib2d3dConf.CalcProgressConf(
            areadefinition_filepathes=_path_list(
                ini.get("Calib2d3d_CalcProgress", "areadefinition_filepathes"),
            ),
            progress_threshold=ini.getfloat(
                "Calib2d3d_CalcProgress", "progress_threshold"
            ),
            subblock_overwrite_src=ini.get(
                "Calib2d3d_CalcProgress", "subblock_overwrite_src"
            ),
        ),
        CalcCorrespondence=Calib2d3dConf.CalcCorrespondenceConf(
            postprocess_mat=_path(
                ini.get("Calib2d3d_CalcCorrespondence", "postprocess_mat"),
            ),
            calcmethod=ini.get("Calib2d3d_CalcCorrespondence", "calcmethod"),
            opt_lambda_center_r=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "opt_lambda_center_r")
            ),
            opt_lambda_center_t=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "opt_lambda_center_t")
            ),
            opt_lambda_axis_r=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "opt_lambda_axis_r")
            ),
            opt_lambda_axis_t=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "opt_lambda_axis_t")
            ),
            optparam_initialvector=_path(
                ini.get("Calib2d3d_CalcCorrespondence", "optparam_initialvector"),
            ),
            corrpoint_mode=ini.get("Calib2d3d_CalcCorrespondence", "corrpoint_mode"),
            enable_recalc_center3d_z=ini.getboolean(
                "Calib2d3d_CalcCorrespondence", "enable_recalc_center3d_z"
            ),
            bbox_center3d_z_ratio=ini.getfloat(
                "Calib2d3d_CalcCorrespondence", "bbox_center3d_z_ratio"
            ),
            use_centerpoint_x_min=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "use_centerpoint_x_min")
            ),
            use_centerpoint_x_max=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "use_centerpoint_x_max")
            ),
            use_centerpoint_y_min=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "use_centerpoint_y_min")
            ),
            use_centerpoint_y_max=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "use_centerpoint_y_max")
            ),
            corner_rangefilter_mode=ini.get(
                "Calib2d3d_CalcCorrespondence", "corner_rangefilter_mode"
            ),
            corner_rangefilter_x_min=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "corner_rangefilter_x_min")
            ),
            corner_rangefilter_x_max=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "corner_rangefilter_x_max")
            ),
            corner_rangefilter_y_min=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "corner_rangefilter_y_min")
            ),
            corner_rangefilter_y_max=parse_float_list(
                ini.get("Calib2d3d_CalcCorrespondence", "corner_rangefilter_y_max")
            ),
            bbox_center3d_z_ratio_area_xmin=parse_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence", "bbox_center3d_z_ratio_area_xmin"
                )
            ),
            bbox_center3d_z_ratio_area_xmax=parse_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence", "bbox_center3d_z_ratio_area_xmax"
                )
            ),
            bbox_center3d_z_ratio_area_ymin=parse_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence", "bbox_center3d_z_ratio_area_ymin"
                )
            ),
            bbox_center3d_z_ratio_area_ymax=parse_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence", "bbox_center3d_z_ratio_area_ymax"
                )
            ),
            center3d_zratio_coord_A_X=_optional_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence",
                    "center3d_zratio_coord_A_X",
                    fallback="",
                )
            ),
            center3d_zratio_coord_B_X=_optional_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence",
                    "center3d_zratio_coord_B_X",
                    fallback="",
                )
            ),
            center3d_zratio_coord_A_Y=_optional_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence",
                    "center3d_zratio_coord_A_Y",
                    fallback="",
                )
            ),
            center3d_zratio_coord_B_Y=_optional_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence",
                    "center3d_zratio_coord_B_Y",
                    fallback="",
                )
            ),
            center3d_zratio_path=_path_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence",
                    "center3d_zratio_path",
                    fallback="",
                )
            ),
            center3d_zratio_val_DEFAULT=_optional_float_list(
                ini.get(
                    "Calib2d3d_CalcCorrespondence",
                    "center3d_zratio_val_DEFAULT",
                    fallback="",
                )
            ),
        ),
        CalcAccuracy=Calib2d3dConf.CalcAccuracyConf(
            check_enable=ini.getboolean("Calib2d3d_CalcAccuracy", "check_enable"),
            accvalue=ini.getfloat("Calib2d3d_CalcAccuracy", "accvalue"),
        ),
        Diagnosis=Calib2d3dConf.DiagnosisConf(
            continuation_seconds=ini.getfloat(
                "Calib2d3d_Diagnosis", "continuation_seconds", fallback=20.0
            ),
            long_duration_seconds=ini.getfloat(
                "Calib2d3d_Diagnosis", "long_duration_seconds", fallback=600.0
            ),
            detection_rate_window_seconds=ini.getfloat(
                "Calib2d3d_Diagnosis",
                "detection_rate_window_seconds",
                fallback=20.0,
            ),
            detection_rate_threshold=ini.getfloat(
                "Calib2d3d_Diagnosis", "detection_rate_threshold", fallback=0.2
            ),
            expected_person_count=ini.getint(
                "Calib2d3d_Diagnosis", "expected_person_count", fallback=1
            ),
            brightness_threshold=ini.getfloat(
                "Calib2d3d_Diagnosis", "brightness_threshold", fallback=20.0
            ),
            brightness_sample_stride=ini.getint(
                "Calib2d3d_Diagnosis", "brightness_sample_stride", fallback=16
            ),
            walking_area_enabled=ini.getboolean(
                "Calib2d3d_Diagnosis", "walking_area_enabled", fallback=False
            ),
            walking_area_corners=[
                _parse_xy_quadrilateral(
                    ini.get(
                        "Calib2d3d_Diagnosis",
                        f"walking_area_corners_camera{camera_id}",
                    )
                )
                for camera_id in range(
                    ini.getint("DataCapture_Camera", "count")
                )
            ],
        ),
    )


@dataclass(frozen=True)
class FacadeConf:
    write_dummydata: bool


def FacadeConf_read(ini: ConfigParser) -> FacadeConf:
    return FacadeConf(write_dummydata=ini.getboolean("Facade", "write_dummydata"))


@dataclass(frozen=True)
class Calib3d3dCalibParamsConf:
    lidar0_calib_path: str
    lidar1_calib_path: str
    lidar2_calib_path: str
    lidar3_calib_path: str
    lidar4_calib_path: str
    lidar5_calib_path: str
    lidars_calib_path: list[str]
    voxel_size_for_ground_pts: float
    voxel_size_for_calib: float
    crane_profile_path: str
    thr_radius_L2L: list[float]
    thr_radius_L2C: list[float]


def Calib3d3dCalibParamsConf_read(
    ini: ConfigParser,
) -> Calib3d3dCalibParamsConf:
    return Calib3d3dCalibParamsConf(
        lidar0_calib_path=_path(ini.get("Calib3d3d_CalibParams", "lidar0_calib_path")),
        lidar1_calib_path=_path(ini.get("Calib3d3d_CalibParams", "lidar1_calib_path")),
        lidar2_calib_path=_path(ini.get("Calib3d3d_CalibParams", "lidar2_calib_path")),
        lidar3_calib_path=_path(ini.get("Calib3d3d_CalibParams", "lidar3_calib_path")),
        lidar4_calib_path=_path(ini.get("Calib3d3d_CalibParams", "lidar4_calib_path")),
        lidar5_calib_path=_path(ini.get("Calib3d3d_CalibParams", "lidar5_calib_path")),
        lidars_calib_path=_path_list(
            ini.get("Calib3d3d_CalibParams", "lidars_calib_path")
        ),
        voxel_size_for_ground_pts=ini.getfloat(
            "Calib3d3d_CalibParams", "voxel_size_for_ground_pts"
        ),
        voxel_size_for_calib=ini.getfloat(
            "Calib3d3d_CalibParams", "voxel_size_for_calib"
        ),
        crane_profile_path=_path(
            ini.get("Calib3d3d_CalibParams", "crane_profile_path")
        ),
        thr_radius_L2L=parse_float_list(
            ini.get("Calib3d3d_CalibParams", "thr_radius_L2L")
        ),
        thr_radius_L2C=parse_float_list(
            ini.get("Calib3d3d_CalibParams", "thr_radius_L2C")
        ),
    )


@dataclass(frozen=True)
class Calib3d3dNdtParamsConf:
    voxel_size: float
    ds_voxel_xy: float
    min_points_per_voxel: int
    neighbor_top_k: int
    neighbor_maha2_gate: float
    weight_temperature: float
    geom_sigma: float
    geom_gate: float
    max_iters: int
    lm_lambda_init: float
    step_clip_trans: float
    step_clip_yaw: float
    range_min: float
    range_max: float
    yaw_offset_list: list[float]
    thinning_margin: float


def Calib3d3dNdtParamsConf_read(ini: ConfigParser) -> Calib3d3dNdtParamsConf:
    return Calib3d3dNdtParamsConf(
        voxel_size=ini.getfloat("Calib3d3d_NdtParams", "voxel_size"),
        ds_voxel_xy=ini.getfloat("Calib3d3d_NdtParams", "ds_voxel_xy"),
        min_points_per_voxel=ini.getint("Calib3d3d_NdtParams", "min_points_per_voxel"),
        neighbor_top_k=ini.getint("Calib3d3d_NdtParams", "neighbor_top_k"),
        neighbor_maha2_gate=ini.getfloat("Calib3d3d_NdtParams", "neighbor_maha2_gate"),
        weight_temperature=ini.getfloat("Calib3d3d_NdtParams", "weight_temperature"),
        geom_sigma=ini.getfloat("Calib3d3d_NdtParams", "geom_sigma"),
        geom_gate=ini.getfloat("Calib3d3d_NdtParams", "geom_gate"),
        max_iters=ini.getint("Calib3d3d_NdtParams", "max_iters"),
        lm_lambda_init=ini.getfloat("Calib3d3d_NdtParams", "lm_lambda_init"),
        step_clip_trans=ini.getfloat("Calib3d3d_NdtParams", "step_clip_trans"),
        step_clip_yaw=ini.getfloat("Calib3d3d_NdtParams", "step_clip_yaw"),
        range_min=ini.getfloat("Calib3d3d_NdtParams", "range_min"),
        range_max=ini.getfloat("Calib3d3d_NdtParams", "range_max"),
        yaw_offset_list=parse_float_list(
            ini.get("Calib3d3d_NdtParams", "yaw_offset_list")
        ),
        thinning_margin=ini.getfloat("Calib3d3d_NdtParams", "thinning_margin"),
    )


@dataclass(frozen=True)
class Calib3d3dSimParamsConf:
    max_range: float
    lidar_type: str
    rays_az: int
    rays_el: int
    vfov: list[float]
    mid360_laser_pattern_path: str
    points_accum_time: float


def Calib3d3dSimParamsConf_read(
    ini: ConfigParser,
) -> Calib3d3dSimParamsConf:
    return Calib3d3dSimParamsConf(
        max_range=ini.getfloat("Calib3d3d_SimParams", "max_range"),
        lidar_type=ini.get("Calib3d3d_SimParams", "lidar_type"),
        rays_az=ini.getint("Calib3d3d_SimParams", "rays_az"),
        rays_el=ini.getint("Calib3d3d_SimParams", "rays_el"),
        vfov=parse_float_list(ini.get("Calib3d3d_SimParams", "vfov")),
        mid360_laser_pattern_path=_path(
            ini.get("Calib3d3d_SimParams", "mid360_laser_pattern_path")
        ),
        points_accum_time=ini.getfloat("Calib3d3d_SimParams", "points_accum_time"),
    )


@dataclass(frozen=True)
class CalibCheck2d3dConf:
    onnx_model_path: str
    camera_intrinsics_path: str
    lidar_calib_files: list[str]
    camera_calib_files: list[str]
    new_axis_mode: bool
    image_w: int
    image_h: int
    z_threshold: float
    camera_count: int
    resultfiles: list[str]
    score_accept_count_threshold: int
    score_value_threshold: float
    person_not_detected_sec: float
    frame_info_maxlen: int
    thresh_3dbbox_count_per_frame: int
    thresh_3dbbox_count_mean_ratio: float
    thresh_2dbbox_count_per_frame: int
    thresh_2dbbox_count_mean_ratio: float
    thresh_3dbbox_tracking_idcount: int
    thresh_2dbbox_tracking_idcount: int
    eval_frame_stride: int
    use_legacy_like_metric: bool
    debug_calibcheck_enabled: bool
    debug_capture_ui_video_enabled: bool
    debug_capture_frame_text_enabled: bool
    debug_video_fps: float
    debug_video_prefix: str
    debug_eval_pickle_path: str
    eval_zvalues: tuple[float, float]
    debug_eval_trace_enabled: bool
    debug_eval_trace_all_frames: bool
    debug_eval_trace_range_start: int
    debug_eval_trace_range_end: int


def CalibCheck2d3dConf_read(
    ini: ConfigParser,
) -> CalibCheck2d3dConf:
    eval_zvalues_str = ini.get("CalibCheck2d3d", "eval_zvalues")
    eval_zvalues = parse_float_tuple2(eval_zvalues_str)

    return CalibCheck2d3dConf(
        onnx_model_path=_path(ini.get("CalibCheck2d3d", "onnx_model_path")),
        camera_intrinsics_path=_path(
            ini.get("CalibCheck2d3d", "camera_intrinsics_path")
        ),
        lidar_calib_files=_path_list(ini.get("CalibCheck2d3d", "lidar_calib_files")),
        camera_calib_files=_path_list(ini.get("CalibCheck2d3d", "camera_calib_files")),
        new_axis_mode=ini.getboolean("CalibCheck2d3d", "new_axis_mode"),
        image_w=ini.getint("CalibCheck2d3d", "image_w"),
        image_h=ini.getint("CalibCheck2d3d", "image_h"),
        z_threshold=ini.getfloat("CalibCheck2d3d", "z_threshold"),
        camera_count=ini.getint("CalibCheck2d3d", "camera_count"),
        resultfiles=_path_list(ini.get("CalibCheck2d3d", "resultfiles")),
        score_accept_count_threshold=ini.getint(
            "CalibCheck2d3d", "score_accept_count_threshold"
        ),
        score_value_threshold=ini.getfloat("CalibCheck2d3d", "score_value_threshold"),
        person_not_detected_sec=ini.getfloat(
            "CalibCheck2d3d", "person_not_detected_sec"
        ),
        frame_info_maxlen=ini.getint("CalibCheck2d3d", "frame_info_maxlen"),
        thresh_3dbbox_count_per_frame=ini.getint(
            "CalibCheck2d3d", "thresh_3dbbox_count_per_frame"
        ),
        thresh_3dbbox_count_mean_ratio=ini.getfloat(
            "CalibCheck2d3d", "thresh_3dbbox_count_mean_ratio"
        ),
        thresh_2dbbox_count_per_frame=ini.getint(
            "CalibCheck2d3d", "thresh_2dbbox_count_per_frame"
        ),
        thresh_2dbbox_count_mean_ratio=ini.getfloat(
            "CalibCheck2d3d", "thresh_2dbbox_count_mean_ratio"
        ),
        thresh_3dbbox_tracking_idcount=ini.getint(
            "CalibCheck2d3d", "thresh_3dbbox_tracking_idcount"
        ),
        thresh_2dbbox_tracking_idcount=ini.getint(
            "CalibCheck2d3d", "thresh_2dbbox_tracking_idcount"
        ),
        eval_frame_stride=ini.getint("CalibCheck2d3d", "eval_frame_stride"),
        use_legacy_like_metric=ini.getboolean(
            "CalibCheck2d3d", "use_legacy_like_metric"
        ),
        debug_calibcheck_enabled=ini.getboolean(
            "CalibCheck2d3d", "debug_calibcheck_enabled"
        ),
        debug_capture_ui_video_enabled=ini.getboolean(
            "CalibCheck2d3d", "debug_capture_ui_video_enabled"
        ),
        debug_capture_frame_text_enabled=ini.getboolean(
            "CalibCheck2d3d", "debug_capture_frame_text_enabled"
        ),
        debug_video_fps=ini.getfloat("CalibCheck2d3d", "debug_video_fps"),
        debug_video_prefix=ini.get("CalibCheck2d3d", "debug_video_prefix"),
        debug_eval_pickle_path=_path(
            ini.get("CalibCheck2d3d", "debug_eval_pickle_path")
        ),
        eval_zvalues=eval_zvalues,
        debug_eval_trace_enabled=ini.getboolean(
            "CalibCheck2d3d", "debug_eval_trace_enabled"
        ),
        debug_eval_trace_all_frames=ini.getboolean(
            "CalibCheck2d3d", "debug_eval_trace_all_frames"
        ),
        debug_eval_trace_range_start=ini.getint(
            "CalibCheck2d3d", "debug_eval_trace_range_start"
        ),
        debug_eval_trace_range_end=ini.getint(
            "CalibCheck2d3d", "debug_eval_trace_range_end"
        ),
    )


class AppConfigCalibration:
    def __init__(
        self,
        configpath="",
        arglist=[],
        verb=False,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> None:
        self.configpath = configpath
        self.arglist = arglist
        self.verb = verb
        self._directory_config = directory_config
        self.reload()

    def reload(self):
        ini = ini_read_and_replace(
            arglist=self.arglist, configpath=self.configpath, verb=self.verb
        )
        self.read(ini)

    def read(self, ini: ConfigParser) -> None:
        validate_calib_settings(ini)
        self._directory_config: paths.DirectoryConfig = resolve_ini_roots(
            ini, self._directory_config
        )
        self.default: DefaultConf = DefaultConf_read(ini)
        self.debug: DebugConf = DebugConf_read(ini)
        self.filepath_io: Filepath_IOConf = Filepath_IOConf_read(ini)
        self.general: GeneralConf = GeneralConf_read(ini)
        self.dataCapture: DataCaptureConf = DataCaptureConf_read(ini)
        self.dataConverter2D3D: DataConverter2D3DConf = DataConverter2D3DConf_read(ini)
        self.calib2d3d: Calib2d3dConf = Calib2d3dConf_read(ini)
        self.facadeConf: FacadeConf = FacadeConf_read(ini)
        self.Calib3d3d_CalibParams: Calib3d3dCalibParamsConf = (
            Calib3d3dCalibParamsConf_read(ini)
        )
        self.Calib3d3d_NdtParams: Calib3d3dNdtParamsConf = Calib3d3dNdtParamsConf_read(
            ini
        )
        self.Calib3d3d_SimParams: Calib3d3dSimParamsConf = Calib3d3dSimParamsConf_read(
            ini
        )
        self.calibCheck2d3d: CalibCheck2d3dConf = CalibCheck2d3dConf_read(ini)


def parse_tuple(text: str) -> tuple[str, ...]:
    t0: str = text.replace("(", "").replace(")", "")
    t1: str = t0.strip(",")
    t2: list[str] = t1.split(",")
    return tuple(t2)


def parse_list(text: str) -> list[str]:
    t0: str = text.replace("[", "").replace("]", "")
    t1: str = t0.strip(",")
    t2: list[str] = t1.split(",")
    return t2


def parse_int_list(text: str) -> list[int]:
    return list(map(int, parse_list(text=text)))


def parse_int_tuple2(text: str) -> tuple[int, int]:
    v1, v2 = map(int, parse_tuple(text=text))
    return v1, v2


def parse_float_tuple2(text: str) -> tuple[float, float]:
    v1, v2 = map(float, parse_tuple(text=text))
    return v1, v2


def parse_int_tuple3(text: str) -> tuple[int, int, int]:
    v1, v2, v3 = map(int, parse_tuple(text=text))
    return v1, v2, v3


def parse_float_tuple3(text: str) -> tuple[float, float, float]:
    v1, v2, v3 = map(float, parse_tuple(text=text))
    return v1, v2, v3


def parse_float_tuple2x3(
    text: str,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
]:
    v1, v2, v3, v4, v5, v6 = map(float, parse_tuple(text=text))
    return ((v1, v2, v3), (v4, v5, v6))


def parse_float_tuple3x3(
    text: str,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    v1, v2, v3, v4, v5, v6, v7, v8, v9 = map(float, parse_tuple(text=text))
    return ((v1, v2, v3), (v4, v5, v6), (v7, v8, v9))


if __name__ == "__main__":
    # ini = ConfigParser(interpolation=ExtendedInterpolation())
    # ini.read("../settings2025/O2502029_2312D2004_reconstructed.ini", "UTF-8")
    # app_config_calib = AppConfigCalibration(ini)
    app_config_calib = AppConfigCalibration(
        "../settings2025/O2502029_2312D2004_reconstructed.ini"
    )
    print(app_config_calib)
    print(app_config_calib.default)
    print(app_config_calib.general)
