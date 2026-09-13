from configparser import ConfigParser, ExtendedInterpolation
from dataclasses import dataclass
from pathlib import Path

from argus_synchro.common import paths


def _path(value: str) -> str:
    return str(paths.normalize_path(value, Path.cwd()))


@dataclass(frozen=True, slots=True)
class DefaultConf:
    home_dir: str
    File_Input: bool
    debug_log: str
    data_dir: str
    print_disabled: bool
    use_shi_lib: bool


@dataclass(frozen=True, slots=True)
class GeneralConf:
    in_factory: bool
    operation_mode: int
    has_external_guard: bool
    external_guard_offset: float
    ground_height: float
    ground_height_margin: float
    rotation_radius: float
    initial_transform_file: str
    process_cpu_affinity_path: str
    enable_cpu_affinity: bool


@dataclass(frozen=True, slots=True)
class CalibrationModeSwitchConf:
    isRunning3D3Dcalib: bool
    isRunning2D3Dcalib: bool
    cameraID: int
    start2D3DCalibCalc: bool
    isRunning2D3Dcheck: bool
    start2D3DCheckCalc: bool
    isRunningInterfaceDebug: bool


@dataclass(frozen=True, slots=True)
class UIIFConf:
    crane_model: str
    UI_mmap: list[str]
    damp_out: bool
    damp_mmap: list[str]
    bbox_3d_num: int
    bbox_3d_distance: float
    show_unk: bool
    collision_depict_dist: float
    collision_attention_dist: float
    collision_warning_dist: float
    cliff_attention_dist: float
    cliff_warning_dist: float
    draw_bbox_3d: bool
    draw_collision: bool


@dataclass(frozen=True, slots=True)
class CalibUIIFConf:
    godot_ui: bool
    UI_mmap: list[str]
    damp_out: bool
    damp_mmap: list[str]
    mmap_assign_json_path: str
    show_trajectory: bool
    show_image2d3d: bool


@dataclass(frozen=True, slots=True)
class AppManagerConf:
    interval: float
    JudegeStopThr: int
    monitor_argus_last_heartbeat_path: str


@dataclass(frozen=True, slots=True)
class MonitorConf:
    show_cam: list[int]
    coeff: float


@dataclass(frozen=True, slots=True)
class ScrutinizerConf:
    s_frame: int
    e_frame: int
    file_input_loop: bool
    v0_file: str
    v1_file: str
    v2_file: str
    fast_th_ms: float
    slow_th_ms: float
    short_que: int
    long_que: int
    get_data_sleep_sec: float


@dataclass(frozen=True, slots=True)
class CANConf:
    config_file: str
    IsOld: bool
    interpretation: int
    yaw_offset_deg: float
    c_file: str
    can_id_map_file: str


@dataclass(frozen=True, slots=True)
class StateEstimatorConf:
    window_sec: float
    delta_db: float
    p_on: float
    p_off: float


@dataclass(frozen=True, slots=True)
class ReducedLoadModeConf:
    many_points_ratio: float
    few_points_ratio: float

    def __post_init__(self) -> None:
        if not 0.0 <= self.few_points_ratio < self.many_points_ratio <= 1.0:
            raise ValueError(
                "ReducedLoadMode ratios must satisfy "
                "0 <= few_points_ratio < many_points_ratio <= 1"
            )


@dataclass(frozen=True, slots=True)
class LidarConf:
    count: int
    path: str
    config_file: str
    accum_time: float
    dev_str: str
    date_str: str
    lidar0_file: str
    lidar1_file: str
    lidar2_file: str
    lidar3_file: str
    lidar4_file: str
    lidar5_file: str
    lidar_files: list[str]


@dataclass(frozen=True, slots=True)
class Detect3dConf:
    eps: float
    min_samples: int


@dataclass(frozen=True, slots=True)
class Detect2dConf:
    is_applied: bool
    core_path: str
    model_path: str
    yolo_class: str
    onnx_model_path: str
    conf_thresh: float
    nms_thresh: float


@dataclass(frozen=True, slots=True)
class CameraConf:
    count: int
    config_file: str
    video_width: int
    video_height: int
    sys_width: int
    sys_height: int
    undistort_backend: str
    porttable: list[str]
    iptable: list[str]


@dataclass(frozen=True, slots=True)
class CalibrationConf:
    calib_lidar2crane: bool
    BothLidars: str
    Lidar0: str
    Lidar1: str
    Lidar2: str
    Lidar3: str
    Lidar4: str
    Lidar5: str
    Lidar_calib_files: list[str]  # CHANGE_LIDAR
    mtx_file: str
    dist_file: str
    list_0_files: str
    list_1_files: str
    list_2_files: str
    fisheye_param_file: str
    center_rotate: tuple[float, float, float]
    center_shift: tuple[float, float, float]
    camera_position: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    lidar_position: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    imu_baselines_file: str


@dataclass(frozen=True, slots=True)
class MachineConf:
    path: str
    vis_machine_dir: str
    json_vis_machine_file: str
    offset_rotate_center: tuple[float, float, float]  # list[float]


@dataclass(frozen=True, slots=True)
class LidarPositionConf:
    x_offset: float
    y_offset: float
    z_offset: float
    x_reverse: bool
    y_reverse: bool
    z_reverse: bool


@dataclass(frozen=True, slots=True)
class LidarGridConf:
    side_max: float
    side_min: float
    fwd_max: float
    fwd_min: float
    grid_size: float


@dataclass(frozen=True, slots=True)
class AccumulationConf:
    accum_point: bool
    num_skip_registration_frames: int
    registration_methods: str
    thr_skip_points: int
    registration_range: float
    voxel_down_sample: float
    voxel_size_for_multi_icp: list[float]
    correspondence_distances: list[float]
    voxel_size_for_accumulated_points: float
    voxel_size_for_accumulated_points_reduced_load: float
    voxel_size_for_accumulated_ground_points: float
    voxel_size_for_accumulated_ground_points_reduced_load: float
    increment_accum_counter: int
    decrement_accum_counter: int
    decrement_speed_threshold: int
    decrement_accum_counter_slow: int
    prob_present_threshold: int
    accum_counter_lower_reset_threshold: int
    accum_counter_max_cap: int
    max_accumulated_frames: int
    max_accumulated_frames_reduced_load: int
    max_accumulated_frames_ground: int
    max_accumulated_frames_ground_reduced_load: int


@dataclass(frozen=True, slots=True)
class OctoTreeConf:
    func_on: bool
    col_machine_dir: str
    json_col_machine_file: str
    max_xyz: list[float]
    min_xyz: list[float]
    max_tree_depth: int
    remove_dist: float
    clustering_tree_depth: int
    use_node_stats: bool


@dataclass(frozen=True, slots=True)
class CollisionDetectionConf:
    func_on: bool
    collision_detector_name: str
    dialate_point_size: int
    distance_threshold: float
    detect_focus_range: list[float]
    coord_method: str
    max_dist: float
    detectable_height: float
    detectable_ground_offset: float
    grid_intervals: tuple[int, int, int]
    min_radius: float
    max_radius: float
    key_num: int


@dataclass(frozen=True, slots=True)
class SceneDescriptionConf:
    coarse_lo: float
    coarse_hi: float
    k_min: float
    h_ref_px: int
    lo_gain: float
    hi_gain: float
    lo_floor: float
    hi_ceil: float
    vertical_w_iou: float
    vertical_w_scale: float
    vertical_w_phi: float
    final_threshold: float
    use_human_gate: bool
    H_min: float
    H_max: float
    W_min: float
    W_max: float
    D_min: float
    D_max: float
    tall_ratio_min: float


@dataclass(frozen=True, slots=True)
class VisualizerConf:
    display_point: bool
    box_size: float
    rotate_grid: bool
    save_dir_path: str
    coeff: float
    radius: float
    display_octree: bool
    display_octree_method: str


@dataclass(frozen=True, slots=True)
class EdgeDetectionConf:
    detector: str
    is_applied: bool
    func_mode: str
    resolution: float
    grid_size: tuple[float, float]
    side_range: tuple[float, float]
    fwd_range: tuple[float, float]
    bev_min_z: float
    bev_max_z: float
    voxel_size: float
    range_properties: list[str]
    detect_range: tuple[float, float]
    detect_offset: tuple[float, float]
    height_strip: float
    edge_width: float
    polar_origin: tuple[float, float, float]
    polar_shape: tuple[int, int]
    polar_min_radius: float
    machine_occ_label: int
    target_edge_dist_th: float
    search_radius_offset: int
    remove_duplicate_label: bool
    edge_z_offset: float
    bin_th: int
    occ_focus_range: int
    edge_filter_size: int
    debug: bool


@dataclass(frozen=True, slots=True)
class LiDARShiftMonitorConf:
    win: int
    hold: int
    thr_slow: float
    win_fast: int
    hold_fast: int
    thr_fast: float
    g_mag: float
    num_sample_k: int
    dt: float
    has_not_calibrated_path: str


@dataclass(frozen=True, slots=True)
class DebugConf:
    record_time_folder_path: str
    draw_yolo: bool


@dataclass(frozen=True, slots=True)
class EvalGeneralConf:
    number_of_frames_used: int
    fdir: str
    fname: str


@dataclass(frozen=True, slots=True)
class EvalCalibConf:
    Lidar1_calib: str
    Lidar2_calib: str
    Lidar3_calib: str
    Lidar4_calib: str

    def get_list(self) -> list[str]:
        return [
            self.Lidar1_calib,
            self.Lidar2_calib,
            self.Lidar3_calib,
            self.Lidar4_calib,
        ]


@dataclass(frozen=True, slots=True)
class EvalDataConf:
    Lidar1_points: str
    Lidar2_points: str
    Lidar3_points: str
    Lidar4_points: str

    def get_list(self) -> list[str]:
        return [
            self.Lidar1_points,
            self.Lidar2_points,
            self.Lidar3_points,
            self.Lidar4_points,
        ]


@dataclass(frozen=True, slots=True)
class EvalCollisionConf:
    func_on: bool
    detect_range: tuple[float, float]
    grid_size: tuple[float, float]
    height_offset: float
    grid_color: tuple[int, int, int]


@dataclass(frozen=True, slots=True)
class JetsonMonitorConf:
    is_applied: bool
    settings_ini: str
    interval: float
    write_interval: float
    window_sec: int
    metrics_path: str
    log_jsonl_ts_name: bool
    jsonl_rotate_mb: int
    jsonl_rotate_keep: int
    disk_guard_gib: float


class AppConfig:
    def __init__(
        self,
        ini: ConfigParser,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> None:
        self.directory_config: paths.DirectoryConfig = directory_config
        self.DEFAULT = DefaultConf(
            home_dir=_path(ini.get("DEFAULT", "home_dir")),
            File_Input=ini.getboolean("DEFAULT", "File_Input"),
            debug_log=_path(ini.get("DEFAULT", "debug_log")),
            data_dir=str(ini.get("DEFAULT", "data_dir")),
            print_disabled=ini.getboolean("DEFAULT", "print_disabled"),
            use_shi_lib=ini.getboolean("DEFAULT", "use_shi_lib"),
        )

        self.CalibMode = CalibrationModeSwitchConf(
            isRunning3D3Dcalib=ini.getboolean("CalibMode", "isRunning3D3Dcalib"),
            isRunning2D3Dcalib=ini.getboolean("CalibMode", "isRunning2D3Dcalib"),
            cameraID=ini.getint("CalibMode", "cameraID"),
            start2D3DCalibCalc=ini.getboolean("CalibMode", "start2D3DCalibCalc"),
            isRunning2D3Dcheck=ini.getboolean("CalibMode", "isRunning2D3Dcheck"),
            start2D3DCheckCalc=ini.getboolean("CalibMode", "start2D3DCheckCalc"),
            isRunningInterfaceDebug=ini.getboolean(
                "CalibMode", "isRunningInterfaceDebug"
            ),
        )

        self.General = GeneralConf(
            in_factory=ini.getboolean("General", "in_factory"),
            operation_mode=ini.getint("General", "operation_mode"),
            has_external_guard=ini.getboolean("General", "has_external_guard"),
            external_guard_offset=ini.getfloat("General", "external_guard_offset"),
            ground_height=ini.getfloat("General", "ground_height"),
            ground_height_margin=ini.getfloat("General", "ground_height_margin"),
            rotation_radius=ini.getfloat("General", "rotation_radius"),
            initial_transform_file=_path(ini.get("General", "initial_transform_file")),
            process_cpu_affinity_path=_path(
                ini.get("General", "process_cpu_affinity_path")
            ),
            enable_cpu_affinity=(ini.getboolean("General", "enable_cpu_affinity")),
        )

        self.UI_IF = UIIFConf(
            crane_model=ini.get("UI_IF", "crane_model"),
            UI_mmap=[_path(p) for p in parse_list(ini.get("UI_IF", "UI_mmap"))],
            damp_out=ini.getboolean("UI_IF", "damp_out"),
            damp_mmap=[_path(p) for p in parse_list(ini.get("UI_IF", "damp_mmap"))],
            bbox_3d_num=ini.getint("UI_IF", "bbox_3d_num"),
            bbox_3d_distance=ini.getfloat("UI_IF", "bbox_3d_distance"),
            show_unk=ini.getboolean("UI_IF", "show_unk"),
            collision_depict_dist=ini.getfloat("UI_IF", "collision_depict_dist"),
            collision_attention_dist=ini.getfloat("UI_IF", "collision_attention_dist"),
            collision_warning_dist=ini.getfloat("UI_IF", "collision_warning_dist"),
            cliff_attention_dist=ini.getfloat("UI_IF", "cliff_attention_dist"),
            cliff_warning_dist=ini.getfloat("UI_IF", "cliff_warning_dist"),
            draw_bbox_3d=ini.getboolean("UI_IF", "draw_bbox_3d"),
            draw_collision=ini.getboolean("UI_IF", "draw_collision"),
        )

        self.CalibUI_IF = CalibUIIFConf(
            godot_ui=ini.getboolean("CalibUI_IF", "godot_ui"),
            UI_mmap=[_path(p) for p in parse_list(ini.get("CalibUI_IF", "UI_mmap"))],
            damp_out=ini.getboolean("CalibUI_IF", "damp_out"),
            damp_mmap=[
                _path(p) for p in parse_list(ini.get("CalibUI_IF", "damp_mmap"))
            ],
            mmap_assign_json_path=_path(ini.get("CalibUI_IF", "mmap_assign_json_path")),
            show_trajectory=ini.getboolean("CalibUI_IF", "show_trajectory"),
            show_image2d3d=ini.getboolean("CalibUI_IF", "show_image2d3d"),
        )

        self.AppManager = AppManagerConf(
            interval=ini.getfloat("AppManager", "interval"),
            JudegeStopThr=ini.getint("AppManager", "JudegeStopThr"),
            monitor_argus_last_heartbeat_path=_path(
                ini.get("AppManager", "monitor_argus_last_heartbeat_path")
            ),
        )

        self.Monitor = MonitorConf(
            show_cam=parse_int_list(ini.get("Monitor", "show_cam")),
            coeff=ini.getfloat("Monitor", "coeff"),
        )

        self.Scrutinizer = ScrutinizerConf(
            s_frame=ini.getint("Scrutinizer", "s_frame"),
            e_frame=ini.getint("Scrutinizer", "e_frame"),
            file_input_loop=ini.getboolean("Scrutinizer", "file_input_loop"),
            v0_file=_path(ini.get("Scrutinizer", "v0_file")),
            v1_file=_path(ini.get("Scrutinizer", "v1_file")),
            v2_file=_path(ini.get("Scrutinizer", "v2_file")),
            fast_th_ms=ini.getfloat("Scrutinizer", "fast_th_ms"),
            slow_th_ms=ini.getfloat("Scrutinizer", "slow_th_ms"),
            short_que=ini.getint("Scrutinizer", "short_que"),
            long_que=ini.getint("Scrutinizer", "long_que"),
            get_data_sleep_sec=ini.getfloat("Scrutinizer", "get_data_sleep_sec"),
        )

        self.CAN = CANConf(
            config_file=_path(ini.get("CAN", "config_file")),
            IsOld=ini.getboolean("CAN", "IsOld"),
            interpretation=ini.getint("CAN", "interpretation"),
            yaw_offset_deg=ini.getfloat("CAN", "yaw_offset_deg"),
            c_file=_path(ini.get("CAN", "c_file")),
            can_id_map_file=_path(ini.get("CAN", "can_id_map_file")),
        )

        self.StateEstimator = StateEstimatorConf(
            window_sec=ini.getfloat("StateEstimator", "window_sec"),
            delta_db=ini.getfloat("StateEstimator", "delta_db"),
            p_on=ini.getfloat("StateEstimator", "p_on"),
            p_off=ini.getfloat("StateEstimator", "p_off"),
        )

        self.ReducedLoadMode = ReducedLoadModeConf(
            many_points_ratio=ini.getfloat(
                "ReducedLoadMode", "many_points_ratio"
            ),
            few_points_ratio=ini.getfloat("ReducedLoadMode", "few_points_ratio"),
        )

        self.Lidar = LidarConf(
            count=ini.getint("Lidar", "count"),
            path=_path(ini.get("Lidar", "path")),
            config_file=_path(ini.get("Lidar", "config_file")),
            accum_time=ini.getfloat("Lidar", "accum_time"),
            dev_str=ini.get("Lidar", "dev_str"),
            date_str=ini.get("Lidar", "date_str"),
            lidar0_file=_path(ini.get("Lidar", "lidar0_file")),
            lidar1_file=_path(ini.get("Lidar", "lidar1_file")),
            lidar2_file=_path(ini.get("Lidar", "lidar2_file")),
            lidar3_file=_path(ini.get("Lidar", "lidar3_file")),
            lidar4_file=_path(ini.get("Lidar", "lidar4_file")),
            lidar5_file=_path(ini.get("Lidar", "lidar5_file")),
            lidar_files=[_path(p) for p in parse_list(ini.get("Lidar", "lidar_files"))],
        )

        self.detect3d = Detect3dConf(
            eps=ini.getfloat("detect3d", "eps"),
            min_samples=ini.getint("detect3d", "min_samples"),
        )

        self.detect2d = Detect2dConf(
            is_applied=ini.getboolean("detect2d", "isApplied"),
            core_path=_path(ini.get("detect2d", "core_path")),
            model_path=_path(ini.get("detect2d", "model_path")),
            yolo_class=_path(ini.get("detect2d", "yolo_class")),
            onnx_model_path=_path(ini.get("detect2d", "onnx_model_path")),
            conf_thresh=ini.getfloat("detect2d", "conf_thresh"),
            nms_thresh=ini.getfloat("detect2d", "nms_thresh"),
        )

        self.camera = CameraConf(
            count=ini.getint("camera", "count"),
            config_file=_path(ini.get("camera", "config_file")),
            video_width=ini.getint("camera", "video_width"),
            video_height=ini.getint("camera", "video_height"),
            sys_width=ini.getint("camera", "sys_width"),
            sys_height=ini.getint("camera", "sys_height"),
            undistort_backend=ini.get("camera", "undistort_backend", fallback="auto"),
            porttable=parse_list(ini.get("camera", "porttable")),
            iptable=parse_list(ini.get("camera", "iptable")),
        )

        self.calibration = CalibrationConf(
            calib_lidar2crane=ini.getboolean("calibration", "calib_lidar2crane"),
            BothLidars=_path(ini.get("calibration", "BothLidars")),
            Lidar0=_path(ini.get("calibration", "Lidar0")),
            Lidar1=_path(ini.get("calibration", "Lidar1")),
            Lidar2=_path(ini.get("calibration", "Lidar2")),
            Lidar3=_path(ini.get("calibration", "Lidar3")),
            Lidar4=_path(ini.get("calibration", "Lidar4")),
            Lidar5=_path(ini.get("calibration", "Lidar5")),
            Lidar_calib_files=[
                _path(p)
                for p in parse_list(
                    ini.get("calibration", "lidar_calib_files")
                )  # CHANGE_LIDAR
            ],
            mtx_file=_path(ini.get("calibration", "mtx_file")),
            dist_file=_path(ini.get("calibration", "dist_file")),
            list_0_files=_path(ini.get("calibration", "list_0_files")),
            list_1_files=_path(ini.get("calibration", "list_1_files")),
            list_2_files=_path(ini.get("calibration", "list_2_files")),
            fisheye_param_file=_path(ini.get("calibration", "fisheye_param_file")),
            center_rotate=parse_float_tuple3(ini.get("calibration", "center_rotate")),
            center_shift=parse_float_tuple3(ini.get("calibration", "center_shift")),
            camera_position=parse_float_tuple3x3(
                ini.get("calibration", "camera_position")
            ),
            lidar_position=parse_float_tuple2x3(
                ini.get("calibration", "lidar_position")
            ),
            imu_baselines_file=_path(ini.get("calibration", "imu_baselines_file")),
        )

        self.machine = MachineConf(
            path=_path(ini.get("machine", "path")),
            vis_machine_dir=_path(ini.get("machine", "vis_machine_dir")),
            json_vis_machine_file=ini.get("machine", "json_vis_machine_file"),
            offset_rotate_center=parse_float_tuple3(
                ini.get("machine", "offset_rotate_center")
            ),
        )

        self.LiDARPosition = LidarPositionConf(
            x_offset=ini.getfloat("LiDARPosition", "x_offset"),
            y_offset=ini.getfloat("LiDARPosition", "y_offset"),
            z_offset=ini.getfloat("LiDARPosition", "z_offset"),
            x_reverse=ini.getboolean("LiDARPosition", "x_reverse"),
            y_reverse=ini.getboolean("LiDARPosition", "y_reverse"),
            z_reverse=ini.getboolean("LiDARPosition", "z_reverse"),
        )

        self.LiDARGrid = LidarGridConf(
            side_max=ini.getfloat("LiDARGrid", "side_max"),
            side_min=ini.getfloat("LiDARGrid", "side_min"),
            fwd_max=ini.getfloat("LiDARGrid", "fwd_max"),
            fwd_min=ini.getfloat("LiDARGrid", "fwd_min"),
            grid_size=ini.getfloat("LiDARGrid", "grid_size"),
        )

        self.Accumulation = AccumulationConf(
            accum_point=ini.getboolean("Accumulation", "accum_point"),
            num_skip_registration_frames=ini.getint(
                "Accumulation", "num_skip_registration_frames"
            ),
            registration_methods=ini.get("Accumulation", "registration_methods"),
            thr_skip_points=ini.getint("Accumulation", "thr_skip_points"),
            registration_range=ini.getfloat("Accumulation", "registration_range"),
            voxel_down_sample=ini.getfloat("Accumulation", "voxel_down_sample"),
            voxel_size_for_multi_icp=parse_float_list(
                ini.get("Accumulation", "voxel_size_for_multi_icp")
            ),
            correspondence_distances=parse_float_list(
                ini.get("Accumulation", "correspondence_distances")
            ),
            voxel_size_for_accumulated_points=ini.getfloat(
                "Accumulation", "voxel_size_for_accumulated_points"
            ),
            voxel_size_for_accumulated_points_reduced_load=ini.getfloat(
                "Accumulation", "voxel_size_for_accumulated_points_reduced_load"
            ),
            voxel_size_for_accumulated_ground_points=ini.getfloat(
                "Accumulation", "voxel_size_for_accumulated_ground_points"
            ),
            voxel_size_for_accumulated_ground_points_reduced_load=ini.getfloat(
                "Accumulation", "voxel_size_for_accumulated_ground_points_reduced_load"
            ),
            increment_accum_counter=ini.getint(
                "Accumulation", "increment_accum_counter"
            ),
            decrement_accum_counter=ini.getint(
                "Accumulation", "decrement_accum_counter"
            ),
            decrement_speed_threshold=ini.getint(
                "Accumulation", "decrement_speed_threshold"
            ),
            decrement_accum_counter_slow=ini.getint(
                "Accumulation", "decrement_accum_counter_slow"
            ),
            prob_present_threshold=ini.getint("Accumulation", "prob_present_threshold"),
            accum_counter_lower_reset_threshold=ini.getint(
                "Accumulation", "accum_counter_lower_reset_threshold"
            ),
            accum_counter_max_cap=ini.getint("Accumulation", "accum_counter_max_cap"),
            max_accumulated_frames=ini.getint("Accumulation", "max_accumulated_frames"),
            max_accumulated_frames_reduced_load=ini.getint(
                "Accumulation", "max_accumulated_frames_reduced_load"
            ),
            max_accumulated_frames_ground=ini.getint(
                "Accumulation", "max_accumulated_frames_ground"
            ),
            max_accumulated_frames_ground_reduced_load=ini.getint(
                "Accumulation", "max_accumulated_frames_ground_reduced_load"
            ),
        )

        self.OctoTree = OctoTreeConf(
            func_on=ini.getboolean("OctoTree", "func_on"),
            col_machine_dir=_path(ini.get("OctoTree", "col_machine_dir")),
            json_col_machine_file=ini.get("OctoTree", "json_col_machine_file"),
            max_xyz=parse_float_list(ini.get("OctoTree", "max_xyz")),
            min_xyz=parse_float_list(ini.get("OctoTree", "min_xyz")),
            max_tree_depth=ini.getint("OctoTree", "max_tree_depth"),
            remove_dist=ini.getfloat("OctoTree", "remove_dist"),
            clustering_tree_depth=ini.getint("OctoTree", "clustering_tree_depth"),
            use_node_stats=ini.getboolean("OctoTree", "use_node_stats"),
        )

        self.CollisionDetection = CollisionDetectionConf(
            func_on=ini.getboolean("CollisionDetection", "func_on"),
            collision_detector_name=ini.get(
                "CollisionDetection", "collision_detector_name"
            ),
            dialate_point_size=ini.getint("CollisionDetection", "dialate_point_size"),
            distance_threshold=ini.getfloat("CollisionDetection", "distance_threshold"),
            detect_focus_range=parse_float_list(
                ini.get("CollisionDetection", "detect_focus_range")
            ),
            coord_method=ini.get("CollisionDetection", "coord_method"),
            max_dist=ini.getfloat("CollisionDetection", "max_dist"),
            detectable_height=ini.getfloat("CollisionDetection", "detectable_height"),
            detectable_ground_offset=ini.getfloat(
                "CollisionDetection", "detectable_ground_offset"
            ),
            grid_intervals=parse_int_tuple3(
                ini.get("CollisionDetection", "grid_intervals")
            ),
            min_radius=ini.getfloat("CollisionDetection", "min_radius"),
            max_radius=ini.getfloat("CollisionDetection", "max_radius"),
            key_num=ini.getint("CollisionDetection", "key_num"),
        )

        self.SceneDescription = SceneDescriptionConf(
            coarse_lo=ini.getfloat("SceneDescription", "coarse_lo"),
            coarse_hi=ini.getfloat("SceneDescription", "coarse_hi"),
            k_min=ini.getfloat("SceneDescription", "k_min"),
            h_ref_px=ini.getint("SceneDescription", "h_ref_px"),
            lo_gain=ini.getfloat("SceneDescription", "lo_gain"),
            hi_gain=ini.getfloat("SceneDescription", "hi_gain"),
            lo_floor=ini.getfloat("SceneDescription", "lo_floor"),
            hi_ceil=ini.getfloat("SceneDescription", "hi_ceil"),
            vertical_w_iou=ini.getfloat("SceneDescription", "vertical_w_iou"),
            vertical_w_scale=ini.getfloat("SceneDescription", "vertical_w_scale"),
            vertical_w_phi=ini.getfloat("SceneDescription", "vertical_w_phi"),
            final_threshold=ini.getfloat("SceneDescription", "final_threshold"),
            use_human_gate=ini.getboolean("SceneDescription", "use_human_gate"),
            H_min=ini.getfloat("SceneDescription", "H_min"),
            H_max=ini.getfloat("SceneDescription", "H_max"),
            W_min=ini.getfloat("SceneDescription", "W_min"),
            W_max=ini.getfloat("SceneDescription", "W_max"),
            D_min=ini.getfloat("SceneDescription", "D_min"),
            D_max=ini.getfloat("SceneDescription", "D_max"),
            tall_ratio_min=ini.getfloat("SceneDescription", "tall_ratio_min"),
        )

        self.Visualizer = VisualizerConf(
            display_point=ini.getboolean("Visualizer", "display_point"),
            box_size=ini.getfloat("Visualizer", "box_size"),
            rotate_grid=ini.getboolean("Visualizer", "rotate_grid"),
            save_dir_path=_path(ini.get("Visualizer", "save_dir_path")),
            coeff=ini.getfloat("Visualizer", "coeff"),
            radius=ini.getfloat("Visualizer", "radius"),
            display_octree=ini.getboolean("Visualizer", "display_octree"),
            display_octree_method=ini.get("Visualizer", "display_octree_method"),
        )

        self.EdgeDetection = EdgeDetectionConf(
            detector=ini.get("EdgeDetection", "detector"),
            is_applied=ini.getboolean("EdgeDetection", "isApplied"),
            func_mode=ini.get("EdgeDetection", "func_mode"),
            resolution=ini.getfloat("EdgeDetection", "resolution"),
            grid_size=parse_float_tuple2(ini.get("EdgeDetection", "grid_size")),
            side_range=parse_float_tuple2(ini.get("EdgeDetection", "side_range")),
            fwd_range=parse_float_tuple2(ini.get("EdgeDetection", "fwd_range")),
            bev_min_z=ini.getfloat("EdgeDetection", "bev_min_z"),
            bev_max_z=ini.getfloat("EdgeDetection", "bev_max_z"),
            voxel_size=ini.getfloat("EdgeDetection", "voxel_size"),
            range_properties=parse_list(ini.get("EdgeDetection", "range_properties")),
            detect_range=parse_float_tuple2(ini.get("EdgeDetection", "detect_range")),
            detect_offset=parse_float_tuple2(ini.get("EdgeDetection", "detect_offset")),
            height_strip=ini.getfloat("EdgeDetection", "height_strip"),
            edge_width=ini.getfloat("EdgeDetection", "edge_width"),
            polar_origin=parse_float_tuple3(ini.get("EdgeDetection", "polar_origin")),
            polar_shape=parse_int_tuple2(ini.get("EdgeDetection", "polar_shape")),
            polar_min_radius=ini.getfloat("EdgeDetection", "polar_min_radius"),
            machine_occ_label=ini.getint("EdgeDetection", "machine_occ_label"),
            target_edge_dist_th=ini.getfloat("EdgeDetection", "target_edge_dist_th"),
            search_radius_offset=ini.getint("EdgeDetection", "search_radius_offset"),
            remove_duplicate_label=ini.getboolean(
                "EdgeDetection", "remove_duplicate_label"
            ),
            edge_z_offset=ini.getfloat("EdgeDetection", "edge_z_offset"),
            bin_th=ini.getint("EdgeDetection", "bin_th"),
            occ_focus_range=ini.getint("EdgeDetection", "occ_focus_range"),
            edge_filter_size=ini.getint("EdgeDetection", "edge_filter_size"),
            debug=ini.getboolean("EdgeDetection", "debug"),
        )

        self.LiDARShiftMonitor = LiDARShiftMonitorConf(
            win=ini.getint("LiDARShiftMonitor", "win"),
            hold=ini.getint("LiDARShiftMonitor", "hold"),
            thr_slow=ini.getfloat("LiDARShiftMonitor", "thr_slow"),
            win_fast=ini.getint("LiDARShiftMonitor", "win_fast"),
            hold_fast=ini.getint("LiDARShiftMonitor", "hold_fast"),
            thr_fast=ini.getfloat("LiDARShiftMonitor", "thr_fast"),
            g_mag=ini.getfloat("LiDARShiftMonitor", "g_mag"),
            num_sample_k=ini.getint("LiDARShiftMonitor", "num_sample_k"),
            dt=ini.getfloat("LiDARShiftMonitor", "dt"),
            has_not_calibrated_path=ini.get(
                "LiDARShiftMonitor", "has_not_calibrated_path"
            ),
        )

        self.debug = DebugConf(
            record_time_folder_path=_path(ini.get("debug", "record_time_folder_path")),
            draw_yolo=ini.getboolean("debug", "draw_yolo"),
        )

        self.eval_general = EvalGeneralConf(
            number_of_frames_used=ini.getint("eval_general", "number_of_frames_used"),
            fdir=_path(ini.get("eval_general", "fdir")),
            fname=ini.get("eval_general", "fname"),
        )

        self.eval_calib = EvalCalibConf(
            Lidar1_calib=_path(ini.get("eval_calib", "Lidar1_calib")),
            Lidar2_calib=_path(ini.get("eval_calib", "Lidar2_calib")),
            Lidar3_calib=_path(ini.get("eval_calib", "Lidar3_calib")),
            Lidar4_calib=_path(ini.get("eval_calib", "Lidar4_calib")),
        )

        self.eval_data = EvalDataConf(
            Lidar1_points=_path(ini.get("eval_data", "Lidar1_points")),
            Lidar2_points=_path(ini.get("eval_data", "Lidar2_points")),
            Lidar3_points=_path(ini.get("eval_data", "Lidar3_points")),
            Lidar4_points=_path(ini.get("eval_data", "Lidar4_points")),
        )

        sec_name = "eval_collision"
        self.eval_collision = EvalCollisionConf(
            func_on=ini.getboolean(sec_name, "func_on"),
            detect_range=parse_float_tuple2(ini.get(sec_name, "detect_range")),
            grid_size=parse_float_tuple2(ini.get(sec_name, "grid_size")),
            height_offset=ini.getfloat(sec_name, "height_offset"),
            grid_color=parse_int_tuple3(ini.get(sec_name, "grid_color")),
        )
        self.jetson_monitor = JetsonMonitorConf(
            is_applied=ini.getboolean("JetsonMonitor", "isApplied"),
            settings_ini=_path(ini.get("JetsonMonitor", "settings_ini")),
            interval=ini.getfloat("JetsonMonitor", "interval"),
            write_interval=ini.getfloat("JetsonMonitor", "write_interval"),
            window_sec=ini.getint("JetsonMonitor", "window_sec"),
            metrics_path=_path(ini.get("JetsonMonitor", "metrics_path")),
            log_jsonl_ts_name=ini.getboolean("JetsonMonitor", "log_jsonl_ts_name"),
            jsonl_rotate_mb=ini.getint("JetsonMonitor", "jsonl_rotate_mb"),
            jsonl_rotate_keep=ini.getint("JetsonMonitor", "jsonl_rotate_keep"),
            disk_guard_gib=ini.getfloat("JetsonMonitor", "disk_guard_gib"),
        )


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


def parse_float_list(text: str) -> list[float]:
    return list(map(float, parse_list(text=text)))


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
    ini = ConfigParser(interpolation=ExtendedInterpolation())
    ini.read("./config/settings.ini", "UTF-8")
    app_config = AppConfig(ini)
    print(app_config)
    print(app_config.calibration)
    print(app_config.Lidar)
