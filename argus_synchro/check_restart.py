from argus_synchro.config.app_config import AppConfig


def check_restart_is_required(old: AppConfig, new: AppConfig) -> bool:
    """
    もし再起動が必要ならTrue,不要ならFalse

    Parameters:
    old_app_config: 過去のapp_config
    new_app_config: 新しいapp_config
    """

    return (
        # root directories (config_dir / log_dir / mmap_dir)
        old.directory_config != new.directory_config
        or
        # DEFAULT
        old.DEFAULT.home_dir != new.DEFAULT.home_dir
        or old.DEFAULT.File_Input != new.DEFAULT.File_Input
        or old.DEFAULT.debug_log != new.DEFAULT.debug_log
        or old.DEFAULT.data_dir != new.DEFAULT.data_dir
        or old.DEFAULT.use_shi_lib != new.DEFAULT.use_shi_lib
        # General
        or old.General.process_cpu_affinity_path
        != new.General.process_cpu_affinity_path
        or old.General.enable_cpu_affinity != new.General.enable_cpu_affinity
        or old.General.in_factory != new.General.in_factory
        # UI_IF
        or old.UI_IF.crane_model != new.UI_IF.crane_model
        or old.UI_IF.UI_mmap != new.UI_IF.UI_mmap
        or old.UI_IF.damp_out != new.UI_IF.damp_out
        or old.UI_IF.damp_mmap != new.UI_IF.damp_mmap
        # AppManager
        or old.AppManager.interval != new.AppManager.interval
        # Monitor
        # Scrutinizer
        or old.Scrutinizer.s_frame != new.Scrutinizer.s_frame
        or old.Scrutinizer.v0_file != new.Scrutinizer.v0_file
        or old.Scrutinizer.v1_file != new.Scrutinizer.v1_file
        or old.Scrutinizer.v2_file != new.Scrutinizer.v2_file
        # CAN
        or old.CAN.config_file != new.CAN.config_file
        or old.CAN.IsOld != new.CAN.IsOld
        or old.CAN.interpretation != new.CAN.interpretation
        or old.CAN.c_file != new.CAN.c_file
        or old.CAN.can_id_map_file != new.CAN.can_id_map_file
        # Lidar
        or old.Lidar.count != new.Lidar.count
        or old.Lidar.path != new.Lidar.path
        or old.Lidar.config_file != new.Lidar.config_file
        or old.Lidar.dev_str != new.Lidar.dev_str
        or old.Lidar.date_str != new.Lidar.date_str
        or old.Lidar.lidar_files != new.Lidar.lidar_files
        # detect3d
        # detect2d
        or old.detect2d.core_path != new.detect2d.core_path
        or old.detect2d.model_path != new.detect2d.model_path
        or old.detect2d.yolo_class != new.detect2d.yolo_class
        or old.detect2d.onnx_model_path != new.detect2d.onnx_model_path
        # camera
        or old.camera.count != new.camera.count
        or old.camera.config_file != new.camera.config_file
        or old.camera.video_width != new.camera.video_width
        or old.camera.video_height != new.camera.video_height
        or old.camera.sys_width != new.camera.sys_width
        or old.camera.sys_height != new.camera.sys_height
        or old.camera.porttable != new.camera.porttable
        or old.camera.iptable != new.camera.iptable
        # machine
        or old.machine.path != new.machine.path
        or old.machine.vis_machine_dir != new.machine.vis_machine_dir
        or old.machine.json_vis_machine_file != new.machine.json_vis_machine_file
        # LiDARGrid
        or old.LiDARGrid.side_max != new.LiDARGrid.side_max
        or old.LiDARGrid.side_min != new.LiDARGrid.side_min
        or old.LiDARGrid.fwd_max != new.LiDARGrid.fwd_max
        or old.LiDARGrid.fwd_min != new.LiDARGrid.fwd_min
        or old.LiDARGrid.grid_size != new.LiDARGrid.grid_size
        # Accumulation
        or old.Accumulation.accum_point != new.Accumulation.accum_point
        # OctoTree
        or old.OctoTree.func_on != new.OctoTree.func_on
        or old.OctoTree.col_machine_dir != new.OctoTree.col_machine_dir
        or old.OctoTree.json_col_machine_file != new.OctoTree.json_col_machine_file
        or old.OctoTree.max_xyz != new.OctoTree.max_xyz
        or old.OctoTree.min_xyz != new.OctoTree.min_xyz
        or old.OctoTree.max_tree_depth != new.OctoTree.max_tree_depth
        or old.OctoTree.clustering_tree_depth != new.OctoTree.clustering_tree_depth
        # CollisionDetection
        or old.CollisionDetection.collision_detector_name
        != new.CollisionDetection.collision_detector_name
        # LiDARShiftMonitor
        or old.LiDARShiftMonitor.win != new.LiDARShiftMonitor.win
        or old.LiDARShiftMonitor.hold != new.LiDARShiftMonitor.hold
        or old.LiDARShiftMonitor.win_fast != new.LiDARShiftMonitor.win_fast
        or old.LiDARShiftMonitor.hold_fast != new.LiDARShiftMonitor.hold_fast
        # visualizer
        or old.Visualizer.display_point != new.Visualizer.display_point
        or old.Visualizer.save_dir_path != new.Visualizer.save_dir_path
        or old.debug.record_time_folder_path != new.debug.record_time_folder_path
        # eval_general
    )
