from argus_synchro.config.app_config_calibration import (
    AppConfigCalibration,
    DataCaptureConf,
)
from argus_synchro.shared_app_config import SharedAppConfig


def lidar_calib_filepath_loader(
    sac: SharedAppConfig, app_config_calib: AppConfigCalibration
) -> list[str]:
    if not sac.read().CalibMode.isRunning2D3Dcalib:
        return app_config_calib.dataConverter2D3D.Lidar.lidar_calib_files_for_othermode
    if sac.read().CalibMode.cameraID == 0:
        return app_config_calib.dataConverter2D3D.Lidar.lidar_calib_files_for_cam0calib
    if sac.read().CalibMode.cameraID == 1:
        return app_config_calib.dataConverter2D3D.Lidar.lidar_calib_files_for_cam1calib
    if sac.read().CalibMode.cameraID == 2:
        return app_config_calib.dataConverter2D3D.Lidar.lidar_calib_files_for_cam2calib
    return app_config_calib.dataConverter2D3D.Lidar.lidar_calib_files_for_othermode


def video_filepath_loader(
    sac: SharedAppConfig, cameraConf: DataCaptureConf.CameraConf
) -> list[str]:
    if not sac.read().CalibMode.isRunning2D3Dcalib:
        return cameraConf.video_files_for_othermode
    if sac.read().CalibMode.cameraID == 0:
        return cameraConf.video_files_for_cam0calib
    if sac.read().CalibMode.cameraID == 1:
        return cameraConf.video_files_for_cam1calib
    if sac.read().CalibMode.cameraID == 2:
        return cameraConf.video_files_for_cam2calib
    return cameraConf.video_files_for_othermode


def lidar_filepath_loader(
    sac: SharedAppConfig, lidarConf: DataCaptureConf.LidarConf
) -> list[str]:
    if not sac.read().CalibMode.isRunning2D3Dcalib:
        return lidarConf.lidar_files_for_othermode
    if sac.read().CalibMode.cameraID == 0:
        return lidarConf.lidar_files_for_cam0calib
    if sac.read().CalibMode.cameraID == 1:
        return lidarConf.lidar_files_for_cam1calib
    if sac.read().CalibMode.cameraID == 2:
        return lidarConf.lidar_files_for_cam2calib
    return lidarConf.lidar_files_for_othermode
