from argus_synchro.config.app_config_calibration import AppConfigCalibration

# 静止点群除去、範囲制限をこちらに持ってくる


class points_preprocess:
    def __init__(self, app_config_calib: AppConfigCalibration) -> None:
        self.app_config_calib = app_config_calib
