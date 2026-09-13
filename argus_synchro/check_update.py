from argus_synchro.config.app_config import LidarPositionConf


def is_required_initial_offsets_update(
    initial_offsets: tuple[float, float, float],
    lidarposition: LidarPositionConf,
) -> bool:
    return initial_offsets != (
        lidarposition.x_offset,
        lidarposition.y_offset,
        lidarposition.z_offset,
    )
