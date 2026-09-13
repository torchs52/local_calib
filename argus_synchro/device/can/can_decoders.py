from collections.abc import Callable
from typing import TypeAlias

from argus_synchro.common.app_logger import AppLogger

DecoderFn: TypeAlias = Callable[[str, float, AppLogger], tuple[float, ...]]


def handle_angle_oldcan(
    raw_can_data: str, yaw_offset_deg: float, logger: AppLogger
) -> tuple[float]:
    """
    旋回角度取得(old_can)
    """
    current_degree = 360.0 - (int(raw_can_data[4:8], 16) / 10.0)
    yaw_angle_deg = current_degree + yaw_offset_deg
    logger.info("handle_angle_oldcan, can deg(old, handler): %f", yaw_angle_deg)
    return (yaw_angle_deg,)


def handle_angle_newcan(
    raw_can_data: str, yaw_offset_deg: float, logger: AppLogger
) -> tuple[float]:
    """
    旋回角度取得(new_can)
    """
    current_degree = (
        int(raw_can_data[:2], 16) + int(raw_can_data[2:4], 16) * 255
    ) / (13400.0 / 360.0)
    yaw_angle_deg = current_degree - yaw_offset_deg
    logger.info("handle_angle_newcan, can deg(new, handler): %f", yaw_angle_deg)
    return (yaw_angle_deg,)


def handle_angle_newcan_inverted(
    raw_can_data: str, yaw_offset_deg: float, logger: AppLogger
) -> tuple[float]:
    """
    旋回角度取得(new_can)
    """
    current_degree = (
        int(raw_can_data[:2], 16) + int(raw_can_data[2:4], 16) * 255
    ) / (13400.0 / 360.0)
    current_degree = (360.0 - current_degree) % 360.0 # 旋回角度の正方向を合わせるために符号反転
    yaw_angle_deg = current_degree - yaw_offset_deg
    logger.info(
        "handle_angle_newcan_inverted, can deg(new, handler): %f",
        yaw_angle_deg,
    )
    return (yaw_angle_deg,)


def handle_lever(
    raw_can_data: str, _yaw_offset_deg: float, logger: AppLogger
) -> tuple[float, ...]:
    """
    レバー圧力取得
    """
    logger.info("handle_lever, lever handler data: %s", raw_can_data)
    payload = raw_can_data[:-2]  # 末尾2桁は DLC
    payload_bytes = bytes.fromhex(payload)
    lever_pressure = tuple(
        int.from_bytes(payload_bytes[index : index + 2], "big") * 0.001  # unsigned 16 bit x 0.001
        for index in (0, 2, 4, 6)
    )
    logger.info(
        "handle_lever, lever handler data: %f, %f, %f, %f",
        *lever_pressure,
    )
    return lever_pressure


def handle_angle_can_scx2000(
    raw_can_data: str, yaw_offset_deg: float, logger: AppLogger
) -> tuple[float]:
    """
    旋回角度取得(can scx2000)
    """
    # 200t機旋回角度　開始ビット48, 1文字4bit, 16文字64bit 48-64は12-15文字 CanMsg_To_Dataで1ワード8bitに直しているので結局6,7を読めばよい

    #tmp_string: str = (
    #    msg_data[7 * 2 : 7 * 2 + 2] + msg_data[6 * 2 : 6 * 2 + 2]
    #)  # 7byte目(7*2~7*2+1)と6byte目(6*2~6*2+1)を結合

    raw_angle = raw_can_data[14:16] + raw_can_data[12:14]
    current_degree = int(raw_angle, 16) / 10.0
    # よくわからないので一旦オフセット無しで。0.1度単位のintで送られてくるので/10して角度へ

    yaw_angle_deg = (-(current_degree - yaw_offset_deg)) % 360.0
    logger.info(
        "handle_angle_can_scx2000, can deg(can scx2000, handler): %f",
        yaw_angle_deg,
    )
    return (yaw_angle_deg,)


DECODER_REGISTRY: dict[str, DecoderFn] = {
    "handle_angle_oldcan": handle_angle_oldcan,
    "handle_angle_newcan": handle_angle_newcan,
    "handle_angle_newcan_inverted": handle_angle_newcan_inverted,
    "handle_lever": handle_lever,
    "handle_angle_can_scx2000": handle_angle_can_scx2000,
}


# ...同様に必要があれば他のIDごとにメソッドを追加...
