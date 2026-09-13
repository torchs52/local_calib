from __future__ import annotations

import json
import math
import socket
import struct
from typing import TYPE_CHECKING, Any, LiteralString

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory

if TYPE_CHECKING:
    from argus_synchro.config.app_config import AppConfig


CHANNEL_BLOCK_COUNT = 128  # Channel blocks per Azimuth block
OS_128_CHANNELS: tuple[int, ...] = tuple(i for i in range(CHANNEL_BLOCK_COUNT))

# packet
PACKET_SIZE = 24896
TICKS_PER_REVOLUTION = 90112
AZIMUTH_BLOCK_COUNT = 16  # Azimuth blocks per packet
RANGE_BIT_MASK = 0x000FFFFF
CHANNEL_BLOCK = (
    "I"  # Range (20 bits, 12 unused)
    "H"  # Reflectivity
    "H"  # Signal photons
    "H"  # Noise photons
    "H"  # Unused
)
CHANNEL_BLOCK_SIZE: int = len(CHANNEL_BLOCK)
AZIMUTH_BLOCK = (
    "Q"  # Timestamp
    "H"  # Measurement ID
    "H"  # Frame ID
    "I"  # Encoder Count
    f"{CHANNEL_BLOCK * CHANNEL_BLOCK_COUNT}"  # Channel Data
    "I"  # Status
)
AZIMUTH_BLOCK_SIZE: int = len(AZIMUTH_BLOCK)
PACKET: LiteralString = "<" + (AZIMUTH_BLOCK * AZIMUTH_BLOCK_COUNT)
RADIANS_360: float = 2 * math.pi

# Only compile the format string once
_unpack = struct.Struct(PACKET).unpack


def unpack(raw_packet: bytes) -> tuple[int, ...]:
    return _unpack(raw_packet)


def azimuth_block(n: int, packet: tuple[int, ...]) -> tuple[int, ...]:
    offset = n * AZIMUTH_BLOCK_SIZE
    return packet[offset : offset + AZIMUTH_BLOCK_SIZE]


def azimuth_timestamp(azimuth_block: tuple[int, ...]) -> int:
    return azimuth_block[0]


def azimuth_measurement_id(azimuth_block: tuple[int, ...]) -> int:
    return azimuth_block[1]


def azimuth_frame_id(azimuth_block: tuple[int, ...]) -> int:
    return azimuth_block[2]


def azimuth_encoder_count(azimuth_block: tuple[int, ...]) -> int:
    return azimuth_block[3]


def azimuth_angle(azimuth_block: tuple[int, ...]) -> float:
    return RADIANS_360 * azimuth_block[3] / TICKS_PER_REVOLUTION


def azimuth_valid(azimuth_block: tuple[int, ...]) -> bool:
    return azimuth_block[-1] != 0


def channel_block(n: int, azimuth_block: tuple[int, ...]) -> tuple[int, ...]:
    offset = 4 + n * CHANNEL_BLOCK_SIZE
    return azimuth_block[offset : offset + CHANNEL_BLOCK_SIZE]


def channel_range(channel_block: tuple[int, ...]) -> int:
    return channel_block[0] & RANGE_BIT_MASK


def channel_reflectivity(channel_block: tuple[int, ...]) -> int:
    return channel_block[1]


def channel_signal_photons(channel_block: tuple[int, ...]) -> int:
    return channel_block[2]


def channel_noise_photons(channel_block: tuple[int, ...]) -> int:
    return channel_block[3]


class OS0ConfigurationError(Exception):
    pass


class OS0API:
    def __init__(self, host: str, port: int = 7501) -> None:
        self.address: tuple[str, int] = (host, port)
        self._error: str | None = None

    def get_sensor_info(self) -> str:
        return self._send("get_sensor_info")

    def get_beam_intrinsics(self) -> str:
        return self._send("get_beam_intrinsics")

    def get_time_info(self) -> str:
        return self._send("get_time_info")

    def get_imu_intrinsics(self) -> str:
        return self._send("get_imu_intrinsics")

    def get_lidar_intrinsics(self) -> str:
        return self._send("get_lidar_intrinsics")

    def get_config_param(self, *args: str) -> str:
        command: str = "get_config_param {}".format(" ".join(args))
        return self._send(command)

    def set_config_param(self, *args: str) -> str:
        command: str = "set_config_param {}".format(" ".join(args))
        return self._send(command)

    def reinitialize(self) -> str:
        return self._send("reinitialize")

    def raise_for_error(self) -> None:
        if self.has_error:
            raise OS0ConfigurationError(self._error)

    @property
    def has_error(self) -> bool:
        return self._error is not None

    def _send(self, command: str, *args: str) -> str:
        self._error = None
        payload: bytes = " ".join([command, *args]).encode("utf-8") + b"\n"
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect(self.address)
            sock.sendall(payload)
            response = b""
            while not response.endswith(b"\n"):
                response += sock.recv(1024)
        self._error_check(response)
        return response.decode("utf-8")

    def _error_check(self, response: bytes) -> None:
        res: str = response.decode("utf-8")
        if res.startswith("error"):
            self._error = res
        else:
            self._error = None


class OS0128PointsFile:
    def __init__(
        self,
        index: int,
        app_conf: AppConfig,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._index = index
        self._app_conf = app_conf

    def connect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: connect()"
        raise NotImplementedError(err_msg)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def get_points(self) -> tuple[list[tuple[float, float, float, int]], float]:
        err_msg = f"class: {self.__class__.__name__}, method: get_points()"
        raise NotImplementedError(err_msg)


class OS0128Points:
    def __init__(
        self,
        index: int,
        lidar_config: dict[str, str | int],
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._index: int = index

        # 点群取得用の接続情報
        self._host_ip: str = str(lidar_config["dest_ip"])
        self._port_pnt: int = int(lidar_config["port_pnt"])
        self.connect()

        # API操作用の接続情報
        self._id: str = str(lidar_config["hostname"])
        self._port_api: int = int(lidar_config["port_tcp"])
        # API確立
        self._os0_api = OS0API(f"os-{self._id}.local", port=self._port_api)

    def __del__(self) -> None:
        self.disconnect()

    @staticmethod
    def _connect(host_ip: str, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((host_ip, port))
        return sock

    def connect(self) -> None:
        self._point: socket.socket = self._connect(self._host_ip, self._port_pnt)
        self._logger.info("CONNECTION: OK")

    def disconnect(self) -> None:
        self._point.close()

    def build_trig_table(
        self,
        _trig_table: list[list[float]],
        beam_altitude_angles: list[float],
        beam_azimuth_angles: list[float],
    ) -> list[list[float]]:
        if not _trig_table:
            for i in range(CHANNEL_BLOCK_COUNT):
                _trig_table.append(
                    [
                        math.sin(beam_altitude_angles[i] * math.radians(1)),
                        math.cos(beam_altitude_angles[i] * math.radians(1)),
                        beam_azimuth_angles[i] * math.radians(1),
                    ],
                )
        return _trig_table

    def xyz_point(
        self,
        channel_n: int,
        azimuth_block: tuple[int, ...],
        _trig_table: list[list[float]],
    ) -> tuple[float, float, float, int]:
        channel: tuple[int, ...] = channel_block(channel_n, azimuth_block)
        table_entry: list[float] = _trig_table[channel_n]
        range = channel_range(channel) / 1000  # to meters
        r: int = channel_reflectivity(channel)
        adjusted_angle: float = table_entry[2] + azimuth_angle(azimuth_block)
        x: float = -range * table_entry[1] * math.cos(adjusted_angle)
        y: float = range * table_entry[1] * math.sin(adjusted_angle)
        z: float = range * table_entry[0]

        return (x, y, z, r)

    def get_points(self) -> tuple[list[tuple[float, float, float, int]], float]:
        """
        点群データを取得
        """
        _trig_table: list[list[float]] = []
        xyzr: list[tuple[float, float, float, int]] = []
        ts: float = 0.0

        # TODO: 初期化時に1回だけ実施すれば良いか調査し対応する。実機無いため後日対応とする。 (NSW)
        # APIでOS0128からパラメータを取得する処理。
        beam_intrinsics: dict[str, Any] = json.loads(
            self._os0_api.get_beam_intrinsics()
        )

        # LiDARパラメータの取得
        _trig_table = self.build_trig_table(
            _trig_table,
            beam_intrinsics["beam_altitude_angles"],
            beam_intrinsics["beam_azimuth_angles"],
        )

        # TODO: ここにAPI経由でLiDARのPortを設定するプログラム挿入 (NSW)
        dst_byte = b""  # 座標データが格納される変数

        dst_byte, _ = self._point.recvfrom(25000)
        packet: tuple[int, ...] = unpack(dst_byte)

        for b in range(AZIMUTH_BLOCK_COUNT):
            block: tuple[float, ...] = azimuth_block(b, packet)
            ts = azimuth_timestamp(block)

            if not azimuth_valid(block):
                continue

            for c in OS_128_CHANNELS:
                points: tuple[float, float, float, int] = self.xyz_point(
                    c, block, _trig_table
                )
                xyzr.append(points)

        return xyzr, ts
