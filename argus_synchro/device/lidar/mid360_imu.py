import socket
import struct
import time

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig


class MID360ImuFile:
    def __init__(self, index: int, app_conf: AppConfig) -> None:
        self._index: int = index
        self._app_conf: AppConfig = app_conf

    def connect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: connect()"
        raise NotImplementedError(err_msg)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def get_imu(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
        time.sleep(0.1)  # LiDARのテストデータの更新頻度に合わせてスリープする
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)


class MID360Imu:
    def __init__(
        self,
        index: int,
        lidar_config: dict[str, str | int],
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

        self._index: int = index
        self._host_ip: str = str(lidar_config["dest_ip"])
        self._port: int = int(lidar_config["port_imu"])

        self._logger.info("Host: %s", self._host_ip)
        self._logger.info("Port(IMU): %s", self._port)

        self._socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # mid360クラスを宣言したタイミングでMID360にも接続する。
        # imu: IMUデータ受け取り用。
        self.connect()

    def __del__(self) -> None:
        self.disconnect()

    def connect(self) -> None:
        self._socket.bind((self._host_ip, self._port))
        self._socket.settimeout(0.1)
        self._logger.info("CONNECTION: OK")

    def disconnect(self) -> None:
        self._socket.close()
        self._logger.info("DISCONNECT: OK")

    @staticmethod
    def _get_reflect(dst_byte: bytes, init_idx: int, offset: int, i: int) -> int:
        return int.from_bytes(
            dst_byte[init_idx + 4 * 3 + offset * i : init_idx + 4 * 3 + 1 + offset * i],
            "little",
            signed=True,
        )

    @staticmethod
    def _get_timestamp(dst_byte: bytes) -> float:
        # xyz[mm]から[m]に変換するために/1000する
        return (
            int.from_bytes(
                dst_byte[
                    (1 + 2 + 2 + 2 + 2 + 1 + 1 + 1 + 12 + 4) : (
                        1 + 2 + 2 + 2 + 2 + 1 + 1 + 1 + 12 + 4 + 8
                    )
                ],
                "little",
                signed=True,
            )
            / 1e9
        )

    def get_imu(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
        init_idx = 36

        # パケットデータを取得　全体で60byteだが、少し多めに
        try:
            imu_byte = self._socket.recv(100)
        except socket.timeout:
            # TODO(NSW): IMU接続エラーはSHI様実装のため、仮実装
            self._logger.warning("IMUデータの受信がタイムアウトしました。")
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)

        # ジャイロ[rad/s]、加速度[g]
        # 最初の文字を "<"にすることでバイトオーダをlittle-endianに指定
        # ">"だとbig-endian
        gyro_x: float = struct.unpack("<f", imu_byte[init_idx : init_idx + 4])[0]
        gyro_y: float = struct.unpack("<f", imu_byte[init_idx + 4 : init_idx + 8])[0]
        gyro_z: float = struct.unpack("<f", imu_byte[init_idx + 8 : init_idx + 12])[0]
        acce_x: float = struct.unpack("<f", imu_byte[init_idx + 12 : init_idx + 16])[0]
        acce_y: float = struct.unpack("<f", imu_byte[init_idx + 16 : init_idx + 20])[0]
        acce_z: float = struct.unpack("<f", imu_byte[init_idx + 20 : init_idx + 24])[0]

        ts: float = self._get_timestamp(imu_byte)
        return (gyro_x, gyro_y, gyro_z), (acce_x, acce_y, acce_z), ts
