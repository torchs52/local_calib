import socket
import time
from typing import Final

import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import LidarConf


class MID360PointsFile:
    def __init__(
        self,
        index: int,
        lidar_file_path: str,
        lidar_conf: LidarConf,
        start_frame: int,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.file: str = lidar_file_path
        self._accum_time = lidar_conf.accum_time
        self._ref_t = start_frame

    def connect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: connect()"
        raise NotImplementedError(err_msg)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def change_file_name_index(self, file_name: str) -> None:
        self.file = file_name

    @staticmethod
    def _read_lidar_file(
        ref_t: int,
        lidar_path: str,
        tot_frames: int,
    ) -> NDArray[np.float64]:
        """
        評価モードでLiDARデータを読み込む
        """
        frame_numbers: list[str] = [f"{ref_t + idx:06d}" for idx in range(tot_frames)]

        # 構築された各フレームのファイルパスをリスト化
        lidar_files: list[str] = [f"{lidar_path}{frame}.npy" for frame in frame_numbers]

        # 各ファイルからデータを読み込み、リストに格納
        lidar_frames: list[NDArray[np.float64]] = [
            np.load(file, allow_pickle=True) for file in lidar_files
        ]

        # フレームが1つの場合はそのまま、複数の場合は結合
        if tot_frames == 1:
            xyz: NDArray[np.float64] = lidar_frames[0]
        else:
            xyz: NDArray[np.float64] = np.concatenate(lidar_frames, axis=0)

        return np.ascontiguousarray(xyz, dtype=np.float64)

    def get_points(
        self,
        ref_t: int | None = None,
    ) -> tuple[NDArray[np.float64], float]:
        # 変数初期化
        ts: Final[float] = 0.0
        tot_frames: Final[int] = 1
        if ref_t is not None:
            self._ref_t = ref_t

        # データ取得処理
        packet_data = self._read_lidar_file(
            self._ref_t,
            self.file,
            tot_frames,
        )

        return packet_data, ts


class MID360Points:
    _POINT_HEADER_SIZE: Final[int] = 36
    _POINT_SIZE: Final[int] = 14
    _POINT_COUNT: Final[int] = 96
    _POINT_DTYPE: Final[np.dtype] = np.dtype(
        [
            ("x", "<i4"),
            ("y", "<i4"),
            ("z", "<i4"),
            ("reflect", "i1"),
            ("tag", "u1"),
        ]
    )

    def __init__(
        self,
        index: int,
        lidar_config: dict[str, str | int],
        app_logger_factory: AppLoggerFactory,
        dot_num_low_threshold: int = 50,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)

        self._index = index
        self._host_ip: str = str(lidar_config["dest_ip"])
        self._port: int = int(lidar_config["port_pnt"])
        self._logger.info("Host: %s", self._host_ip)

        self._logger.info("Host: %s", self._host_ip)
        self._logger.info("Port(PNT): %s", self._port)

        # mid360クラスを宣言したタイミングでMID360にも接続する。
        # point: 点群データ受け取り用。
        self._socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SE008: パケット品質監視用
        self._prev_udp_cnt: int | None = None
        self._last_quality_degraded = 0.0
        self._dot_num_low_threshold: Final[int] = dot_num_low_threshold
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

    def _check_packet_quality(self, dst_byte: bytes) -> bool:
        """パケット品質をチェックし、異常があれば True を返す。
        udp_cnt 欠番 / dot_num 低下を検出する。
        """
        dot_num: int = int.from_bytes(dst_byte[5:7], "little", signed=False)
        udp_cnt: int = int.from_bytes(dst_byte[7:9], "little", signed=False)

        has_issue: bool = False

        if udp_cnt == 0:
            # フレーム境界: カウンタをリセットして欠番判定しない
            self._prev_udp_cnt = 0
        elif self._prev_udp_cnt is not None and udp_cnt != self._prev_udp_cnt + 1:
            has_issue = True  # udp_cnt ギャップ検出
        self._prev_udp_cnt = udp_cnt

        if dot_num < self._dot_num_low_threshold:
            has_issue = True  # dot_num 低下

        return has_issue

    @property
    def last_quality_degraded(self) -> float:
        """最後に品質低下イベントが発生した単調時刻 (0.0=未発生)"""
        return self._last_quality_degraded

    @classmethod
    def _decode_points(cls, dst_byte: bytes) -> NDArray[np.float64]:
        expected_size = cls._POINT_HEADER_SIZE + cls._POINT_SIZE * cls._POINT_COUNT
        if len(dst_byte) < expected_size:
            raise ValueError(
                f"MID360 point packet is too short: {len(dst_byte)} < {expected_size}"
            )
        raw_points = np.frombuffer(
            dst_byte,
            dtype=cls._POINT_DTYPE,
            count=cls._POINT_COUNT,
            offset=cls._POINT_HEADER_SIZE,
        )
        points = np.empty((cls._POINT_COUNT, 4), dtype=np.float64)
        points[:, 0] = raw_points["x"] / 1e3
        points[:, 1] = raw_points["y"] / 1e3
        points[:, 2] = raw_points["z"] / 1e3
        points[:, 3] = raw_points["reflect"]
        return points

    def get_points(self) -> tuple[NDArray[np.float64], float]:
        # パケットデータを取得 全体1380byteだが、少し多めに
        dst_byte, _ = self._socket.recvfrom(1500)

        # 品質チェック: udp_cnt欠番 / dot_num低下
        if self._check_packet_quality(dst_byte):
            self._last_quality_degraded = time.perf_counter()

        ts: float = self._get_timestamp(dst_byte)
        return self._decode_points(dst_byte), ts
