import socket
import struct
import time

import numpy as np


class mid360:
    def __init__(
        self, host_ip="192.168.1.4", port={"pnt": 56301, "imu": 56401}, debug=False
    ):
        self.host_ip = host_ip
        self.point_port = port["pnt"]
        self.imu_port = port["imu"]

        print(f"Host: {self.host_ip}")
        print(f"Port(PNT): {self.point_port}")
        print(f"Port(IMU): {self.imu_port}")

        # mid360クラスを宣言したタイミングでMID360にも接続する
        # point: 点群データ受け取り用、imu: IMUデータ受け取り用
        self.address = (self.host_ip, self.point_port)
        self.point = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(self.address)
        self.point.bind(self.address)
        # self.address = (self.host_ip, self.imu_port)
        # self.imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.imu.bind(self.address)

        print("CONNECTION: OK")

        # 時間計測用(点群のタイムスタンプではない)
        # このクラスがどのくらい稼働していたか測定用
        self.t = time.time()

        # デバッグしたかったらTRUE。デバッグ表示したい情報は未定
        self.debug = debug

        self.firstframe = True

    def connect_mid360(self, host_ip, port):
        """
        使っていない関数
        """

        address = (host_ip, port)
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        return udp.bind(address)

    def disconnect_mid360(self):
        """
        接続を切断
        """
        self.point.close()
        # self.imu.close()
        print("DISCONNECT: OK")
        print(f"Connected TIME was: {time.time() - self.t}")

    def get_points(self, max_accum_time=0.1):
        """
        点群データを取得
        max_accum_time: 点群の蓄積時間を指定。デフォルト0.1秒
        """

        offset = 14  # 1ポイント毎のバイト数
        init_idx = 36  # 先頭から36バイト目以降に点群データが格納されている
        accum_time = 0  # 点群の蓄積時間
        count = 0

        dst_byte = b""  # 座標データが格納される変数
        packet_data = []  # LiDARから送信されるデータ格納用(Byte)
        ts_data = []
        while accum_time < max_accum_time:  # max_accum_timeまで点群を蓄積する
            dst_byte, _ = self.point.recvfrom(
                1500
            )  # パケットデータを取得 全体1380byteだが、少し多めに

            for i in range(
                96
            ):  # 1フレームごとに 14*96 byteのデータが送信される（96点分)
                # xyz[mm]から[m]に変換するために/1000する
                x = (
                    int.from_bytes(
                        dst_byte[init_idx + offset * i : init_idx + 4 + offset * i],
                        "little",
                        signed=True,
                    )
                    / 1e3
                )
                y = (
                    int.from_bytes(
                        dst_byte[init_idx + 4 + offset * i : init_idx + 8 + offset * i],
                        "little",
                        signed=True,
                    )
                    / 1e3
                )
                z = (
                    int.from_bytes(
                        dst_byte[
                            init_idx + 8 + offset * i : init_idx + 12 + offset * i
                        ],
                        "little",
                        signed=True,
                    )
                    / 1e3
                )
                reflect = int.from_bytes(
                    dst_byte[
                        init_idx + 4 * 3 + offset * i : init_idx
                        + 4 * 3
                        + 1
                        + offset * i
                    ],
                    "little",
                    signed=True,
                )
                packet_data.append([x, y, z, reflect])

            # timestamp: [nsec]を[sec]へ変換するために/1e9する
            if count == 0:
                ts_init = (
                    int.from_bytes(
                        dst_byte[
                            1 + 2 + 2 + 2 + 2 + 1 + 1 + 1 + 12 + 4 : 1
                            + 2
                            + 2
                            + 2
                            + 2
                            + 1
                            + 1
                            + 1
                            + 12
                            + 4
                            + 8
                        ],
                        "little",
                        signed=True,
                    )
                    / 1e9
                )  # timestamp
                ts_data.append(ts_init)
            else:
                ts = (
                    int.from_bytes(
                        dst_byte[
                            1 + 2 + 2 + 2 + 2 + 1 + 1 + 1 + 12 + 4 : 1
                            + 2
                            + 2
                            + 2
                            + 2
                            + 1
                            + 1
                            + 1
                            + 12
                            + 4
                            + 8
                        ],
                        "little",
                        signed=True,
                    )
                    / 1e9
                )  # timestamp
                ts_data.append(ts)
                accum_time = ts - ts_init
            count += 1

        if self.debug:
            import open3d as o3d

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(packet_data)[:, :3])
            o3d.visualization.draw_geometries(
                geometry_list=[pcd], window_name="THIS APPEARES WHNE DEBUG MODE IS TRUE"
            )

        return np.array(packet_data)

    def get_imu(self):
        """
        imuデータを取得
        """

        init_idx = 36
        imu_byte = (
            b""  # IMUデータが格納される変数。なくても動く。（宣言したほうが効率よい？）
        )
        imu_byte, _ = self.imu.recvfrom(
            100
        )  # パケットデータを取得　全体で60byteだが、少し多めに

        # ジャイロ[rad/s]、加速度[g]
        # 最初の文字を "<"にすることでバイトオーダをlittle-endianに指定
        # ">"だとbig-endian
        gyro_x = struct.unpack("<f", imu_byte[init_idx : init_idx + 4])[0]
        gyro_y = struct.unpack("<f", imu_byte[init_idx + 4 : init_idx + 8])[0]
        gyro_z = struct.unpack("<f", imu_byte[init_idx + 8 : init_idx + 12])[0]
        acce_x = struct.unpack("<f", imu_byte[init_idx + 12 : init_idx + 16])[0]
        acce_y = struct.unpack("<f", imu_byte[init_idx + 16 : init_idx + 20])[0]
        acce_z = struct.unpack("<f", imu_byte[init_idx + 20 : init_idx + 24])[0]

        ts = (
            int.from_bytes(
                imu_byte[
                    1 + 2 + 2 + 2 + 2 + 1 + 1 + 1 + 12 + 4 : 1
                    + 2
                    + 2
                    + 2
                    + 2
                    + 1
                    + 1
                    + 1
                    + 12
                    + 4
                    + 8
                ],
                "little",
                signed=True,
            )
            / 1e9
        )  # timestamp

        return np.array([gyro_x, gyro_y, gyro_z, acce_x, acce_y, acce_z]), ts
