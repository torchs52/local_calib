from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from argus_synchro.message.input_message import ImuData
from argus_synchro.shared_excepts import SharedLidarShiftMonitorExcept


def _load_rt(csv_path: str | Path) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    mat: NDArray[np.float64] = np.loadtxt(csv_path, delimiter=",", dtype=np.float64)
    if mat.shape != (4, 4):
        raise ValueError(f"{csv_path}の形状が4×4ではありません")
    return mat[:3, :3], mat[:3, 3]


class LidarShiftMonitor:
    """
    メインLiDAR(=IMU-1)を基準に、各IMU(=IMU-2..N)の取り付けズレをリアルタイム監視するクラス
    """

    def __init__(
        self,
        calib_files: list[str | Path],  # 各IMUcalibファイルのPath（0番は基準IMU）
        *,
        # Fast（瞬間ズレ)
        win_fast: int = 5,  # 例: 5=25ms@200Hz
        hold_fast: int = 40,
        fast_abs: float = 0.05,  # ズレと判定用しきい値 [rad/s]
        # Slow（定常ズレ：今は何もしていない）
        win: int = 200,
        hold: int = 100,
        slow_abs: float = 0.02,
    ) -> None:
        # 4x4行列から回転成分R_iのみ使用（並進成分t_iは未使用）
        self.R_list: list[NDArray[np.float64]] = [
            R for R, _ in (_load_rt(p) for p in calib_files)
        ]

        # IMU台数（>=2）
        self.n_imu: int = len(self.R_list)
        if self.n_imu < 2:
            raise ValueError("IMUは2台以上必要です")

        # パラメータ（固定しきい値）
        self.Ws: int = int(win)
        self.Hs: int = int(hold)
        self.Wf: int = int(win_fast)
        self.Hf: int = int(hold_fast)
        self.fast_thr: float = float(fast_abs)
        self.slow_thr: float = float(slow_abs)

        # バッファ（IMUごと）
        self._latch_fast: NDArray[np.int32] = np.zeros(self.n_imu, dtype=np.int32)

        # 変化率計算用
        self._prev_dw: NDArray[np.float64] = np.zeros(self.n_imu, dtype=np.float64)

        # 連続カウンタ
        self._consec_fast: NDArray[np.int32] = np.zeros(self.n_imu, dtype=np.int32)

    def detect_lidar_shift(
        self,
        imu_data: NDArray[np.float64],
        dt: float,
    ) -> dict[int, dict[str, float | bool]]:
        domega = self._compute_domega(imu_data)
        result: dict[int, dict[str, float | bool]] = {}

        for idx in range(1, self.n_imu):
            dwi = float(domega[idx])
            status_fast = self._update_fast(idx, dwi, dt)
            status_slow = self._update_slow(idx, dwi)
            result[idx + 1] = {"status_fast": status_fast, "status_slow": status_slow}

        # 前回値更新
        self._prev_dw[:] = domega
        return result

    def _compute_domega(self, imu_data: NDArray[np.float64]) -> NDArray[np.float64]:
        if imu_data.shape[0] != self.n_imu or imu_data.shape[1] < 6:
            raise ValueError("imu_dataの形状が不正です（期待:(N,6)）")
        gyro_list: NDArray[np.float64] = imu_data[:, :3]
        wB_stack = np.stack([R @ w for R, w in zip(self.R_list, gyro_list)], axis=0)
        return np.linalg.norm(wB_stack[0] - wB_stack, axis=1)

    def _update_fast(self, idx: int, dwi: float, dt: float) -> bool:
        # 瞬間ズレ検出部
        over_fast: bool = dwi > self.fast_thr
        # 連続カウント
        if over_fast:
            self._consec_fast[idx] += 1
        else:
            self._consec_fast[idx] = 0
        return bool(self._consec_fast[idx] >= self.Hf)

    def _update_slow(self, idx: int, dwi: float) -> bool:
        # 定常ズレ検出部
        # いまは未実装、常にFalseと判定させる
        return False

    def detect_lidar_shift_from_k_samples(
        self,
        imu_input_data: tuple[ImuData, ...],
        sec_lidar_sm: SharedLidarShiftMonitorExcept,
        k: int = 20,
        dt: float = 0.005,
    ) -> None:
        """
        共有バッファから各IMUの最新kサンプルを取り出し、時系列（古→新）に処理。
        """

        # 1) 共有バッファ読み出し
        imu_deques = [shared_buf.imu for shared_buf in imu_input_data]

        # 2) 各IMU：最新k件を(T,6)に整形
        per_imu: list[NDArray[np.float64]] = []
        for buf in imu_deques:
            arr = np.asarray(buf, dtype=np.float64)
            if arr.size == 0:
                per_imu.append(np.empty((0, 6), dtype=np.float64))
                return
            sel = arr[-k:]
            sel = np.asarray(sel, dtype=np.float64).reshape(-1, sel.shape[-1])
            per_imu.append(sel[:, :6])

        # 3) 共通フレーム長
        if len(per_imu) != self.n_imu:
            return

        T = min(seq.shape[0] for seq in per_imu)
        if T == 0:
            return

        # 4) 古→新で1フレームずつ検出
        for j in range(T):
            frame = np.stack(
                [per_imu[i][j] for i in range(self.n_imu)], axis=0
            )  # (N,6)
            res = self.detect_lidar_shift(imu_data=frame, dt=dt)

            # 5) 共有フラグ更新（Fast優先→Slow）
            fast_any = any(v["status_fast"] for v in res.values())
            slow_any = any(v["status_slow"] for v in res.values())

            if fast_any:
                sec_lidar_sm.is_shifted_fast.value = True
            if slow_any:
                sec_lidar_sm.is_shifted_slow.value = True
