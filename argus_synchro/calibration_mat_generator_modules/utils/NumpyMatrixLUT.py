from __future__ import annotations

import os

import numpy as np


class NumpyMatrixLUT:
    """
    (x_i, y_i) に対して NumPy 2次元行列を参照し、値をそのまま η として返す。

    初期化:
      - dummy_config: str（将来の設定クラス読込のダミー）
      - NumPy行列を読み込み（.npy / .npz）
      - 行列サイズ(width,height)を保持
      - 読めない場合 FileNotFoundError

    evaluate:
      - 例外を出さず、範囲外/NaN/Inf/その他失敗は default を返す
      - x_r = a_x*x_i + b_x, y_r = a_y*y_i + b_y
      - xr_i=int(round(x_r)), yr_i=int(round(y_r))
      - eta = mat[yr_i, xr_i] を返す（配列は [y, x]）
    """

    def __init__(
        self,
        A_X: float,
        B_X: float,
        A_Y: float,
        B_Y: float,
        ARRAY_PATH: str,
        DEFAULT_VALUE: float,
    ) -> None:
        # ---- 設定値をグローバル変数からコピー（要件どおり）----
        self.a_x: float = float(A_X)
        self.b_x: float = float(B_X)
        self.a_y: float = float(A_Y)
        self.b_y: float = float(B_Y)

        self.array_path: str = str(ARRAY_PATH)

        self.default_value: float = float(DEFAULT_VALUE)

        # ---- 行列読込 ----
        self._mat = self._load_matrix(self.array_path)

        # 2次元であることを前提（必要なら3次元対応にも拡張可）
        if self._mat.ndim != 2:
            raise ValueError(
                f"Loaded array must be 2D, but got shape={self._mat.shape}, ndim={self._mat.ndim}"
            )

        self.height: int = int(self._mat.shape[0])
        self.width: int = int(self._mat.shape[1])

    @staticmethod
    def _load_matrix(path: str) -> np.ndarray:
        """
        .npy / .npz を読み込んで 2D 行列を返す。
        読めなければ FileNotFoundError。
        """
        # ファイル存在チェック（メッセージを明確に）
        if not os.path.exists(path):
            raise FileNotFoundError(f"Matrix file not found: {path}")

        ext = os.path.splitext(path)[1].lower()

        try:
            if ext == ".npy":
                mat = np.load(path, allow_pickle=False)
                return mat
            # 拡張子が想定外でも読み込み失敗として扱う
            raise FileNotFoundError(
                f"Unsupported matrix file extension: {ext} ({path})"
            )
        except FileNotFoundError:
            raise
        except Exception as e:
            # 「読み込めなかった場合 FileNotFoundError」要件に合わせて統一
            raise FileNotFoundError(f"Failed to load matrix file: {path}") from e

    def evaluate(self, x_i: np.float64, y_i: np.float64) -> float:
        """
        例外無しで評価値 η を返す（round版）。
        """
        try:
            # 1) 参照座標計算
            x_r = self.a_x * float(x_i) + self.b_x
            y_r = self.a_y * float(y_i) + self.b_y

            # NaN/Inf ガード
            if not (np.isfinite(x_r) and np.isfinite(y_r)):
                # print(f"NumpyMatrixLUT: {x_i},{y_i} -> {x_r},{y_r} : inf/NaNガード")
                return self.default_value

            # floor でインデックス化
            xr_i = int(np.floor(x_r))
            yr_i = int(np.floor(y_r))

            # 範囲外チェック
            if x_r < 0.0 or x_r >= self.width or y_r < 0.0 or y_r >= self.height:
                # print(f"NumpyMatrixLUT: {x_i},{y_i} -> {x_r},{y_r} : 範囲外")
                return self.default_value

            # 4) 行列参照（配列は [y, x]）
            eta = self._mat[yr_i, xr_i]

            # NaNが嫌なのでここで default に落とす:
            if not np.isfinite(float(eta)):
                # print(f"NumpyMatrixLUT: {x_i},{y_i} -> {x_r},{y_r} : inf/NaNガード")
                return self.default_value
            # print(f"NumpyMatrixLUT: {x_i},{y_i} -> {x_r},{y_r} : {eta=}")
            return float(eta)

        except Exception:
            # 要件：計算メソッドは例外無し
            return self.default_value
