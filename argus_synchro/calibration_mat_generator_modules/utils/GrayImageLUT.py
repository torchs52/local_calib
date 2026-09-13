import cv2
import numpy as np


class GrayImageLUT:
    """
    OpenCVで白黒画像を読み込み、round()で参照画素を決めて
    画素値を一次変換して評価値ηを返すクラス。

    初期化:
      - dummy_config: str（将来の設定クラス読込のダミー）
      - 画像読込（grayscale）
      - サイズ(width,height)を保持
      - 読めない場合 FileNotFoundError

    evaluate:
      - 例外を出さず、範囲外/NaN/Inf/その他失敗は default を返す
      - x_r = a_x*x_i+b_x, y_r = a_y*y_i+b_y
      - xr_i=int(round(x_r)), yr_i=int(round(y_r))
      - alpha = img[yr_i, xr_i] (0..255)
      - eta = a_eta*alpha + b_eta
    """

    def __init__(
        self,
        A_X: float,
        B_X: float,
        A_Y: float,
        B_Y: float,
        IMAGE_PATH: str,
        A_ETA: float,
        B_ETA: float,
        DEFAULT_VALUE: float,
    ) -> None:
        # ---- 設定値をグローバル変数からコピー（要件どおり）----
        self.a_x: float = float(A_X)
        self.b_x: float = float(B_X)
        self.a_y: float = float(A_Y)
        self.b_y: float = float(B_Y)

        self.image_path: str = str(IMAGE_PATH)

        self.a_eta: float = float(A_ETA)
        self.b_eta: float = float(B_ETA)

        self.default_value: float = float(DEFAULT_VALUE)

        # ---- 画像読込（OpenCV）----
        # cv2.imreadは失敗すると None を返す（例外は基本投げない）ため明示的にチェック
        img = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)

        if img is None:
            # より分かりやすいメッセージを付けて FileNotFoundError 扱いにする
            # (存在するが読み込めない場合もあり得るが、要件が「読めなかったらFileNotFoundError」なので統一)
            raise FileNotFoundError(
                f"Failed to load grayscale image: {self.image_path}"
            )

        # OpenCVの画像は shape=(H, W) の numpy.ndarray（uint8）
        self._img_np: np.ndarray = img
        self.height: int = int(img.shape[0])
        self.width: int = int(img.shape[1])

    def evaluate(self, x_i: np.float64, y_i: np.float64) -> float:
        """例外無しで評価値ηを返す（round版）"""
        try:
            # 1) 参照座標計算
            x_r = self.a_x * float(x_i) + self.b_x
            y_r = self.a_y * float(y_i) + self.b_y

            # NaN/Inf ガード
            if not (np.isfinite(x_r) and np.isfinite(y_r)):
                # print(f"GrayImageLUT: {x_i},{y_i} -> {x_r},{y_r} : inf/NaNガード")
                return self.default_value

            # floor でインデックス化
            xr_i = int(np.floor(x_r))
            yr_i = int(np.floor(y_r))

            # 範囲外チェック
            if x_r < 0.0 or x_r >= self.width or y_r < 0.0 or y_r >= self.height:
                # print(f"GrayImageLUT: {x_i},{y_i} -> {x_r},{y_r} : 範囲外")
                return self.default_value

            # 3) 画素値参照（配列は [y, x]）
            alpha = int(self._img_np[yr_i, xr_i])  # 0..255

            # 4) スケーリング
            eta = self.a_eta * alpha + self.b_eta
            # print(f"GrayImageLUT: {x_i},{y_i} -> {x_r},{y_r} : {eta=}")
            return float(eta)

        except Exception:
            # 要件：計算メソッドは例外無し
            return self.default_value
