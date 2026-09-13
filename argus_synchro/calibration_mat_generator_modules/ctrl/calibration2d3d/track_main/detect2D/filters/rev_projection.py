import cv2
import numpy as np
from numpy.typing import NDArray


def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """L2正規化（ゼロ除算を回避）"""
    n = np.linalg.norm(v)
    if n < eps:
        return v
    return v / n


def pixel_to_cam_ray(
    pixel: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray | None = None,
    use_opencv_undistort: bool = True,
) -> np.ndarray:
    """
    画像ピクセル座標 (u, v) と内部行列 K から、カメラ座標系での単位視線ベクトル dir_c を返す。

    - dist が与えられ、かつ OpenCV が利用可能な場合は、cv2.undistortPoints を用いて歪み補正。
      -> 正規化画像座標 (x', y') を得て dir_c = [x', y', 1]^T を正規化。
    - dist が None の場合は、dir_c ~ K^{-1} [u, v, 1]^T を正規化。

    Parameters
    ----------
    pixel : (2,) array_like
        [u, v]（ピクセル単位）
    K : (3,3) array_like
        内部パラメータ行列
    dist : (k,) array_like or None
        歪み係数。OpenCV 互換 [k1, k2, p1, p2, k3, ...]
    use_opencv_undistort : bool
        True かつ dist が与えられ、OpenCV が使える場合に undistortPoints を用いる

    Returns
    -------
    dir_c : (3,) ndarray
        カメラ座標系での**単位**視線ベクトル
    """
    u, v = float(pixel[0]), float(pixel[1])

    if dist is not None and use_opencv_undistort:
        # undistortPoints: 入力は Nx1x2、出力は正規化画像座標（新Kを与えなければ）
        pts = np.array([[[u, v]]], dtype=np.float64)
        K = np.asarray(K, dtype=np.float64)
        dist = np.asarray(dist, dtype=np.float64)
        norm_xy = cv2.undistortPoints(pts, K, dist, P=None)  # -> Nx1x2, 正規化座標
        x_p, y_p = norm_xy[0, 0, 0], norm_xy[0, 0, 1]
        dir_c = np.array([x_p, y_p, 1.0], dtype=np.float64)
        return _normalize(dir_c)

    # 歪みなし：K 逆行列で正規化
    Kinv = np.linalg.inv(K)
    pix_h = np.array([u, v, 1.0], dtype=np.float64)
    dir_c = Kinv @ pix_h
    return _normalize(dir_c)


def camera_center_and_rotation_world(
    R: np.ndarray,
    t: np.ndarray,
    extrinsic_convention: str = "world_to_cam",
) -> tuple[np.ndarray, np.ndarray]:
    """
    外部パラメータから、世界座標系でのカメラ中心 C_w と、カメラ→世界の回転 R_cw を求める。

    - world_to_cam（OpenCV流）: X_c = R * X_w + t
        -> R_cw = R^T, C_w = -R^T * t
    - cam_to_world           : X_w = R * X_c + t
        -> R_cw = R,   C_w = t

    Returns
    -------
    C_w : (3,) ndarray
        世界座標系でのカメラ中心
    R_cw : (3,3) ndarray
        カメラ座標系から世界座標系への回転
    """
    R = np.asarray(R, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64).reshape(3)

    if extrinsic_convention == "world_to_cam":
        R_cw = R.T
        C_w = -R.T @ t
    elif extrinsic_convention == "cam_to_world":
        R_cw = R
        C_w = t
    else:
        raise ValueError(
            "extrinsic_convention は 'world_to_cam' か 'cam_to_world' を指定してください。"
        )
    return C_w, R_cw


def intersect_ray_with_plane(
    ray_origin_w: np.ndarray,
    ray_dir_w: np.ndarray,
    plane_abcd: np.ndarray,
    eps: float = 1e-12,
) -> tuple[bool, np.ndarray | None]:
    """
    世界座標系での半直線（始点 O、方向 d）と平面 ax+by+cz+d=0 との交点を求める。
    パラメトリック：X(s) = O + s d。平面法線 n=(a,b,c)。

    解： s = -(d + n·O) / (n·d)

    Returns
    -------
    (ok, X)
        ok=True なら交点あり（X は (3,)）。False の場合、X=None（平行または数値的不安定）。
        s<0 の場合はカメラ後方側の交点（必要に応じて呼び出し側で弾いてください）。
    """
    a, b, c, d0 = [float(v) for v in plane_abcd]
    n = np.array([a, b, c], dtype=np.float64)

    denom = n.dot(ray_dir_w)
    if abs(denom) < eps:
        return False, None  # 平面と視線がほぼ平行

    numer = -(d0 + n.dot(ray_origin_w))
    s = numer / denom
    X = ray_origin_w + s * ray_dir_w
    return True, X


def intersect_pixel_ray_with_world_plane(
    pixel_xy: tuple[float, float],
    K: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    plane_abcd: tuple[float, float, float, float],
    dist: np.ndarray | None = None,
    extrinsic_convention: str = "world_to_cam",
    return_extra: bool = False,
) -> tuple[NDArray[np.float64], NDArray[np.bool_], dict] | None:
    """
    画像上の1点 (u,v) と焦点を通る直線（視線）と、世界座標系の平面 ax+by+cz+d=0 の交点を返す。

    Parameters
    ----------
    pixel_xy : (u, v)
        画像座標（ピクセル）
    K : (3,3)
        内部パラメータ
    R, t :
        外部パラメータ。`extrinsic_convention` で意味が変わる（下記参照）
    plane_abcd : (a, b, c, d)
        世界座標系での平面係数
    dist : array_like or None
        歪み係数（OpenCV 互換）。指定時は OpenCV を用いて undistortPoints で正規化。
    extrinsic_convention : {"world_to_cam", "cam_to_world"}
        - "world_to_cam": X_c = R * X_w + t  （OpenCV標準）
        - "cam_to_world": X_w = R * X_c + t
    return_extra : bool
        True の場合、付加情報（カメラ中心、レイ、交点が前方か等）を dict で返す

    Returns
    -------
    X_w : (3,) ndarray
        交点の世界座標
    or (X_w, info) if return_extra=True
    """
    K = np.asarray(K, dtype=np.float64)
    R = np.asarray(R, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64).reshape(3)
    plane = np.asarray(plane_abcd, dtype=np.float64).reshape(4)

    # 1) カメラ中心 C_w と回転 R_cw を取得
    C_w, R_cw = camera_center_and_rotation_world(
        R, t, extrinsic_convention=extrinsic_convention
    )
    # print(f"{C_w=},{R_cw}")

    # 2) 画像座標 -> カメラ座標系の視線ベクトル
    # print(f"{pixel_xy = }")
    dir_c = pixel_to_cam_ray(
        np.array(pixel_xy, dtype=np.float64), K, dist=dist, use_opencv_undistort=True
    )
    # print(f"{dir_c=}")

    # 3) 視線を世界座標系へ
    dir_w = R_cw @ dir_c
    dir_w = _normalize(dir_w)
    # print(f"{dir_w=}")

    # 4) 平面と交差
    ok, X = intersect_ray_with_plane(C_w, dir_w, plane)
    if not ok:
        return None
        # raise RuntimeError("視線と平面が平行で、交点が求まりません。")
    assert X is not None

    # s の符号で前方/後方を判定したい場合に備えて s を計算
    a, b, c, d0 = plane.tolist()
    n = np.array([a, b, c], dtype=np.float64)
    denom = n.dot(dir_w)
    s: np.bool_ = (
        -(d0 + n.dot(C_w)) / denom
    )  # denom は intersect 内ですでにゼロでないことを確認済み
    is_front = np.array([s >= 0.0], dtype=np.bool_)
    # print(f"{X=}")

    if return_extra or True:
        info = {
            "camera_center_world": C_w,
            "ray_dir_world": dir_w,
            "ray_dir_camera": dir_c,
            "is_in_front_of_camera": bool(s >= 0.0),  # False ならカメラ後方の交点
            "param_s": s,
        }
        # print(info)
        return X, is_front, info

    return X, is_front, {}
