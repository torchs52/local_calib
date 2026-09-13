from __future__ import annotations

import json
import math
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
import pandas as pd

from argus_synchro.common import paths
from argus_synchro.common.paths import normalize_path
from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.app_config_calibration import AppConfigCalibration


def _read_mat4_csv(
    path: str,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
) -> np.ndarray:
    try:
        T = pd.read_csv(path, header=None).values
    except (OSError, UnicodeError, pd.errors.EmptyDataError, pd.errors.ParserError) as error:
        if file_io_error_reporter is not None:
            file_io_error_reporter(str(path), "read 3D-3D matrix CSV", error)
        raise
    if T.shape != (4, 4):
        raise ValueError(f"{path}: 4x4行列が必要ですが shape={T.shape}")
    return T.astype(float, copy=False)


def _orthonormalize_so3(R: np.ndarray) -> np.ndarray:
    # SVDで最近傍の回転行列に射影（det<0 の場合は反転修正）
    U, _, Vt = np.linalg.svd(R)
    R_ = U @ Vt
    if np.linalg.det(R_) < 0:
        U[:, -1] *= -1
        R_ = U @ Vt
    return R_


def _euler_zyx_from_R(R: np.ndarray) -> tuple[float, float, float]:
    """
    ZYX（Rz→Ry→Rx）のyaw(z), pitch(y), roll(x) を度で返す
    """
    # 数値安定化のため再直交化
    R = _orthonormalize_so3(R)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-8
    if not singular:
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))  # Z
        pitch = math.degrees(math.atan2(-R[2, 0], sy))  # Y
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))  # X
    else:
        # ギンバルロック近傍
        yaw = math.degrees(math.atan2(-R[0, 1], R[1, 1]))
        pitch = math.degrees(math.atan2(-R[2, 0], sy))
        roll = 0.0
    return yaw, pitch, roll


def csv_to_make_T_args(
    csv_path: str,
    order: str = "ZYX",
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
):
    """
    4x4 CSVから make_T_from_deg に渡す引数一式を抽出して返す。
    戻り値: (tx,ty,tz, yaw_deg, pitch_deg, roll_deg, order)
    """
    T = _read_mat4_csv(csv_path, file_io_error_reporter)
    tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
    R = T[:3, :3]
    yaw, pitch, roll = _euler_zyx_from_R(R)
    return tx, ty, tz, yaw, pitch, roll, order


class Pose:
    def __init__(self, tx, ty, tz, yaw=0.0, pitch=0.0, roll=0.0, order="ZYX"):
        self.tx, self.ty, self.tz = tx, ty, tz
        self.yaw, self.pitch, self.roll, self.order = yaw, pitch, roll, order


def _pose_from_dict(d: dict[str, Any]) -> Pose:
    """
    Jsonで定義されたPose相当の辞書からPoseインスタンスを生成する
    期待するキー:
      tx,ty,tz (必須)
      yaw,pitch,roll,order (省略可)
    """
    return Pose(
        tx=d["tx"],
        ty=d["ty"],
        tz=d["tz"],
        yaw=d.get("yaw", 0.0),
        pitch=d.get("pitch", 0.0),
        roll=d.get("roll", 0.0),
        order=d.get("order", "ZYX"),
    )


def load_crane_profiles(
    json_path: str | Path,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
) -> dict[str, dict[str, Any]]:
    """
    Jsonファイルを読み込み、LiDAR0/LiDAR1をPoseオブジェクトに復元した辞書を返す

    戻り値例:
      profiles["900HSC"]["LiDAR0"]はPoseインスタンス
      profiles["900HSC"]["LiDAR1"]もPoseインスタンス
    """
    json_path = Path(json_path)
    try:
        with json_path.open("r", encoding="utf-8") as file:
            raw = json.load(file)
    except (OSError, UnicodeError, json.JSONDecodeError, KeyError, TypeError) as error:
        if file_io_error_reporter is not None:
            file_io_error_reporter(
                str(json_path), "read 3D-3D crane profile JSON", error
            )
        raise

    try:
        profiles: dict[str, dict[str, Any]] = {}
        for crane_name, cfg in raw.items():
            cfg_copy = dict(cfg)
            # LiDAR0,LiDAR1をPose化する(あれば)
            if "LiDAR0" in cfg_copy and cfg_copy["LiDAR0"] is not None:
                cfg_copy["LiDAR0"] = _pose_from_dict(cfg_copy["LiDAR0"])
            if "LiDAR1" in cfg_copy and cfg_copy["LiDAR1"] is not None:
                cfg_copy["LiDAR1"] = _pose_from_dict(cfg_copy["LiDAR1"])
            profiles[crane_name.upper()] = cfg_copy
    except (KeyError, TypeError, AttributeError) as error:
        if file_io_error_reporter is not None:
            file_io_error_reporter(
                str(json_path), "interpret 3D-3D crane profile JSON", error
            )
        raise

    return profiles


def get_profile(
    crane: str,
    profile_path: Path,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
) -> dict[str, Any]:
    """
    クレーン名に対応するプロファイル辞書を返す
    例:
      p = get_profile("900HSC")
      p["ground_z"] -> float
      p["LiDAR0"]   -> Pose
    """

    # TODO profiles.jsonはsettings.iniからPath指定する
    _CRANE_PROFILES: dict[str, dict[str, Any]] = load_crane_profiles(
        profile_path, file_io_error_reporter
    )
    k = crane.upper()
    if k not in _CRANE_PROFILES:
        raise ValueError(f"Unknown crane: {crane}. Available: {list(_CRANE_PROFILES)}")
    return _CRANE_PROFILES[k]


def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def make_T_from_deg(
    tx, ty, tz, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0, order="ZYX"
) -> np.ndarray:
    y, p, r = map(math.radians, (yaw_deg, pitch_deg, roll_deg))
    mats = {"X": rotx(r), "Y": roty(p), "Z": rotz(y)}
    R = np.eye(3)
    for ax in order:
        R = R @ mats[ax]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def apply_local_rotation(T: np.ndarray, R_local: np.ndarray) -> np.ndarray:
    T2 = T.copy()
    T2[:3, :3] = T[:3, :3] @ R_local
    return T2


def transform_points(T: np.ndarray, P: np.ndarray) -> np.ndarray:
    P3 = as_xyz(P, dtype=np.float32)
    ones = np.ones((len(P3), 1), P3.dtype)
    Ph = np.concatenate([P3, ones], 1)
    return (Ph @ T.T)[:, :3]


def as_xyz(P: np.ndarray, dtype=np.float32) -> np.ndarray:
    if P.ndim != 2 or P.shape[1] < 3:
        raise ValueError(f"Expect (N,>=3), got {P.shape}")
    return P[:, :3].astype(dtype, copy=False)


def make_rays_airy(
    num_az: int, num_el: int, *, vdeg: float = 51.0, el_bias_deg: float = 0.0
) -> np.ndarray:
    az = np.deg2rad(np.linspace(-180, 180, num_az, endpoint=False))
    el_min = -7 + el_bias_deg
    el_max = 51 + el_bias_deg
    el = np.deg2rad(np.linspace(el_min, el_max, num_el))
    A, E = np.meshgrid(az, el)
    dirs = np.stack(
        [np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1
    ).reshape(-1, 3)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True) + 1e-18
    return dirs.astype(np.float32)


def _is_radian(a: np.ndarray) -> bool:
    if a.size == 0 or not np.isfinite(a).any():
        return False
    return np.nanmax(np.abs(a)) <= (math.pi + 0.2)


def _read_angles_deg(
    csv_path: str | Path,
    az_col: str | None,
    el_col: str | None,
    limit: int,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
):
    csv_path = Path(csv_path)
    try:
        df = pd.read_csv(csv_path, nrows=limit)
    except (OSError, UnicodeError, pd.errors.EmptyDataError, pd.errors.ParserError) as error:
        if file_io_error_reporter is not None:
            file_io_error_reporter(str(csv_path), "read 3D-3D angle CSV", error)
        raise
    az = df["Az"].to_numpy(float)
    el = df["El"].to_numpy(float)
    az_deg = np.rad2deg(az) if _is_radian(az) else az
    el_deg = np.rad2deg(el) if _is_radian(el) else el
    return az_deg, el_deg


def make_rays_mid360_from_csv(
    csv_path: str | Path,
    az_col=None,
    el_col=None,
    limit: int = 20000,
    el_offset: int = 90,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
) -> np.ndarray:
    az_deg, el_deg = _read_angles_deg(
        csv_path, az_col, el_col, limit, file_io_error_reporter
    )
    el_deg = (-1.0 * el_deg) + el_offset
    az = np.deg2rad(az_deg)
    el = np.deg2rad(el_deg)
    dirs = np.stack(
        [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1
    )
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True) + 1e-18
    return dirs.astype(np.float32)


def _merge_meshes(
    meshes: Iterable[o3d.geometry.TriangleMesh],
) -> o3d.geometry.TriangleMesh:
    meshes = list(meshes)
    if not meshes:
        return o3d.geometry.TriangleMesh()
    verts_all, tris_all, v_off = [], [], 0
    for m in meshes:
        v = np.asarray(m.vertices)
        t = np.asarray(m.triangles)
        verts_all.append(v)
        tris_all.append(t + v_off)
        v_off += len(v)
    out = o3d.geometry.TriangleMesh(
        vertices=o3d.utility.Vector3dVector(np.vstack(verts_all)),
        triangles=o3d.utility.Vector3iVector(np.vstack(tris_all)),
    )
    out.compute_vertex_normals()
    return out


def load_crane_mesh(
    crane: str,
    profile,
    directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
):
    """
    Load/compose crane mesh by model key.

    - '900HSC' / '1200' / '2000'
        - 設定パスが「ファイル」→従来通り単体OBJを読み込み
        - 設定パスが「フォルダ」→直下にサブフォルダがあれば、
          サブフォルダ毎に *.obj を再帰的に結合 → parts_dict として返す（任意）
          すべて結合した merged も作って返す
        - OBJは mm とみなし m へ /1000、Z軸π回転（center=(0,0,0)）は従来通り
    """
    crane = crane.upper()

    def _cleanup(m: o3d.geometry.TriangleMesh):
        m.remove_duplicated_vertices()
        m.remove_degenerate_triangles()
        if not m.has_vertex_normals():
            m.compute_vertex_normals()
        return m

    def _read_obj_scaled_zpi(path: Path) -> o3d.geometry.TriangleMesh:
        m = o3d.io.read_triangle_mesh(str(path))
        if m is None or len(m.triangles) == 0:
            raise ValueError(f"Failed to read triangles from {path}")
        V = np.asarray(m.vertices, dtype=np.float64) / 1000.0  # mm→m
        m.vertices = o3d.utility.Vector3dVector(V)
        Rz_pi = m.get_rotation_matrix_from_axis_angle([0, 0, np.pi])
        m.rotate(Rz_pi, center=(0, 0, 0))
        _cleanup(m)
        m.paint_uniform_color([0.7, 0.7, 0.7])
        return m

    config_dir: Path = paths.get_config_dir(directory_config)
    src = Path(normalize_path(profile["obj_fpath"], config_dir))
    parts: dict[str, o3d.geometry.TriangleMesh] = {}

    if src.is_file():
        merged = _read_obj_scaled_zpi(src)
        return (merged, parts)

    if not src.exists():
        raise FileNotFoundError(src)

    if src.is_dir():
        # 直下サブフォルダを列挙（深さ1）。無ければフォルダ全体を1つとして扱う
        subfolders = [p for p in src.iterdir() if p.is_dir()]
        if not subfolders:
            obj_paths = list(src.rglob("*.obj")) + list(src.rglob("*.OBJ"))
            if not obj_paths:
                raise FileNotFoundError(f"No .obj under {src}")
            merged = _merge_meshes([_read_obj_scaled_zpi(p) for p in sorted(obj_paths)])
            return (merged, parts)

        # サブフォルダごとに集約
        for sub in sorted(subfolders):
            obj_paths = list(sub.rglob("*.obj")) + list(sub.rglob("*.OBJ"))
            if not obj_paths:
                continue
            parts[sub.name] = _merge_meshes(
                [_read_obj_scaled_zpi(p) for p in sorted(obj_paths)]
            )

        if not parts:
            raise FileNotFoundError(f"No .obj in any subfolder of {src}")

        merged = _merge_meshes(list(parts.values()))
        return (merged, parts)

    merged = _merge_meshes(list(parts.values()))
    return (merged, parts)


def frame_at(T: np.ndarray, size=0.3) -> o3d.geometry.TriangleMesh:
    f = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    f.transform(T.astype(np.float64))
    return f


def make_sensor_body(
    T: np.ndarray, radius=0.06, height=0.08, color=(0.1, 0.1, 0.1)
) -> o3d.geometry.TriangleMesh:
    cyl = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius, height=height, resolution=40
    )
    cyl.translate([0, 0, height / 2], relative=True)
    cyl.paint_uniform_color(color)
    cyl.compute_vertex_normals()
    cyl.transform(T.astype(np.float64))
    return cyl


def make_ground_plane_mesh(
    z: float = -1.38, size: float = 30.0, color=(0.95, 0.95, 0.95)
) -> o3d.geometry.TriangleMesh:
    plane = o3d.geometry.TriangleMesh.create_box(width=size, height=size, depth=0.02)
    plane.paint_uniform_color(color)
    plane.compute_vertex_normals()
    plane.translate([0, 0, z - 0.01], relative=False)
    return plane


def build_scene(mesh_B: o3d.geometry.TriangleMesh):
    scene = o3d.t.geometry.RaycastingScene()
    id_to_name: dict[int, str] = {}
    name_to_id: dict[str, int] = {}

    # CAD
    tmesh_cad = o3d.t.geometry.TriangleMesh.from_legacy(mesh_B)
    gid_cad = int(scene.add_triangles(tmesh_cad))
    id_to_name[gid_cad] = "cad"
    name_to_id["cad"] = gid_cad
    return scene, id_to_name, name_to_id


def raycast(
    scene: o3d.t.geometry.RaycastingScene,
    id_to_name: dict[int, str],
    lidar_pos: np.ndarray,
    rays_local: np.ndarray,
    *,
    max_range: float = 20.0,
    ground_z: float = -1.38,
    colors: dict[str, tuple] = None,
):
    """
    この関数は単一センサーから放射される複数のレイをシーンに対してレイキャストし、
    各レイが
      1) Open3DのRaycastingScene中のメッシュ(例:クレーンCAD等)
      2) 地面
    のどこに最初に当たるかを計算する。

    主な処理の流れ:
    - センサー座標系Lで与えられたレイ方向ベクトルを、LiDAR0でワールド座標系B(またはベース座標系B)へ変換する
    - Open3Dシーン内のメッシュ衝突距離t_sceneを取得する
    - 別途、解析的に地面(ground_z平面)との交点距離t_groundを計算する
    - より手前側(距離が短い方)のヒットを採用する
    - ヒット種別ごとに色分けしてOpen3Dジオメトリ(球インスタンスまたはPointCloud)を生成する
    """
    if colors is None:
        colors = {}

    # 色のデフォルト設定
    col_cad = colors.get("cad", (0.0, 0.6, 1.0))  # CADメッシュ用(青系)
    col_ground_base = colors.get("ground", (0.2, 0.8, 0.2))  # 基本地面用(緑系)
    col_ground_step = colors.get("ground_step", (1.0, 0.2, 0.2))  # 段差領域用(赤系)

    # 1. センサー位置とレイ方向をワールド座標系Bに変換する
    # センサー原点位置o_BをB座標系で取得する
    o_B = transform_points(lidar_pos, np.zeros((1, 3), np.float32))[0]

    # 回転成分Rを取り出す(センサー座標系L→B座標系)
    R = lidar_pos[:3, :3]

    # rays_local(各レイ方向,L系)をB系の方向ベクトルへ変換
    dB = (R @ rays_local.T).T  # 形状は(N,3)

    # 正規化して単位方向ベクトルにする
    dB /= np.linalg.norm(dB, axis=1, keepdims=True) + 1e-18

    # 2. Open3Dシーン上のメッシュとの交差判定
    # Open3DのRaycastingSceneはレイを[ox,oy,oz, dx,dy,dz]形式で受け取る
    rays6 = np.zeros((len(dB), 6), np.float32)
    rays6[:, :3] = o_B  # 全レイ同じ原点
    rays6[:, 3:] = dB  # 各レイの方向

    ans = scene.cast_rays(o3d.core.Tensor(rays6))

    # t_scene[i]はレイがメッシュに当たるまでの距離。ヒットなしはinf
    t_scene = ans["t_hit"].numpy().astype(np.float32)

    # gid_arr[i]はそのヒット対象メッシュのgeometry_id
    gid_arr = ans["geometry_ids"].numpy()

    # 3. 地面との交差計算(解析的に平面交差を解く)
    # 各レイ方向ベクトルdBのz成分
    dz = dB[:, 2]

    # 地面はz=ground_zの平面とみなす
    # o_B[2] + dz * t = ground_z → t = (ground_z - o_B[2]) / dz
    with np.errstate(divide="ignore", invalid="ignore"):
        t_ground_base = (ground_z - o_B[2]) / dz  # (N,)

    # 有効な交点条件: 有限値かつt>0(センサーの前側)
    valid_ground_base = np.isfinite(t_ground_base) & (t_ground_base > 0)

    # t_ground_effは「そのレイにおける有効な地面ヒット距離」
    # 最初は基準地面のみを考慮する。ヒットしない場合はinf
    t_ground_eff = np.where(valid_ground_base, t_ground_base, np.inf).astype(np.float32)

    # src_groundは地面のどの種類に当たったかを示すラベル
    # 0:地面ヒットなし
    # 1:基準地面(ground_z)
    # 2以上:段差領域(stepsのインデックス+2)
    src_ground = np.where(valid_ground_base, 1, 0).astype(np.int32)

    # 5. メッシュヒットと地面ヒットのうち、手前側(小さい距離)を決める
    t_scene_eff = t_scene.astype(np.float32)  # メッシュまでの距離
    t_near = np.minimum(t_scene_eff, t_ground_eff)  # 近い方を採用

    # 無限大のものはinfのまま扱いつつ、max_rangeでクリップする
    t_clip = np.where(np.isfinite(t_near), t_near, np.inf).astype(np.float32)
    t_clip = np.minimum(t_clip, max_range)

    # 各レイの最終的なヒット点(もしくはmax_range地点)をワールド座標で算出
    # endpoints[i] = o_B + dB[i] * t_clip[i]
    endpoints = o_B[None, :] + dB * t_clip[:, None]  # (N,3)

    # 6. レイごとの結果を分類する
    # 地面に当たり、かつ距離的にメッシュより手前で、かつmax_range以内
    hit_ground_any = (
        np.isfinite(t_ground_eff)
        & (t_ground_eff <= max_range)
        & (t_ground_eff < t_scene_eff)
    )

    # その中でsrc_ground==1は基準地面
    hit_ground_base = hit_ground_any & (src_ground == 1)

    # その中でsrc_ground>=2は段差領域(steps)
    hit_ground_step = hit_ground_any & (src_ground >= 2)

    # メッシュヒット: t_scene_effが有限かつmax_range以内かつ地面より手前
    hit_scene = (
        np.isfinite(t_scene_eff)
        & (t_scene_eff <= max_range)
        & (t_scene_eff <= t_ground_eff)
    )

    # missはどこにも当たらなかったレイ
    # miss = ~(hit_ground_any | hit_scene)

    # 7. 可視化用のOpen3Dジオメトリを生成する
    geoms: list[o3d.geometry.Geometry] = []

    def _emit_markers(mask: np.ndarray, color_tuple):
        """
        maskで指定されたレイのendpoints位置にマーカを配置する。
        hit_marker=="sphere"の場合は多数の小球をインスタンス化したメッシュを作る。
        hit_marker=="point"の場合はPointCloudを作る。
        sphere_stride>1なら間引いて描画負荷を軽くする。
        """
        idx = np.where(mask)[0]
        if len(idx) == 0:
            return

        # PointCloudとして出力する場合
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(endpoints[idx].astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(
            np.tile(np.array(color_tuple, float), (len(idx), 1))
        )
        geoms.append(pcd)

    # 基準地面ヒット点を描画(緑系)
    _emit_markers(hit_ground_base, col_ground_base)

    # 段差領域ヒット点を描画(赤系)
    _emit_markers(hit_ground_step, col_ground_step)

    # シーン中メッシュヒット点はメッシュIDごとに色分けする
    if hit_scene.any():
        for guniq in np.unique(gid_arr[hit_scene]):
            mask = hit_scene & (gid_arr == guniq)

            # geometry_id→名前
            name = id_to_name.get(int(guniq), f"gid_{int(guniq)}")

            # 個別色が定義されていればそれを使い、なければCADデフォルト色を使う
            col = colors.get(name, col_cad)

            _emit_markers(mask, col)

    # 9. 可視化ジオメトリを返す
    return (
        geoms,
        endpoints[hit_scene].astype(np.float32),
        endpoints[hit_ground_base].astype(np.float32),
        endpoints[hit_ground_step].astype(np.float32),
    )


def rotate_mesh_deg(
    m: o3d.geometry.TriangleMesh, *, yaw=0.0, pitch=0.0, roll=0.0, center="center"
):
    """
    Z(=yaw) → Y(=pitch) → X(=roll) の順に度指定で回転。
    center: "center" | "origin" | (x,y,z)
    """
    cz = (
        m.get_center()
        if center == "center"
        else ((0, 0, 0) if center == "origin" else center)
    )
    Rz = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, math.radians(yaw)])
    Ry = o3d.geometry.get_rotation_matrix_from_axis_angle([0, math.radians(pitch), 0])
    Rx = o3d.geometry.get_rotation_matrix_from_axis_angle([math.radians(roll), 0, 0])
    m.rotate(Rz @ Ry @ Rx, center=cz)


def euler_deg_to_R(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, order="ZYX") -> np.ndarray:
    """Compose rotation from degrees. Default ZYX: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r = np.deg2rad(roll_deg)
    p = np.deg2rad(pitch_deg)
    y = np.deg2rad(yaw_deg)
    mats = {"X": rotx(r), "Y": roty(p), "Z": rotz(y)}
    R = np.eye(3, dtype=np.float64)
    for ax in order:
        R = R @ mats[ax]
    return R


def simulate_crane_pts(
    lidar_pos: list,
    angle_data: float,
    app_config: AppConfig,
    calib_app_config: AppConfigCalibration,
    file_io_error_reporter: Callable[[str, str, Exception], None] | None = None,
):
    prof = get_profile(
        crane=app_config.UI_IF.crane_model,
        profile_path=calib_app_config.Calib3d3d_CalibParams.crane_profile_path,
        file_io_error_reporter=file_io_error_reporter,
    )
    ground_z = prof["ground_z"]

    # Crane mesh
    mesh_B, parts = load_crane_mesh(
        app_config.UI_IF.crane_model,
        prof,
        app_config.directory_config,
    )

    # 下部走行体を回転させる
    # クレーンはCCWで回転する（上部旋回体がCCW）
    # なので、下部走行体はCAN angleとは逆回転させると辻褄があう
    if parts:
        for name, m in parts.items():
            if name == "mobile":
                # TODO ここはCAN旋回データに差し替え
                rotate_mesh_deg(m, yaw=-1 * angle_data, center="origin")

    mesh_B = _merge_meshes(list(parts.values()))
    scene, id_to_name, _ = build_scene(mesh_B)

    # Sensors: 1〜N台に対応
    lidar_Ts: list[np.ndarray] = []

    for i in range(len(lidar_pos)):
        tx, ty, tz, yaw, pitch, roll, order = csv_to_make_T_args(
            lidar_pos[i], file_io_error_reporter=file_io_error_reporter
        )

        T = make_T_from_deg(
            tx,
            ty,
            tz=-0.10,  # センサ位置をCWから地面方向に10cm離す。近づけすぎるとCW内部にセンサが埋まってしまう
            yaw_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            order="ZYX",
        )
        lidar_Ts.append(T)

    if not lidar_Ts:
        raise ValueError(
            "lidar_pos が空です。少なくとも1台のLiDAR情報を渡してください。"
        )

    # 機体点群をLiDAR点群の向きに揃えるために上下反転させる
    if prof.get("local_flip_x", False):
        Rflip = euler_deg_to_R(180, 0, 0)
        lidar_Ts = [apply_local_rotation(T, Rflip) for T in lidar_Ts]

    # 実際のLiDARの回転に合わせるため、90度回転
    # MID360が360度計測するので, yaw_degを変えてもみた目だとあまりわからない
    R_local = euler_deg_to_R(roll_deg=0.0, pitch_deg=0.0, yaw_deg=90, order="ZYX")
    lidar_Ts = [apply_local_rotation(T, R_local) for T in lidar_Ts]

    # Rays
    if calib_app_config.Calib3d3d_SimParams.lidar_type == "airy":
        rays_local = make_rays_airy(
            calib_app_config.Calib3d3d_SimParams.rays_az,
            calib_app_config.Calib3d3d_SimParams.rays_el,
            vdeg=51.0,
            el_bias_deg=0.0,
        )
    elif calib_app_config.Calib3d3d_SimParams.lidar_type == "mid360":
        rays_local = make_rays_mid360_from_csv(
            calib_app_config.Calib3d3d_SimParams.mid360_laser_pattern_path,
            limit=800000,  # 読み込み行数の最大値
            el_offset=90,
            file_io_error_reporter=file_io_error_reporter,
        )

    # Colors（センサごとに色スキームをローテーション）
    base_color_schemes = [
        {
            "cad": (0.0, 0.6, 1.0),
            "ground": (0.2, 0.8, 0.2),
            "miss": (0.6, 0.6, 0.6),
            "ground_step": (1.0, 0.2, 0.2),
        },
        {
            "cad": (0.0, 0.6, 1.0),
            "ground": (0.8, 0.2, 0.8),
            "miss": (0.8, 0.2, 0.8),
            "ground_step": (1.0, 0.2, 0.2),
        },
    ]

    # Raycast: N台分ループ
    all_geoms: list[list[o3d.geometry.Geometry]] = []
    all_hit_cad: list[np.ndarray] = []
    all_hit_ground: list[np.ndarray] = []
    all_hit_step: list[np.ndarray] = []

    for idx_sensor, T_LiDAR in enumerate(lidar_Ts):
        colors = base_color_schemes[idx_sensor % len(base_color_schemes)]

        geoms_i, hit_cad_i, hit_ground_i, hit_step_i = raycast(
            scene,
            id_to_name,
            T_LiDAR,
            rays_local,
            max_range=calib_app_config.Calib3d3d_SimParams.max_range,
            ground_z=ground_z,
            colors=colors,
        )

        all_geoms.append(geoms_i)
        all_hit_cad.append(hit_cad_i)
        all_hit_ground.append(hit_ground_i)
        all_hit_step.append(hit_step_i)

    # ヒット点の集約（全センサ分）
    def _stack_hits(arr_list: list[np.ndarray]) -> np.ndarray:
        valid = [
            a.astype(np.float32, copy=False).reshape(-1, 3)
            for a in arr_list
            if isinstance(a, np.ndarray) and a.size > 0
        ]
        if not valid:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(valid)

    # センサごとのヒット（cad + ground + step）を作る
    hits_per_sensor: list[np.ndarray] = []
    for i in range(len(lidar_Ts)):
        h_i = _stack_hits([all_hit_cad[i], all_hit_ground[i], all_hit_step[i]])
        hits_per_sensor.append(h_i)

    # 全センサ分をまとめた hits_pts（従来互換）
    hits_pts = _stack_hits(hits_per_sensor)

    # Visual helpers
    visualize = False
    if visualize:
        axes = [frame_at(T, size=0.35) for T in lidar_Ts]
        sensors = [make_sensor_body(T, radius=0.06, height=0.08) for T in lidar_Ts]
        ground_plane = make_ground_plane_mesh(
            z=ground_z, size=50.0, color=(0.95, 0.95, 0.95)
        )

        # 全センサの geoms をフラットに
        ray_geoms = [g for geoms_i in all_geoms for g in geoms_i]

        vis = o3d.visualization.Visualizer()
        vis.create_window(
            "LiDAR Rays + CAD + Ground (N sensors)", width=1600, height=900
        )
        for g in [
            mesh_B,
            ground_plane,
            *axes,
            *sensors,
            *ray_geoms,
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=3),
        ]:
            vis.add_geometry(g)

        opt = vis.get_render_option()
        opt.mesh_show_wireframe = True
        opt.mesh_show_back_face = True
        opt.point_size = 2.0
        vis.run()
        vis.destroy_window()

    return hits_pts, hits_per_sensor
