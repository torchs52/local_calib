import glob
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import open3d as o3d


def triangle_areas(vertices: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """
    三角形ごとの面積 [同一単位^2]
    vertices: (N, 3), triangles: (M, 3) int
    """
    v1 = vertices[triangles[:, 0]]
    v2 = vertices[triangles[:, 1]]
    v3 = vertices[triangles[:, 2]]
    cross = np.cross(v2 - v1, v3 - v1)
    areas = 0.5 * np.linalg.norm(cross, axis=1)
    return areas


def sample_barycentric_in_triangles(
    vertices: np.ndarray,
    triangles: np.ndarray,
    samples_per_triangle: np.ndarray,
    rng: np.random.Generator,
) -> np.ndarray:
    """
    各三角形に指定個数の点をバリセントリック乱数でサンプリング（均一分布）
    """
    # 累積で総サンプル数を把握し、あらかじめ配列を確保
    total = int(samples_per_triangle.sum())
    if total == 0:
        return np.empty((0, 3), dtype=np.float32)

    out = np.empty((total, 3), dtype=np.float32)
    write_idx = 0

    for tri_idx, k in enumerate(samples_per_triangle):
        if k <= 0:
            continue
        i1, i2, i3 = triangles[tri_idx]
        a = vertices[i1]
        b = vertices[i2]
        c = vertices[i3]

        # Turk method: (u,v)~U(0,1)^2, if u+v>1, (u,v)=(1-u,1-v)
        u = rng.random(int(k))
        v = rng.random(int(k))
        mask = (u + v) > 1.0
        u[mask] = 1.0 - u[mask]
        v[mask] = 1.0 - v[mask]
        w = 1.0 - u - v  # 重み w for vertex a

        pts = (
            w[:, None] * a[None, :] + u[:, None] * b[None, :] + v[:, None] * c[None, :]
        )

        out[write_idx : write_idx + int(k), :] = pts
        write_idx += int(k)

    return out


def assign_samples_by_density(
    areas: np.ndarray, points_per_m2: float, scale_to_m: float
) -> np.ndarray:
    """
    面積に比例してサンプル数を割り当てる。
    - モデル単位を 'scale_to_m' [m/モデル単位] で m に換算して面積[m^2]→目標点数を計算。
    - 期待値から丸め、合計点数の偏りを抑えるよう誤差を調整。
    """
    # 面積を m^2 に変換
    areas_m2 = areas * (scale_to_m**2)
    expected = areas_m2 * points_per_m2
    base = np.floor(expected).astype(np.int64)
    residual = expected - base
    # 残差の大きい順に +1 して、総数を期待値に近づける
    need = int(round(expected.sum() - base.sum()))
    if need > 0:
        idx = np.argsort(-residual)[:need]
        base[idx] += 1
    return base


def mesh_to_pointcloud(
    obj_path: Path,
    scale_to_m: float = 1.0,
    points_per_m2: float = 2000.0,
    points_per_triangle: int = None,
    include_original_vertices: bool = False,
    voxel_size_m: float = 0.02,
    seed: int = 0,
) -> o3d.geometry.PointCloud:
    """
    OBJ を読み込み、三角形面から一様サンプリングで点群を生成。
    - scale_to_m: モデル単位→メートルの変換係数（例: mm モデルなら 0.001）
    - points_per_m2: 面密度（推奨）。None の場合は points_per_triangle を使用。
    - points_per_triangle: 各三角形あたりの固定サンプル数
    - include_original_vertices: 元頂点も点群に含める
    - voxel_size_m: ダウンサンプリング体素サイズ（None で無効）
    """
    mesh = o3d.io.read_triangle_mesh(str(obj_path), enable_post_processing=True)
    if mesh.is_empty():
        raise ValueError(f"Failed to load mesh: {obj_path}")

    # 法線・三角形化（OBJ に四角形/多角形があれば）
    if not mesh.has_triangles():
        mesh = mesh.triangulate()

    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_unreferenced_vertices()

    V = np.asarray(mesh.vertices, dtype=np.float64)
    T = np.asarray(mesh.triangles, dtype=np.int32)

    # スケール変換（→メートル）
    V_m = V * scale_to_m

    # 面積とサンプル数割り当て
    areas = triangle_areas(V, T)  # ここは元単位（丸め誤差小さくするため）
    rng = np.random.default_rng(seed)

    if points_per_m2 is not None and points_per_triangle is None:
        k_per_tri = assign_samples_by_density(areas, points_per_m2, scale_to_m)
    elif points_per_triangle is not None:
        k_per_tri = np.full(len(T), int(points_per_triangle), dtype=np.int64)
    else:
        raise ValueError(
            "points_per_m2 か points_per_triangle のどちらか一方を指定してください。"
        )

    # サンプリング
    pts = sample_barycentric_in_triangles(V_m, T, k_per_tri, rng)

    # 元頂点も含めたい場合
    if include_original_vertices:
        pts = np.vstack([pts, V_m.astype(np.float32)])

    # Open3D PointCloud 作成
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))

    # 任意のダウンサンプル
    if voxel_size_m is not None and voxel_size_m > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size_m)

    return pcd


def save_pointcloud(pcd: o3d.geometry.PointCloud, out_base: Path):
    """
    .ply（バイナリ）と .csv（xyz）で保存
    """
    out_base.parent.mkdir(parents=True, exist_ok=True)
    # PLY
    ply_path = out_base.with_suffix(".ply")
    o3d.io.write_point_cloud(str(ply_path), pcd, write_ascii=False, compressed=True)
    # CSV (x y z 空白区切り)
    csv_path = out_base.with_suffix(".csv")
    print(f"csv_path = {csv_path}")
    np.savetxt(csv_path, np.asarray(pcd.points), fmt="%.8f")
    return ply_path, csv_path


# =========================
# フォルダ一括処理
# =========================


def process_one(
    obj_path: str,
    output_dir: str,
    scale_to_m: float,
    points_per_m2: float,
    points_per_triangle: int,
    include_original_vertices: bool,
    voxel_size_m: float,
    seed: int,
):
    obj_path = Path(obj_path)
    stem = obj_path.stem
    out_base = Path(output_dir) / f"{stem}"
    try:
        pcd = mesh_to_pointcloud(
            obj_path=obj_path,
            scale_to_m=scale_to_m,
            points_per_m2=points_per_m2,
            points_per_triangle=points_per_triangle,
            include_original_vertices=include_original_vertices,
            voxel_size_m=voxel_size_m,
            seed=seed,
        )
        ply_path, csv_path = save_pointcloud(pcd, out_base)
        return (obj_path, ply_path, csv_path, len(pcd.points), None)
    except Exception as e:
        return (obj_path, None, None, 0, str(e))


def batch_process_objs(
    input_dir: str,
    output_dir: str,
    recursive: bool = True,
    scale_to_m: float = 1.0,
    points_per_m2: float = 2000.0,  # small is good
    points_per_triangle: int = None,
    include_original_vertices: bool = False,
    voxel_size_m: float = 0.02,  # large is good
    seed: int = 0,
    workers: int = max(1, os.cpu_count() // 2),
):
    """
    input_dir 内の *.obj をまとめて処理
    """
    pattern = "**/*.obj" if recursive else "*.obj"
    obj_files = sorted(glob.glob(str(Path(input_dir) / pattern), recursive=recursive))
    if not obj_files:
        raise FileNotFoundError(f"No OBJ files in {input_dir}")

    results = []
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futures = [
            ex.submit(
                process_one,
                obj_path,
                output_dir,
                scale_to_m,
                points_per_m2,
                points_per_triangle,
                include_original_vertices,
                voxel_size_m,
                seed + i,  # 各ファイルで seed ずらす
            )
            for i, obj_path in enumerate(obj_files)
        ]
        for fu in as_completed(futures):
            results.append(fu.result())

    # ログ出力
    ok = 0
    for obj_path, ply_path, csv_path, npts, err in results:
        if err is None:
            ok += 1
            print(f"[OK] {obj_path} -> {ply_path.name} / {csv_path.name} ({npts} pts)")
        else:
            print(f"[NG] {obj_path}\n     Error: {err}")

    print(f"\nDone. {ok}/{len(results)} files processed.")
    return results


def visualize_all_plys(
    output_dir: str,
    results=None,
    voxel_size_m: float = None,  # 表示用にさらに軽くしたいなら値を入れる（例 0.02）
    colorize: bool = True,  # 各点群を自動で別色にする
    layout: str = "overlay",  # "overlay"（重ねる） or "row"（横一列に並べる）
    gap_ratio: float = 0.10,  # "row" のとき、各モデル幅に対する隙間割合
):
    """
    output_dir 内（または results に含まれる）.ply を全部読み込んで表示
    """
    # 1) .ply の一覧を作る（results があればそれを優先）
    ply_paths = []
    if results is not None:
        for _obj, ply_path, _csv, _npts, err in results:
            if err is None and ply_path is not None:
                ply_paths.append(str(ply_path))
    if not ply_paths:
        # results がない/空ならフォルダから拾う
        ply_paths = sorted(
            glob.glob(str(Path(output_dir) / "**/*.ply"), recursive=True)
        )

    if not ply_paths:
        print("No PLY files to visualize.")
        return

    # 2) 読み込み & オプションの軽量化
    pcds = []
    for p in ply_paths:
        pcd = o3d.io.read_point_cloud(p)
        if voxel_size_m is not None and voxel_size_m > 0:
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size_m)
        pcds.append(pcd)

    # 3) 任意：色分け
    if colorize:
        N = len(pcds)
        # HSV を等間隔に回して色を作る（Open3D は [0,1] RGB）
        for i, pcd in enumerate(pcds):
            h = i / max(1, N)  # [0,1)
            # 簡易 HSV→RGB（彩度・明度固定）
            s, v = 0.85, 0.95
            k = int(h * 6)
            f = h * 6 - k
            p = v * (1 - s)
            q = v * (1 - f * s)
            t = v * (1 - (1 - f) * s)
            if k % 6 == 0:
                r, g, b = v, t, p
            elif k % 6 == 1:
                r, g, b = q, v, p
            elif k % 6 == 2:
                r, g, b = p, v, t
            elif k % 6 == 3:
                r, g, b = p, q, v
            elif k % 6 == 4:
                r, g, b = t, p, v
            else:
                r, g, b = v, p, q
            col = np.array([[r, g, b]], dtype=np.float64)
            pcd.colors = o3d.utility.Vector3dVector(
                np.repeat(col, len(pcd.points), axis=0)
            )

    # 4) 任意：横一列レイアウトで配置（重なり回避）
    if layout == "row":
        offsets = []
        cursor_x = 0.0
        for pcd in pcds:
            bbox = pcd.get_axis_aligned_bounding_box()
            extent = bbox.get_extent()
            width = extent[0]
            gap = width * gap_ratio
            offsets.append(
                cursor_x - bbox.get_min_bound()[0]
            )  # 原点寄せ + 横方向に並べる
            cursor_x += width + gap
        for pcd, offx in zip(pcds, offsets):
            pcd.translate([offx, 0.0, 0.0], relative=False)

    # 5) 軸フレームを追加して描画
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    o3d.visualization.draw_geometries([*pcds, coord])


if __name__ == "__main__":
    print(os.getcwd())
    # 例）モデル単位が mm の場合は 0.001 にする
    input_dir = "./config/crane3d/visualize/SCX2000-3/immobile/"
    # output_dir = "./crane3d/SCX2000-3/mobile_pts/"
    output_dir = "./config/tmp/immobile/"

    # 面密度で指定（例：1 m^2 あたり 3000 点）
    results = batch_process_objs(
        input_dir=input_dir,
        output_dir=output_dir,
        recursive=True,
        scale_to_m=0.001,  # mm → m なら 0.001
        points_per_m2=100.0,  # 面密度（推奨）
        points_per_triangle=None,  # 三角形ごとの固定個数を使う場合は値を入れて points_per_m2 を None に
        include_original_vertices=False,
        # voxel_size_m=None,  # 最終点群を 5cm 体素でダウンサンプル（不要なら None）
        voxel_size_m=0.05,  # 最終点群を 5cm 体素でダウンサンプル（不要なら None）
        seed=42,
        workers=max(1, os.cpu_count() - 1),
    )

    visualize_all_plys(
        output_dir=output_dir,
        results=results,
        voxel_size_m=0.01,
        colorize=True,
        layout="overlay",
        gap_ratio=0,
    )
