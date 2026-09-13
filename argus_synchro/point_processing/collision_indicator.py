from __future__ import annotations

import numpy as np
import open3d as o3d
from numpy.typing import NDArray

from argus_synchro.common.common import t_py_col_res


def rotation_matrix_from_vectors(
    vec1: NDArray[np.float64],
    vec2: NDArray[np.float64],
) -> NDArray[np.float64]:
    """vec1からvec2への回転行列を計算する"""
    a, b = (
        (vec1 / np.linalg.norm(vec1)).reshape(3),
        (vec2 / np.linalg.norm(vec2)).reshape(3),
    )
    v: NDArray[np.float64] = np.cross(a, b).astype(np.float64)
    c: NDArray[np.float64] = np.dot(a, b)
    s = np.linalg.norm(v)

    # ベクトルが平行または逆平行な場合の処理
    if s < 1e-10:  # 数値精度の問題を考慮した小さな閾値
        if c > 0:
            return np.eye(3)  # 平行なベクトルの場合、回転は不要
        # 逆平行なベクトルの場合、180度回転
        orthogonal_vec = (
            np.array([1, 0, 0]) if np.abs(a[0]) < 0.9 else np.array([0, 1, 0])
        )
        v = np.cross(a, orthogonal_vec).astype(np.float64)
        v = v / np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + 2 * kmat.dot(kmat)

    # vのスキュー対称クロス積行列を計算
    kmat: NDArray[np.float64] = np.array(
        [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]],
    )
    rotation_matrix: NDArray[np.float64] = (
        np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))
    )
    return rotation_matrix


def create_cylinder(
    obj_dict: t_py_col_res,
    radius: float = 0.05,
) -> tuple[o3d.utility.Vector3dVector, o3d.utility.Vector3iVector]:
    obj_dict.pop(-1)

    if len(obj_dict) == 0:
        empty_mesh = o3d.geometry.TriangleMesh()
        return empty_mesh.vertices, empty_mesh.triangles

    all_vertices: list[NDArray[np.float64]] = []
    all_triangles: list[NDArray[np.int32]] = []
    offset = 0

    skip_count = 0

    for _, _, p1_t, p2_t, _, _ in obj_dict.values():
        p1: NDArray[np.float64] = np.array(p1_t)
        p2: NDArray[np.float64] = np.array(p2_t)
        direction = p2 - p1
        center = (p1 + p2) / 2
        length: float = float(np.linalg.norm(direction))

        if (
            length == 0
        ):  # length == 0の場合、cylinderが生成できずエラーになるのでスキップする
            skip_count += 1
            if (
                (len(obj_dict) - skip_count) == 0
            ):  # スキップした結果、len(obj_dict)==0になった場合、空のオブジェクトを返す
                empty_mesh = o3d.geometry.TriangleMesh()
                return empty_mesh.vertices, empty_mesh.triangles
            continue

        cylinder: o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_cylinder(
            radius=radius,
            height=length,
        )
        # cylinder.paint_uniform_color([1.0, 0, 0])  # オプション:色を設定
        rotation_matrix: NDArray[np.float64] = rotation_matrix_from_vectors(
            np.array([0, 0, 1]),
            direction,
        )
        trans: NDArray[np.float64] = np.eye(4)
        trans[:3, :3] = rotation_matrix
        trans[:3, 3] = center
        cylinder.transform(trans)

        current_vertices: NDArray[np.float64] = np.asarray(cylinder.vertices)
        current_triangles: NDArray[np.int32] = np.asarray(cylinder.triangles) + offset
        offset += len(current_vertices)

        all_vertices.append(current_vertices)
        all_triangles.append(current_triangles)

    # 最終的なverticesとtrianglesを結合
    final_vertices: NDArray[np.float64] = (
        np.vstack(all_vertices) if all_vertices else np.array([])
    )
    final_triangles: NDArray[np.int32] = (
        np.vstack(all_triangles) if all_triangles else np.array([])
    )

    # 新しいTriangleMeshオブジェクトを生成
    final_mesh = o3d.geometry.TriangleMesh()
    final_mesh.vertices = o3d.utility.Vector3dVector(final_vertices)
    final_mesh.triangles = o3d.utility.Vector3iVector(final_triangles)
    # final_mesh.compute_vertex_normals()  # 法線の計算

    return o3d.utility.Vector3dVector(final_vertices), o3d.utility.Vector3iVector(
        final_triangles,
    )
