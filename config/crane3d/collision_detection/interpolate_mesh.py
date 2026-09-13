"""
各mesh(三角形)面にバリセントリック法を用いて点を生成。
meshオブジェクトを高密度な点群オブジェクトへの変換が目的。
"""

import numpy as np
import open3d as o3d


def interpolate_in_triangle(v1, v2, v3, num_points=10):
    """三角形内に点を補間する"""
    points = []
    for i in range(1, num_points):
        for j in range(1, num_points - i):
            w1 = i / num_points
            w2 = j / num_points
            w3 = 1 - w1 - w2
            point = w1 * v1 + w2 * v2 + w3 * v3
            points.append(point)
    return points


def load_obj_and_interpolate_faces(obj_file_path, num_points=10):
    vertices = []
    faces = []

    with open(obj_file_path) as file:
        for line in file:
            if line.startswith("v "):
                parts = line.split()
                vertices.append(
                    np.array([float(parts[1]), float(parts[2]), float(parts[3])]),
                )
            elif line.startswith("f "):
                parts = line.split()
                faces.append([int(part.split("/")[0]) - 1 for part in parts[1:]])

    interpolated_points = []
    for face in faces:
        v1, v2, v3 = vertices[face[0]], vertices[face[1]], vertices[face[2]]
        interpolated_points.extend(interpolate_in_triangle(v1, v2, v3, num_points))

    all_points = np.array(vertices + interpolated_points, dtype=np.float32)
    return all_points


def read_saved_points(file_path):
    points = np.genfromtxt(file_path, delimiter=" ")
    return points


# OBJファイルから点群データを読み込み、面内に点を補間
obj_file_path = "../crane3d/SCX900.obj"  # OBJファイルのパス
point_cloud = load_obj_and_interpolate_faces(obj_file_path, num_points=10)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3] / 1000)
pcd = pcd.voxel_down_sample(0.05)
coord_arrow = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=1,
    origin=[0, 0, 0],
)
o3d.visualization.draw_geometries(geometry_list=[pcd, coord_arrow])

# 生成した点群データを保存
output_file_path = "../crane3d/interpolated_SCX900_09_crawler_left.csv"
# np.savetxt(output_file_path, np.array(pcd.points))

# 保存した点群データを読み込み
points = read_saved_points(output_file_path)
