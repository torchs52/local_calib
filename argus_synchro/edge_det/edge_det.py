"""
崖検出のメインの処理が入っているモジュール
"""

import itertools
from configparser import ConfigParser, ExtendedInterpolation
from itertools import product

import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from argus_synchro_lib.edge_det import AggName
from argus_synchro_lib.octotree import NodeEntity, OctoTree
from numpy.typing import NDArray

from argus_synchro.common.common import (
    Err,
    NDImage,
    NDPoint2fArray,
    NDPoint2i,
    NDPoint2iArray,
    NDPoint3f,
    NDPoint3fArray,
    NDSeries,
    Ok,
    Point2f,
    Point2i,
    Point3f,
    RangeF,
    Result,
    SizeF,
    map_,
    t_np_uint,
    unwrap,
)
from argus_synchro.config import app_config as app_config_module
from argus_synchro.config.app_config import AppConfig
from argus_synchro.edge_det.base import (
    BorderDetector,
    EdgeDetectionIF,
    EdgeDetectionResult,
)
from argus_synchro.edge_det.border import BorderExtractor
from argus_synchro.edge_det.common import apply_morphology_close, thresh_based_edge2bin
from argus_synchro.edge_det.const import (
    DETECT_MESH_COLOR,
    BevCoord,
    BinMethod,
    DediscretizeMethod,
    EdgeFilterType,
    ScanDirType,
)
from argus_synchro.edge_det.detect_range import (
    RangeProperty,
    RangePropertyBase,
)
from argus_synchro.edge_det.edge_extract import (
    apply_DoG_filter,
    bev_masking,
    extract_edge,
)
from argus_synchro.edge_det.occlusion import (
    check_occlusion_from_other_origin,
    mask_img_by_value,
)
from argus_synchro.edge_det.polar import (
    calc_max_radius_each_theta,
    calc_rect_max_dist,
    postproc_for_polar,
    preproc_edge,
    py_octotree2bev,
)
from argus_synchro.edge_det.transform import polar_grid_to_lidar_coord
from argus_synchro.edge_det.typedef import (
    EdgeDetError,
    EdgeDetInvalidArgumenError,
    EdgeDetLogicError,
    PxPyTup,
    XYMinMax,
    XYTup,
    XYZTup,
)
from argus_synchro.edge_det.utils import get_empty_points_lines_length


def np_to_pcd(np_mat: NDPoint3fArray) -> o3d.geometry.PointCloud:
    """
    numpyをPointCloudに変換する

    :param np_mat: n*3行列
    :type np_mat: NDPoint3fArray
    :return: point cloud
    :rtype: PointCloud
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_mat)
    return pcd


def np_voxel_downsample(points: NDPoint3f, voxel_size: float) -> NDPoint3fArray:
    """ndarrayにopen3dのvoxel_down_sampleを適用して、ndarrayにして返す"""
    return np.array(np_to_pcd(points).voxel_down_sample(voxel_size).points)


class EdgeDetection(EdgeDetectionIF):
    grid_size: XYTup  # 鳥観図の格子サイズ
    range_property: (
        RangePropertyBase | None
    )  # Note: どの範囲の崖検出をしているか明確にするため、インスタンス変数として保持している
    side_range: XYTup  # y軸の検出範囲
    fwd_range: XYTup  # x軸の検出範囲
    side_length: float  # y軸の幅
    fwd_length: float  # x軸の幅
    edge_size: PxPyTup  # エッジ点の大きさ
    scaled_bev: bool  # スケーリング有無
    edge_filter: EdgeFilterType  # 用いるエッジフィルタ
    scan_direction: ScanDirType  # 走査する方向 (+x, -x, +y, -y)がある
    remove_duplicate_label: bool  # 後方の崖を除外するか
    voxel_size: float | None  # 点群の粒度を制御するパラメータ
    border_det: BorderDetector  # 検出範囲に関連する処理を記述しておくインスタンス

    def __init__(
        self,
        range_property: RangePropertyBase | None = None,
        grid_size: SizeF = (0.1, 0.1),
        side_range: RangeF | None = (-10, 10),
        fwd_range: RangeF | None = (-10, 10),
        voxel_size: float | None = 0.1,
        height_strips: float = 5,
        edge_width: float = 1,
        scaled_bev: bool = True,
        remove_duplicate_label: bool = True,
        default_edge_filter: EdgeFilterType = EdgeFilterType.SOBEL,
        default_scan_direction: ScanDirType = ScanDirType.PLUS_Y,
        debug: bool = True,
    ) -> None:
        """
        崖検出処理を行うインスタンス, 最初に作った崖検出インスタンス
        後方のみが検出対象で各y軸で、x軸方向が正の方向にエッジを見るようなロジック
        途中途中で画像を鈍らせたりするが、そういった細かい処理を除くと、
        LiDAR座標 -> 鳥観図 -> エッジ画像生成 -> 二値化 -> ラベリング画像生成 -> エッジ走査 -> 結果出力

        :param self: 説明
        :param range_property: x,y軸の検出範囲を計算したりするためのクラス
        :type range_property: RangePropertyBase | None
        :param grid_size: 実座標における鳥瞰図の幅
        :type grid_size: SizeF
        :param side_range: y軸方向の検出範囲
        :type side_range: RangeF | None
        :param fwd_range: x軸方向の検出範囲
        :type fwd_range: RangeF | None
        :param voxel_size: voxel_down_sampleのパラメータ, Noneの場合ダウンサンプルしない
        :type voxel_size: float | None
        :param height_strips: 検出した崖のz方向のオフセット
        :type height_strips: float
        :param edge_width: 検出したエッジの長さ
        :type edge_width: float
        :param scaled_bev: 鳥観図を正規化するかどうか, 基本True
        :type scaled_bev: bool
        :param remove_duplicate_label: ある崖エッジの後方の崖を検知するか, Trueの場合は検知しない
        :type remove_duplicate_label: bool
        :param default_edge_filter: エッジ画像生成で用いるエッジフィルタの列挙型
        :type default_edge_filter: EdgeFilterType
        :param default_scan_direction: エッジ走査の方向
        :type default_scan_direction: ScanDirType
        :param debug: デバッグの有無, Trueの場合、open3dの表示が行われる
        :type debug: bool
        """

        self.range_property = range_property
        self.grid_size: SizeF = grid_size
        self.height_strips = height_strips
        self.edge_size = (
            int(edge_width / grid_size[1]),
            int(edge_width / grid_size[0]),
        )
        self.scaled_bev = scaled_bev
        self.edge_filter = (
            range_property.edge_filter_type if range_property else default_edge_filter
        )
        self.scan_direction = (
            range_property.scan_direction if range_property else default_scan_direction
        )
        self.remove_duplicate_label = remove_duplicate_label
        self.voxel_size = voxel_size
        self.debug = debug

        # range_property, side_range, fwd_rangeの組み合わせで検出範囲の設定を分ける
        match (range_property, side_range, fwd_range):
            case (range_prop, _, _) if range_prop is not None:
                self.side_range: RangeF = range_prop.calculate_side_range()
                self.fwd_range: RangeF = range_prop.calculate_fwd_range()
            case (None, side, fwd) if side is not None and fwd is not None:
                self.side_range = side
                self.fwd_range = fwd
            case _:
                raise ValueError(
                    "range_calculatorか、side_rangeとfwd_rangeのどちらかは値を持っている必要がある"
                    f"range_calculator={range_property}, "
                    f"side_range={side_range}, fwd_range={fwd_range}"
                )

        self.side_length = abs(self.side_range[1] - self.side_range[0])
        self.fwd_length = abs(self.fwd_range[1] - self.fwd_range[0])

        self.border_det = BorderDetector(
            self.scan_direction,
            self.side_range,
            self.fwd_range,
            self.side_length,
            self.fwd_length,
            self.grid_size,
            self.height_strips,
        )

    def __str__(self) -> str:
        """EdgeDetecionの状態を表す設定値を文字列で固めて返す"""
        return f""" grid_size = {self.grid_size},
    range_calculator = {self.range_property},
    side_range = {self.side_range},
    fwd_range = {self.fwd_range},
    height_strips = {self.height_strips},
    edge_size = {self.edge_size},
    side_lenth = {self.side_length},
    fwd_length = {self.fwd_length},
    scaled_bev = {self.scaled_bev},
    edge_filter = {self.edge_filter},
    scan_direction = {self.scan_direction},
    remove_duplicate_label = {self.remove_duplicate_label},
    debug = {self.debug},
    border_det = {self.border_det}
    """

    def min_max(
        self,
        x: cv2.typing.MatLike,
        axis: int | None = None,
    ) -> cv2.typing.MatLike:
        """
        xの各要素を[0,1]に正規化する

        :param self: 説明
        :param x: 説明
        :type x: cv2.typing.MatLike
        :param axis: 説明
        :type axis: int | None
        :return: 説明
        :rtype: MatLike
        """
        min = x.min(axis=axis, keepdims=True)
        max = x.max(axis=axis, keepdims=True)
        return (x - min) / (max - min)

    def remove_outliers_from_point_cloud(
        self,
        data: NDPoint3fArray,
        x_min: float | None = None,
        x_max: float | None = None,
        y_min: float | None = None,
        y_max: float | None = None,
        z_min: float | None = None,
        z_max: float | None = None,
    ) -> NDPoint3fArray:
        """
        指定された範囲外の点を点群データから除外する。

        パラメータ:
            data: 点群データ。Nx3のNumPy配列とする。
            x_min, x_max: x軸に関する最小値と最大値の範囲。
            y_min, y_max: y軸に関する最小値と最大値の範囲。
            z_min, z_max: z軸に関する最小値と最大値の範囲。

        戻り値:
            範囲内の点のみを含む点群データ
        """

        # 各軸に対するフィルタ条件を作成
        filter_condition = np.ones(len(data), dtype=bool)

        if x_min is not None and x_max is not None:
            filter_condition &= (data[:, 0] >= x_min) & (data[:, 0] < x_max)

        if y_min is not None and y_max is not None:
            filter_condition &= (data[:, 1] >= y_min) & (data[:, 1] < y_max)

        if z_min is not None and z_max is not None:
            filter_condition &= (data[:, 2] >= z_min) & (data[:, 2] < z_max)

        # フィルタ条件に基づいてデータをフィルタリング
        filtered_data = data[filter_condition]

        return filtered_data

    def scale_to_255(
        self, pixel_values: NDPoint3f, min_value: float, max_value: float
    ) -> NDSeries:
        """
        ピクセル値を0から255の範囲にスケーリングする。
        パラメータ:
            pixel_values: スケーリングするピクセル値の配列
            min: スケーリングのための最小値
            max: スケーリングのための最大値

        戻り値:
            スケーリングされたピクセル値の配列
        """
        # 最小値と最大値の間で正規化
        normalized = (pixel_values - min_value) / (max_value - min_value)
        # 0から255の範囲にスケーリング
        scaled = np.clip(normalized * 255, 0, 255).astype(np.uint8)
        return scaled

    def save_img(self, im: NDImage, path: str) -> None:
        """
        imをpathに保存する

        :param self: 説明
        :param im: 説明
        :type im: NDImage
        :param path: 説明
        :type path: str
        """
        plt.figure(figsize=(10, 10))
        plt.imshow(im, "gray", interpolation="nearest", aspect="auto")
        plt.savefig(f"{path}", format="png", dpi=300)

    def convert_points_to_bev_image(
        self,
        points: NDPoint3fArray,
        min_z: float = -1.88,
        max_z: float = -0.88,
    ) -> NDImage:
        """
        点群データをBird's Eye View (BEV) グリッドマップに変換する。

        LiDAR点群データを受け取り、指定されたグリッドサイズ、サイド範囲、前方範囲に基づいてBEVマップに変換。Z座標は特定の範囲でスケーリングされ、グリッドマップに対応する高さ値として使用

        パラメータ:
            points (numpy.ndarray): LiDAR点群データ。Nx3の形状を持ち、各行は[x, y, z]の形式。
            grid_size (tuple): グリッドマップの解像度。形式は(x_resolution, y_resolution)。
            side_range (tuple): x軸の範囲。形式は(min_x, max_x)。
            fwd_range (tuple): y軸の範囲。形式は(min_y, max_y)。

        戻り値:
            grid_map_scaled (numpy.ndarray): Z座標がスケーリングされたBEVグリッドマップ。
            grid_map (numpy.ndarray): スケーリングされていないZ座標を持つBEVグリッドマップ。

        処理の流れ：
        1. LiDAR座標をグリッドマップ座標に変換。
        2. グリッドマップ座標を指定された範囲内にシフト。
        3. Z座標を特定の範囲でスケーリングし、対応するグリッドに高さ値として割り当て。
        4. モルフォロジカル変換を適用してグリッドマップを滑らかにする。
        """

        # 点群を計算対象に絞る
        target_lidar_points = self.remove_outliers_from_point_cloud(
            points,
            x_min=self.fwd_range[0],
            x_max=self.fwd_range[1],
            y_min=self.side_range[0],
            y_max=self.side_range[1],
        )

        # 絞った点群に対してBEV画像を作る
        grid_map = self.border_det.lidar2pixel(
            lidar_points=target_lidar_points,
            min_z=min_z,
            max_z=max_z,
            is_scaled=self.scaled_bev,
        )

        # モルフォロジカル変換の適用
        kernel = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(grid_map, cv2.MORPH_CLOSE, kernel).astype(np.uint8)

    def mask_img_by_gradient(
        self,
        filtered_img: cv2.typing.MatLike,
        max_gradient: float = 30,
        min_gradient: float = -30,
    ) -> cv2.typing.MatLike:
        """
        画像の勾配に基づいて、各ピクセルの勾配方向をマスクとして出力する

        この関数は、入力された2次元配列（画像の勾配を表す）の各ピクセルに対してマスク処理を行う
        勾配値がgrad_strength以下のピクセルは-1としてマークされ、強い負の勾配を示す
        勾配値がgrad_strength以上のピクセルは1としてマークされ、強い正の勾配を示す
        その他のピクセルは0としてマークされ、勾配が弱いまたは無いことを示す

        パラメータ:
        - filtered_img: 勾配値の2次元numpy配列

        戻り値:
        - masked_img: 勾配方向に基づいて、各要素が-1、0、または1になっている2次元numpy配列
        """
        return np.where(
            filtered_img <= min_gradient,
            -1,
            np.where(filtered_img >= max_gradient, 1, 0),
        )

    def check_occlusion(
        self,
        masked_img: cv2.typing.MatLike,
        pixel_range: int = 5,
    ) -> NDArray[np.bool_]:
        """
        画像の勾配方向マスクに基づいてオクルージョンをチェックする

        この関数は、マスクされた画像の各ピクセルをスキャンし、y軸方向の上下指定されたピクセル範囲内に異なる符号のピクセルが存在するかどうかをチェックする
        そのようなピクセルが見つかった場合、オクルージョンとみなされ、出力配列の対応する位置はTrueとしてマークする

        パラメータ:
        - masked_img: 勾配方向マスクを表す-1、0、または1の各要素を持つ2次元numpy配列
        - y_range: y軸方向のチェック範囲(ピクセル数)

        戻り値:
        - output: オクルージョンを示すTrueの位置を持つ2次元のブール型numpy配列
        """
        arr = masked_img
        output = np.full(arr.shape, False, dtype=bool)
        height, width = arr.shape
        for y in range(pixel_range, height - pixel_range):
            for x in range(pixel_range, width):
                if arr[y, x] == 0:
                    continue
                # 中心ピクセルを囲むウィンドウを取得
                window = arr[
                    max(0, y - pixel_range) : min(height, y + pixel_range + 1), x
                ]
                # 中心ピクセルと異なる符号を持つピクセルが存在するかチェック
                if np.any(window * arr[y, x] == -1):
                    output[y, x] = True
        return output

    def _check_occlusion(self, masked_img: NDImage) -> NDImage:
        """
        画像の勾配方向マスクに基づいてオクルージョンをチェックする

        この関数は、マスクされた画像の各ピクセルをスキャンし、y軸方向の上下3ピクセル以内に異なる符号のピクセルが存在するかどうかをチェックする
        そのようなピクセルが見つかった場合、オクルージョンとみなされ、出力配列の対応する位置はTrueとしてマークする

        パラメータ:
        - masked_img: 勾配方向マスクを表す-1、0、または1の各要素を持つ2次元numpy配列

        戻り値:
        - output: オクルージョンを示すTrueの位置を持つ2次元のブール型numpy配列
        """
        arr = masked_img
        output = np.full(arr.shape, False, dtype=bool)
        height, width = arr.shape

        for y in range(3, height - 3):
            for x in range(width):
                if arr[y, x] == 0:
                    continue
                window = arr[y - 3 : y + 4, x]
                if np.any(window * arr[y, x] < 0):
                    output[y, x] = True
        return output

    def apply_occlusion_mask(
        self,
        filtered_img: cv2.typing.MatLike,
    ) -> cv2.typing.MatLike:
        """
        勾配方向マスクに基づいて画像にオクルージョンマスクを適用する

        この関数はまず入力画像に勾配方向マスクを適用し、次にオクルージョンをチェックする
        オクルージョンと識別されたピクセルは出力画像で0に設定される

        パラメータ:
        - filtered_img: 処理対象の2次元numpy配列の画像

        戻り値:
        - result_img: オクルージョンに基づいて更新された画像を表す2次元numpy配列
        """
        masked_img = self.mask_img_by_gradient(filtered_img)
        occlusion_mask = self.check_occlusion(masked_img)
        return np.where(occlusion_mask, 0, masked_img)

    def preproc_edge(self, img: cv2.typing.MatLike) -> cv2.typing.MatLike:
        """エッジ検出を行う前の前処理を行って、その結果を返す"""
        bi = cv2.bilateralFilter(img, 10, 20, 20)
        for _ in range(2):
            bi = cv2.bilateralFilter(img, 10, 20, 20)
        return bi

    def extract_edge(self, img: cv2.typing.MatLike) -> cv2.typing.MatLike:
        """画像のエッジ検出とオクルージョン処理を行う
        今後検出範囲を複数に分けて考えて、統合する事を考えると、機能を分割したくなりそうなので、border_detectionを細分化する
        """
        im_filter = self.edge_filter

        # sobelフィルタ or ラプラシアンフィルタで輪郭検出
        if im_filter == EdgeFilterType.SOBEL:
            # デフォルトは、これまでのsobel変換を行う
            proc_img = img.astype(np.float32)
            kernel = np.array([[0, 0, 0], [0, 1, 0], [0, -1, 0]])
            edge_detected_img = cv2.filter2D(proc_img, -1, kernel)
        elif im_filter == EdgeFilterType.SOBEL_X_FORWARD:
            proc_img = img.astype(np.float32)
            kernel = np.array([[0, 0, 0], [0, 1, 0], [0, -1, 0]])
            edge_detected_img = cv2.filter2D(proc_img, -1, kernel)
        elif im_filter == EdgeFilterType.SOBEL_X_BACKWARD:
            proc_img = img.astype(np.float32)
            kernel = np.array([[0, -1, 0], [0, 1, 0], [0, 0, 0]])
            edge_detected_img = cv2.filter2D(proc_img, -1, kernel)
        elif im_filter == EdgeFilterType.SOBEL_Y_FORWARD:
            proc_img = img.astype(np.float32)
            kernel = np.array([[0, 0, 0], [0, 1, -1], [0, 0, 0]])
            edge_detected_img = cv2.filter2D(proc_img, -1, kernel)
        elif im_filter == EdgeFilterType.SOBEL_Y_BACKWARD:
            proc_img = img.astype(np.float32)
            kernel = np.array([[0, 0, 0], [-1, 1, 0], [0, 0, 0]])
            edge_detected_img = cv2.filter2D(proc_img, -1, kernel)
        elif im_filter == EdgeFilterType.LAPLACIAN:
            edge_detected_img = cv2.Laplacian(img, cv2.CV_64F, ksize=3)
        elif im_filter == EdgeFilterType.DoG:
            edge_detected_img = apply_DoG_filter(img)
        else:
            raise ValueError(
                f""" im_filterは以下のどれかが実装されています:
                SOBEL, SOBEL_X_FORWARD, SOBEL_X_BACKWARD,
                SOBEL_Y_FORWARD, SOBEL_Y_BACKWARD,
                LAPLACIAN, DoG.
                今回の入力: im_filter={im_filter}
                """
            )

        return edge_detected_img

    def edge2bin(self, edge_img: cv2.typing.MatLike) -> cv2.typing.MatLike:
        """
        エッジ画像を二値画像に変換する
        2値画像に変換した後で、morphology変換もしている

        :param self: 説明
        :param edge_img: エッジ画像
        :type edge_img: cv2.typing.MatLike
        :return: 2値画像
        :rtype: MatLike

        """
        # floatでないと二値化できないので、float変換している
        z_normalized_img = (self.min_max(edge_img) * 255).astype(np.float64)
        _, binalized_img = cv2.threshold(z_normalized_img, 150, 255, cv2.THRESH_BINARY)

        # ラベリング
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(
            binalized_img,
            cv2.MORPH_CLOSE,
            kernel,
        ).astype(np.uint8)

    def choose_edges(
        self,
        labeled_imgs: cv2.typing.MatLike,
        loop_limit: int = 1000,
    ) -> list[int]:
        """あるエッジの後方にあるエッジを消して、関連があるエッジを選択する"""
        # Remark: scan方向と同じ方向のエッジを除去しているが、これで良いかは要検討
        target_entity = self.scan_direction

        if target_entity == ScanDirType.PLUS_Y:
            pick_field_name = "min_y"
            remove_fields = ("min_x", "max_x")
            pick_func = min
        elif target_entity == ScanDirType.PLUS_X:
            pick_field_name = "min_x"
            remove_fields = ("min_y", "max_y")
            pick_func = min
        elif target_entity == ScanDirType.MINUS_Y:
            pick_field_name = "min_y"
            remove_fields = ("min_x", "max_x")
            pick_func = max
        elif target_entity == ScanDirType.MINUS_X:
            pick_field_name = "min_x"
            remove_fields = ("min_y", "max_y")
            pick_func = max
        else:
            msg = "target_entityはScanDir.PLUS_X, ScanDir.PLUS_Y, ScanDir.MINUS_X, ScanDir.MINUS_Yしか想定されていないです"
            raise ValueError(msg)

        # labeled_imgs = 0の部分はグループ対象ではないので、無視する
        whole_labels = np.unique(labeled_imgs[labeled_imgs > 0])

        if len(whole_labels) == 0:
            # 範囲内にエッジがないので、空の配列を返す
            return []

        # 各ラベルの最大, 最小のx,y座標を取得する
        label_minmax: dict[int, XYMinMax] = {}
        for label_no in whole_labels:
            pos_y, pos_x = np.where(labeled_imgs == label_no)
            label_minmax[label_no] = XYMinMax(
                pos_x.min(),
                pos_x.max(),
                pos_y.min(),
                pos_y.max(),
            )

        chosen_labels: set[int] = set()  # 選ぶindex

        count = 0
        while count < loop_limit:
            # target_entity最小のindexを一つ選ぶ, pick_field_name=min_yの場合, min_yのminを選ぶ
            picked_label: int = pick_func(
                label_minmax.items(),
                key=lambda kv: getattr(kv[1], pick_field_name),
            )[0]

            # 選んだindexを返り値の集合に追加して、選択対象から外す, remove_fields=("min_x", "max_x")の場合、
            min_val = getattr(label_minmax[picked_label], remove_fields[0])
            max_val = getattr(label_minmax[picked_label], remove_fields[1])
            chosen_labels.add(picked_label)
            del label_minmax[picked_label]

            # 後ろ側に存在するindexを選択対象から外す
            removed_labels = dict(
                filter(
                    lambda kv: (
                        (
                            min_val <= getattr(kv[1], remove_fields[0])
                            and getattr(kv[1], remove_fields[0]) <= max_val
                        )
                        or (
                            min_val <= getattr(kv[1], remove_fields[1])
                            and getattr(kv[1], remove_fields[1]) <= max_val
                        )
                        or (
                            getattr(kv[1], remove_fields[0]) <= min_val
                            and max_val <= getattr(kv[1], remove_fields[1])
                        )
                    ),
                    label_minmax.items(),
                ),
            )
            for removed_label in removed_labels:
                del label_minmax[removed_label]

            if len(label_minmax) == 0:
                break
            count += 1
        return list(chosen_labels)

    def img_labeling(
        self,
        bin_img: cv2.typing.MatLike,
        edge_th: int,
    ) -> tuple[cv2.typing.MatLike, cv2.typing.MatLike, list[int]]:
        """
        画像をラベリングする
        ラベリング結果の中で、有効なラベルを選ぶ部分まで行う

        :param self: 説明
        :param bin_img: 説明
        :type bin_img: cv2.typing.MatLike
        :param edge_th: 説明
        :type edge_th: int
        :return: 説明
        :rtype: tuple[MatLike, MatLike, list[int]]
        """
        # ラベリング処理
        labels, labeled_imgs, bbox, _ = cv2.connectedComponentsWithStats(
            bin_img,
            connectivity=8,
        )

        # 領域が小さいラベルを取得
        small_region_ind = np.where(bbox[:, 4] <= edge_th)[0]

        # scan方向に応じて用いるラベルを決める
        if self.remove_duplicate_label:
            _labeled_imgs = labeled_imgs.copy()
            _labeled_imgs[np.isin(_labeled_imgs, small_region_ind)] = 0
            chosen_label = self.choose_edges(_labeled_imgs)
        else:
            chosen_label = np.setdiff1d(np.arange(labels), small_region_ind).tolist()

        return labeled_imgs, bbox, chosen_label

    def _border_detection(
        self,
        labeled_imgs: NDImage,
        z_offset: float,
        chosen_label: list[int],
    ) -> tuple[NDPoint3fArray, list]:
        """
        ラベリング画像から操作方向に基づいて、3d座標を計算する

        パラメータ:
            labeled_img (numpy.ndarray): エッジ検出で得られた二値画像をラベリングした結果の画像
            z_offset (float): Z座標のオフセット値。
            chosen_label (list): labeled_imgの中でedge_onesideの計算を行うラベル番号のリスト

        戻り値:
            (tuple):
                - edges_oneside (numpy.ndarray): 検出された輪郭の3D座標。
                - length_edge (list): 各輪郭の長さ。

        """

        # ラベルインデックスを画素数でソート
        # label_indices = bbox[:, 4].argsort()[::-1]
        # chosen_label = np.unique(labeled_imgs[labeled_imgs > 0])

        edges_oneside = []
        length_edge = []

        # ラベル番号0は、黒い画像部分になるので、除外
        _chosen_label = filter(lambda elem: elem != 0, chosen_label)

        if self.scan_direction == ScanDirType.PLUS_X:
            # 各ラベルのエッジ座標を検出
            # x_range = abs(self.fwd_range[1] - self.fwd_range[0])
            x_range = abs(self.side_length)
            x_max = int(x_range / self.grid_size[0])

            # for idx in range(1, labels):  # ラベル0（背景）を除く各ラベルに対してループ
            for label_no in _chosen_label:
                # filter(lambda elem: elem != 0, chosen_label):  # ラベル0（背景）を除く各ラベルに対してループ
                # ラベルに一致するy座標のインデックスを取得
                y_coords = np.where(labeled_imgs == label_no)[0]
                unique_y_coords = np.unique(
                    y_coords,
                )  # 同じy座標は一度見れば良い（x方向のエッジだけみたいから）
                length_edge.append(
                    len(unique_y_coords),
                )  # ユニークなy座標の数をエッジの長さとして記録

                for y in unique_y_coords:
                    # x座標を走査し、ラベルに一致する最初のx座標を見つける
                    for x in range(x_max):
                        if labeled_imgs[y, x] == label_no:
                            # 対応するZ座標を計算
                            # z_val = im[y, x] + z_offset
                            z_val = z_offset
                            # エッジ座標をリストに追加（y, x, zの順）
                            edges_oneside.append((y, x, z_val))
                            break  # 最初の一致点を見つけたら、そのy座標でのループを終了

        elif self.scan_direction == ScanDirType.PLUS_Y:
            # 各ラベルのエッジ座標を検出
            y_range = abs(self.fwd_length)
            y_max = int(y_range / self.grid_size[1])

            # for idx in range(1, labels):  # ラベル0（背景）を除く各ラベルに対してループ
            for label_no in _chosen_label:
                # if bbox[label_indices[idx], 4] < label_size_thr:
                #     continue

                # ラベルに一致するx座標のインデックスを取得
                # x_coords = np.where(labeled_imgs == label_indices[idx])[1]
                x_coords = np.where(labeled_imgs == label_no)[1]
                unique_x_coords = np.unique(
                    x_coords,
                )  # 同じx座標は一度見れば良い（x方向のエッジだけみたいから）
                length_edge.append(
                    len(unique_x_coords),
                )  # ユニークなx座標の数をエッジの長さとして記録

                for x in unique_x_coords:
                    # x座標を走査し、ラベルに一致する最初のx座標を見つける
                    for y in range(y_max - 1, -1, -1):
                        if labeled_imgs[y, x] == label_no:
                            # 対応するZ座標を計算
                            # z_val = im[y-1, x] + z_offset
                            z_val = z_offset
                            # エッジ座標をリストに追加（y, x, zの順）
                            edges_oneside.append((y, x, z_val))
                            break  # 最初の一致点を見つけたら、そのy座標でのループを終了
        else:
            raise NotImplementedError("他の場合はまだ未実装")

        return np.array(edges_oneside), length_edge

    def create_detect_area(self, plane_depth: float = 0.1) -> o3d.geometry.TriangleMesh:
        """
        検出範囲た対応するTriangleMeshを生成する, EdgeDetectionIFからのオーバーライド
        TODO: ope3dを使わないようにしたいので、dgeDetectionIFから除く

        :param self: 説明
        :param plane_depth: 説明
        :type plane_depth: float
        :return: 説明
        :rtype: TriangleMesh

        """
        return (
            o3d.geometry.TriangleMesh.create_box(
                width=self.fwd_length,
                height=self.side_length,
                depth=plane_depth,
            )
            .paint_uniform_color(DETECT_MESH_COLOR)
            .translate(np.array([self.fwd_range[0], self.side_range[0], 0]))
        )

    def _pixel2lidar(
        self,
        pixel_val: NDPoint2fArray,
        z_val: NDPoint2fArray | float,
    ) -> tuple[NDSeries, NDSeries, NDSeries]:
        """与えられた画素部分に対応する点群上の座標を返す関数, 高さはz_valで与えられる"""

        # x_centerを使っていたが、変更していて、これで問題ないか要確認
        lidar_pos = np.zeros((len(pixel_val), 3), dtype=float)
        lidar_pos[:, 0] = pixel_val[:, 0] * self.grid_size[1] + self.fwd_range[0]
        lidar_pos[:, 1] = pixel_val[:, 1] * self.grid_size[0] + self.side_range[0]
        lidar_pos[:, 2] = z_val
        return lidar_pos[:, 0], lidar_pos[:, 1], lidar_pos[:, 2]

    def _lidar2pixel(
        self,
        lidar_points: NDPoint3fArray,
        grid_size: tuple[float, float],
        fwd_range: tuple[float, float],
        side_range: tuple[float, float],
        min_z: float = -1.88,
        max_z: float = -0.88,
        is_scaled: bool = True,
    ) -> NDImage:
        """LiDAR点群を解像度に合わせて、BEVに変換する
        lidar_pointsは絞った点群として、与えられたlidar_pointsをBEVに変換するだけの関数
        インスタンス変数を使ったりしないので、クラスメソッドにする
        """

        side_length = side_range[1] - side_range[0]
        fwd_length = fwd_range[1] - fwd_range[0]
        x_lidar = lidar_points[:, 0]
        y_lidar = lidar_points[:, 1]
        z_lidar = lidar_points[:, 2]

        # LiDAR 座標をグリッドマップ座標に変換
        x_img = np.floor((y_lidar - side_range[0]) / grid_size[0]).astype(np.int32)
        y_img = np.floor((x_lidar - fwd_range[0]) / grid_size[1]).astype(np.int32)

        # グリッドマップの初期化
        x_max = int(np.ceil(side_length / grid_size[0]))
        y_max = int(np.ceil(fwd_length / grid_size[1]))

        grid_map = np.zeros([y_max, x_max], dtype=np.uint8)
        if is_scaled:
            grid_map[y_img, x_img] = self.scale_to_255(
                z_lidar, min_value=min_z, max_value=max_z
            )
        else:
            grid_map[y_img, x_img] = z_lidar
        return grid_map

    def _convert_2DBoader_to_3D(
        self, edges_oneside: tuple[NDSeries, NDSeries, NDSeries], length_edge: list[int]
    ) -> tuple[
        o3d.geometry.LineSet,
        o3d.utility.Vector3dVector,
        o3d.utility.Vector2iVector,
    ]:
        """
        2Dの境界線座標から3DのLineSetオブジェクトを生成する。

        この関数は、BEV画像から抽出された2Dの境界線座標を3D空間にマッピングし、それらの座標を用いてLineSetオブジェクトを生成する。各境界線は、3D空間内の線分として表現される。

        パラメータ:
            edges_oneside (numpy.ndarray): 2Dの境界線座標を含む配列。
            length_edge (list of int): 各境界線の長さを表すリスト。
            side_range (tuple of float): BEV画像のX軸に対する範囲 (min_x, max_x)。
            fwd_range (tuple of float): BEV画像のY軸に対する範囲 (min_y, max_y)。
            resolution (float): BEV画像の解像度。

        戻り値:
            o3d.geometry.LineSet: 生成された3D LineSetオブジェクト。

        処理の流れ:
            1. 2D座標を3D座標に変換。
            2. 変換された座標を用いて複数の線分を生成。
            3. これらの線分を結合してLineSetオブジェクトを作成。
        """
        # if len(edges_oneside) == 0:
        #    # 崖が存在しない場合は空のLineSetを返す
        #    return o3d.geometry.LineSet()

        multi_points: list[Point3f] = []
        multi_lines: list[Point2i] = []
        pc_len = 0

        for _, edge_len in enumerate(length_edge):
            # 各エッジの3D座標
            x_pc = edges_x[pc_len : pc_len + edge_len]
            y_pc = edges_y[pc_len : pc_len + edge_len]
            z_pc = edges_z[pc_len : pc_len + edge_len]

            # 3D座標と線の作成
            obj_len = len(multi_points)
            for i in range(edge_len):
                points = [
                    [x_pc[i], y_pc[i], z_pc[i]],
                    [x_pc[i], y_pc[i], z_pc[i] + self.height_strips],
                ]
                multi_points.extend(points)

                if i < edge_len - 1:
                    lines = [
                        [obj_len + 2 * i, obj_len + 1 + 2 * i],
                        [obj_len + 2 * i, obj_len + 2 + 2 * i],
                        [obj_len + 1 + 2 * i, obj_len + 3 + 2 * i],
                        [obj_len + 2 + 2 * i, obj_len + 3 + 2 * i],
                    ]
                    multi_lines.extend(lines)

            pc_len += edge_len

        # 3D LineSetの作成
        line_set_bb = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(multi_points),
            lines=o3d.utility.Vector2iVector(multi_lines),
        )

        point_cloud = o3d.utility.Vector3dVector(multi_points)
        lines = o3d.utility.Vector2iVector(multi_lines)

        return line_set_bb, point_cloud, lines

    def update(self, edgedetection: app_config_module.EdgeDetectionConf) -> None:
        """
        設定値を更新時に行う処理

        :param self: 説明
        :param edgedetection: 説明
        :type edgedetection: app_config_module.EdgeDetectionConf
        """
        self.grid_size = edgedetection.grid_size
        self.height_strips = edgedetection.height_strip
        self.edge_width = (
            int(edgedetection.edge_width / edgedetection.grid_size[1]),
            int(edgedetection.edge_width / edgedetection.grid_size[0]),
        )
        self.debug = edgedetection.debug
        self.border_det = BorderDetector(
            self.scan_direction,
            self.side_range,
            self.fwd_range,
            self.side_length,
            self.fwd_length,
            self.grid_size,
            self.height_strips,
        )

    def get_edge_size(self) -> int:
        """自分自身の走査方向に対応したedge_sizeを返す, いくつかの箇所で呼びたいので、メソッドにした"""
        if self.scan_direction in (ScanDirType.PLUS_X, ScanDirType.MINUS_X):
            return self.edge_size[0]
        if self.scan_direction in (ScanDirType.PLUS_Y, ScanDirType.MINUS_Y):
            return self.edge_size[1]
        msg = "他の場合はまだ未実装"
        raise NotImplementedError(msg)

    def main(
        self,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        edge_conf: app_config_module.EdgeDetectionConf,
        general_conf: app_config_module.GeneralConf,
    ) -> EdgeDetectionResult | EdgeDetError:
        """
        崖検出のメイン処理

        :param self: 説明
        :param octree_obj: 説明
        :type octree_obj: OctoTree
        :param ground_entities: 説明
        :type ground_entities: list[NodeEntity]
        :param edge_conf: 説明
        :type edge_conf: app_config_module.EdgeDetectionConf
        :param general_conf: 説明
        :type general_conf: app_config_module.GeneralConf
        :return: 説明
        :rtype: EdgeDetectionResult
        """
        # 対応する点群を八分木から取り出す
        points = octree_obj.get_np_from_entity_octonodes_by_chunk(ground_entities)
        min_z = edge_conf.bev_min_z
        max_z = edge_conf.bev_max_z
        z_offset = edge_conf.edge_z_offset
        if points.shape[0] == 0:
            edge_points, edge_lines, edge_length = get_empty_points_lines_length()

            return EdgeDetectionResult(
                0,
                0,
                edge_points=edge_points,
                edge_lines=edge_lines,
                edge_length=edge_length,
            )

        if self.voxel_size:
            _points = np_voxel_downsample(points.copy(), self.voxel_size)
        else:
            _points = points.copy()

        bev_img = self.convert_points_to_bev_image(_points, min_z, max_z)

        # エッジ検出の前処理
        bev_img = self.preproc_edge(bev_img)

        # エッジ検出
        edge_img = self.extract_edge(bev_img)
        # edge_img = self.bev2edge(bev_img)

        # オクルージョン対策
        proc_edge_img = self.apply_occlusion_mask(edge_img)

        # 二値化
        bin_img = self.edge2bin(proc_edge_img)

        labeled_imgs, _, chosen_label = self.img_labeling(bin_img, self.get_edge_size())

        # 軸に沿ったエッジ検出
        edges_oneside, length_edge = self.border_det.border_detection(
            labeled_imgs, z_offset, chosen_label
        )

        if len(length_edge) == 0 or len(edges_oneside) == 0:
            edge_points, edge_lines, edge_length = get_empty_points_lines_length()

            return EdgeDetectionResult(
                0,
                0,
                edge_points=edge_points,
                edge_lines=edge_lines,
                edge_length=edge_length,
            )
            # box_points = empty_line.points
            # box_lines = empty_line.lines

        # Open3D Viewer上で崖位置を表示するためのLineSetを作成
        lidar_edges = self.border_det.pixel2lidar(
            edges_oneside[:, :2], -1 * edges_oneside[:, 2]
        )
        edge_points, edge_lines, edge_length = self.border_det.convert_2DBoader_to_3D(
            lidar_edges,
            length_edge,
        )

        if self.debug:
            # o3d_lineset = from_points_to_lineset(edge_points, edge_lines)
            o3d_lineset = o3d.geometry.LineSet(
                o3d.utility.Vector3dVector(edge_points),
                o3d.utility.Vector2iVector(edge_lines),
            )
            pcd = np_to_pcd(points)
            coord_arrow = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=2,
                origin=[0, 0, 0],
            )
            o3d.visualization.draw_geometries(
                geometry_list=[
                    pcd,
                    o3d_lineset,
                    coord_arrow,
                    self.create_detect_area(),
                ]
            )

        return EdgeDetectionResult(
            0,
            0,
            edge_points=edge_points,
            edge_lines=edge_lines,
            edge_length=edge_length,
        )


class MultiEdgeDetection(EdgeDetectionIF):
    range_properties: list[
        RangePropertyBase
    ]  # 検出範囲を生成するクラスを左右後方で持っている
    side_range: RangeF  # 全体のside_range
    fwd_range: RangeF  # 全体のfwd_range
    side_length: float  # 全体のy軸の長さ
    fwd_length: float  # 全体のx軸の長さ
    voxel_size: (
        float | None
    )  # ダウンサンプルのサイズ, Noneの場合点群をダウンサンプルしない
    grid_size: RangeF  # 鳥観図の格子実座標におけるサイズ
    edge_detectors: list[EdgeDetection]  # 左右後方の崖検出インスタンスのリスト
    border_dets: list[
        BorderDetector
    ]  # 各edge検出に対して、それに対応するborder検出がある
    debug: bool  # デバッグ有無

    def __init__(
        self,
        range_properties: list[RangePropertyBase],
        grid_size: RangeF,
        voxel_size: float | None = None,
        height_strips: float = 5,
        edge_width: float = 1,
        scaled_bev: bool = True,
        remove_duplicate_label: bool = True,
        debug: bool = True,
    ) -> None:
        """
        複数の範囲に対して崖検出を行うクラス
        複数の崖検出クラスを作って、それらにmain関数で必要な制御を行う

        :param self: 説明
        :param range_properties: 説明
        :type range_properties: list[RangePropertyBase]
        :param grid_size: 説明
        :type grid_size: RangeF
        :param voxel_size: 説明
        :type voxel_size: float | None
        :param height_strips: 検出した崖のz方向のオフセット
        :type height_strips: float
        :param edge_width: 説明
        :type edge_width: float
        :param scaled_bev: 説明
        :type scaled_bev: bool
        :param remove_duplicate_label: 後方の崖を除外するか
        :type remove_duplicate_label: bool
        :param debug: 説明
        :type debug: bool
        """

        self.range_properties = range_properties

        self.edge_detectors = [
            EdgeDetection(
                range_property=range_calculator,
                grid_size=grid_size,
                voxel_size=voxel_size,
                height_strips=height_strips,
                edge_width=edge_width,
                scaled_bev=scaled_bev,
                remove_duplicate_label=remove_duplicate_label,
            )
            for range_calculator in range_properties
        ]
        self.grid_size = grid_size
        self.voxel_size = voxel_size

        self.side_range = (
            min([edge_det.side_range[0] for edge_det in self.edge_detectors]),
            max([edge_det.side_range[1] for edge_det in self.edge_detectors]),
        )
        self.side_length = self.side_range[1] - self.side_range[0]
        self.fwd_range = (
            min([edge_det.fwd_range[0] for edge_det in self.edge_detectors]),
            max([edge_det.fwd_range[1] for edge_det in self.edge_detectors]),
        )
        self.fwd_length = self.fwd_range[1] - self.fwd_range[0]

        self.border_dets = [
            BorderDetector(
                scan_direction=edge_det.scan_direction,
                side_range=self.side_range,
                fwd_range=self.fwd_range,
                side_length=self.side_length,
                fwd_length=self.fwd_length,
                grid_size=grid_size,
                height_strips=height_strips,
            )
            for edge_det in self.edge_detectors
        ]
        self.debug = debug

    def main(
        self,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        # points: t_np_float,
        edge_conf: app_config_module.EdgeDetectionConf,
        general_conf: app_config_module.GeneralConf,
    ) -> EdgeDetectionResult | EdgeDetError:
        """崖検出のメイン処理
        各検出範囲に対して、二値化を行いまとめて、全体のエッジを計算する
        """
        points = octree_obj.get_np_from_entity_octonodes_by_chunk(ground_entities)
        # 対象の点群がない場合は空の結果を返す
        if points.shape[0] == 0:
            edge_points, edge_lines, edge_length = get_empty_points_lines_length()
            return EdgeDetectionResult(
                0,
                0,
                edge_points=edge_points,
                edge_lines=edge_lines,
                edge_length=edge_length,
            )

        min_z = edge_conf.bev_min_z
        max_z = edge_conf.bev_max_z
        z_offset = edge_conf.edge_z_offset

        _points = points.copy()
        if self.voxel_size:
            _points = np_voxel_downsample(points, self.voxel_size)

        bin_imgs = []
        for edge_det in self.edge_detectors:
            # bev画像への変換
            bev_img = edge_det.convert_points_to_bev_image(_points, min_z, max_z)

            # エッジ検出の前処理
            bev_img = edge_det.preproc_edge(bev_img)

            # エッジ検出
            edge_img = edge_det.extract_edge(bev_img)

            # オクルージョン対策
            proc_edge_img = edge_det.apply_occlusion_mask(edge_img)

            # 二値化
            bin_img = edge_det.edge2bin(proc_edge_img)
            bin_imgs.append(bin_img)

        # 画像の統合
        integrated_bin_img, target_regions = unwrap(self.integrate_img(bin_imgs))

        # ラベリング
        labels, labeled_imgs, bbox, _ = cv2.connectedComponentsWithStats(
            integrated_bin_img,
            connectivity=8,
        )

        multi_lineset = o3d.geometry.LineSet()
        multi_edge_length = np.empty((0,), int)
        for edge_det, border_det, target_region in zip(
            self.edge_detectors,
            self.border_dets,
            target_regions,
            strict=False,
        ):
            # 各崖検出と関連があるラベリングを取り出す
            chosen_label = self.choose_search_label(
                target_labels=np.arange(labels),
                labeled_img=labeled_imgs.copy(),
                bbox=bbox,
                edge_det=edge_det,
                target_region=target_region,
            )

            # 対象となるラベルに対してborder検出を行う
            edge_oneside, length_edge = border_det.border_detection(
                labeled_imgs,
                z_offset,
                chosen_label,
            )
            if len(edge_oneside) == 0:
                # 崖がない場合の処理. 後続処理を行わなければ良い
                one_lineset = o3d.geometry.LineSet()
            else:
                # 崖検出のLineSetオブジェクトを作る
                lidar_edges = border_det.pixel2lidar(
                    edge_oneside[:, :2],
                    -1 * edge_oneside[:, 2],
                )
                edge_points, edge_lines, edge_length = (
                    border_det.convert_2DBoader_to_3D(lidar_edges, length_edge)
                )
                one_lineset = o3d.geometry.LineSet(
                    o3d.utility.Vector3dVector(edge_points),
                    o3d.utility.Vector2iVector(edge_lines),
                )
                multi_edge_length = np.append(multi_edge_length, edge_length)

                # from_points_to_lineset(edge_points, edge_lines)

            if self.debug:
                pcd = np_to_pcd(points)
                coord_arrow = o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=2,
                    origin=[0, 0, 0],
                )
                o3d.visualization.draw_geometries(
                    geometry_list=[
                        one_lineset,
                        pcd,
                        coord_arrow,
                        self.create_detect_area(),
                    ],
                )
            multi_lineset += one_lineset

        edge_result = EdgeDetectionResult(
            0,
            0,
            np.array(multi_lineset.points),
            np.array(multi_lineset.lines),
            multi_edge_length,
        )
        return edge_result

    def integrate_img(
        self,
        target_imgs: list[cv2.typing.MatLike],
    ) -> Result[tuple[NDImage, list[XYMinMax]], EdgeDetLogicError]:
        """複数の範囲で得られた画像データを統合して一つにまとめる
        各検出範囲を包括する画像データを用意して、それぞれの位置する画像を埋め込む
        """

        if len(target_imgs) != len(self.edge_detectors):
            return Err(
                EdgeDetLogicError(
                    "target_imgsとself.edge_detectorsは同じ長さである必要があります, "
                    f"len(target_imgs) = {len(target_imgs)}, len(self.edge_detectors) == {len(self.edge_detectors)}"
                )
            )

        total_grid_size = self.edge_detectors[0].grid_size
        # 全体のside_range, fwd_rangeを計算する
        total_side_range = self.side_range

        total_fwd_range = self.fwd_range

        # 統合画像の大きさを計算
        x_max = int(
            np.ceil((total_side_range[1] - total_side_range[0]) / total_grid_size[0]),
        )
        y_max = int(
            np.ceil((total_fwd_range[1] - total_fwd_range[0]) / total_grid_size[1]),
        )

        # 統合画像に各画像を埋め込む
        integrated_img: NDImage = np.zeros([y_max, x_max], dtype=np.uint8)
        target_regions: list[XYMinMax] = []
        for edge_det, target_img in zip(self.edge_detectors, target_imgs, strict=False):
            x_offset = int(
                (edge_det.side_range[0] - total_side_range[0]) / total_grid_size[0],
            )
            y_offset = int(
                (edge_det.fwd_range[0] - total_fwd_range[0]) / total_grid_size[1],
            )
            y_len, x_len = target_img.shape

            # 画像の(x,y)を入れているので、順番がy, xの順で入れている
            target_region = XYMinMax(
                y_offset,
                y_offset + y_len,
                x_offset,
                x_offset + x_len,
            )

            target_regions.append(target_region)
            integrated_img[
                target_region.min_x : target_region.max_x,
                target_region.min_y : target_region.max_y,
            ] = target_img

        return Ok((integrated_img, target_regions))

    def choose_search_label(
        self,
        target_labels: NDArray[np.int32],
        labeled_img: cv2.typing.MatLike,
        bbox: cv2.typing.MatLike,
        edge_det: EdgeDetection,
        target_region: XYMinMax,
        do_copy: bool = True,
    ) -> list[int]:
        """走査の対象としたいラベルを選ぶ
        target_labels: 対象となる全てのラベル
        引数のlabeled_imgを書き換える処理を行っているため、do_copyという引数を用意している
        """
        if do_copy:
            labeled_img = labeled_img.copy()
        (min_x, max_x, min_y, max_y) = target_region

        # 見たい走査方向の粒度に応じて、小さすぎるグループは除去
        _edge_size = edge_det.get_edge_size()
        labeled_img[np.isin(labeled_img, np.where(bbox[:, -1] <= _edge_size)[0])] = 0

        # 全体の中で、対応する崖検出の範囲に存在していないラベリングは除去
        remove_inds = np.setdiff1d(
            target_labels,
            np.unique(labeled_img[min_x : (max_x + 1), min_y : (max_y + 1)]),
        )
        labeled_img[np.isin(labeled_img, remove_inds)] = 0

        # border検出の対象となるラベルを決める
        if edge_det.remove_duplicate_label:
            chosen_label = edge_det.choose_edges(labeled_img)
        else:
            chosen_label = np.unique(labeled_img).tolist()

        return chosen_label

    def create_detect_area(self, plane_depth: float = 0.1) -> o3d.geometry.TriangleMesh:
        """崖検出の検出範囲を表す直方体を作成する"""
        box = o3d.geometry.TriangleMesh()
        for edge_det in self.edge_detectors:
            box += edge_det.create_detect_area(plane_depth=plane_depth)
        return box

    def convert_img_to_mesh(
        self,
        img: t_np_uint,
        z_offset: float = 0,
        depth: float = 0.1,
    ) -> o3d.geometry.TriangleMesh:
        """img画像を高さz_offsetの位置に厚さdepthでTriangleMeshとして配置する関数"""
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_rgb = img_rgb.astype(np.float32) / 255.0
        x_num, y_num = img.shape

        total_mesh = o3d.geometry.TriangleMesh()
        for x_ind, y_ind in product(range(x_num), range(y_num)):
            x_pos = x_ind * self.grid_size[0] + self.fwd_range[0]
            y_pos = y_ind * self.grid_size[1] + self.side_range[0]
            color = img_rgb[x_ind, y_ind]

            _mesh = (
                o3d.geometry.TriangleMesh.create_box(
                    width=self.grid_size[0],
                    height=self.grid_size[1],
                    depth=depth,
                )
                .translate([x_pos, y_pos, z_offset])
                .paint_uniform_color(color)
            )
            total_mesh += _mesh
        return total_mesh

    def update(self, edgedetection: app_config_module.EdgeDetectionConf) -> None:
        """
        設定値が変更された場合に呼び出される関数

        :param self: 説明
        :param edgedetection: 説明
        :type edgedetection: app_config_module.EdgeDetectionConf
        """
        self.grid_size = edgedetection.grid_size
        self.height_strips = edgedetection.height_strip
        self.edge_width = (
            int(edgedetection.edge_width / edgedetection.grid_size[1]),
            int(edgedetection.edge_width / edgedetection.grid_size[0]),
        )
        self.debug = edgedetection.debug


class EdgeDetectionPolar(EdgeDetectionIF):
    # 検出範囲に関するクラス
    range_property: RangePropertyBase

    # デカルト座標の検出範囲
    fwd_range_cartesian: RangeF
    side_range_cartesian: RangeF

    # 極座標の格子サイズ
    grid_size_polar: Point2f

    # デカルト座標の格子サイズ
    grid_size_cartesian: Point2f

    # 極座標の格子の数, デカルト座標の時と同じように範囲指定しづらいので、格子の数で表現
    grid_shape_polar: Point2i

    # 極座標の中心
    origin: Point3f

    # 格子極座標と実極座標のオフセット, 色々変わりそうなのでコンストラクタで与えるものにする
    grid_offset: Point2i

    # lidar中心, オクルージョン除去で用いる
    lidar_centers: list[Point3f]

    # 鳥観図における地面の画素値
    pixel_ground_height: int

    # エッジフィルタの設定
    edge_filter_type: EdgeFilterType

    # 2値化の閾値
    bin_th: int

    # オクルージョン判定の検出窓のサイズ
    occ_focus_range: int

    # エッジ画像のフィルタサイズ
    edge_filter_size: int

    # 範囲計算をするクラス
    border_extractor: BorderExtractor

    # 崖検出を行う方向
    scan_direction: ScanDirType

    # 短冊の高さ
    height_strips: float

    # 全ての計算対象の角度における最大角度
    max_grid_radiuses: NDSeries

    # radius方向の検出範囲に載せるオフセット
    search_radius_offset: int

    # 後方の崖を除外するか
    remove_duplicate_label: bool

    # 最終的に崖検知のとして使われる崖の長さの最小値
    target_edge_dist_th: float

    # デバッグ有無
    debug: bool

    # 極座標変換時のオフセット, 定数扱いで良さそうなので、定数にしている
    REAL_OFFSET: XYTup = (0.0, -np.pi)

    def __init__(
        self,
        range_property: RangePropertyBase,
        grid_size_polar: XYTup,
        grid_size_cartesian: XYTup,
        grid_shape_polar: PxPyTup,
        origin: XYZTup,
        grid_offset: PxPyTup,
        lidar_centers: list[XYZTup],
        height_strips: float,
        pixel_ground_height: int = 120,
        bin_th: int = 110,
        occ_focus_range: int = 30,
        edge_filter_size: int = 3,
        search_radius_offset: int = 8,
        target_edge_dist_th: float = 2.0,
        remove_duplicate_label: bool = True,
        edge_size: tuple[float, float] = (50.0, 50.0),
        max_grid_radius_offset: int = 1,
        debug: bool = False,
    ) -> None:
        """
        極座標形式の崖検出を行うクラス
        (radius, theta)の想定でデータを持っている

        :param self: 説明
        :param range_property: 検出範囲を計算したりするインスタンス
        :type range_property: RangePropertyBase
        :param grid_size_polar: 極座標における鳥瞰図の格子1つの実座標における大きさ
        Remark: デカルト座標で鳥瞰図を作るのと違い、(radius, theta)で鳥瞰図を作ると、thetaの位置に応じて格子一つの大きさは違うはずだが、大きさを一定にしているため、上手くいかない部分が出る可能性はある
        :type grid_size_polar: t_xy
        :param grid_size_cartesian: デカルト座標における鳥瞰図の格子1つの実座標における大きさ
        :type grid_size_cartesian: t_xy
        :param grid_shape_polar: 極座標における鳥観図の画像サイズ(radius, theta)
        :type grid_shape_polar: t_pxpy
        :param origin: 極座標の実座標における原点, x,y座標
        :type origin: t_xyz
        :param grid_offset: 鳥観図に移した後, 格子上で発生するオフセット
        :type grid_offset: t_pxpy
        :param lidar_centers: 実座標において各LiDARが付いているxyzの位置
        :type lidar_centers: list[t_xyz]
        :param height_strips: 検出した崖のz方向のオフセット
        :type height_strips: float
        :param pixel_ground_height: 鳥観図における地面の画素値
        :type pixel_ground_height: int
        :param bin_th: 2値化の閾値
        :type bin_th: int
        :param occ_focus_range: オクルージョン判定の検出窓のサイズ
        :type occ_focus_range: int
        :param edge_filter_size: エッジ画像計算のフィルタサイズ
        :type occ_focus_range: int
        :param search_radius_offset: radius方向の検出範囲に載せるオフセット
        :type search_radius_offset: int
        :param target_edge_dist_th: 最終的に崖検知のとして使われる崖の長さの最小値
        :type target_edge_dist_th: float
        :param remove_duplicate_label: 後方の崖を除外するか
        :type remove_duplicate_label: bool
        """

        self.range_property = range_property
        self.grid_size_polar = grid_size_polar
        self.grid_size_cartesian = grid_size_cartesian
        self.grid_shape_polar = grid_shape_polar
        self.origin = origin
        self.grid_offset = grid_offset
        self.lidar_centers = lidar_centers
        self.height_strips = height_strips
        self.pixel_ground_height = pixel_ground_height  # TODO: 後で逆算できるようにする
        self.bin_th = bin_th
        self.occ_focus_range = occ_focus_range
        self.edge_filter_size = edge_filter_size
        self.search_radius_offset = search_radius_offset
        self.target_edge_dist_th = target_edge_dist_th
        self.remove_duplicate_label = remove_duplicate_label
        self.debug = debug

        # put fields derived from range_property
        self.fwd_range_cartesian = range_property.calculate_fwd_range()
        self.side_range_cartesian = range_property.calculate_side_range()
        self.scan_direction = range_property.scan_direction
        self.edge_filter_type = range_property.edge_filter_type

        self.border_extractor = BorderExtractor(
            scan_direction=self.scan_direction,
            side_range=self.side_range_cartesian,
            fwd_range=self.fwd_range_cartesian,
            grid_size=self.grid_size_cartesian,
            edge_size=edge_size,
            height_strips=self.height_strips,
            target_edge_dist_th=self.target_edge_dist_th,
        )

        # 鳥観図の各thetaに対する最大動径方向を計算する
        # コンストラクタでResult型を返すと警告が出たので、unwrapすることにする
        self.max_grid_radiuses = unwrap(
            calc_max_radius_each_theta(
                origin=self.origin,
                x_range=self.fwd_range_cartesian,
                y_range=self.side_range_cartesian,
                grid_size_polar=self.grid_size_polar,
                real_offset=EdgeDetectionPolar.REAL_OFFSET,
                grid_offset=self.grid_offset,
            )
        )

        ## max_grid_radiusesを微妙に小さくしないと、オクルージョン処理で
        ## 検出範囲のエッジに反応してしまうので、入れているが
        ## search_radius_offsetで制御できる気もするので、処理を確認して修正したほうが良いかもしれない
        self.max_grid_radiuses -= max_grid_radius_offset

    def __str__(self) -> str:
        """EdgeDetecionの状態を表す設定値を文字列で固めて返す"""
        return f""" grid_size_polar = {self.grid_size_polar},
    grid_size_cartesian = {self.grid_size_cartesian},
    grid_shape_polar = {self.grid_shape_polar},
    origin = {self.origin},
    grid_offset = {self.grid_offset},
    lidar_centers = {self.lidar_centers},
    range_calculator = {self.range_property},
    side_range_cartesian = {self.side_range_cartesian},
    fwd_range_cartesian = {self.fwd_range_cartesian},
    height_strips = {self.height_strips},
    edge_filter = {self.edge_filter_type},
    scan_direction = {self.scan_direction},
    remove_duplicate_label = {self.remove_duplicate_label},
    debug = {self.debug},
    """

    def _create_polar_bev(
        self,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        agg_name: AggName = AggName.MAX,
        front_grid_radius: int = 10,
        front_kernel_size: PxPyTup = (3, 16),
        rear_kernel_size: PxPyTup = (7, 8),
        min_scale_z: float = -1.88,
        max_scale_z: float = -0.88,
        # max_scale_z: float = -1.08,
        min_bev_val: int = 0,
        max_bev_val: int = 255,
    ) -> Result[cv2.typing.MatLike, EdgeDetError]:
        """
        必要な前処理を行って、極座標の鳥瞰図を作成する

        :param self: 説明
        :param octree_obj: 八分木インスタンス
        :type octree_obj: OctoTree
        :param ground_entities: 八分木インスタンスのentity_octonodesの中で鳥瞰図に使うNodeEntityのリスト
        :type ground_entities: list[NodeEntity]
        :param agg_name: 鳥観図のz値の集約方法
        :type agg_name: AggName
        :param front_grid_radius: 鳥観図のradiusが小さい場合と大きい場合でモルフォロジー変換の方法を変えていて、それを鳥瞰図のどの格子にするかを決める閾値
        :type front_grid_radius: int
        :param front_kernel_size: 鳥観図のradiusが小さい側に適用するモルフォロジー変換のkernel size
        :type front_kernel_size: t_pxpy
        :param rear_kernel_size: 鳥瞰図のradiusが大きい側に適用するモルフォロジー変換のkernel size
        :type rear_kernel_size: t_pxpy
        :param min_scale_z: 地面の最小z値, これ以下のz値はmin_scale_zにclipされる
        :type min_scale_z: float
        :param max_scale_z: 地面の最大z値, これ以上のz値はmax_scale_zにclipされる
        :type max_scale_z: float
        :param min_bev_val: 鳥観図の最小値, これ以下の鳥瞰図の画素値はmin_bev_valにclipされる
        :type min_bev_val: int
        :param max_bev_val: 鳥観図の最大値, これ以上鳥観図の画素値はmax_bev_valにclipされる
        :type max_bev_val: int
        :return: 途中処理でエラーがなければ、鳥瞰図と同じ大きさの画像データ, 途中の処理で失敗した場合はEdgeDetErrorをResult型にして返す
        :rtype: Result[MatLike, EdgeDetError]
        """
        # 鳥観図を作成する
        polar_bev = py_octotree2bev(
            octree_obj=octree_obj,
            fwd_range=self.fwd_range_cartesian,
            side_range=self.side_range_cartesian,
            grid_size=self.grid_size_polar,
            bev_shape=self.grid_shape_polar,
            target_entities=ground_entities,
            bev_coord=BevCoord.POLAR,
            agg_name=agg_name,
            coord_origin=self.origin,
            discrete_origin=EdgeDetectionPolar.REAL_OFFSET,
            min_scale_z=min_scale_z,
            max_scale_z=max_scale_z,
            min_bev_val=min_bev_val,
            max_bev_val=max_bev_val,
        )

        # 鳥観図の加工処理
        front_radius = self.grid_size_polar[0] * front_grid_radius
        polar_bev = map_(
            polar_bev,
            lambda ok_val: postproc_for_polar(
                ok_val,
                self.grid_offset[0],
                front_radius,
                polar_grid=self.grid_size_polar,
                front_kernel_size=front_kernel_size,
                rear_kernel_size=rear_kernel_size,
            ),
        )

        # エッジ計算の手前の画像処理部分だが、この処理の後でオクルージョン処理に使う画像の大きい部分, 小さい部分を分けているので、preproc_edgeという名前で扱うのは問題があるかもしれない
        polar_bev = map_(
            polar_bev,
            lambda ok_val: preproc_edge(ok_val, 2),
        )
        # 最後にモルフォロジー変換を行って、その結果を返す
        return map_(polar_bev, lambda ok_val: apply_morphology_close(ok_val, (5, 5)))

    def _check_occlusion(
        self,
        occ_target_img: cv2.typing.MatLike,
        focus_range: int = 30,
        n_happen: int = 1,
    ) -> tuple[NDPoint2i, NDPoint2i]:
        """
        オクルージョン箇所を特定する

        :param self: 説明
        :param occ_target_img: オクルージョン判定を行う画像
        :type occ_target_img: cv2.typing.MatLike
        :param focus_range: オクルージョン判定を行うwindowサイズ
        :type focus_range: int
        :param n_happen: 何個オクルージョン判定されるとオクルージョン扱いされるか
        :type n_happen: int
        :return: 地面に該当する部分とオクルージョンに該当する部分の画素上のx,y座標のtuple
        :rtype: tuple[NDPoint2i, NDPoint2i]
        """
        real_offset_minus = (
            -EdgeDetectionPolar.REAL_OFFSET[0],
            -EdgeDetectionPolar.REAL_OFFSET[1],
        )
        origin_xy = (self.origin[0], self.origin[1])

        # lidar中心毎にオクルージョン判定を行う
        occ_grid_lidars: list[list[NDPoint2i]] = []
        for lidar_center in self.lidar_centers:
            occ_grid_lidars.append(
                check_occlusion_from_other_origin(
                    masked_img=occ_target_img,
                    max_radius=self.max_grid_radiuses,
                    other_origin=(lidar_center[0], lidar_center[1]),
                    origin=origin_xy,
                    grid_size=self.grid_size_polar,
                    real_offset=real_offset_minus,
                    grid_offset=self.grid_offset,
                    focus_range=focus_range,
                    n_happen=n_happen,
                )
            )

        # LiDAR中心毎に格子座標ができているので、その部分をflattenする
        _occ_grid_lidars = itertools.chain.from_iterable(occ_grid_lidars)

        # オクルージョン境界とそれ以外を分ける
        pixel_ground: list[NDPoint2i] = []
        pixel_occ: list[NDPoint2i] = []

        for occ_area in _occ_grid_lidars:
            if len(occ_area) > 1:
                pixel_ground.append(occ_area[:-1])
                pixel_occ.append(occ_area[-1])
            else:
                # 要素が一つだけの場合はそれをオクルージョン境界とする
                pixel_occ.append(occ_area[0])

        if len(pixel_ground) == 0:
            _pixel_ground = np.empty((0, 2), dtype=int)
        else:
            _pixel_ground = np.vstack(pixel_ground)

        if len(pixel_occ) == 0:
            _pixel_occ = np.empty((0, 2), dtype=int)
        else:
            _pixel_occ = np.vstack(pixel_occ)

        return _pixel_ground, _pixel_occ

    def _postproc_occ(
        self,
        img_shape: Point2i,
        occ_grid: NDPoint2i,
        morphology_kernel_size: Point2i = (2, 5),
        dilate_kernel_size: Point2i = (5, 5),
    ) -> cv2.typing.MatLike:
        """
        画像上のオクルージョン箇所をモルフォロジー変換で広げて、オクルージョン検出の離散化誤差を吸収する
        Remark: オクルージョン検出は各LiDAR中心における極座標を離散化した空間の上で行われて、それを復元した結果を使うので、離散化誤差が発生している

        :param self: 説明
        :param img_shape: 説明
        :type img_shape: Point2i
        :param occ_grid: 説明
        :type occ_grid: NDPoint2i
        :param morphology_kernel_size: 説明
        :type morphology_kernel_size: Point2i
        :param dilate_kernel_size: 説明
        :type dilate_kernel_size: Point2i
        :return: 説明
        :rtype: MatLike
        """
        occ_mat = np.zeros(img_shape, dtype=np.uint8)
        if len(occ_grid) == 0:
            return occ_mat

        # オクルージョン検出を1として、1部分を膨らませる
        occ_mat[occ_grid[:, 0], occ_grid[:, 1]] = 1
        occ_mat = apply_morphology_close(occ_mat, morphology_kernel_size)
        return cv2.dilate(occ_mat, np.ones(dilate_kernel_size, np.uint8), iterations=3)

    def _remove_detect_outside_edge(
        self,
        bin_img: cv2.typing.MatLike,
        is_copy: bool = True,
    ) -> cv2.typing.MatLike:
        """検出範囲ぎりぎりのエッジを削除する
        検出範囲境界のエッジを検知してしまうので、bin_imgの時点で除去しようとしている

        :param self: 説明
        :param bin_img: 二値画像
        :type bin_img: MatLike
        :param is_copy: 画像をコピーして除去する場合はTrueにする
        :type is_copy: bool
        :return: 除去後の二値画像
        :rtype: MatLike
        """
        _img = bin_img.copy() if is_copy else bin_img
        outside_detect = self.max_grid_radiuses - self.search_radius_offset
        rows = np.arange(_img.shape[0])[:, None]
        thr = outside_detect[None, :]
        mask = rows >= thr
        _img[mask] = 0
        return _img

    def _create_cliff_points(
        self,
        edge_oneside: NDPoint2i,
        length_edge: list[int],
        repr_method: DediscretizeMethod = DediscretizeMethod.MED,
    ) -> tuple[NDPoint3fArray, NDPoint2iArray, NDSeries]:
        """
        崖検出で得られた内容を加工して、EdgeDetectionResultで使う点群・線・クラスタ長を作る
        """
        if len(edge_oneside) == 0:
            edge_points, edge_lines, edge_length = get_empty_points_lines_length()
            return edge_points, edge_lines, edge_length

        real_offset_minus = (
            -EdgeDetectionPolar.REAL_OFFSET[0],
            -EdgeDetectionPolar.REAL_OFFSET[1],
        )
        origin_xy = (self.origin[0], self.origin[1])

        edge_lidar_coords = polar_grid_to_lidar_coord(
            edge_oneside[:, :2],
            grid_size=self.grid_size_polar,
            grid_offset=self.grid_offset,
            real_offset=real_offset_minus,
            origin=origin_xy,
            repr_method=repr_method,
        )

        # 各崖の実空間上の長さを見てEdgeDetectionResultに含めるか決める
        edge_lidar_coords, length_edge = self.border_extractor.filter_target_edge(
            edge_points=edge_lidar_coords, edge_length=length_edge
        )

        # 元のEdgeDetectionがzを-1*edge_oneside[:,2]にしていたのでそれに則った変数を用意したが、
        # 要らない気もする
        lidar_edges = (
            edge_lidar_coords[:, 0],
            edge_lidar_coords[:, 1],
            -edge_oneside[:, 2],
        )

        edge_points, edge_lines, edge_length = (
            self.border_extractor.convert_2dborder_to_3d(
                lidar_edges,
                length_edge,
            )
        )

        return (edge_points, edge_lines, edge_length)

    def main(
        self,
        octree_obj: OctoTree,
        ground_entities: list[NodeEntity],
        edge_conf: app_config_module.EdgeDetectionConf,
        general_conf: app_config_module.GeneralConf,
    ) -> EdgeDetectionResult | EdgeDetError:
        # 1. 極座標の鳥瞰図を作成する
        polar_bev = self._create_polar_bev(
            octree_obj=octree_obj,
            ground_entities=ground_entities,
        )
        # TODO: 他の処理でもEdgeDetErrorを返す関数が増えてきたら、and_thenやmap_を使ってエラーを伝播させる
        if isinstance(polar_bev, Err):
            return polar_bev.error
        polar_bev = polar_bev.value

        # 2. 鳥観図の高い部分, 低い部分, その間を分ける
        high_low_img = mask_img_by_value(polar_bev)

        # 3. エッジ検出を行う
        masked_polar_bev = bev_masking(
            high_low_img=high_low_img,
            polar_bev=polar_bev,
            completed_value=1,
            completing_value=0,
            is_copy=True,
        )
        polar_edge = extract_edge(
            img=masked_polar_bev,
            im_filter=self.edge_filter_type,
            ksize=self.edge_filter_size,
        )

        # 4. 二値化
        bin_img = thresh_based_edge2bin(polar_edge, self.bin_th, BinMethod.ABS)
        bin_img = cv2.dilate(bin_img, np.ones((3, 3), dtype=np.uint8))

        # 5. オクルージョン検出を行う
        pixel_ground, pixel_occ = self._check_occlusion(
            occ_target_img=high_low_img,
            focus_range=self.occ_focus_range,
        )
        occ_grid = np.vstack([pixel_ground, pixel_occ])
        occ_img = self._postproc_occ(
            (polar_bev.shape[0], polar_bev.shape[1]),
            occ_grid,
        )

        # 6. 二値画像で不要な箇所の削除を行う
        ## オクルージョン箇所をマスクする
        occ_row, occ_col = np.where(occ_img == 1)
        masked_bin_img = np.ma.array(bin_img)
        masked_bin_img[occ_row, occ_col] = np.ma.masked

        ## 動径方向の最大検出位置より向こう側で1が立っている部分は無視したいので、この段階で除去
        masked_bin_img = self._remove_detect_outside_edge(masked_bin_img)

        # 7. ラベリング
        labels, labeled_imgs, bbox, _ = cv2.connectedComponentsWithStats(
            masked_bin_img.filled(0),
            connectivity=4,
            # bin_img, connectivity=4
        )
        masked_labeled_imgs: np.ma.MaskedArray = np.ma.array(labeled_imgs)
        masked_labeled_imgs[occ_row, occ_col] = np.ma.masked

        # 8. 取り出すラベルが存在する位置を取り出す
        # 崖検知の結果として取り出すラベルを決める
        chosen_label = self.border_extractor.choose_search_label(
            target_labels=np.arange(labels),
            labeled_img=masked_labeled_imgs.filled(0),
            # labeled_img=labeled_imgs,
            bbox=bbox,
            target_region=XYMinMax(
                0, self.grid_shape_polar[0] - 1, 0, self.grid_shape_polar[1] - 1
            ),
            remove_duplicate_label=self.remove_duplicate_label,
        )
        edge_oneside, length_edge = self.border_extractor.border_detection(
            # labeled_imgs=labeled_imgs,
            labeled_imgs=masked_labeled_imgs,
            scan_direction=self.scan_direction,
            z_offset=edge_conf.edge_z_offset,
            chosen_label=chosen_label,
            search_range=(self.max_grid_radiuses - self.search_radius_offset).tolist(),
            is_exc_occ=True,
        )

        # 9. 結果を返すための後処理を行う
        edge_points, edge_lines, edge_length = self._create_cliff_points(
            edge_oneside=edge_oneside,
            length_edge=length_edge,
        )
        return EdgeDetectionResult(
            frame=0,
            time=0,
            edge_points=edge_points,
            edge_lines=edge_lines,
            edge_length=edge_length,
        )

    def update(self, edgedetection: app_config_module.EdgeDetectionConf) -> None:
        """
        変に更新すると、副作用がありそうなので、影響のなさそうなパラメータのみ更新するようにする
        TODO: create_edge_detection関数でを呼んで、崖検出クラスを再度作った方が筋は良さそう
        """
        self.debug = edgedetection.debug
        self.target_edge_dist_th = edgedetection.target_edge_dist_th

    def create_detect_area(self, plane_depth: float = 0.1) -> o3d.geometry.TriangleMesh:
        """崖検出の検出範囲を表す直方体を作成する"""
        fwd_range = self.fwd_range_cartesian
        fwd_length = fwd_range[1] - fwd_range[0]

        side_range = self.side_range_cartesian
        side_length = side_range[1] - side_range[0]
        return (
            o3d.geometry.TriangleMesh.create_box(
                width=fwd_length,
                height=side_length,
                depth=plane_depth,
            )
            .paint_uniform_color(DETECT_MESH_COLOR)
            .translate(np.array([fwd_range[0], side_range[0], 0]))
        )


def create_edge_detection(
    app_config: AppConfig,
    crawler_points: NDPoint3fArray,
) -> Result[EdgeDetectionIF, EdgeDetError]:
    """崖検出を行うインスタンスを作成する
    im_filtersとscan_directionsが3であれば、MultiEdgeDetectionのインスタンスを作って、そうでなければ、EdgeDetectionのインスタンスを作る
    """
    edge_conf = app_config.EdgeDetection
    crawler_min = crawler_points.min(axis=0)
    crawler_max = crawler_points.max(axis=0)

    match edge_conf.detector:
        case "EdgeDetectionPolar":
            range_property = RangeProperty["CIRCUM"].value(
                crawler_max=crawler_max,
                crawler_min=crawler_min,
                detect_range=edge_conf.detect_range,
                offsets=edge_conf.detect_offset,
            )

            origin = edge_conf.polar_origin
            n_radius, n_angle = edge_conf.polar_shape
            min_radius = edge_conf.polar_min_radius

            fwd_range_cartesian = range_property.calculate_fwd_range()
            side_range_cartesian = range_property.calculate_side_range()

            max_radius = calc_rect_max_dist(
                (fwd_range_cartesian[0], side_range_cartesian[0]),
                (fwd_range_cartesian[1], side_range_cartesian[1]),
                origin,
            )
            radius_range = (0, max_radius)
            angle_range = (-np.pi, np.pi)
            grid_size_polar = (
                (radius_range[1] - radius_range[0]) / n_radius,
                (angle_range[1] - angle_range[0]) / n_angle,
            )
            edge_size = (
                edge_conf.edge_width / grid_size_polar[0],
                edge_conf.edge_width / grid_size_polar[1],
            )
            min_radius_grid_offset = np.floor(min_radius / grid_size_polar[0]).astype(
                int
            )
            lidar_centers = [
                tuple(np.loadtxt(file, delimiter=",")[:3, 3])
                for file in app_config.calibration.Lidar_calib_files
            ]
            return Ok(
                EdgeDetectionPolar(
                    range_property=range_property,
                    grid_size_polar=grid_size_polar,
                    grid_size_cartesian=edge_conf.grid_size,
                    grid_shape_polar=(n_radius, n_angle),
                    origin=origin,
                    grid_offset=(min_radius_grid_offset, 0),
                    lidar_centers=lidar_centers,
                    height_strips=edge_conf.height_strip,
                    bin_th=edge_conf.bin_th,
                    edge_filter_size=edge_conf.edge_filter_size,
                    occ_focus_range=edge_conf.occ_focus_range,
                    search_radius_offset=edge_conf.search_radius_offset,
                    target_edge_dist_th=edge_conf.target_edge_dist_th,
                    remove_duplicate_label=edge_conf.remove_duplicate_label,
                    edge_size=edge_size,
                    debug=edge_conf.debug,
                )
            )
        case "EdgeDetection":
            range_property = edge_conf.range_properties[0]
            range_property = RangeProperty[range_property].value(
                crawler_max=crawler_max,
                crawler_min=crawler_min,
                detect_range=edge_conf.detect_range,
                offsets=edge_conf.detect_offset,
            )
            return Ok(
                EdgeDetection(
                    range_property=range_property,
                    grid_size=edge_conf.grid_size,
                    side_range=None,
                    fwd_range=None,
                    voxel_size=edge_conf.voxel_size,
                    height_strips=edge_conf.height_strip,
                    debug=edge_conf.debug,
                    edge_width=edge_conf.edge_width,
                    remove_duplicate_label=edge_conf.remove_duplicate_label,
                )
            )
        case "MultiEdgeDetection":
            range_properties = [
                RangeProperty[range_calculator].value(
                    crawler_max=crawler_max,
                    crawler_min=crawler_min,
                    detect_range=edge_conf.detect_range,
                    offsets=edge_conf.detect_offset,
                )
                for range_calculator in edge_conf.range_properties
            ]
            return Ok(
                MultiEdgeDetection(
                    range_properties=range_properties,
                    voxel_size=edge_conf.voxel_size,
                    grid_size=edge_conf.grid_size,
                    height_strips=edge_conf.height_strip,
                    debug=edge_conf.debug,
                    remove_duplicate_label=edge_conf.remove_duplicate_label,
                    edge_width=edge_conf.edge_width,
                )
            )
        case x:
            return Err(
                EdgeDetInvalidArgumenError(
                    f"detector should be EdgeDetectionPolar, EdgeDetection, or MultiEdgeDetection, detector={x}"
                )
            )


if __name__ == "__main__":
    # 各種設定ファイル
    app_ini = ConfigParser(interpolation=ExtendedInterpolation())
    app_ini.read("./config/settings.ini", "UTF-8")
    app_config = AppConfig(app_ini)
    edge_conf = app_config.EdgeDetection
    rng = np.random.default_rng()
    points = rng.random(3, 1000)
    resolution = edge_conf.resolution
    edgeDet = EdgeDetection(
        grid_size=(resolution, resolution),
        side_range=edge_conf.side_range,
        fwd_range=edge_conf.fwd_range,
        height_strips=edge_conf.height_strip,
        edge_width=edge_conf.edge_width,
    )
    # edgeDet = EdgeDetection(
    #    app_config=app_config,
    #    resolution=app_config.EdgeDetection.resolution,
    #    side_range=app_config.EdgeDetection.side_range,
    #    fwd_range=app_config.EdgeDetection.fwd_range,
    #    height_strips=app_config.EdgeDetection.height_strip,
    #    debug=app_config.EdgeDetection.debug,
    #    edge_width=app_config.EdgeDetection.edge_width,
    # )
    edgeDet.main(points)
