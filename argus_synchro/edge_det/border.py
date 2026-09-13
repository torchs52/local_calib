"""空間の範囲に対して行うクラスに関するモジュール"""

from collections.abc import Callable, Iterable

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.common import (
    NDPoint2f,
    NDPoint2i,
    NDPoint3f,
    NDPoint3fArray,
    NDSeries,
    Point2i,
    Point3f,
)
from argus_synchro.edge_det.const import ScanDirType
from argus_synchro.edge_det.typedef import XYMinMax, XYTup, XYZTup
from argus_synchro.edge_det.utils import (
    calc_edge_length_piecewise,
    get_empty_points_lines_length,
    scale_to_255,
)


class BorderExtractor:
    scan_direction: ScanDirType  # 崖のエッジをどの方向に見ていくかを表す列挙型
    side_range: XYTup  # y方向の検出範囲
    fwd_range: XYTup  # x方向の検出範囲
    side_length: float  # y方向の検出範囲の長さ
    fwd_length: float  # x方向の検出範囲の長さ
    grid_size: XYTup  # (x,y)の1画素の実空間上の格子サイズ
    edge_size: XYTup  # 1m辺りの画素数, grid_sizeの逆数
    height_strips: float  # 崖のエッジ点に持たせるz値のオフセット
    target_edge_dist_th: float  # 一つの崖に対する総距離の閾値

    def __init__(
        self,
        scan_direction: ScanDirType,
        side_range: XYTup,
        fwd_range: XYTup,
        grid_size: XYTup,
        edge_size: XYTup,
        height_strips: float,
        target_edge_dist_th: float,
    ) -> None:
        """
        BorderDetectorで座標変換部分を取り除いたクラス
        元々BorderDetectorを使っていたが、極座標での鳥瞰図などを作る過程で
        座標変換部分をこのクラスが担当すると扱いにくくなってきたので、座標変換部分は外に出した形のクラス

        :param self: 説明
        :param scan_direction: 崖のエッジをどの方向に見ていくかを表す列挙型
        :type scan_direction: ScanDirType
        :param side_range: y方向の検出範囲
        :type side_range: t_xy
        :param fwd_range: x方向の検出範囲
        :type fwd_range: t_xy
        :param grid_size: (x,y)の1画素の実空間上の格子サイズ
        :type grid_size: t_xy
        :param height_strips: 崖のエッジ点に持たせるz値のオフセット
        :type height_strips: float
        :param target_edge_dist_th: 一つの崖に対する総距離の閾値
        :type target_edge_dist_th: float
        """
        self.scan_direction = scan_direction
        self.side_range = side_range
        self.fwd_range = fwd_range
        self.side_length = side_range[1] - side_range[0]
        self.fwd_length = fwd_range[1] - fwd_range[0]
        self.grid_size = grid_size
        self.edge_size = edge_size
        # self.edge_size = (1 / grid_size[0], 1 / grid_size[1])
        self.height_strips = height_strips
        self.target_edge_dist_th = target_edge_dist_th

    def _edge_search_by_label_x(
        self,
        chosen_label: Iterable[int],
        labeled_img: cv2.typing.MatLike | np.ma.MaskedArray,
        z_offset: float,
        is_inverse: bool = False,
        search_range: list[int] | int | None = None,
        is_exc_mask: bool = True,
    ) -> tuple[list[XYZTup], list[int]]:
        """
        ラベリング画像であるlabeled_imgに対して、各ラベル毎に各x座標の最小のy座標を見つける

        :param self: 説明
        :param chosen_label: 対象となるラベル番号
        :type chosen_label: Iterable[int]
        :param labeled_img: ラベリング画像, マスクされた画像でもOK
        :type labeled_img: cv2.typing.MatLike | np.ma.MaskedArray
        :param z_offset: エッジの高さの値
        :type z_offset: float
        :param is_inverse: 小さい順に走査するか, 大きい順に走査するか
        :type is_inverse: bool
        :param search_range: 見る範囲, x毎に決める場合はlist, 一定値で見る範囲を与える場合はint, 全部を見る場合はNoneを設定する
        :type search_range: list[int] | int | None
        :param is_exc_mask: ラベリング画像がマスクされたもので、マスク箇所を除外するかどうか, Trueの場合除外する
        :type is_exc_mask: bool
        :return: 崖エッジの画素上の(x,y,z), 崖の一つのクラスタの長さ
        :rtype: tuple[list[t_xyz], list[int]]
        """
        edges_oneside: list[Point3f] = []
        length_edge: list[int] = []
        y_range = abs(self.fwd_length)
        y_max = int(y_range / self.grid_size[1])
        # search_range = range(y_max - 1, -1, -1) if is_inverse else range(y_max)

        for label_no in chosen_label:
            # ラベルに一致するx座標のインデックスを取得
            x_coords = np.where(labeled_img == label_no)[1]
            unique_x_coords = np.unique(
                x_coords,
            )  # 同じx座標は一度見れば良い（x方向のエッジだけみたいから）

            _length_edge = 0  # ユニークなx座標の数をエッジの長さとして記録
            for x in unique_x_coords:
                # x座標を走査し、ラベルに一致する最初のx座標を見つける
                # for y in range(y_max - 1, -1, -1):
                match search_range:
                    case None:
                        _search_range = range(y_max)
                    case int(val):
                        _search_range = range(val)
                    case list(val):
                        _search_range = range(max(val[x] - 1, 0))
                if is_inverse:
                    _search_range = reversed(_search_range)

                for y in _search_range:
                    if (
                        is_exc_mask
                        and isinstance(labeled_img, np.ma.MaskedArray)
                        and np.ma.is_masked(labeled_img[y, x])
                    ):
                        # print(f"(y, x, val) = {(y, x, labeled_img[y, x])}")
                        # skip mask position if is_exc_mask is True
                        continue

                    if labeled_img[y, x] == label_no:
                        # 対応するZ座標を計算
                        # z_val = im[y-1, x] + z_offset
                        z_val = z_offset
                        # エッジ座標をリストに追加（y, x, zの順）
                        edges_oneside.append((y, x, z_val))
                        _length_edge += 1
                        break  # 最初の一致点を見つけたら、そのy座標でのループを終了
            length_edge.append(_length_edge)
        return edges_oneside, length_edge

    def _edge_search_by_label_y(
        self,
        chosen_label: Iterable[int],
        labeled_img: cv2.typing.MatLike | np.ma.MaskedArray,
        z_offset: float,
        is_inverse: bool = False,
        search_range: list[int] | int | None = None,
        is_exc_mask: bool = True,
    ) -> tuple[list[Point3f], list[int]]:
        """
        ラベリング画像であるlabeled_imgに対して、各ラベル毎に各y座標の最小のx座標を見つける

        :param self: 説明
        :param chosen_label: 対象となるラベル番号
        :type chosen_label: Iterable[int]
        :param labeled_img: ラベリング画像, マスクされた画像でもOK
        :type labeled_img: cv2.typing.MatLike | np.ma.MaskedArray
        :param z_offset: エッジの高さの値
        :type z_offset: float
        :param is_inverse: 小さい順に走査するか, 大きい順に走査するか
        :type is_inverse: bool
        :param search_range: 見る範囲, y毎に決める場合はlist, 一定値で見る範囲を与える場合はint, 全部を見る場合はNoneを設定する
        :type search_range: list[int] | int | None
        :param is_exc_mask: ラベリング画像がマスクされたもので、マスク箇所を除外するかどうか, Trueの場合除外する
        :type is_exc_mask: bool
        :return: 崖エッジの画素上の(x,y,z), 崖の一つのクラスタの長さ
        :rtype: tuple[list[Point3f], list[int]]

        """
        edges_oneside: list[Point3f] = []
        length_edge: list[int] = []
        x_range = abs(self.side_length)
        x_max = int(x_range / self.grid_size[0])

        # reverseの場合は、xが大きい側から走査している

        # for idx in range(1, labels):  # ラベル0（背景）を除く各ラベルに対してループ
        for label_no in chosen_label:
            # ラベルに一致するy座標のインデックスを取得
            y_coords = np.where(labeled_img == label_no)[0]
            unique_y_coords = np.unique(
                y_coords,
            )  # 同じy座標は一度見れば良い（x方向のエッジだけみたいから）

            _length_edge = 0  # ユニークなx座標の数をエッジの長さとして記録
            for y in unique_y_coords:
                # x座標を小さい方から走査し、ラベルに一致する最初のx座標を見つける
                # set search range
                match search_range:
                    case None:
                        _search_range = range(x_max)
                    case int(val):
                        _search_range = range(val)
                    case list(val):
                        _search_range = range(val[y])
                if is_inverse:
                    _search_range = reversed(_search_range)

                for x in _search_range:
                    if (
                        is_exc_mask
                        and isinstance(labeled_img, np.ma.MaskedArray)
                        and np.ma.is_masked(labeled_img[y, x])
                    ):
                        # print(f"(y, x, val) = {(y, x, labeled_img[y, x])}")
                        # skip mask position if is_exc_mask is True
                        continue
                    if labeled_img[y, x] == label_no:
                        # 対応するZ座標を計算
                        # z_val = im[y, x] + z_offset
                        z_val = z_offset
                        # エッジ座標をリストに追加（y, x, zの順）
                        edges_oneside.append((y, x, z_val))
                        _length_edge += 1
                        break  # 最初の一致点を見つけたら、そのy座標でのループを終了
            length_edge.append(_length_edge)

        return edges_oneside, length_edge

    def border_detection(
        self,
        labeled_imgs: cv2.typing.MatLike,
        scan_direction: ScanDirType,
        z_offset: float,
        chosen_label: list[int],
        search_range: list[int] | int | None = None,
        is_exc_occ: bool = False,
    ) -> tuple[NDPoint3fArray, list[int]]:
        """
        ラベリング画像から走査方向に基づいて、3d座標を計算する

        :param self: 説明
        :param labeled_imgs: エッジ検出で得られた二値画像をラベリングした結果の画像
        :type labeled_imgs: cv2.typing.MatLike
        :param scan_direction: 走査する方向を表す列挙型
        :type scan_direction: ScanDirType
        :param z_offset: Z座標のオフセット値
        :type z_offset: float
        :param chosen_label: labeled_imgの中でedge_onesideの計算を行うラベル番号のリスト
        :type chosen_label: list[int]
        :param search_range: 走査をどこまで行うかを表す, 各点で走査する範囲が決まっている場合はlist, 全て一定ならint, 全体ならNoneを指定
        :type search_range: list[int] | int | None
        :param is_exc_occ: ラベリング画像がマスクされたもので、マスク箇所を除外するかどうか, Trueの場合除外する
        :type is_exc_occ: bool
        :return: 検出された輪郭の3D座標, 各輪郭の長さのtuple
        :rtype: tuple[NDPoint3fArray, list[int]]
        """

        # ラベル番号0は、黒い画像部分になるので、除外
        _chosen_label = filter(lambda elem: elem != 0, chosen_label)
        edge_search_func: Callable[
            [
                Iterable[int],
                cv2.typing.MatLike | np.ma.MaskedArray[np.int32],
                float,
                bool,
                list[int] | int | None,
                bool,
            ],
            tuple[list[XYZTup], list[int]],
        ]
        # case, matchで探索する方向に応じて崖検出を行う
        match scan_direction:
            case ScanDirType.PLUS_X:
                edge_search_func = self._edge_search_by_label_y
                is_reverse = False
            case ScanDirType.MINUS_X:
                edge_search_func = self._edge_search_by_label_y
                is_reverse = True
            case ScanDirType.PLUS_Y:
                edge_search_func = self._edge_search_by_label_x
                is_reverse = False
            case ScanDirType.MINUS_Y:
                edge_search_func = self._edge_search_by_label_x
                is_reverse = True

        # 選んだ走査関数と走査の順転逆転に応じて走査する
        edges_oneside, edge_length = edge_search_func(
            _chosen_label,
            labeled_imgs,
            z_offset,
            is_reverse,
            search_range,
            is_exc_occ,
        )
        return np.array(edges_oneside), edge_length

    def _choose_edges(
        self,
        labeled_imgs: cv2.typing.MatLike,
        scan_direction: ScanDirType,
        loop_limit: int = 1000,
    ) -> list[int]:
        """
        あるエッジの後方にあるエッジを消して、関連があるエッジを選択する

        :param self: 説明
        :param labeled_imgs: ラベリング画像
        :type labeled_imgs: cv2.typing.MatLike
        :param scan_direction: 後方判定をする方向を表す列挙型
        :type scan_direction: ScanDirType
        :param loop_limit: エッジ候補が見つかる限り繰り返すので、ループ回数に上限を設けている
        :type loop_limit: int
        :return: 選ばれたエッジのラベリング画像上での番号
        :rtype: list[int]
        """
        # Remark: scan方向と同じ方向のエッジを除去しているが、これで良いかは要検討

        pick_field_name: str
        remove_fields: tuple[str, str]
        match scan_direction:
            case ScanDirType.PLUS_Y:
                pick_field_name = "min_y"
                remove_fields = ("min_x", "max_x")
                pick_func = min
            case ScanDirType.PLUS_X:
                pick_field_name = "min_x"
                remove_fields = ("min_y", "max_y")
                pick_func = min
            case ScanDirType.MINUS_Y:
                pick_field_name = "min_y"
                remove_fields = ("min_x", "max_x")
                pick_func = max
            case ScanDirType.MINUS_X:
                pick_field_name = "min_x"
                remove_fields = ("min_y", "max_y")
                pick_func = max

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
                pos_x.min(), pos_x.max(), pos_y.min(), pos_y.max()
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

    def get_edge_size(self) -> float:
        """自分自身の走査方向に対応したedge_sizeを返す, いくつかの箇所で呼びたいので、メソッドにした"""
        match self.scan_direction:
            case ScanDirType.PLUS_X | ScanDirType.MINUS_X:
                return self.edge_size[1]
            case ScanDirType.PLUS_Y | ScanDirType.MINUS_Y:
                return self.edge_size[0]

    def filter_target_edge(
        self,
        edge_points: NDPoint3f,
        edge_length: list[int],
    ) -> tuple[NDPoint2f, list[int]]:
        """
        崖検出で得られた崖のエッジ点に対するフィルタ処理を行うメソッド

        :param self: 説明
        :param edge_points: 崖検出で得られたエッジ点
        :type edge_points: NDPoint3f
        :param edge_length: 各崖クラスタの長さ
        :type edge_length: list[int]
        :return: フィルター後のedge_points, edge_length
        :rtype: tuple[NDPoint2f, list[int]], 何も選ばれなければ、(0,2)の行列と空のlistが返される
        """
        pc_len = 0
        target_edge_point: list[NDPoint3f] = []
        target_edge_lenth: list[int] = []

        # 各崖クラスタの点の総距離でフィルタリング
        for edge_len in edge_length:
            edge_point = edge_points[pc_len : pc_len + edge_len]
            edge_dist = calc_edge_length_piecewise(edge_point)

            if edge_dist > self.target_edge_dist_th:
                target_edge_point.append(edge_point)
                target_edge_lenth.append(edge_len)
            pc_len += edge_len
        if len(target_edge_point) == 0:
            return np.empty((0, 2)), []
        return np.vstack(target_edge_point), target_edge_lenth

    def choose_search_label(
        self,
        target_labels: NDSeries,
        labeled_img: cv2.typing.MatLike,
        bbox: NDArray[np.uint32],
        target_region: XYMinMax,
        do_copy: bool = True,
        remove_duplicate_label: bool = True,
    ) -> list[int]:
        """
        走査の対象としたいラベルを選ぶ
        引数のlabeled_imgを書き換える処理を行っているため、do_copyという引数を用意している

        :param self: 説明
        :param target_labels: 対象となる全てのラベル
        :type target_labels: NDSeries
        :param labeled_img: ラベリング画像
        :type labeled_img: cv2.typing.MatLike
        :param bbox: 各ラベルのbbox
        :type bbox: NDArray[np.uint32]
        :param target_region: 選ぶラベルを見つけに行く範囲
        :type target_region: XYMinMax
        :param do_copy: ラベリング画像をコピーするかどうか, ラベリング画像の書き換える処理を行っているため用意している
        :type do_copy: bool
        :param remove_duplicate_label: あるラベルの後方のラベル加えるかどうか
        :type remove_duplicate_label: bool
        :return: エッジ抽出に加えるラベル番号のリスト
        :rtype: list[int]
        """
        if do_copy:
            labeled_img = labeled_img.copy()
        (min_x, max_x, min_y, max_y) = target_region

        # 見たい走査方向の粒度に応じて、小さすぎるグループは除去
        _edge_size = self.get_edge_size()
        labeled_img[np.isin(labeled_img, np.where(bbox[:, -1] <= _edge_size)[0])] = 0

        # 全体の中で、対応する崖検出の範囲に存在していないラベリングは除去
        remove_inds = np.setdiff1d(
            target_labels,
            np.unique(labeled_img[min_x : (max_x + 1), min_y : (max_y + 1)]),
        )
        labeled_img[np.isin(labeled_img, remove_inds)] = 0

        # border検出の対象となるラベルを決める
        if remove_duplicate_label:
            chosen_label = self._choose_edges(
                labeled_img,
                scan_direction=self.scan_direction,
            )
        else:
            chosen_label = np.unique(labeled_img).tolist()

        return chosen_label

    def convert_2dborder_to_3d(
        self,
        lidar_edges: tuple[NDSeries, NDSeries, NDSeries],
        length_edge: list[int],
    ) -> tuple[NDPoint3f, NDPoint2i, NDSeries]:
        """
        2Dの境界線座標から3DのLineSetオブジェクトを生成する。

        この関数は、BEV画像から抽出された2Dの境界線座標を3D空間にマッピングし、それらの座標を用いてエッジ点, エッジlineのindex, エッジ長さを計算する

        処理の流れ:
            1. 2D座標を3D座標に変換。
            2. 変換された座標を用いて複数の線分を生成。
            3. これらの線分を結合してLineSetオブジェクトを作成。

        :param self: 説明
        :param lidar_edges: エッジのx,y,zをtupleで分けた変数
        :type lidar_edges: tuple[NDSeries, NDSeries, NDSeries]
        :param length_edge: 各クラスタにおけるエッジの長さ
        :type length_edge: list[int]
        :return: エッジの座標, エッジ同士をどのようにつなげるか(open3d.geometry.LineSetの名残), 各クラスタでのエッジの長さのtuple
        Remark: エッジの座標は複数のクラスタの座標が順番に入っているので、各クラスタのエッジ点が欲しければ、クラスタのエッジの長さを見ながら順番にばらす必要がある
        :rtype: tuple[NDPoint3f, NDPoint2i, NDSeries]
        """
        if len(lidar_edges[0]) == 0:
            # 崖が存在しない場合は空のLineSetを返す
            return get_empty_points_lines_length()
        (edges_x, edges_y, edges_z) = lidar_edges

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

        return np.array(multi_points), np.array(multi_lines), np.array(length_edge)


class BorderDetector:
    scan_direction: ScanDirType  # 崖のエッジをどの方向に見ていくかを表す列挙型
    side_range: XYTup  # y方向の検出範囲
    fwd_range: XYTup  # x方向の検出範囲
    side_length: float  # y方向の検出範囲の長さ
    fwd_length: float  # x方向の検出範囲の長さ
    grid_size: XYTup  # (x,y)の1画素の実空間上の格子サイズ
    height_strips: float  # 崖のエッジ点に持たせるz値のオフセット

    def __init__(
        self,
        scan_direction: ScanDirType,
        side_range: XYTup,
        fwd_range: XYTup,
        side_length: float,
        fwd_length: float,
        grid_size: XYTup,
        height_strips: float,
    ) -> None:
        """
        EdgeDetectionとMultiEdgeDetectionで空間の範囲に対して処理をするものをまとめたクラス
        EdgeDetectionやMultiEdgeDetectionで使いたいが、継承するものとも違うように感じたので、クラスとして作っておいて、
        各クラスが参照する形でクラスを作成した
        最新の崖検出で使わなくなった

        :param self: 説明
        :param scan_direction: 崖のエッジをどの方向に見ていくかを表す列挙型
        :type scan_direction: ScanDirType
        :param side_range: y方向の検出範囲
        :type side_range: t_xy
        :param fwd_range: x方向の検出範囲
        :type fwd_range: t_xy
        :param side_length: y方向の検出範囲の長さ
        :type side_length: float
        :param fwd_length: x方向の検出範囲の長さ
        :type fwd_length: float
        :param grid_size: (x,y)の1画素の実空間上の格子サイズ
        :type grid_size: t_xy
        :param height_strips: 崖のエッジ点に持たせるz値のオフセット
        :type height_strips: float
        """
        self.scan_direction = scan_direction
        self.side_range = side_range
        self.fwd_range = fwd_range
        self.side_length = side_length
        self.fwd_length = fwd_length
        self.grid_size = grid_size
        self.height_strips = height_strips

    def _edge_search_by_label_x(
        self,
        chosen_label: list[int],
        labeled_img: NDArray[np.uint8],
        z_offset: float,
        is_inverse: bool = False,
    ) -> tuple[list[XYZTup], list[int]]:
        """ラベリング画像であるlabeled_imgに対して、各ラベル毎に各x座標の最小のy座標を見つける"""
        edges_oneside = []
        length_edge = []
        y_range = abs(self.fwd_length)
        y_max = int(y_range / self.grid_size[1])
        search_range = range(y_max - 1, -1, -1) if is_inverse else range(y_max)

        for label_no in chosen_label:
            # ラベルに一致するx座標のインデックスを取得
            x_coords = np.where(labeled_img == label_no)[1]
            unique_x_coords = np.unique(
                x_coords,
            )  # 同じx座標は一度見れば良い（x方向のエッジだけみたいから）

            _length_edge = 0  # ユニークなx座標の数をエッジの長さとして記録
            for x in unique_x_coords:
                # x座標を走査し、ラベルに一致する最初のx座標を見つける
                # for y in range(y_max - 1, -1, -1):
                for y in search_range:
                    if labeled_img[y, x] == label_no:
                        # 対応するZ座標を計算
                        # z_val = im[y-1, x] + z_offset
                        z_val = z_offset
                        # エッジ座標をリストに追加（y, x, zの順）
                        edges_oneside.append((y, x, z_val))
                        _length_edge += 1
                        break  # 最初の一致点を見つけたら、そのy座標でのループを終了
            length_edge.append(_length_edge)
        return edges_oneside, length_edge

    def _edge_search_by_label_y(
        self,
        chosen_label: list[int],
        labeled_img: NDArray[np.uint8],
        z_offset: float,
        is_inverse: bool = False,
    ) -> tuple[list[XYZTup], list[int]]:
        """ラベリング画像であるlabeled_imgに対して、各ラベル毎に各y座標の最小のx座標を見つける"""
        edges_oneside = []
        length_edge = []
        x_range = abs(self.side_length)
        x_max = int(x_range / self.grid_size[0])

        # reverseの場合は、xが大きい側から走査している
        search_range = range(x_max - 1, -1, -1) if is_inverse else range(x_max)

        # for idx in range(1, labels):  # ラベル0（背景）を除く各ラベルに対してループ
        for label_no in chosen_label:
            # ラベルに一致するy座標のインデックスを取得
            y_coords = np.where(labeled_img == label_no)[0]
            unique_y_coords = np.unique(
                y_coords,
            )  # 同じy座標は一度見れば良い（x方向のエッジだけみたいから）

            _length_edge = 0  # ユニークなx座標の数をエッジの長さとして記録
            for y in unique_y_coords:
                # x座標を小さい方から走査し、ラベルに一致する最初のx座標を見つける
                for x in search_range:
                    if labeled_img[y, x] == label_no:
                        # 対応するZ座標を計算
                        # z_val = im[y, x] + z_offset
                        z_val = z_offset
                        # エッジ座標をリストに追加（y, x, zの順）
                        edges_oneside.append((y, x, z_val))
                        _length_edge += 1
                        break  # 最初の一致点を見つけたら、そのy座標でのループを終了
            length_edge.append(_length_edge)

        return edges_oneside, length_edge

    def border_detection(
        self,
        labeled_imgs: NDArray[np.uint8],
        z_offset: float,
        chosen_label: list[int],
    ) -> tuple[NDArray[np.float64], list[int]]:
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

        # ラベル番号0は、黒い画像部分になるので、除外
        _chosen_label = filter(lambda elem: elem != 0, chosen_label)

        # 探索する方向に応じて崖検出を行う
        if self.scan_direction == ScanDirType.PLUS_X:
            edges_oneside, length_edge = self._edge_search_by_label_y(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=False,
            )
        elif self.scan_direction == ScanDirType.PLUS_Y:
            edges_oneside, length_edge = self._edge_search_by_label_x(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=False,
            )
        elif self.scan_direction == ScanDirType.MINUS_X:
            edges_oneside, length_edge = self._edge_search_by_label_y(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=True,
            )
        elif self.scan_direction == ScanDirType.MINUS_Y:
            edges_oneside, length_edge = self._edge_search_by_label_x(
                _chosen_label,
                labeled_imgs,
                z_offset,
                is_inverse=True,
            )
        else:
            raise NotImplementedError("他の場合はまだ未実装")

        return np.array(edges_oneside), length_edge

    def convert_2DBoader_to_3D(
        self,
        lidar_edges: tuple[
            NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]
        ],
        length_edge: list[int],
    ) -> tuple[
        NDArray[np.float64], NDArray[np.float64], NDArray[np.int32]
    ]:  # o3d.geometry.LineSet:
        """
        2Dの境界線座標から3DのLineSetオブジェクトを生成する。

        この関数は、BEV画像から抽出された2Dの境界線座標を3D空間にマッピングし、それらの座標を用いてLineSetオブジェクトを生成する。各境界線は、3D空間内の線分として表現される。

        パラメータ:
            lidar_edges: 2Dの境界線座標を含むx,y,zのtuple
            length_edge (list of int): 各境界線の長さを表すリスト。

        戻り値:
            崖のエッジ点, 崖点のindex同士の繋がり, 崖クラスタの長さ

        処理の流れ:
            1. 2D座標を3D座標に変換。
            2. 変換された座標を用いて複数の線分を生成。
            3. これらの線分を結合してLineSetオブジェクトを作成。
        """
        if len(lidar_edges[0]) == 0:
            # 崖が存在しない場合は空のLineSetを返す
            return get_empty_points_lines_length()
        (edges_x, edges_y, edges_z) = lidar_edges

        multi_points = []
        multi_lines = []
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

        return np.array(multi_points), np.array(multi_lines), np.array(length_edge)

    def pixel2lidar(
        self,
        pixel_val: NDArray,
        z_val: NDArray[np.float64] | float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """与えられた画素部分に対応する点群上の座標を返す関数, 高さはz_valで与えられる"""

        # x_centerを使っていたが、変更していて、これで問題ないか要確認
        lidar_pos = np.zeros((len(pixel_val), 3), dtype=float)
        lidar_pos[:, 0] = pixel_val[:, 0] * self.grid_size[1] + self.fwd_range[0]
        lidar_pos[:, 1] = pixel_val[:, 1] * self.grid_size[0] + self.side_range[0]
        lidar_pos[:, 2] = z_val

        return lidar_pos[:, 0], lidar_pos[:, 1], lidar_pos[:, 2]

    def lidar2pixel(
        self,
        lidar_points: NDArray[np.float64],
        min_z: float = -1.88,
        max_z: float = -0.88,
        is_scaled: bool = True,
    ) -> NDArray:
        """LiDAR点群を解像度に合わせて、BEVに変換する
        lidar_pointsは絞った点群として、与えられたlidar_pointsをBEVに変換するだけの関数
        インスタンス変数を使ったりしないので、クラスメソッドにする
        """

        x_lidar = lidar_points[:, 0]
        y_lidar = lidar_points[:, 1]
        z_lidar = lidar_points[:, 2]

        # LiDAR 座標をグリッドマップ座標に変換
        x_img = np.floor((y_lidar - self.side_range[0]) / self.grid_size[0]).astype(
            np.int32
        )
        y_img = np.floor((x_lidar - self.fwd_range[0]) / self.grid_size[1]).astype(
            np.int32
        )

        # グリッドマップの初期化
        x_max = int(np.ceil(self.side_length / self.grid_size[0]))
        y_max = int(np.ceil(self.fwd_length / self.grid_size[1]))

        grid_map = np.zeros([y_max, x_max], dtype=np.uint8)
        if is_scaled:
            grid_map[y_img, x_img] = scale_to_255(
                z_lidar, min_value=min_z, max_value=max_z
            )
        else:
            grid_map[y_img, x_img] = z_lidar
        return grid_map
