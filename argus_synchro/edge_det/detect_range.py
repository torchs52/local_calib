"""崖検出の範囲を決めるためのモジュール
左右、後方の崖をまず検出したいが、今後機種変更などが起こってくると範囲の長さなどは変わってくるように思えたので、新しくファイルを分けて、実装する
"""

from abc import ABC, abstractmethod
from enum import Enum

from argus_synchro.common.common import NDPoint3f, Point2f, RangeF
from argus_synchro.edge_det.const import EdgeFilterType, ScanDirType


class RangePropertyBase(ABC):
    """検出範囲に対する情報を持っている抽象クラス"""

    def __init__(
        self,
        crawler_min: NDPoint3f,
        crawler_max: NDPoint3f,
        detect_range: RangeF,
        scan_direction: ScanDirType = ScanDirType.PLUS_Y,
        edge_filter_type: EdgeFilterType = EdgeFilterType.DoG,
        offsets: Point2f = (0.0, 1.0),
    ) -> None:
        """インタフェースの実装先も基本的にクローラーのmin, maxと検出範囲とオフセットを基に
        side_rangeやfwd_rangeは決めるはずはので、コンストラクタを定義"""
        self.crawler_min: NDPoint3f = crawler_min
        self.crawler_max: NDPoint3f = crawler_max
        self.detect_range: RangeF = detect_range
        self.scan_direction: ScanDirType = scan_direction
        self.edge_filter_type: EdgeFilterType = edge_filter_type
        self.offsets: Point2f = offsets

    @abstractmethod
    def calculate_side_range(
        self,
        **params: object,
    ) -> RangeF:
        """対応する場所に対する検出範囲を計算する抽象メソッド
        side_rangeを返す
        """
        raise NotImplementedError(
            "崖検出の該当する範囲のside_rangeを返すメソッドを実装する必要があります"
        )

    @abstractmethod
    def calculate_fwd_range(
        self,
        **params: object,
    ) -> RangeF:
        """対応する場所に対する検出範囲を計算する抽象メソッド
        fwd_rangeを返す
        """
        raise NotImplementedError(
            "崖検出の該当する範囲のfwd_rangeを返すメソッドを実装する必要があります"
        )


class BackRangePropertyBase(RangePropertyBase):
    """後方の検出範囲を計算するクラス"""

    def __init__(
        self,
        crawler_min: NDPoint3f,
        crawler_max: NDPoint3f,
        detect_range: RangeF,
        scan_direction: ScanDirType = ScanDirType.PLUS_Y,
        edge_filter_type: EdgeFilterType = EdgeFilterType.DoG,
        offsets: Point2f = (0.0, 1.0),
    ) -> None:
        super().__init__(
            crawler_min,
            crawler_max,
            detect_range,
            scan_direction,
            edge_filter_type,
            offsets,
        )

    def calculate_side_range(
        self,
        **params: object,
    ) -> RangeF:
        return (
            self.crawler_min[1] - self.detect_range[1] - self.offsets[1],
            self.crawler_max[1] + self.detect_range[1] + self.offsets[1],
        )

    def calculate_fwd_range(
        self,
        **params: object,
    ) -> RangeF:
        return (
            self.crawler_max[0] + self.offsets[0],
            self.crawler_max[0] + self.detect_range[0] + self.offsets[0],
        )


class LeftRangePropertyBase(RangePropertyBase):
    """左側の検出範囲を計算するクラス"""

    def __init__(
        self,
        crawler_min: NDPoint3f,
        crawler_max: NDPoint3f,
        detect_range: RangeF,
        scan_direction: ScanDirType = ScanDirType.PLUS_X,
        edge_filter_type: EdgeFilterType = EdgeFilterType.DoG,
        offsets: Point2f = (0.0, 1.0),
    ) -> None:
        super().__init__(
            crawler_min,
            crawler_max,
            detect_range,
            scan_direction,
            edge_filter_type,
            offsets,
        )

    def calculate_side_range(
        self,
        **params: object,
    ) -> RangeF:
        return (
            self.crawler_max[1] + self.offsets[1],
            self.crawler_max[1] + self.detect_range[1] + self.offsets[1],
        )

    def calculate_fwd_range(
        self,
        **params: object,
    ) -> RangeF:
        return (
            self.crawler_min[0] + self.offsets[0],
            self.crawler_max[0] + self.offsets[0],
        )


class RightRangePropertyBase(RangePropertyBase):
    """右側の検出範囲を計算するクラス"""

    def __init__(
        self,
        crawler_min: NDPoint3f,
        crawler_max: NDPoint3f,
        detect_range: RangeF,
        scan_direction: ScanDirType = ScanDirType.MINUS_X,
        edge_filter_type: EdgeFilterType = EdgeFilterType.DoG,
        offsets: Point2f = (0.0, 1.0),
    ) -> None:
        super().__init__(
            crawler_min,
            crawler_max,
            detect_range,
            scan_direction,
            edge_filter_type,
            offsets,
        )

    def calculate_side_range(
        self,
        **params: object,
    ) -> RangeF:
        return (
            self.crawler_min[1] - self.detect_range[1] - self.offsets[1],
            self.crawler_min[1] - self.offsets[1],
        )

    def calculate_fwd_range(
        self,
        **params: object,
    ) -> RangeF:
        fwd_range = (
            self.crawler_min[0] + self.offsets[0],
            self.crawler_max[0] + self.offsets[0],
        )

        return fwd_range


class CircumstanceRangePropertyBase(RangePropertyBase):
    def __init__(
        self,
        crawler_min: NDPoint3f,
        crawler_max: NDPoint3f,
        detect_range: RangeF,
        scan_direction: ScanDirType = ScanDirType.PLUS_Y,
        edge_filter_type: EdgeFilterType = EdgeFilterType.SOBEL_X_FORWARD,
        offsets: RangeF = (0.0, 1.0),
    ) -> None:
        """左右後方は関係なく、外周に対して定義される検出範囲のクラス
        極座標による崖検出をする場合に用いる
        """
        super().__init__(
            crawler_min,
            crawler_max,
            detect_range,
            scan_direction,
            edge_filter_type,
            offsets,
        )

    def calculate_side_range(
        self,
        **params: object,
    ) -> RangeF:
        return (
            self.crawler_min[1] - self.detect_range[1] - self.offsets[1],
            self.crawler_max[1] + self.detect_range[1] + self.offsets[1],
        )

    def calculate_fwd_range(
        self,
        **params: object,
    ) -> RangeF:
        return (
            self.crawler_min[0] + self.offsets[0],
            self.crawler_max[0] + self.detect_range[0] + self.offsets[0],
        )


class RangeProperty(Enum):
    BACK = BackRangePropertyBase
    LEFT = LeftRangePropertyBase
    RIGHT = RightRangePropertyBase
    CIRCUM = CircumstanceRangePropertyBase
