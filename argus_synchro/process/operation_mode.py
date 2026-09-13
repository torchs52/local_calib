from enum import IntEnum, auto


# 動作モード
class OPERATION_MODE(IntEnum):
    SCRUT = 0  # 通常周辺監視
    CALIB = 1  # 校正パラメータ生成


class CalibMode(IntEnum):
    IsRunning3D3Dcalib = auto()
    IsRunning2D3Dcalib = auto()
    IsRunning2D3Dcheck = auto()
    wait_app = auto()
