# 2025/2/14 argus本体アプリから移植（中村さん協力） detect2d.pyより。いつかは本体アプリと統合する必要がある。

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

# from tensorflow.python.saved_model import tag_constants


class ObjDetectionInterface(Protocol):
    """物体検知を行うクラスが共通で持つメソッドを規定したもの
    外部とのインターフェイスとなる処理は、物体検知だけなので、そのメソッドだけ入出力を規定している
    """

    def object_detect(
        self,
        frame: NDArray[np.uint8],
    ) -> list[NDArray]:
        raise NotImplementedError(
            "このクラスを継承したクラスは、object_detectを実装する必要があります",
        )


@dataclass
class detect2d:
    isApplied: bool = True  # この辺は使っていないが、settingsにあるので置いている
    core_path: str = (
        "./core/config"  # この辺は使っていないが、settingsにあるので置いている
    )
    model_path: str = "./checkpoints/yolov4-tiny-416"  # この辺は使っていないが、settingsにあるので置いている
    yolo_class: str = "./config/classes/coco.names"  # この辺は使っていないが、settingsにあるので置いている
    onnx_model_path: str = "./checkpoints/damoyolo_large.onnx"
    conf_thresh: float = 0.7
    nms_thresh: float = 0.5
    use_onnx: bool = True  # この辺は使っていないが、settingsにあるので置いている
    is_DAMO_YOLO: bool = True  # この辺は使っていないが、settingsにあるので置いている


@dataclass
class camera:
    camera_config_file: str = "./config/camera_three.json"  # この辺は使っていないが、settingsにあるので置いている
    MOTEC: bool = True  # この辺は使っていないが、settingsにあるので置いている
    video_width: int = 1280  # この辺は使っていないが、settingsにあるので置いている
    video_height: int = 720  # この辺は使っていないが、settingsにあるので置いている
    sys_width: int = 1280
    sys_height: int = 720
    porttable: tuple[int, int, int] = (
        10750,
        10760,
        10770,
    )  # この辺は使っていないが、settingsにあるので置いている
    iptable: tuple[str, str, str] = (
        "192.168.1.75",
        "192.168.1.76",
        "192.168.1.77",
    )  # この辺は使っていないが、settingsにあるので置いている
