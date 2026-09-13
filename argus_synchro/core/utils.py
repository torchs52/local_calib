from __future__ import annotations

from collections.abc import Callable
from typing import Final

import cv2
import numpy as np
from numpy.typing import NDArray


def read_class_names(
    class_file_name: str,
    on_error: Callable[[str, Exception], None] | None = None,
) -> dict[int, str]:
    names: dict[int, str] = {}
    try:
        with open(class_file_name) as data:
            for id_, name in enumerate(data):
                names[id_] = name.strip("\n")
    except (OSError, UnicodeError) as error:
        if on_error is not None:
            on_error(class_file_name, error)
        raise
    return names


class FastResize:
    def __init__(
        self,
        width: int,
        height: int,
        interpolation: int = cv2.INTER_NEAREST,
    ) -> None:
        self._size: Final[cv2.typing.Size] = (width, height)
        self._interpolation: Final[int] = interpolation
        self._dst: NDArray[np.uint8] = np.empty((height, width, 3), np.uint8)

    def apply(self, image: cv2.typing.MatLike) -> NDArray[np.uint8]:
        cv2.resize(
            image,
            self._size,
            dst=self._dst,
            interpolation=self._interpolation,
        )
        return self._dst
