from argus_synchro.config.app_config import AppConfig


class AIRY96PointsFile:
    def __init__(self, index: int, app_conf: AppConfig) -> None:
        self._index: int = index
        self._app_conf: AppConfig = app_conf

    def connect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: connect()"
        raise NotImplementedError(err_msg)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def get_points(self) -> tuple[list[tuple[float, float, float, int]], float]:
        err_msg = f"class: {self.__class__.__name__}, method: get_points()"
        raise NotImplementedError(err_msg)


class AIRY96Points:
    def __init__(self, index: int, app_conf: AppConfig) -> None:
        self._index: int = index
        self._app_conf: AppConfig = app_conf

    def connect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: connect()"
        raise NotImplementedError(err_msg)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def get_points(self) -> tuple[list[tuple[float, float, float, int]], float]:
        err_msg = f"class: {self.__class__.__name__}, method: get_points()"
        raise NotImplementedError(err_msg)
