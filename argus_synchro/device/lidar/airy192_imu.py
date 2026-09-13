from argus_synchro.config.app_config import AppConfig


class AIRY192ImuFile:
    def __init__(self, index: int, app_conf: AppConfig) -> None:
        self._index: int = index
        self._app_conf: AppConfig = app_conf

    def connect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: connect()"
        raise NotImplementedError(err_msg)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def get_imu(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
        err_msg = f"class: {self.__class__.__name__}, method: get_imu()"
        raise NotImplementedError(err_msg)


class AIRY192Imu:
    def __init__(self, index: int, app_conf: AppConfig) -> None:
        self._index: int = index
        self._app_conf: AppConfig = app_conf

    def connect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: connect()"
        raise NotImplementedError(err_msg)

    def disconnect(self) -> None:
        err_msg = f"class: {self.__class__.__name__}, method: disconnect()"
        raise NotImplementedError(err_msg)

    def get_imu(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
        err_msg = f"class: {self.__class__.__name__}, method: get_imu()"
        raise NotImplementedError(err_msg)
