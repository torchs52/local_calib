from __future__ import annotations

import builtins
import contextlib
from typing import TYPE_CHECKING

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import CANConf
from argus_synchro.device.can.can_receiver import CanHandler, DecodedCanMessage

if TYPE_CHECKING:
    import pybind_shi_sensor_lib as shi
with contextlib.suppress(builtins.BaseException):
    import pybind_shi_sensor_lib as shi


class ShiLibCan:
    def __init__(
        self,
        index: int,
        can_conf: CANConf,
        crane_model: str,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.can_dev = shi.Can()
        self._index = index
        self.can_recv_msg = shi.CanMessageData()

        self.handle = 0
        self.isLatest = True

        self._handler = CanHandler(can_conf, crane_model, app_logger_factory)
        self.update(can_conf)

    def receive_can_data(self) -> DecodedCanMessage | None:
        return self._get_data()

    def update(self, can_conf: CANConf) -> None:
        self._yaw_offset_deg: float = can_conf.yaw_offset_deg
        if hasattr(self, "_handler"):
            self._handler.update(can_conf)

    def _get_data(self) -> DecodedCanMessage | None:
        timestamp_us = 1000000
        isLatest = True
        handle: int = self._index
        rts = self.can_dev.getSensorData(
            handle,
            self.can_recv_msg,
            timestamp_us,
            isLatest,
        )
        if rts == shi.ApiStatus.SUCCESS:
            can_id = f"{self.can_recv_msg.id:X}"
            self._logger.info(
                "CAN-ID:%x, TIMESTAMP:%d, SIZE:%d, DATA:%s",
                self.can_recv_msg.id,
                self.can_recv_msg.timestamp_ms,
                self.can_recv_msg.size,
                self.can_recv_msg.data[0:8],
            )
            raw_data = "".join(f"{byte:02X}" for byte in self.can_recv_msg.data)
            return self._handler.dispatch(can_id, raw_data)
        else:
            self._logger.info("get can mesg error. status:%s", rts)
        return None
