from __future__ import annotations
import typing
__all__: list[str] = ['StatusCode', 'StatusMMAP']
class StatusCode:
    """
    Members:
    
      INIT
    
      REBOOT
    
      BOOTING
    
      RUNNING
    
      ERROR
    
      SHUTDOWN
    """
    BOOTING: typing.ClassVar[StatusCode]  # value = <StatusCode.BOOTING: 2>
    ERROR: typing.ClassVar[StatusCode]  # value = <StatusCode.ERROR: -2>
    INIT: typing.ClassVar[StatusCode]  # value = <StatusCode.INIT: 0>
    REBOOT: typing.ClassVar[StatusCode]  # value = <StatusCode.REBOOT: 1>
    RUNNING: typing.ClassVar[StatusCode]  # value = <StatusCode.RUNNING: 3>
    SHUTDOWN: typing.ClassVar[StatusCode]  # value = <StatusCode.SHUTDOWN: -1>
    __members__: typing.ClassVar[dict[str, StatusCode]]  # value = {'INIT': <StatusCode.INIT: 0>, 'REBOOT': <StatusCode.REBOOT: 1>, 'BOOTING': <StatusCode.BOOTING: 2>, 'RUNNING': <StatusCode.RUNNING: 3>, 'ERROR': <StatusCode.ERROR: -2>, 'SHUTDOWN': <StatusCode.SHUTDOWN: -1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class StatusMMAP:
    @staticmethod
    def get_status_name(arg0: typing.SupportsInt | typing.SupportsIndex) -> str:
        ...
    @staticmethod
    def is_recent(timeout: typing.SupportsFloat | typing.SupportsIndex = 5.0) -> bool:
        ...
    def __init__(self, path: str, create: bool = False) -> None:
        ...
    def close(self) -> None:
        ...
    def read_status(self) -> int:
        ...
    @typing.overload
    def write_status(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def write_status(self, arg0: StatusCode) -> None:
        ...
