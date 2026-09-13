"""
shi sensor library, can
"""
from __future__ import annotations
import collections.abc
import numpy
import typing

import numpy.typing
__all__ = ['ALREADY_RUN', 'ApiStatus', 'CANNOT_GET_RING_BUFFER', 'CONFIG_FILE_NOT_PARSED', 'Camera', 'CameraImageData', 'Can', 'CanMessageData', 'DIRECTORY_NOT_FOUND', 'FAILED_CLOSE_FILE', 'FAILED_SETUP_CAMERA', 'FAILED_SETUP_LIDAR', 'FAILED_SETUP_SOCKET_CAN', 'FAILED_WRITE_DATA', 'FILE_NOT_FOUND', 'FILE_NOT_OPEN', 'INVALID_ARGUMENT', 'INVALID_CAN_IF_NAME', 'INVALID_HANDLE', 'INVALID_STATE', 'Lidar', 'LidarImuData', 'LidarPointCloudArray', 'LidarPointCloudData', 'NOT_INITIALIZED', 'NOT_RUNNING', 'SUCCESS', 'TIMEOUT']
class ApiStatus:
    """
    Members:
    
      SUCCESS
    
      NOT_INITIALIZED
    
      NOT_RUNNING
    
      TIMEOUT
    
      INVALID_ARGUMENT
    
      INVALID_HANDLE
    
      INVALID_STATE
    
      INVALID_CAN_IF_NAME
    
      FILE_NOT_FOUND
    
      DIRECTORY_NOT_FOUND
    
      CONFIG_FILE_NOT_PARSED
    
      FAILED_SETUP_SOCKET_CAN
    
      FAILED_SETUP_CAMERA
    
      FAILED_SETUP_LIDAR
    
      FAILED_WRITE_DATA
    
      FAILED_CLOSE_FILE
    
      CANNOT_GET_RING_BUFFER
    
      ALREADY_RUN
    
      FILE_NOT_OPEN
    """
    ALREADY_RUN: typing.ClassVar[ApiStatus]  # value = <ApiStatus.ALREADY_RUN: 17>
    CANNOT_GET_RING_BUFFER: typing.ClassVar[ApiStatus]  # value = <ApiStatus.CANNOT_GET_RING_BUFFER: 16>
    CONFIG_FILE_NOT_PARSED: typing.ClassVar[ApiStatus]  # value = <ApiStatus.CONFIG_FILE_NOT_PARSED: 10>
    DIRECTORY_NOT_FOUND: typing.ClassVar[ApiStatus]  # value = <ApiStatus.DIRECTORY_NOT_FOUND: 9>
    FAILED_CLOSE_FILE: typing.ClassVar[ApiStatus]  # value = <ApiStatus.FAILED_CLOSE_FILE: 15>
    FAILED_SETUP_CAMERA: typing.ClassVar[ApiStatus]  # value = <ApiStatus.FAILED_SETUP_CAMERA: 12>
    FAILED_SETUP_LIDAR: typing.ClassVar[ApiStatus]  # value = <ApiStatus.FAILED_SETUP_LIDAR: 13>
    FAILED_SETUP_SOCKET_CAN: typing.ClassVar[ApiStatus]  # value = <ApiStatus.FAILED_SETUP_SOCKET_CAN: 11>
    FAILED_WRITE_DATA: typing.ClassVar[ApiStatus]  # value = <ApiStatus.FAILED_SETUP_LIDAR: 13>
    FILE_NOT_FOUND: typing.ClassVar[ApiStatus]  # value = <ApiStatus.FILE_NOT_FOUND: 8>
    FILE_NOT_OPEN: typing.ClassVar[ApiStatus]  # value = <ApiStatus.FILE_NOT_OPEN: 18>
    INVALID_ARGUMENT: typing.ClassVar[ApiStatus]  # value = <ApiStatus.INVALID_ARGUMENT: 4>
    INVALID_CAN_IF_NAME: typing.ClassVar[ApiStatus]  # value = <ApiStatus.INVALID_CAN_IF_NAME: 7>
    INVALID_HANDLE: typing.ClassVar[ApiStatus]  # value = <ApiStatus.INVALID_HANDLE: 5>
    INVALID_STATE: typing.ClassVar[ApiStatus]  # value = <ApiStatus.INVALID_STATE: 6>
    NOT_INITIALIZED: typing.ClassVar[ApiStatus]  # value = <ApiStatus.NOT_INITIALIZED: 1>
    NOT_RUNNING: typing.ClassVar[ApiStatus]  # value = <ApiStatus.NOT_RUNNING: 2>
    SUCCESS: typing.ClassVar[ApiStatus]  # value = <ApiStatus.SUCCESS: 0>
    TIMEOUT: typing.ClassVar[ApiStatus]  # value = <ApiStatus.TIMEOUT: 3>
    __members__: typing.ClassVar[dict[str, ApiStatus]]  # value = {'SUCCESS': <ApiStatus.SUCCESS: 0>, 'NOT_INITIALIZED': <ApiStatus.NOT_INITIALIZED: 1>, 'NOT_RUNNING': <ApiStatus.NOT_RUNNING: 2>, 'TIMEOUT': <ApiStatus.TIMEOUT: 3>, 'INVALID_ARGUMENT': <ApiStatus.INVALID_ARGUMENT: 4>, 'INVALID_HANDLE': <ApiStatus.INVALID_HANDLE: 5>, 'INVALID_STATE': <ApiStatus.INVALID_STATE: 6>, 'INVALID_CAN_IF_NAME': <ApiStatus.INVALID_CAN_IF_NAME: 7>, 'FILE_NOT_FOUND': <ApiStatus.FILE_NOT_FOUND: 8>, 'DIRECTORY_NOT_FOUND': <ApiStatus.DIRECTORY_NOT_FOUND: 9>, 'CONFIG_FILE_NOT_PARSED': <ApiStatus.CONFIG_FILE_NOT_PARSED: 10>, 'FAILED_SETUP_SOCKET_CAN': <ApiStatus.FAILED_SETUP_SOCKET_CAN: 11>, 'FAILED_SETUP_CAMERA': <ApiStatus.FAILED_SETUP_CAMERA: 12>, 'FAILED_SETUP_LIDAR': <ApiStatus.FAILED_SETUP_LIDAR: 13>, 'FAILED_WRITE_DATA': <ApiStatus.FAILED_SETUP_LIDAR: 13>, 'FAILED_CLOSE_FILE': <ApiStatus.FAILED_CLOSE_FILE: 15>, 'CANNOT_GET_RING_BUFFER': <ApiStatus.CANNOT_GET_RING_BUFFER: 16>, 'ALREADY_RUN': <ApiStatus.ALREADY_RUN: 17>, 'FILE_NOT_OPEN': <ApiStatus.FILE_NOT_OPEN: 18>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Camera:
    def __init__(self) -> None:
        ...
    def getRecordState(self, arg0: typing.SupportsInt) -> tuple[ApiStatus, bool]:
        ...
    def getSensorData(self, arg0: typing.SupportsInt, arg1: CameraImageData, arg2: typing.SupportsInt, arg3: bool) -> ApiStatus:
        ...
    def init(self, arg0: str) -> ApiStatus:
        ...
    def run(self) -> ApiStatus:
        ...
    def setRecordState(self, arg0: typing.SupportsInt, arg1: bool) -> ApiStatus:
        ...
class CameraImageData:
    def __init__(self) -> None:
        ...
    @property
    def height(self) -> int:
        ...
    @height.setter
    def height(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def image(self) -> numpy.typing.NDArray[numpy.uint8]:
        ...
    @image.setter
    def image(self) -> None:
        ...
    @property
    def pitch(self) -> int:
        ...
    @pitch.setter
    def pitch(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def size(self) -> int:
        ...
    @size.setter
    def size(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def timestamp_ms(self) -> int:
        ...
    @timestamp_ms.setter
    def timestamp_ms(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def width(self) -> int:
        ...
    @width.setter
    def width(self, arg0: typing.SupportsInt) -> None:
        ...
class Can:
    def __init__(self) -> None:
        ...
    def getRecordState(self, arg0: typing.SupportsInt) -> tuple[ApiStatus, bool]:
        ...
    def getSensorData(self, arg0: typing.SupportsInt, arg1: CanMessageData, arg2: typing.SupportsInt, arg3: bool) -> ApiStatus:
        ...
    def init(self, arg0: str) -> ApiStatus:
        ...
    def run(self) -> ApiStatus:
        ...
    def setRecordState(self, arg0: typing.SupportsInt, arg1: bool) -> ApiStatus:
        ...
class CanMessageData:
    def __init__(self) -> None:
        ...
    @property
    def data(self) -> numpy.typing.NDArray[numpy.uint8]:
        ...
    @data.setter
    def data(self) -> None:
        ...
    @property
    def id(self) -> int:
        ...
    @id.setter
    def id(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def size(self) -> int:
        ...
    @size.setter
    def size(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def timestamp_ms(self) -> int:
        ...
    @timestamp_ms.setter
    def timestamp_ms(self, arg0: typing.SupportsInt) -> None:
        ...
class Lidar:
    def __init__(self) -> None:
        ...
    def getRecordState(self, arg0: typing.SupportsInt) -> tuple[ApiStatus, bool]:
        ...
    @typing.overload
    def getSensorData(self, arg0: typing.SupportsInt, arg1: LidarPointCloudArray, arg2: typing.SupportsInt, arg3: bool) -> ApiStatus:
        ...
    @typing.overload
    def getSensorData(self, arg0: typing.SupportsInt, arg1: LidarImuData, arg2: typing.SupportsInt, arg3: bool) -> ApiStatus:
        ...
    def init(self, arg0: str) -> ApiStatus:
        ...
    def run(self) -> ApiStatus:
        ...
    def setRecordState(self, arg0: typing.SupportsInt, arg1: bool) -> ApiStatus:
        ...
class LidarImuData:
    def __init__(self) -> None:
        ...
    @property
    def acc_x(self) -> float:
        ...
    @acc_x.setter
    def acc_x(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def acc_y(self) -> float:
        ...
    @acc_y.setter
    def acc_y(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def acc_z(self) -> float:
        ...
    @acc_z.setter
    def acc_z(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def gyro_x(self) -> float:
        ...
    @gyro_x.setter
    def gyro_x(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def gyro_y(self) -> float:
        ...
    @gyro_y.setter
    def gyro_y(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def gyro_z(self) -> float:
        ...
    @gyro_z.setter
    def gyro_z(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def timestamp_ms(self) -> int:
        ...
    @timestamp_ms.setter
    def timestamp_ms(self, arg0: typing.SupportsInt) -> None:
        ...
class LidarPointCloudArray:
    def __init__(self) -> None:
        ...
    @property
    def data(self) -> typing.Annotated[list[LidarPointCloudData], "FixedSize(100)"]:
        ...
    @data.setter
    def data(self, arg0: typing.Annotated[collections.abc.Sequence[LidarPointCloudData], "FixedSize(100)"]) -> None:
        ...
    @property
    def data_num(self) -> int:
        ...
    @data_num.setter
    def data_num(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def timestamp_ms(self) -> int:
        ...
    @timestamp_ms.setter
    def timestamp_ms(self, arg0: typing.SupportsInt) -> None:
        ...
class LidarPointCloudData:
    def __init__(self) -> None:
        ...
    @property
    def reflectivity(self) -> int:
        ...
    @reflectivity.setter
    def reflectivity(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def x(self) -> int:
        ...
    @x.setter
    def x(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def y(self) -> int:
        ...
    @y.setter
    def y(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def z(self) -> int:
        ...
    @z.setter
    def z(self, arg0: typing.SupportsInt) -> None:
        ...
ALREADY_RUN: ApiStatus  # value = <ApiStatus.ALREADY_RUN: 17>
CANNOT_GET_RING_BUFFER: ApiStatus  # value = <ApiStatus.CANNOT_GET_RING_BUFFER: 16>
CONFIG_FILE_NOT_PARSED: ApiStatus  # value = <ApiStatus.CONFIG_FILE_NOT_PARSED: 10>
DIRECTORY_NOT_FOUND: ApiStatus  # value = <ApiStatus.DIRECTORY_NOT_FOUND: 9>
FAILED_CLOSE_FILE: ApiStatus  # value = <ApiStatus.FAILED_CLOSE_FILE: 15>
FAILED_SETUP_CAMERA: ApiStatus  # value = <ApiStatus.FAILED_SETUP_CAMERA: 12>
FAILED_SETUP_LIDAR: ApiStatus  # value = <ApiStatus.FAILED_SETUP_LIDAR: 13>
FAILED_SETUP_SOCKET_CAN: ApiStatus  # value = <ApiStatus.FAILED_SETUP_SOCKET_CAN: 11>
FAILED_WRITE_DATA: ApiStatus  # value = <ApiStatus.FAILED_SETUP_LIDAR: 13>
FILE_NOT_FOUND: ApiStatus  # value = <ApiStatus.FILE_NOT_FOUND: 8>
FILE_NOT_OPEN: ApiStatus  # value = <ApiStatus.FILE_NOT_OPEN: 18>
INVALID_ARGUMENT: ApiStatus  # value = <ApiStatus.INVALID_ARGUMENT: 4>
INVALID_CAN_IF_NAME: ApiStatus  # value = <ApiStatus.INVALID_CAN_IF_NAME: 7>
INVALID_HANDLE: ApiStatus  # value = <ApiStatus.INVALID_HANDLE: 5>
INVALID_STATE: ApiStatus  # value = <ApiStatus.INVALID_STATE: 6>
NOT_INITIALIZED: ApiStatus  # value = <ApiStatus.NOT_INITIALIZED: 1>
NOT_RUNNING: ApiStatus  # value = <ApiStatus.NOT_RUNNING: 2>
SUCCESS: ApiStatus  # value = <ApiStatus.SUCCESS: 0>
TIMEOUT: ApiStatus  # value = <ApiStatus.TIMEOUT: 3>
