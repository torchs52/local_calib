from __future__ import annotations

import socket
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import CANConf
from argus_synchro.device.can.can_decoders import DECODER_REGISTRY, DecoderFn
from argus_synchro.diagnosis.error_diagnosis import ResultDiagnosis
from argus_synchro.shared_errors import SharedErrors, StateErrorIndex
from argus_synchro.shared_excepts import SharedCANExcept

if TYPE_CHECKING:
    import pandas as pd

CONVERTER_IP = "192.168.1.100"
CONVERTER_PORT = 2000
MASK_PGN = 0x1F
YAW_ANGLE_SIGNAL = "yaw_angle"
LEVER_PRESSURE_SIGNAL = "lever_pressure"
SUPPORTED_SIGNAL_TYPES = {YAW_ANGLE_SIGNAL, LEVER_PRESSURE_SIGNAL}


@dataclass(frozen=True, slots=True)
class DecodedCanMessage:
    can_id: str
    signal_type: str
    values: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class CanMapEntry:
    signal_type: str
    decoder: DecoderFn


class CanIdMapError(ValueError):
    def __init__(self, file_path: str, error: Exception) -> None:
        self.file_path = file_path
        super().__init__(
            f"CAN ID map error: path={file_path}, "
            f"detail={type(error).__name__}: {error}"
        )


def normalize_can_id(can_id: str) -> str:
    return can_id.strip().upper().removeprefix("0X")


def handle_on_new_receive_return_canstr(
    bin_str: bytes,
    ip_address: str,
    port: int,
) -> tuple[bool, str, str]:
    """
    Parameters
    ----------
    bin_str : bytes
        J1939 コンバータから受信した生バイト列
    ip_address : str
        送信元 IP
    port : int
        送信元ポート

    Returns
    -------
    tuple[bool, Optional[str], Optional[str]]
        (成功フラグ, PGN 文字列, CAN データ文字列)
        マッチしない場合は (False, None, None)
    """

    # global pre_rotation_degree
    # CONVERTER_IP = ip_address #localhost経由でテストする際コメントアウト
    # CONVERTER_PORT = port
    if ip_address == CONVERTER_IP and port == CONVERTER_PORT:
        # バイト列 → 可変長リスト[int]
        bin_table: list[int] = list(bin_str)

        # Mask for deleting extra bit which is added by J1939 converter
        bin_table[0] &= MASK_PGN

        # Check PGNs (Priority, Reserved, Data Page, PDU Format, PDU Specific)

        # TODO: ここ常にTrueで良い?
        is_pgn1_matched: bool = (
            True  # all(bin_table[j] == HSC_CRANE_PGN1[j] for j in range(4))
        )
        is_pgn2_matched: bool = (
            True  # all(bin_table[j] == HSC_CRANE_PGN2[j] for j in range(4))
        )

        # Process rotation angle data
        if not (is_pgn1_matched or is_pgn2_matched):
            return (False, None, None)

        # 0-3 バイト: PGN, 4-7 バイト: データ
        pgn_string: str = "".join(f"{byte:02X}" for byte in bin_table[0:4])
        can_string: str = "".join(f"{byte:02X}" for byte in bin_table[4:])

        return (True, pgn_string, can_string)
    return (False, "", "")


class Can:
    def __init__(
        self,
        index: int,
        can: CANConf,
        crane_model: str,
        app_logger_factory: AppLoggerFactory,
        ser: SharedErrors,
        sec_can: SharedCANExcept,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._index: int = index
        self._handler: CanHandler = CanHandler(can, crane_model, app_logger_factory)
        self.failedcount = 0
        self.udp_socket: socket.socket = self._create_udp_socket()
        self._ser: SharedErrors = ser
        self._sec_can: SharedCANExcept = sec_can
        self._last_received_time: float = 0.0
        self._timestamp: float | None = None
        self._err_config_load()

    def _err_config_load(self) -> None:
        self._err_config = self._ser.shared_err_conf.read()

        self._ser.state_errors_A_C[StateErrorIndex.CAN_COMM_QUALITY_DEGRADED].update(
            self._err_config
        )
        self._ser.state_errors_A_C[StateErrorIndex.CAN_COMM_QUALITY_ERROR].update(
            self._err_config
        )
        self._ser.state_errors_A_C[StateErrorIndex.CAN_INVALID_DATA_DIAGNOSIS].update(
            self._err_config
        )
        self._ser.state_errors_A_C[StateErrorIndex.CAN_INVALID_DATA_DIAGNOSIS].update(
            self._err_config
        )

    def _create_udp_socket(self) -> socket.socket:
        # Set up UDP
        sock: socket.socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        sock.bind(("0.0.0.0", 2000))  # Bind to all interfaces
        sock.settimeout(0.1)
        return sock

    def receive_can_data(self) -> DecodedCanMessage | None:
        """
        Returns
        -------
        can_id_str : str
            受信したCANIDを16進数で表した文字列
        recv_data : tuple[float]
            受信したデータを物理値にした値のタプル(1メッセージから複数の値を取得する場合のためタプル)
        """
        # self._logger.info("receive_start_without_lib start")
        decoded_message = None
        data: bytes = b""
        (ip_address, port) = ("", 0)

        sucsess: bool = False
        is_data_received: bool = False
        canid: str = ""
        candata: str = ""
        try:
            data, (ip_address, port) = self.udp_socket.recvfrom(1024)
        except TimeoutError:
            self.failedcount += 1
        else:
            sucsess = True
            self.failedcount = 0
            self._timestamp = time.perf_counter()
            self._sec_can.last_received.value = self._timestamp
            is_data_received, canid, candata = handle_on_new_receive_return_canstr(
                data,
                ip_address,
                port,
            )

        now: float = time.perf_counter()
        can_comm_quality_degraded = self._ser.state_errors_A_C[
            StateErrorIndex.CAN_COMM_QUALITY_DEGRADED
        ]
        result = can_comm_quality_degraded.errors_diagnosis(
            canid,
            self.failedcount,
            self._timestamp,
            now,
        )
        can_comm_quality_degraded.log_output(
            *result, StateErrorIndex.CAN_COMM_QUALITY_DEGRADED
        )

        can_comm_quality_error = self._ser.state_errors_A_C[
            StateErrorIndex.CAN_COMM_QUALITY_ERROR
        ]
        result = can_comm_quality_error.errors_diagnosis(
            canid,
            self.failedcount,
            self._timestamp,
            now,
        )
        can_comm_quality_error.log_output(
            *result, StateErrorIndex.CAN_COMM_QUALITY_ERROR
        )

        if sucsess:
            can_invalid_data = self._ser.state_errors_A_C[
                StateErrorIndex.CAN_INVALID_DATA_DIAGNOSIS
            ]
            result: tuple[ResultDiagnosis, ResultDiagnosis] = (
                can_invalid_data.errors_diagnosis(canid, candata, now)
            )
            can_invalid_data.log_output(
                *result, StateErrorIndex.CAN_INVALID_DATA_DIAGNOSIS
            )
            candata_invalid: ResultDiagnosis = result[0]

            if is_data_received and candata_invalid in (
                ResultDiagnosis.NORMAL,
                ResultDiagnosis.RECOVERY,
            ):
                # resultdatから対応するアドレスのhandlerを呼び出してdecodeする
                decoded_message = self._handler.dispatch(canid, candata)
            else:
                pass
                # rts = "udp_socket - handle_on_new_receive error"
                # self._logger.info("setRecordState error. status:%d", rts)

        return decoded_message

    def get_yaw_angle_deg(self) -> int:
        err_msg = f"class: {self.__class__.__name__}, method: get_yaw_angle_deg()"
        raise NotImplementedError(err_msg)


class CanHandler:
    """
    CANフレームを受信し、IDごとに登録されたハンドラへディスパッチする
    """

    def __init__(
        self,
        can_conf: CANConf,
        crane_model: str,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._crane_model = crane_model
        self.update(can_conf)

    def update(self, can_conf: CANConf) -> None:
        try:
            self._load_map(can_conf)
        except CanIdMapError:
            raise
        except (OSError, UnicodeError, ValueError, TypeError, KeyError) as error:
            raise CanIdMapError(can_conf.can_id_map_file, error) from error

    def _load_map(self, can_conf: CANConf) -> None:
        import pandas as pd

        self._logger.info("CanHandler start")

        can_id_map: pd.DataFrame = pd.read_csv(  # type: ignore
            can_conf.can_id_map_file,
            dtype=str,
            comment="#",
        )
        self._yaw_offset_deg: float = can_conf.yaw_offset_deg

        self._logger.info(f"CanHandler - can_id_map: {can_id_map}")

        required_columns = {"crane_model", "can_id", "signal_type", "decoder"}
        missing_columns = required_columns.difference(can_id_map.columns)
        if missing_columns:
            missing = ", ".join(sorted(missing_columns))
            raise ValueError(f"Missing CAN ID map columns: {missing}")

        model_column = can_id_map["crane_model"].astype(str).str.strip()
        common_map = can_id_map[model_column == "*"]
        model_map = can_id_map[model_column == self._crane_model]
        if common_map.empty and model_map.empty:
            raise ValueError(
                f"CAN ID map has no entries for crane_model: {self._crane_model}"
            )

        self._handler_map: dict[str, CanMapEntry] = {}
        for selected_map in (common_map, model_map):
            normalized_ids = selected_map["can_id"].map(
                lambda value: normalize_can_id(str(value))
            )
            duplicate_ids = normalized_ids[normalized_ids.duplicated()].unique()
            if len(duplicate_ids) > 0:
                duplicate = str(duplicate_ids[0])
                raise ValueError(f"Duplicate CAN ID in csv: {duplicate}")

        model_can_ids = {
            normalize_can_id(str(can_id)) for can_id in model_map["can_id"]
        }
        model_signal_types = {
            str(signal_type).strip() for signal_type in model_map["signal_type"]
        }
        common_map = common_map[
            ~common_map["can_id"].map(
                lambda value: normalize_can_id(str(value)) in model_can_ids
            )
            & ~common_map["signal_type"].astype(str).str.strip().isin(
                model_signal_types
            )
        ]
        selected_map = pd.concat([common_map, model_map], ignore_index=True)
        for _, key, signal_type, funcname in selected_map.itertuples(index=False):
            signal_type = str(signal_type).strip()
            self._logger.info(
                "CanHandler - (key,signal_type,funcname): (%s, %s, %s)",
                key,
                signal_type,
                funcname,
            )
            method = DECODER_REGISTRY.get(str(funcname).strip())
            if method is None:
                err_msg = f"Unknown CAN decoder function in csv: {funcname}"
                raise ValueError(err_msg)
            can_id = normalize_can_id(str(key))
            if not signal_type:
                raise ValueError(f"Empty CAN signal type in csv: {can_id}")
            if signal_type not in SUPPORTED_SIGNAL_TYPES:
                raise ValueError(f"Unknown CAN signal type in csv: {signal_type}")
            self._handler_map[can_id] = CanMapEntry(signal_type, method)

        yaw_angle_count = sum(
            entry.signal_type == YAW_ANGLE_SIGNAL
            for entry in self._handler_map.values()
        )
        if yaw_angle_count != 1:
            raise ValueError("CAN ID map must contain exactly one yaw_angle signal")

        self._logger.info(f"CanHandler - self._handler_map: {self._handler_map}")

    def can_id_for(self, signal_type: str) -> str:
        matching_ids = [
            can_id
            for can_id, entry in self._handler_map.items()
            if entry.signal_type == signal_type
        ]
        if len(matching_ids) != 1:
            raise ValueError(
                f"CAN ID map must contain exactly one {signal_type} signal"
            )
        return matching_ids[0]

    def dispatch(self, can_id: str, data: str) -> DecodedCanMessage | None:
        """受信フレームを適切なハンドラへ振り分け"""
        can_id_str = normalize_can_id(can_id)

        # 登録済みハンドラーに存在すれば呼び出し
        entry = self._handler_map.get(can_id_str)
        if entry is None:
            return None

        return DecodedCanMessage(
            can_id=can_id_str,
            signal_type=entry.signal_type,
            values=entry.decoder(data, self._yaw_offset_deg, self._logger),
        )


class LoadTableDataInterface(ABC):
    @abstractmethod
    def get_angle_data(
        self,
        can_data: pd.DataFrame,
        is_old: bool,
    ) -> pd.Series: ...

    @abstractmethod
    def get_raw_table_data(self, c_filepath: str) -> pd.DataFrame: ...

    @abstractmethod
    def get_lever_data(self, can_data: pd.DataFrame) -> pd.Series: ...


class LoadFileTableData(LoadTableDataInterface):
    def get_angle_data(
        self,
        can_data: pd.DataFrame,
        is_old: bool,
    ) -> pd.Series:
        import pandas as pd

        if can_data.empty:
            angle_data: pd.Series = pd.Series()

        else:
            angle_data = self._get_angle_data(
                can_data,
                is_old,
            )

        return angle_data

    def _get_angle_data(self, can_df: pd.DataFrame, is_old: bool) -> pd.Series:
        if is_old:
            angle_data: pd.Series = can_df["o_msg"]
        else:
            angle_data: pd.Series = can_df["n_msg"]

        return angle_data

    def get_raw_table_data(
        self,
        c_filepath: str,
    ) -> pd.DataFrame:
        import pandas as pd

        if c_filepath != "None":
            angle_data: pd.DataFrame = pd.read_csv(c_filepath)  # type: ignore
        else:
            # ファイル名Noneの時
            angle_data = pd.DataFrame()
        return angle_data

    def get_lever_data(self, can_data: pd.DataFrame) -> pd.Series:
        import pandas as pd

        if "lever_msg" in can_data.columns:
            lever_data: pd.Series = can_data["lever_msg"]
        else:
            lever_data = pd.Series()
        return lever_data


class CanFile:
    def __init__(
        self,
        can_conf: CANConf,
        crane_model: str,
        app_logger_factory: AppLoggerFactory,
    ) -> None:
        """
        argus_synchro
        Scrutinizerクラスの__init__を参考に
        """
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.update(can_conf)
        self._handler = CanHandler(can_conf, crane_model, app_logger_factory)
        # ファイル読み込みの時は、最初にテーブルデータ読み込み.
        self.load_table_data: LoadTableDataInterface = LoadFileTableData()

        can_data: pd.DataFrame = self.load_table_data.get_raw_table_data(
            self.c_filepath,
        )
        self.angle_data: pd.Series = self.load_table_data.get_angle_data(
            can_data=can_data,
            is_old=self.is_old,
        )

    def update(self, can_conf: CANConf) -> None:
        """CAN設定を更新する"""
        self._yaw_offset_deg: float = can_conf.yaw_offset_deg

        self.is_old = can_conf.IsOld
        self.c_filepath = can_conf.c_file
        if hasattr(self, "_handler"):
            self._handler.update(can_conf)

    def pick_row(self, df: pd.Series, row_num: int) -> str:
        return df[row_num]

    @staticmethod
    def _normalize_raw_msg(raw_msg: object) -> str | None:
        if raw_msg is None:
            return None

        msg = str(raw_msg).strip().upper().removeprefix("0X")
        if not msg or msg == "NAN" or len(msg) % 2 != 0:
            return None
        if any(character not in "0123456789ABCDEF" for character in msg):
            return None
        return msg

    def receive_can_data(self, ref_t: int) -> DecodedCanMessage | None:
        """
        ファイルからyaw_angle_dataを呼んで角度を返す

        Scrutinizerクラスの607行目でコールしている関数・クラスを参考に
        """

        """
        Parameters
        ----------
        sc_can : SharedClasses.Shared_can
            共有メモリ (yaw 角などを保持)
        can_conf : CANConf
            システム設定
        ls_can_det : LShared_can
            呼び出し元保持の構造体 (結果を書き戻す)
        angle_data : pandas.DataFrame
            CANログを読み込んだDataFrame
        ref_t : int
            何行目をデコードするか

        Returns
        -------
        LShared_can
            更新済み構造体
        """
        # 1) ログが空ならオフセットだけ設定して終了
        if self.angle_data.empty:
            return None

        # 3) 対象行を抽出しmsg文字列を取り出す (列名はサブクラス依存)
        raw_msg: str = self.pick_row(self.angle_data, ref_t)
        # raw_msg: str = row_dict[self.msg_col]
        normalized_msg = self._normalize_raw_msg(raw_msg)
        if normalized_msg is None:
            self._logger.warning(
                "CanFile - invalid msg row. ref_t=%s raw_msg=%r",
                ref_t,
                raw_msg,
            )
            return None

        return self._handler.dispatch(
            self._handler.can_id_for(YAW_ANGLE_SIGNAL),
            normalized_msg,
        )

    def change_file_name_index(self, file_name: str) -> None:
        self.c_filepath = file_name
        can_data: pd.DataFrame = self.load_table_data.get_raw_table_data(
            self.c_filepath,
        )
        self.angle_data: pd.Series = self.load_table_data.get_angle_data(
            can_data=can_data,
            is_old=self.is_old,
        )
