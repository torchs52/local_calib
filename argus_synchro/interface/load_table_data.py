from abc import ABC, abstractmethod

import pandas as pd

from argus_synchro import SubScrutinizer


class LoadTableDataInterface(ABC):
    @abstractmethod
    def get_angle_data(
        self,
        can_data: pd.DataFrame,
        is_old: bool,
    ) -> pd.Series: ...

    @abstractmethod
    def get_raw_table_data(self, c_filepath: str) -> pd.DataFrame: ...


class LoadFileTableData(LoadTableDataInterface):
    def get_angle_data(
        self,
        can_data: pd.DataFrame,
        is_old: bool,
    ) -> pd.Series:
        if can_data.empty:
            return pd.Series()

        return SubScrutinizer.get_angle_data(
            can_data,
            is_old,
        )

    def get_raw_table_data(
        self,
        c_filepath: str,
    ) -> pd.DataFrame:
        if c_filepath != "None":
            return pd.read_csv(c_filepath)

        # ファイル名Noneの時
        return pd.DataFrame()
