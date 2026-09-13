from __future__ import annotations

import faulthandler  # シグナル受信時にスタックトレース出力を有効化。実用時困るようなら無効化のこと。
import multiprocessing
import multiprocessing.sharedctypes
import signal
import time
from abc import ABC, abstractmethod
from typing import Self

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory


class MultiSensorManagerBase(ABC):
    def __init__(
        self,
        num_processes,
        app_logger_factory: AppLoggerFactory,
    ):
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self.num_processes: int = num_processes
        self.processes: list[multiprocessing.Process] = []
        self.loopEnable: multiprocessing.sharedctypes.Synchronized[bool] = (
            multiprocessing.Value("i", 1)
        )  # 0 or 1 0でループを抜ける（→フリーズ等でない限りjoinを抜けられるようになる）

    def __del__(self):
        try:
            self._logger.info("__del__ called")
            self.join_all(timeout=1)
            self.terminate_all()
        except Exception:
            pass

    def terminate_all(self):
        self._logger.info("terminate_all called")
        self.loopEnable.value = False
        for p in self.processes:
            if p.is_alive():
                p.terminate()
            p.join(timeout=0.1)

        self.processes: list[multiprocessing.Process] = []

    def join_all(self, timeout=None):
        self._logger.info("join_all called")
        self.loopEnable.value = False
        if timeout is None:
            is_alive = True
            while is_alive:
                is_alive = False
                for ix, p in enumerate(self.processes):
                    if p.is_alive():
                        is_alive = True
                        print(f"run(wait - join): waiting... sensorbase {ix}")
                time.sleep(1)
        for p in self.processes:
            p.join(timeout)

    @abstractmethod
    def add_taskinfo(self, ix: int, argsK: dict) -> dict:
        """
        マネージャ側： 該当センサ番号のプロセス開始直前に呼ばれ、プロセス作成のための追加情報を登録。
        ix: センサ番号、argsK: run関数で与えられる引数名と引数のdict　※この引数で追加・削除等加工して返すこと
        """
        return argsK

    def run(self, argsK_list: list[dict], target, name_suffix: str = ""):
        print(f"run process, {argsK_list}")
        self.loopEnable.value = True
        for ix, argsK in enumerate(argsK_list):
            argsK["task_index"] = ix
            argsK = self.add_taskinfo(ix, argsK)
            print(f"new process start, {ix, argsK}")
            p = multiprocessing.Process(
                target=target, kwargs=argsK, name=f"worker{ix}_{name_suffix}"
            )
            self.processes.append(p)
            p.start()
            self._logger.info(
                f"starting worker process: worker{ix}_{name_suffix}, pid: {p.pid}"
            )


class MultiSensorWorkerBase(ABC):
    @classmethod
    def entrypoint(cls, *args, **kwargs) -> None:
        # SIGUSR1を受け取ったらスタックトレースを出力
        faulthandler.register(
            signal.SIGUSR1
        )  # 指定したシグナルでスタックトレースを出力。
        faulthandler.enable()  # 致命的なエラー時に自動でスタックトレースを出力。

        inst: Self = cls(*args, **kwargs)
        inst.task()

    @abstractmethod
    def task(self):
        pass
