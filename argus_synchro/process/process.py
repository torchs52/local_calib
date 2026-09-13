from __future__ import annotations

import contextlib
import fnmatch
import json
import multiprocessing as mp
import os
import time
from abc import ABC, abstractmethod
from multiprocessing import Process
from multiprocessing.synchronize import Event
from pathlib import Path
from typing import TYPE_CHECKING, Any, Generic, Self, TypeVar, cast, final

import psutil
from tqdm import tqdm

from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.config.app_config import AppConfig
from argus_synchro.core.closable import Closable, IClosable
from argus_synchro.process.message import Consumer, MessageFlow, Producer
from argus_synchro.process.synchronizer import ProcessActivator
from argus_synchro.profiler import ProfMode, ProfSharedReader
from argus_synchro.shared_excepts import SharedProcessExcept

if TYPE_CHECKING:
    from viztracer.viztracer import VizTracer

T = TypeVar("T")
IT = TypeVar("IT")
OT = TypeVar("OT")


def _is_trace_target_process(
    process_name: str,
    target_processes: tuple[str, ...],
) -> bool:
    if len(target_processes) == 0:
        return True
    return any(fnmatch.fnmatch(process_name, target) for target in target_processes)


def _to_trace_safe_name(name: str) -> str:
    return "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in name)


class CoreAffinityError(RuntimeError):
    pass


class ProcessCpuAffinityConfigError(RuntimeError):
    """CPU affinity JSONの読み込みまたは検証に失敗した場合に送出する。"""


class ProcessBase(IClosable, ABC):
    """
    プロセスの基底クラス

    * 複数入力、または複数出力のプロセスを作成する場合は、このクラスを継承する
    * 入出力が単一クラスの場合はTransformProcess、InputProcess、OutputProcessを継承する
    """

    __slots__ = (
        "_ProcessBase__flows",
        "_ProcessBase__process",
        "_directory_config",
        "_is_closed",
        "_is_restart_finish_wait",
        "_logger",
        "_loop_wait",
        "_spe",
        "_startup_wait",
        "name",
    )

    def __init__(
        self,
        spe: SharedProcessExcept,
        activator: ProcessActivator,
        name: str | None = None,
    ) -> None:
        super().__init__()
        self._is_closed = False
        self._spe: SharedProcessExcept = spe
        if name is None:
            name = self.__class__.__name__
        self.name: str = name
        self.__process: Process | None = None
        self.__flows: list[MessageFlow[Any]] = []
        self._process_activator: ProcessActivator = activator
        self._directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG
        self._startup_wait: Event = mp.Event()
        self._loop_wait: Event = mp.Event()
        # processを落とさないrestart処理が完了したことを知らせる変数
        self._is_restart_finish_wait: Event = mp.Event()
        self.affinity_cores: list[int] | None = None

    @property
    def is_alive(self) -> bool:
        if self.__process is None:
            return False
        return self.__process.is_alive()

    @property
    def exitcode(self) -> int | None:
        if self.__process is None:
            return None
        return self.__process.exitcode

    def _cpu_affinity_base(self) -> None:
        """プロセスにコア指定"""
        if self.pid is None or self.affinity_cores is None:
            raise CoreAffinityError(f"CPU affinity failed : name={self.name}")
        process_instance = psutil.Process(self.pid)

        try:
            process_instance.cpu_affinity(self.affinity_cores)
            actual = set(process_instance.cpu_affinity())
            if not set(self.affinity_cores).issubset(actual):
                raise CoreAffinityError(f"CPU affinity failed : name={self.name}")
        except (
            psutil.NoSuchProcess,
            psutil.AccessDenied,
            psutil.Error,
            ValueError,
        ) as e:
            raise CoreAffinityError(f"CPU affinity failed : name={self.name}") from e

    def set_cpu_affinity(self) -> None:
        self._cpu_affinity_base()

    def update_cpu_affinity(self) -> None:
        # NOTE: 生成済みのスレッドに対してコアを指定する機能は非対応
        self._cpu_affinity_base()

    @property
    def enable(self) -> bool:
        return (
            all(flow.activator.value for flow in self.__flows)
            and self._process_activator.value
            and not self._process_activator.is_restart_required
        )

    def _startup(self) -> None:
        pass

    def _log_register(self) -> None:
        pass

    def _set_logger(self, path: str | None) -> None:
        self._app_logger_factory = AppLoggerFactory(to_file=path)
        self._logger: AppLogger = self._app_logger_factory.register_from_name(self.name)
        self._log_register()
        self._app_logger_factory.update()

    def _shutdown(self) -> None:
        pass

    @abstractmethod
    def _loop(self) -> None: ...

    def _create_process_profiler(self) -> VizTracer | None:
        info = ProfSharedReader.get()
        if info.mode not in (ProfMode.VizTracereMain, ProfMode.VizTracereTarget):
            return None

        target_processes = info.target_processes
        if not _is_trace_target_process(self.name, target_processes):
            return None

        out_dir = Path(info.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        output_file = (
            out_dir / f"trace_{_to_trace_safe_name(self.name)}_{os.getpid()}.json"
        )

        from viztracer.viztracer import VizTracer

        tracer = VizTracer(
            tracer_entries=info.tracer_entries,
            verbose=0,
            output_file=str(output_file),
            max_stack_depth=info.max_stack_depth,
            ignore_c_function=info.ignore_c_function,
            log_sparse=True,
            pid_suffix=False,
            file_info=False,
            register_global=True,
            dump_raw=True,
            minimize_memory=info.minimize_memory,
        )
        self._logger.info(
            f"enabled process trace: name={self.name}, output={output_file}"
        )
        return tracer

    def _save_process_profiler(self, tracer: VizTracer) -> None:
        tracer.stop(stop_option="flush_as_finish")
        tracer.save()
        tracer.terminate()

    def _main(self, path: str, directory_config: paths.DirectoryConfig) -> None:
        tracer: VizTracer | None = None
        try:
            self._directory_config = directory_config
            self._set_logger(path)
            tracer = self._create_process_profiler()
            self._startup()
            self._startup_wait.set()
            self._loop_wait.wait()
            try:
                while True:
                    self._loop()
                    if not self._process_activator.is_restart_required:
                        break
                    self.start_restart()
                    self.create_producer_and_consumer()
                    self.restart_completed()
            except Exception as e:
                self._except(e)
            finally:
                self._shutdown()
        except Exception as e:
            self._except(e)
        finally:
            self._startup_wait.set()
            if tracer is not None:
                self._save_process_profiler(tracer)
            self.close()

    def _except(self, e: Exception) -> None:
        if not self._spe.IsFinished.value:
            self._logger.exception(
                f"class: {self.__class__.__name__}, except: {e}",
            )
        self._spe.IsFinished.value = True

    def _subscribe(self, flow: MessageFlow[T]) -> MessageFlow[T]:
        self.__flows.append(flow)
        return flow

    def _unsubscribe(self) -> None:
        logger = getattr(self, "_logger", None)
        if logger is not None:
            logger.warning("%s unsubscribing message flows", self.name)
        self._startup_wait.set()
        self._loop_wait.set()
        for flow in self.__flows:
            flow.synchronizer.stop()

    def add_to(self, manager: ProcessManager) -> Self:
        manager.append(self)
        return self

    @abstractmethod
    def create_producer_and_consumer(self) -> None: ...

    @abstractmethod
    def restart_completed(self) -> None: ...

    def _start_log(self, process: Process, path: str) -> None:
        factory = AppLoggerFactory(to_file=path)
        _logger = factory.register_from_name(self.name)
        _logger.info(f"{process.name}Start")
        _logger.info(f"{process.name} started pid={process.pid}")

    def start(self, path: str, directory_config: paths.DirectoryConfig) -> None:
        self._startup_wait.clear()
        self._loop_wait.clear()
        self.__process = Process(
            target=self._main, args=(path, directory_config), name=self.name
        )
        self.__process.start()
        self._start_log(self.__process, path)

    def stop(self) -> None:
        self._unsubscribe()
        if self.__process:
            self.__process.join()

    def join(self) -> None:
        if self.__process:
            self.__process.join()
        self._unsubscribe()

    def kill(self, *, unsubscribe: bool = True) -> None:
        if self.__process:
            self.__process.kill()
        if unsubscribe:
            self._unsubscribe()

    def terminate(self, *, unsubscribe: bool = True) -> None:
        if self.__process:
            self.__process.terminate()
        if unsubscribe:
            self._unsubscribe()

    def close(self) -> None:
        if self._is_closed:
            return
        for flow in self.__flows:
            flow.close()
        self._is_closed = True

    def wait_start(self, timeout: float | None = None) -> bool:
        return self._startup_wait.wait(timeout=timeout)

    def wait_restart(self, timeout: float | None = None) -> bool:
        return self._is_restart_finish_wait.wait(timeout=timeout)

    def start_loop(self) -> None:
        self._loop_wait.set()

    def stop_loop(self) -> None:
        self._loop_wait.set()

    def restart(self) -> None:
        if self.__process:
            self._is_restart_finish_wait.clear()
            self._loop_wait.clear()
            self._process_activator.require_restart()

    def start_restart(self) -> None:
        self._start_restart()
        if self.__process:
            self._logger.info(f"{self.__process.name} finished restarting")
            self._is_restart_finish_wait.set()
            self._loop_wait.wait()

    @abstractmethod
    def _start_restart(self) -> None: ...

    """実際にプロセスを落とさないで再起動で実行する処理"""

    @property
    def pid(self) -> int | None:
        if self.__process is None:
            return None
        return self.__process.pid

    def start_diagnosis(self) -> None:
        pass

    def stop_diagnosis(self) -> None:
        pass

class TransformProcess(ProcessBase, ABC, Generic[IT, OT]):
    """
    入出力が単一クラスのプロセス

    * 抽象メソッド_updateをオーバーライドして、処理を実装する
    * 入出力はこのクラスによって自動的に処理される
    """

    __slots__ = ("_TransformProcess__consumer_flow", "_TransformProcess__producer_flow")

    def __init__(
        self,
        consumer_flow: MessageFlow[IT],
        producer_flow: MessageFlow[OT],
        spe: SharedProcessExcept,
        activator: ProcessActivator,
        name: str | None = None,
    ) -> None:
        super().__init__(spe, activator, name=name)
        self.__consumer_flow: MessageFlow[IT] = self._subscribe(consumer_flow)
        self.__producer_flow: MessageFlow[OT] = self._subscribe(producer_flow)

    @abstractmethod
    def _update(self, input_data: IT) -> OT | None: ...

    def _loop(self) -> None:
        consumer: Consumer[IT] = self.__consumer_flow.create_consumer()
        producer: Producer[OT] = self.__producer_flow.create_producer()
        try:
            while self.enable:
                if not consumer.wait():
                    continue
                with consumer.consume() as input_data:
                    if not producer.wait():
                        continue
                    output_data: OT | None = self._update(input_data)
                if output_data is None:
                    continue

                producer.produce(output_data)
        finally:
            self.close()


class InputProcess(ProcessBase, ABC, Generic[OT]):
    """
    入力データを生成するプロセス

    * 抽象メソッド_updateをオーバーライドして、入力データの生成処理を実装する
    """

    __slots__ = ("_producer_flow",)

    def __init__(
        self,
        producer_flow: MessageFlow[OT],
        spe: SharedProcessExcept,
        activator: ProcessActivator,
        name: str | None = None,
    ) -> None:
        super().__init__(spe, activator, name=name)
        self._producer_flow: MessageFlow[OT] = self._subscribe(producer_flow)

    @abstractmethod
    def _update(self) -> OT | None: ...

    def _loop(self) -> None:
        producer: Producer[OT] = self._producer_flow.create_producer()
        try:
            while self.enable:
                if not producer.wait():
                    continue
                output_data: OT | None = self._update()
                if output_data is None:
                    continue
                producer.produce(output_data)
        finally:
            self.close()


class OutputProcess(ProcessBase, ABC, Generic[IT]):
    """
    出力のみのプロセス

    * 抽象メソッド_updateをオーバーライドして、出力処理を実装する
    """

    __slots__ = ("_OutputProcess__consumer_flow",)

    def __init__(
        self,
        consumer_flow: MessageFlow[IT],
        spe: SharedProcessExcept,
        activator: ProcessActivator,
        name: str | None = None,
    ) -> None:
        super().__init__(spe, activator, name=name)
        self.__consumer_flow: MessageFlow[IT] = self._subscribe(consumer_flow)

    @abstractmethod
    def _update(self, input_data: IT) -> None: ...

    def _loop(self) -> None:
        consumer: Consumer[IT] = self.__consumer_flow.create_consumer()
        try:
            while self.enable:
                if not consumer.wait():
                    continue
                with consumer.consume() as input_data:
                    self._update(input_data)
        finally:
            self.close()


@final
class ProcessManager(Closable):
    """
    プロセスの管理を行うクラス

    * プロセスの起動、停止、終了を行う
    * プロセスの状態を管理する
    """

    __slots__ = ("_ProcessManager__processes", "_process_activator")

    STARTUP_TIMEOUT_SEC = 6000

    def __init__(
        self, process_activator: ProcessActivator, app_logger: AppLogger
    ) -> None:
        super().__init__()
        self.__processes: list[ProcessBase] = []
        self._process_activator: ProcessActivator = process_activator
        self._logger: AppLogger = app_logger

    def append(self, process: ProcessBase) -> None:
        self.__processes.append(process)

    def get_process_cpu_affinity(
        self, json_path: str, process_name: str
    ) -> list[int] | None:
        try:
            with open(json_path) as json_file:
                process_cpu_affinity: object = json.load(json_file)
        except (OSError, json.JSONDecodeError, UnicodeError, TypeError, ValueError) as error:
            raise ProcessCpuAffinityConfigError(
                f"read process CPU affinity JSON failed: "
                f"path={json_path}, error={type(error).__name__}: {error}"
            ) from error
        if not isinstance(process_cpu_affinity, dict):
            raise ProcessCpuAffinityConfigError(
                f"process CPU affinity JSON root must be an object: path={json_path}"
            )
        affinity_cores: object = process_cpu_affinity.get(process_name)
        if (
            not isinstance(affinity_cores, list)
            or not affinity_cores
            or not all(type(core) is int for core in affinity_cores)
        ):
            raise ProcessCpuAffinityConfigError(
                f"process CPU affinity entry is missing or invalid: "
                f"path={json_path}, process={process_name}"
            )

        return cast(list[int], affinity_cores)

    def update_cpu_affinity(self, app_config: AppConfig) -> None:
        if not app_config.General.enable_cpu_affinity:
            return

        for process in self.__processes:
            process.affinity_cores = self.get_process_cpu_affinity(
                app_config.General.process_cpu_affinity_path, process.name
            )

            if process.affinity_cores is not None:
                process.update_cpu_affinity()

    def start(
        self,
        log_path: str,
        directory_config: paths.DirectoryConfig,
        app_config: AppConfig | None = None,
    ) -> None:
        try:
            with tqdm(
                total=len(self.__processes),
                desc="process startup",
                unit="proc",
                dynamic_ncols=True,
            ) as pbar:
                for process in self.__processes:
                    pbar.set_postfix_str(f"{process.name}")
                    if (
                        app_config is not None
                        and app_config.General.enable_cpu_affinity
                    ):
                        process.affinity_cores = self.get_process_cpu_affinity(
                            app_config.General.process_cpu_affinity_path, process.name
                        )

                    process.start(log_path, directory_config)

                    if process.affinity_cores is not None:
                        process.set_cpu_affinity()

                for process in self.__processes:
                    pbar.set_postfix_str(f"{process.name}")
                    if not process.wait_start(timeout=self.STARTUP_TIMEOUT_SEC):
                        raise RuntimeError(
                            f"process startup timeout: name={process.name}"
                        )
                    pbar.update(1)
                    if process.affinity_cores is None:
                        msg = f"[process startup] name={process.name} pid={process.pid}"
                    else:
                        msg = f"[process startup] name={process.name} pid={process.pid} coreid={process.affinity_cores}"
                    pbar.write(msg)

        finally:
            for process in self.__processes:
                process.start_loop()

    def stop(self) -> None:
        for process in self.__processes:
            process.stop()

    def restart(self) -> None:
        for process in self.__processes:
            process.restart()
        self._process_activator.require_restart()
        for process in self.__processes:
            process.wait_restart()
        self._process_activator.restart_completed()
        for process in self.__processes:
            process.start_loop()

    def graceful_stop_all(
        self,
        *,
        t_grace: float = 3.0,
        t_term: float = 3.0,
        t_kill: float = 2.0,
    ) -> tuple[list[ProcessBase], list[ProcessBase]]:
        for process in self.__processes:
            self._logger.info(
                f"[{process.name}] state: alive={process.is_alive} pid={process.pid}"
            )
        # --- Phase 1: 優雅停止（自発終了待ち） ---
        self._process_activator.disable()
        self._logger.info(f"phase=graceful wait {t_grace}s (parallel)")
        deadline: float = time.perf_counter() + t_grace
        self._phase_wait_until(deadline, self.__processes)

        # --- Phase 2: terminate 同時送信 ---
        still_alive_term: list[ProcessBase] = [
            p for p in self.__processes if p.is_alive
        ]
        if still_alive_term:
            self._logger.info(f"phase=terminate {len(still_alive_term)} procs")
            for p in still_alive_term:
                if p.is_alive:
                    self._logger.info(f"[{p.name}] sending terminate (pid={p.pid})")
                    try:
                        p.terminate(unsubscribe=False)
                    except Exception as e:
                        self._logger.error(f"[{p.name}] terminate error: {e!r}")
            deadline = time.perf_counter() + t_term
            self._phase_wait_until(deadline, still_alive_term)

        # --- Phase 3: kill 同時送信 ---
        still_alive_kill: list[ProcessBase] = [
            p for p in self.__processes if p.is_alive
        ]
        if still_alive_kill:
            self._logger.info(f"phase=kill {len(still_alive_kill)} procs")
            for p in still_alive_kill:
                if p.is_alive:
                    self._logger.info(f"[{p.name}] sending kill (pid={p.pid})")
                    try:
                        p.kill(unsubscribe=False)
                    except Exception as e:
                        self._logger.error(f"[{p.name}] kill error: {e!r}")
            deadline = time.perf_counter() + t_kill
            self._phase_wait_until(deadline, still_alive_kill)

        # 終了コードログ
        for p in self.__processes:
            with contextlib.suppress(Exception):
                self._logger.info(f"[{p.name}] exitcode={p.exitcode}")
        return still_alive_term, still_alive_kill

    def _phase_wait_until(self, deadline: float, procs: list[ProcessBase]) -> None:
        """deadline まで 100ms 間隔で生存確認をポーリング"""
        while time.perf_counter() < deadline:
            if all(not p.is_alive for p in procs):
                return
            time.sleep(0.1)

    def join(self) -> None:
        for process in self.__processes:
            process.join()

    def kill(self) -> None:
        for process in self.__processes:
            process.kill()

    def _close(self) -> None:
        for process in self.__processes:
            process.close()

    def start_diagnosis(self) -> None:
        for process in self.__processes:
            process.start_diagnosis()

    def stop_diagnosis(self) -> None:
        for process in self.__processes:
            process.stop_diagnosis()
