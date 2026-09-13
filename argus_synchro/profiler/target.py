from __future__ import annotations

import sys
from collections.abc import Callable
from types import TracebackType
from typing import Any, Self

from viztracer import get_tracer
from viztracer.vizevent import VizEvent
from viztracer.viztracer import VizTracer

from argus_synchro.profiler.prof_info import ProfInfo
from argus_synchro.profiler.prof_mode import ProfCategory, ProfMode
from argus_synchro.profiler.shared import ProfSharedReader

log_format: str = "[T]:{name} [{path}]"
frame_format: str = "[Frame]{name}"


def log_main() -> Callable[..., Any]:
    def _log_main(func: Callable[..., Any]) -> Callable[..., Any]:
        info = ProfSharedReader.get()
        if info.mode not in (ProfMode.VizTracereMain, ProfMode.VizTracereTarget):
            return func

        def wrapper(*args: object, **kwargs: object) -> object:
            tracer = get_tracer()
            if tracer is None or not tracer.log_sparse:
                return func(*args, **kwargs)
            if info.mode == ProfMode.VizTracereMain:
                tracer.start()
                ret = func(*args, **kwargs)
                tracer.stop()
                return ret

            start = tracer.getts()
            ret = func(*args, **kwargs)
            dur: float = tracer.getts() - start
            if info.dur_limit_us > 0.0 and dur < info.dur_limit_us:
                return ret
            event_name = func.__qualname__
            file_name = func.__code__.co_filename
            lineno = func.__code__.co_firstlineno
            raw_data = {
                "ph": "X",
                "name": f"{event_name} ({file_name}:{lineno})",
                "ts": start,
                "dur": dur,
                "cat": "FEE",
            }
            tracer.add_raw(raw_data)
            return ret

        return wrapper

    return _log_main


def log_target(
    name: str, category: ProfCategory, format_: str = log_format
) -> Callable[..., Any]:
    """Viztracer測定対象用デコレーター

    デコレーターが付けられている関数を測定対象とする。
    対象の測定はlog_sparseモードの時のみ実行される。

    Args:
        name (str): 測定対象名
        format (str): 測定名出力フォーマット
    """

    def _log_mark(func: Callable[..., Any]) -> Callable[..., Any]:
        info = ProfSharedReader.get()
        if info.mode != ProfMode.VizTracereTarget:
            return func
        if not (info.category & category):
            return func

        def wrapper(*args: object, **kwargs: object) -> object:
            tracer = get_tracer()
            if tracer is None or not tracer.log_sparse:
                return func(*args, **kwargs)
            start = tracer.getts()
            tracer.start()
            ret = func(*args, **kwargs)
            tracer.stop()
            dur: float = tracer.getts() - start
            if info.dur_limit_us > 0.0 and dur < info.dur_limit_us:
                return ret
            event_name = format_.format(name=name, path=func.__qualname__)
            file_name = func.__code__.co_filename
            lineno = func.__code__.co_firstlineno
            raw_data = {
                "ph": "X",
                "name": f"{event_name} ({file_name}:{lineno})",
                "ts": start,
                "dur": dur,
                "cat": "FEE",
            }
            tracer.add_raw(raw_data)
            return ret

        return wrapper

    return _log_mark


class EmptyEvent:
    """with文で測定しない時用の何もしないコンテキストマネージャー"""

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        type_: type[BaseException] | None,
        value: BaseException | None,
        trace: TracebackType | None,
    ) -> None:
        pass


def log_target_area(
    name: str,
    category: ProfCategory,
    func: Callable[..., Any] | str | None = None,
    format_: str = log_format,
) -> EmptyEvent | VizEvent:
    """Viztracer測定対象範囲用コンテキストマネージャーを取得する

    Args:
        name (str): 測定対象名
        func (Union[Callable[..., Any], str]): 関数または関数名
        format (str): 測定名出力フォーマット

    Returns:
        VizEvent | EmptyEvent: 測定対象範囲用コンテキストマネージャー

    Returns:
        Callable: 測定関数
    """
    info: ProfInfo = ProfSharedReader.get()
    if info.mode != ProfMode.VizTracereTarget:
        return EmptyEvent()
    if not (info.category & category):
        return EmptyEvent()
    tracer: VizTracer | None = get_tracer()
    if tracer is None:
        return EmptyEvent()
    call_frame = sys._getframe(1)
    if isinstance(func, Callable):
        qualname: str = func.__qualname__
    elif isinstance(func, str):
        qualname = func
    else:
        qualname = ""
    if tracer.log_sparse:
        return VizEvent(
            tracer,
            format_.format(name=name, path=qualname),
            call_frame.f_code.co_filename,
            call_frame.f_lineno,
        )
    if func is None:
        return EmptyEvent()
    return VizEvent(
        tracer,
        qualname,
        call_frame.f_code.co_filename,
        call_frame.f_lineno,
    )
