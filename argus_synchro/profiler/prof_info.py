from dataclasses import dataclass

from argus_synchro.profiler.prof_mode import ProfCategory, ProfMode


@dataclass
class ProfInfo:
    """プロファイリング共有情報"""

    count: int = 0
    mode: ProfMode = ProfMode.No
    out_dir: str = "result"
    category: ProfCategory = ProfCategory.All
    dur_limit_us: float = 0.0
    target_processes: tuple[str, ...] = ()
    tracer_entries: int = 200000
    max_stack_depth: int = 5
    ignore_c_function: bool = True
    minimize_memory: bool = True
