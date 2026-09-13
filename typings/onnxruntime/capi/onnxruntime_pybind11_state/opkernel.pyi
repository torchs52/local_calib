"""
OpKernel submodule
"""
from __future__ import annotations
__all__: list[str] = ['KernelDef']
class KernelDef:
    @property
    def domain(self) -> str:
        ...
    @property
    def op_name(self) -> str:
        ...
    @property
    def provider(self) -> str:
        ...
    @property
    def type_constraints(self) -> dict[str, list[str]]:
        ...
    @property
    def version_range(self) -> tuple[int, int]:
        ...
