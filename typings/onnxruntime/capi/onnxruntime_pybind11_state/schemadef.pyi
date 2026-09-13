"""
Schema submodule
"""
from __future__ import annotations
import typing
__all__: list[str] = ['OpSchema']
class OpSchema:
    class AttrType:
        """
        Members:
        
          FLOAT
        
          INT
        
          STRING
        
          TENSOR
        
          SPARSE_TENSOR
        
          GRAPH
        
          FLOATS
        
          INTS
        
          STRINGS
        
          TENSORS
        
          SPARSE_TENSORS
        
          GRAPHS
        """
        FLOAT: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.FLOAT: 1>
        FLOATS: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.FLOATS: 6>
        GRAPH: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.GRAPH: 5>
        GRAPHS: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.GRAPHS: 10>
        INT: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.INT: 2>
        INTS: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.INTS: 7>
        SPARSE_TENSOR: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.SPARSE_TENSOR: 11>
        SPARSE_TENSORS: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.SPARSE_TENSORS: 12>
        STRING: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.STRING: 3>
        STRINGS: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.STRINGS: 8>
        TENSOR: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.TENSOR: 4>
        TENSORS: typing.ClassVar[OpSchema.AttrType]  # value = <AttrType.TENSORS: 9>
        __members__: typing.ClassVar[dict[str, OpSchema.AttrType]]  # value = {'FLOAT': <AttrType.FLOAT: 1>, 'INT': <AttrType.INT: 2>, 'STRING': <AttrType.STRING: 3>, 'TENSOR': <AttrType.TENSOR: 4>, 'SPARSE_TENSOR': <AttrType.SPARSE_TENSOR: 11>, 'GRAPH': <AttrType.GRAPH: 5>, 'FLOATS': <AttrType.FLOATS: 6>, 'INTS': <AttrType.INTS: 7>, 'STRINGS': <AttrType.STRINGS: 8>, 'TENSORS': <AttrType.TENSORS: 9>, 'SPARSE_TENSORS': <AttrType.SPARSE_TENSORS: 12>, 'GRAPHS': <AttrType.GRAPHS: 10>}
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
    class Attribute:
        @property
        def _default_value(self) -> bytes:
            ...
        @property
        def description(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def required(self) -> bool:
            ...
        @property
        def type(self) -> ...:
            ...
    class FormalParameter:
        @property
        def description(self) -> str:
            ...
        @property
        def isHomogeneous(self) -> bool:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def option(self) -> OpSchema.FormalParameterOption:
            ...
        @property
        def typeStr(self) -> str:
            ...
        @property
        def types(self) -> set[str]:
            ...
    class FormalParameterOption:
        """
        Members:
        
          Single
        
          Optional
        
          Variadic
        """
        Optional: typing.ClassVar[OpSchema.FormalParameterOption]  # value = <FormalParameterOption.Optional: 1>
        Single: typing.ClassVar[OpSchema.FormalParameterOption]  # value = <FormalParameterOption.Single: 0>
        Variadic: typing.ClassVar[OpSchema.FormalParameterOption]  # value = <FormalParameterOption.Variadic: 2>
        __members__: typing.ClassVar[dict[str, OpSchema.FormalParameterOption]]  # value = {'Single': <FormalParameterOption.Single: 0>, 'Optional': <FormalParameterOption.Optional: 1>, 'Variadic': <FormalParameterOption.Variadic: 2>}
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
    class SupportType:
        """
        Members:
        
          COMMON
        
          EXPERIMENTAL
        """
        COMMON: typing.ClassVar[OpSchema.SupportType]  # value = <SupportType.COMMON: 0>
        EXPERIMENTAL: typing.ClassVar[OpSchema.SupportType]  # value = <SupportType.EXPERIMENTAL: 1>
        __members__: typing.ClassVar[dict[str, OpSchema.SupportType]]  # value = {'COMMON': <SupportType.COMMON: 0>, 'EXPERIMENTAL': <SupportType.EXPERIMENTAL: 1>}
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
    class TypeConstraintParam:
        @property
        def allowed_type_strs(self) -> list[str]:
            ...
        @property
        def description(self) -> str:
            ...
        @property
        def type_param_str(self) -> str:
            ...
    @staticmethod
    def is_infinite(arg0: typing.SupportsInt | typing.SupportsIndex) -> bool:
        ...
    @property
    def attributes(self) -> dict[str, ...]:
        ...
    @property
    def deprecated(self) -> bool:
        ...
    @property
    def doc(self) -> str:
        ...
    @property
    def domain(self) -> str:
        ...
    @property
    def file(self) -> str:
        ...
    @property
    def has_type_and_shape_inference_function(self) -> bool:
        ...
    @property
    def inputs(self) -> list[...]:
        ...
    @property
    def line(self) -> int:
        ...
    @property
    def max_input(self) -> int:
        ...
    @property
    def max_output(self) -> int:
        ...
    @property
    def min_input(self) -> int:
        ...
    @property
    def min_output(self) -> int:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def outputs(self) -> list[...]:
        ...
    @property
    def since_version(self) -> int:
        ...
    @property
    def support_level(self) -> ...:
        ...
    @property
    def type_constraints(self) -> list[...]:
        ...
