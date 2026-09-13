"""
pybind11 stateful interface to ONNX runtime
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
import typing_extensions
from . import opkernel
from . import schemadef
__all__: list[str] = ['AdapterFormat', 'ArenaExtendStrategy', 'EPFail', 'EngineError', 'ExecutionMode', 'ExecutionOrder', 'Fail', 'GraphOptimizationLevel', 'InferenceSession', 'InvalidArgument', 'InvalidGraph', 'InvalidProtobuf', 'LoraAdapter', 'ModelCompiler', 'ModelLoadCanceled', 'ModelLoaded', 'ModelMetadata', 'ModelRequiresCompilation', 'NoModel', 'NoSuchFile', 'NodeArg', 'NotFound', 'NotImplemented', 'OrtAllocatorType', 'OrtArenaCfg', 'OrtCompileApiFlags', 'OrtCompiledModelCompatibility', 'OrtDevice', 'OrtDeviceMemoryType', 'OrtEpAssignedNode', 'OrtEpAssignedSubgraph', 'OrtEpDevice', 'OrtExecutionProviderDevicePolicy', 'OrtExternalInitializerInfo', 'OrtHardwareDevice', 'OrtHardwareDeviceType', 'OrtMemType', 'OrtMemoryInfo', 'OrtMemoryInfoDeviceType', 'OrtSparseFormat', 'OrtSyncStream', 'OrtValue', 'OrtValueVector', 'RunOptions', 'RuntimeException', 'SessionIOBinding', 'SessionObjectInitializer', 'SessionOptions', 'SparseBlockSparseView', 'SparseCooView', 'SparseCsrView', 'SparseTensor', 'copy_tensors', 'create_and_register_allocator', 'create_and_register_allocator_v2', 'disable_telemetry_events', 'enable_telemetry_events', 'get_all_operator_schema', 'get_all_opkernel_def', 'get_all_providers', 'get_available_providers', 'get_build_info', 'get_default_session_options', 'get_device', 'get_ep_devices', 'get_model_compatibility_for_ep_devices', 'get_session_initializer', 'get_version_string', 'has_collective_ops', 'is_dlpack_uint8_tensor', 'kNextPowerOfTwo', 'kSameAsRequested', 'opkernel', 'quantize_matmul_2bits', 'quantize_matmul_4bits', 'quantize_matmul_8bits', 'quantize_matmul_bnb4', 'quantize_qdq_matmul_2bits', 'quantize_qdq_matmul_4bits', 'register_execution_provider_library', 'register_tensorrt_plugins_as_custom_ops', 'schemadef', 'set_arena_extend_strategy', 'set_cuda_device_id', 'set_cudnn_conv_algo_search', 'set_default_logger_severity', 'set_default_logger_verbosity', 'set_do_copy_in_default_stream', 'set_global_thread_pool_sizes', 'set_gpu_mem_limit', 'set_seed', 'unregister_execution_provider_library']
class AdapterFormat:
    @staticmethod
    def read_adapter(arg0: str) -> AdapterFormat:
        """
        The function returns an instance of the class that contains a dictionary of name -> numpy arrays
        """
    def __init__(self) -> None:
        ...
    def export_adapter(self, arg0: str) -> None:
        """
        "Save adapter parameters into a onnxruntime adapter file format.
        """
    @property
    def adapter_version(self) -> int:
        """
        "Enables user to read/write adapter version stored in the file"
        """
    @adapter_version.setter
    def adapter_version(self, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def format_version(self) -> int:
        """
        "Enables user to read format version stored in the file"
        """
    @property
    def model_version(self) -> int:
        """
        "Enables user to read/write model version this adapter was created for"
        """
    @model_version.setter
    def model_version(self, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def parameters(self) -> dict:
        """
        "Enables user to read/write adapter version stored in the file"
        """
    @parameters.setter
    def parameters(self, arg1: dict) -> None:
        ...
class ArenaExtendStrategy:
    """
    Members:
    
      kNextPowerOfTwo
    
      kSameAsRequested
    """
    __members__: typing.ClassVar[dict[str, ArenaExtendStrategy]]  # value = {'kNextPowerOfTwo': <ArenaExtendStrategy.kNextPowerOfTwo: 0>, 'kSameAsRequested': <ArenaExtendStrategy.kSameAsRequested: 1>}
    kNextPowerOfTwo: typing.ClassVar[ArenaExtendStrategy]  # value = <ArenaExtendStrategy.kNextPowerOfTwo: 0>
    kSameAsRequested: typing.ClassVar[ArenaExtendStrategy]  # value = <ArenaExtendStrategy.kSameAsRequested: 1>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
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
class EPFail(Exception):
    pass
class EngineError(Exception):
    pass
class ExecutionMode:
    """
    Members:
    
      ORT_SEQUENTIAL
    
      ORT_PARALLEL
    """
    ORT_PARALLEL: typing.ClassVar[ExecutionMode]  # value = <ExecutionMode.ORT_PARALLEL: 1>
    ORT_SEQUENTIAL: typing.ClassVar[ExecutionMode]  # value = <ExecutionMode.ORT_SEQUENTIAL: 0>
    __members__: typing.ClassVar[dict[str, ExecutionMode]]  # value = {'ORT_SEQUENTIAL': <ExecutionMode.ORT_SEQUENTIAL: 0>, 'ORT_PARALLEL': <ExecutionMode.ORT_PARALLEL: 1>}
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
class ExecutionOrder:
    """
    Members:
    
      DEFAULT
    
      PRIORITY_BASED
    
      MEMORY_EFFICIENT
    """
    DEFAULT: typing.ClassVar[ExecutionOrder]  # value = <ExecutionOrder.DEFAULT: 0>
    MEMORY_EFFICIENT: typing.ClassVar[ExecutionOrder]  # value = <ExecutionOrder.MEMORY_EFFICIENT: 2>
    PRIORITY_BASED: typing.ClassVar[ExecutionOrder]  # value = <ExecutionOrder.PRIORITY_BASED: 1>
    __members__: typing.ClassVar[dict[str, ExecutionOrder]]  # value = {'DEFAULT': <ExecutionOrder.DEFAULT: 0>, 'PRIORITY_BASED': <ExecutionOrder.PRIORITY_BASED: 1>, 'MEMORY_EFFICIENT': <ExecutionOrder.MEMORY_EFFICIENT: 2>}
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
class Fail(Exception):
    pass
class GraphOptimizationLevel:
    """
    Members:
    
      ORT_DISABLE_ALL
    
      ORT_ENABLE_BASIC
    
      ORT_ENABLE_EXTENDED
    
      ORT_ENABLE_LAYOUT
    
      ORT_ENABLE_ALL
    """
    ORT_DISABLE_ALL: typing.ClassVar[GraphOptimizationLevel]  # value = <GraphOptimizationLevel.ORT_DISABLE_ALL: 0>
    ORT_ENABLE_ALL: typing.ClassVar[GraphOptimizationLevel]  # value = <GraphOptimizationLevel.ORT_ENABLE_ALL: 99>
    ORT_ENABLE_BASIC: typing.ClassVar[GraphOptimizationLevel]  # value = <GraphOptimizationLevel.ORT_ENABLE_BASIC: 1>
    ORT_ENABLE_EXTENDED: typing.ClassVar[GraphOptimizationLevel]  # value = <GraphOptimizationLevel.ORT_ENABLE_EXTENDED: 2>
    ORT_ENABLE_LAYOUT: typing.ClassVar[GraphOptimizationLevel]  # value = <GraphOptimizationLevel.ORT_ENABLE_LAYOUT: 3>
    __members__: typing.ClassVar[dict[str, GraphOptimizationLevel]]  # value = {'ORT_DISABLE_ALL': <GraphOptimizationLevel.ORT_DISABLE_ALL: 0>, 'ORT_ENABLE_BASIC': <GraphOptimizationLevel.ORT_ENABLE_BASIC: 1>, 'ORT_ENABLE_EXTENDED': <GraphOptimizationLevel.ORT_ENABLE_EXTENDED: 2>, 'ORT_ENABLE_LAYOUT': <GraphOptimizationLevel.ORT_ENABLE_LAYOUT: 3>, 'ORT_ENABLE_ALL': <GraphOptimizationLevel.ORT_ENABLE_ALL: 99>}
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
class InferenceSession:
    """
    This is the main class used to run a model.
    """
    def __init__(self, arg0: SessionOptions, arg1: str, arg2: bool, arg3: bool) -> None:
        ...
    def end_profiling(self) -> str:
        ...
    def get_provider_graph_assignment_info(self) -> list[OrtEpAssignedSubgraph]:
        """
        Returns information on the subgraph/nodes assigned to execution providers in the session.
        """
    def get_provider_options(self) -> dict[str, dict[str, str]]:
        ...
    def get_providers(self) -> list[str]:
        ...
    def get_tuning_results(self) -> list:
        ...
    def initialize_session(self, arg0: collections.abc.Sequence[str], arg1: collections.abc.Sequence[collections.abc.Mapping[str, str]], arg2: collections.abc.Set[str]) -> None:
        """
        Load a model saved in ONNX or ORT format.
        """
    def run(self, arg0: collections.abc.Sequence[str], arg1: collections.abc.Mapping[str, typing.Any], arg2: RunOptions) -> list:
        ...
    def run_async(self, arg0: collections.abc.Sequence[str], arg1: collections.abc.Mapping[str, typing.Any], arg2: collections.abc.Callable[[collections.abc.Sequence[typing.Any], typing.Any, str], None], arg3: typing.Any, arg4: RunOptions) -> None:
        ...
    def run_with_iobinding(self, arg0: ..., arg1: RunOptions) -> None:
        ...
    def run_with_ort_values(self, arg0: dict, arg1: collections.abc.Sequence[str], arg2: RunOptions) -> ...:
        ...
    def run_with_ortvaluevector(self, arg0: RunOptions, arg1: collections.abc.Sequence[str], arg2: ..., std: ..., arg3: collections.abc.Sequence[str], arg4: ..., std: ..., arg5: collections.abc.Sequence[OrtDevice]) -> None:
        ...
    def set_ep_dynamic_options(self, arg0: dict) -> None:
        """
        Set dynamic options for execution providers.
        
                  Args:
                      options (dict): Dictionary of key-value pairs where both keys and values are strings.
                                    These options will be passed to the execution providers to modify
                                    their runtime behavior.
        
                  Example:
                      session.set_ep_dynamic_options({
                          "option1": "value1",
                          "option2": "value2"
                      })
        
                  Raises:
                      RuntimeError: If no options are provided or if setting the options fails.
        """
    def set_tuning_results(self, arg0: list, arg1: bool) -> None:
        ...
    @property
    def get_profiling_start_time_ns(self) -> int:
        ...
    @property
    def input_epdevices(self) -> list:
        ...
    @property
    def input_meminfos(self) -> list:
        ...
    @property
    def inputs_meta(self) -> list[NodeArg]:
        ...
    @property
    def model_meta(self) -> ModelMetadata:
        ...
    @property
    def output_meminfos(self) -> list:
        ...
    @property
    def outputs_meta(self) -> list[NodeArg]:
        ...
    @property
    def overridable_initializers(self) -> list[NodeArg]:
        ...
    @property
    def session_options(self) -> SessionOptions:
        ...
class InvalidArgument(Exception):
    pass
class InvalidGraph(Exception):
    pass
class InvalidProtobuf(Exception):
    pass
class LoraAdapter:
    def Load(self, arg0: str) -> None:
        """
        Memory map the specified file as LoraAdapter
        """
    def __init__(self) -> None:
        ...
class ModelCompiler:
    """
    This is the class used to compile an ONNX model.
    """
    def __init__(self, arg0: SessionOptions, arg1: str, arg2: bool, arg3: bool, arg4: str, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: GraphOptimizationLevel, arg8: collections.abc.Callable[[str, OrtValue, OrtExternalInitializerInfo], OrtExternalInitializerInfo]) -> None:
        ...
    def compile_to_bytes(self) -> bytes:
        """
        Compile an ONNX model into a buffer.
        """
    def compile_to_file(self, arg0: str) -> None:
        """
        Compile an ONNX model into a file.
        """
    def compile_to_stream(self, arg0: collections.abc.Callable[[bytes], None]) -> None:
        """
        Compile an ONNX model into an output stream using the provided write functor.
        """
class ModelLoadCanceled(Exception):
    pass
class ModelLoaded(Exception):
    pass
class ModelMetadata:
    """
    Pre-defined and custom metadata about the model.
    It is usually used to identify the model used to run the prediction and
    facilitate the comparison.
    """
    @property
    def custom_metadata_map(self) -> dict[str, str]:
        """
        additional metadata
        """
    @custom_metadata_map.setter
    def custom_metadata_map(self, arg0: collections.abc.Mapping[str, str]) -> None:
        ...
    @property
    def description(self) -> str:
        """
        description of the model
        """
    @description.setter
    def description(self, arg0: str) -> None:
        ...
    @property
    def domain(self) -> str:
        """
        ONNX domain
        """
    @domain.setter
    def domain(self, arg0: str) -> None:
        ...
    @property
    def graph_description(self) -> str:
        """
        description of the graph hosted in the model
        """
    @graph_description.setter
    def graph_description(self, arg0: str) -> None:
        ...
    @property
    def graph_name(self) -> str:
        """
        graph name
        """
    @graph_name.setter
    def graph_name(self, arg0: str) -> None:
        ...
    @property
    def producer_name(self) -> str:
        """
        producer name
        """
    @producer_name.setter
    def producer_name(self, arg0: str) -> None:
        ...
    @property
    def version(self) -> int:
        """
        version of the model
        """
    @version.setter
    def version(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class ModelRequiresCompilation(Exception):
    pass
class NoModel(Exception):
    pass
class NoSuchFile(Exception):
    pass
class NodeArg:
    """
    Node argument definition, for both input and output,
    including arg name, arg type (contains both type and shape).
    """
    def __str__(self) -> str:
        """
        converts the node into a readable string
        """
    @property
    def name(self) -> str:
        """
        node name
        """
    @property
    def shape(self) -> list[typing.Any]:
        """
        node shape (assuming the node holds a tensor)
        """
    @property
    def type(self) -> str:
        """
        node type
        """
class NotFound(Exception):
    pass
class NotImplemented(Exception):
    pass
class OrtAllocatorType:
    """
    Members:
    
      INVALID
    
      ORT_DEVICE_ALLOCATOR
    
      ORT_ARENA_ALLOCATOR
    """
    INVALID: typing.ClassVar[OrtAllocatorType]  # value = <OrtAllocatorType.INVALID: -1>
    ORT_ARENA_ALLOCATOR: typing.ClassVar[OrtAllocatorType]  # value = <OrtAllocatorType.ORT_ARENA_ALLOCATOR: 1>
    ORT_DEVICE_ALLOCATOR: typing.ClassVar[OrtAllocatorType]  # value = <OrtAllocatorType.ORT_DEVICE_ALLOCATOR: 0>
    __members__: typing.ClassVar[dict[str, OrtAllocatorType]]  # value = {'INVALID': <OrtAllocatorType.INVALID: -1>, 'ORT_DEVICE_ALLOCATOR': <OrtAllocatorType.ORT_DEVICE_ALLOCATOR: 0>, 'ORT_ARENA_ALLOCATOR': <OrtAllocatorType.ORT_ARENA_ALLOCATOR: 1>}
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
class OrtArenaCfg:
    @typing.overload
    def __init__(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsInt | typing.SupportsIndex, arg2: typing.SupportsInt | typing.SupportsIndex, arg3: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: dict) -> None:
        ...
    @property
    def arena_extend_strategy(self) -> int:
        ...
    @arena_extend_strategy.setter
    def arena_extend_strategy(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def initial_chunk_size_bytes(self) -> int:
        ...
    @initial_chunk_size_bytes.setter
    def initial_chunk_size_bytes(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def initial_growth_chunk_size_bytes(self) -> int:
        ...
    @initial_growth_chunk_size_bytes.setter
    def initial_growth_chunk_size_bytes(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def max_dead_bytes_per_chunk(self) -> int:
        ...
    @max_dead_bytes_per_chunk.setter
    def max_dead_bytes_per_chunk(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def max_mem(self) -> int:
        ...
    @max_mem.setter
    def max_mem(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def max_power_of_two_extend_bytes(self) -> int:
        ...
    @max_power_of_two_extend_bytes.setter
    def max_power_of_two_extend_bytes(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class OrtCompileApiFlags:
    """
    Members:
    
      NONE
    
      ERROR_IF_NO_NODES_COMPILED
    
      ERROR_IF_OUTPUT_FILE_EXISTS
    """
    ERROR_IF_NO_NODES_COMPILED: typing.ClassVar[OrtCompileApiFlags]  # value = <OrtCompileApiFlags.ERROR_IF_NO_NODES_COMPILED: 1>
    ERROR_IF_OUTPUT_FILE_EXISTS: typing.ClassVar[OrtCompileApiFlags]  # value = <OrtCompileApiFlags.ERROR_IF_OUTPUT_FILE_EXISTS: 2>
    NONE: typing.ClassVar[OrtCompileApiFlags]  # value = <OrtCompileApiFlags.NONE: 0>
    __members__: typing.ClassVar[dict[str, OrtCompileApiFlags]]  # value = {'NONE': <OrtCompileApiFlags.NONE: 0>, 'ERROR_IF_NO_NODES_COMPILED': <OrtCompileApiFlags.ERROR_IF_NO_NODES_COMPILED: 1>, 'ERROR_IF_OUTPUT_FILE_EXISTS': <OrtCompileApiFlags.ERROR_IF_OUTPUT_FILE_EXISTS: 2>}
    def __and__(self, other: typing.Any) -> typing.Any:
        ...
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __ge__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __gt__(self, other: typing.Any) -> bool:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __invert__(self) -> typing.Any:
        ...
    def __le__(self, other: typing.Any) -> bool:
        ...
    def __lt__(self, other: typing.Any) -> bool:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __or__(self, other: typing.Any) -> typing.Any:
        ...
    def __rand__(self, other: typing.Any) -> typing.Any:
        ...
    def __repr__(self) -> str:
        ...
    def __ror__(self, other: typing.Any) -> typing.Any:
        ...
    def __rxor__(self, other: typing.Any) -> typing.Any:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    def __xor__(self, other: typing.Any) -> typing.Any:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class OrtCompiledModelCompatibility:
    """
    Members:
    
      EP_NOT_APPLICABLE
    
      EP_SUPPORTED_OPTIMAL
    
      EP_SUPPORTED_PREFER_RECOMPILATION
    
      EP_UNSUPPORTED
    """
    EP_NOT_APPLICABLE: typing.ClassVar[OrtCompiledModelCompatibility]  # value = <OrtCompiledModelCompatibility.EP_NOT_APPLICABLE: 0>
    EP_SUPPORTED_OPTIMAL: typing.ClassVar[OrtCompiledModelCompatibility]  # value = <OrtCompiledModelCompatibility.EP_SUPPORTED_OPTIMAL: 1>
    EP_SUPPORTED_PREFER_RECOMPILATION: typing.ClassVar[OrtCompiledModelCompatibility]  # value = <OrtCompiledModelCompatibility.EP_SUPPORTED_PREFER_RECOMPILATION: 2>
    EP_UNSUPPORTED: typing.ClassVar[OrtCompiledModelCompatibility]  # value = <OrtCompiledModelCompatibility.EP_UNSUPPORTED: 3>
    __members__: typing.ClassVar[dict[str, OrtCompiledModelCompatibility]]  # value = {'EP_NOT_APPLICABLE': <OrtCompiledModelCompatibility.EP_NOT_APPLICABLE: 0>, 'EP_SUPPORTED_OPTIMAL': <OrtCompiledModelCompatibility.EP_SUPPORTED_OPTIMAL: 1>, 'EP_SUPPORTED_PREFER_RECOMPILATION': <OrtCompiledModelCompatibility.EP_SUPPORTED_PREFER_RECOMPILATION: 2>, 'EP_UNSUPPORTED': <OrtCompiledModelCompatibility.EP_UNSUPPORTED: 3>}
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
class OrtDevice:
    """
    ONNXRuntime device information.
    """
    @staticmethod
    def cann() -> int:
        ...
    @staticmethod
    def cpu() -> int:
        ...
    @staticmethod
    def cuda() -> int:
        ...
    @staticmethod
    def default_memory() -> int:
        ...
    @staticmethod
    def dml() -> int:
        ...
    @staticmethod
    def fpga() -> int:
        ...
    @staticmethod
    def gpu() -> int:
        ...
    @staticmethod
    def npu() -> int:
        ...
    @staticmethod
    def webgpu() -> int:
        ...
    @typing.overload
    def __init__(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsInt | typing.SupportsIndex, arg2: typing.SupportsInt | typing.SupportsIndex, arg3: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: typing.SupportsInt | typing.SupportsIndex, arg1: typing.SupportsInt | typing.SupportsIndex, arg2: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Constructor with vendor_id defaulted to 0 for backward compatibility.
        """
    def device_id(self) -> int:
        """
        Device Id.
        """
    def device_type(self) -> int:
        """
        Device Type.
        """
    def mem_type(self) -> int:
        """
        Device Memory Type.
        """
    def vendor_id(self) -> int:
        """
        Vendor Id.
        """
class OrtDeviceMemoryType:
    """
    Members:
    
      DEFAULT
    
      HOST_ACCESSIBLE
    """
    DEFAULT: typing.ClassVar[OrtDeviceMemoryType]  # value = <OrtDeviceMemoryType.DEFAULT: 0>
    HOST_ACCESSIBLE: typing.ClassVar[OrtDeviceMemoryType]  # value = <OrtDeviceMemoryType.HOST_ACCESSIBLE: 5>
    __members__: typing.ClassVar[dict[str, OrtDeviceMemoryType]]  # value = {'DEFAULT': <OrtDeviceMemoryType.DEFAULT: 0>, 'HOST_ACCESSIBLE': <OrtDeviceMemoryType.HOST_ACCESSIBLE: 5>}
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
class OrtEpAssignedNode:
    """
    Contains information about a node assigned to an execution
    provider
    """
    @property
    def domain(self) -> str:
        """
        The node's domain
        """
    @property
    def name(self) -> str:
        """
        The node's name
        """
    @property
    def op_type(self) -> str:
        """
        The node's operator type
        """
class OrtEpAssignedSubgraph:
    """
    Contains information about a subgraph assigned to an
    execution provider
    """
    def get_nodes(self) -> list[OrtEpAssignedNode]:
        """
        List of nodes in the subgraph.
        """
    @property
    def ep_name(self) -> str:
        """
        The name of the execution provider to which this subgraph is assigned.
        """
class OrtEpDevice:
    """
    Represents a hardware device that an execution provider supports
    for model inference.
    """
    def create_sync_stream(self) -> OrtSyncStream:
        """
        The OrtSyncStream instance for the OrtEpDevice.
        """
    def memory_info(self, arg0: OrtDeviceMemoryType) -> OrtMemoryInfo:
        """
        The OrtMemoryInfo instance for the OrtEpDevice specific to the device memory type.
        """
    @property
    def device(self) -> OrtHardwareDevice:
        """
        The OrtHardwareDevice instance for the OrtEpDevice.
        """
    @property
    def ep_metadata(self) -> dict[str, str]:
        """
        The execution provider's additional metadata for the OrtHardwareDevice.
        """
    @property
    def ep_name(self) -> str:
        """
        The execution provider's name.
        """
    @property
    def ep_options(self) -> dict[str, str]:
        """
        The execution provider's options used to configure the provider to use the OrtHardwareDevice.
        """
    @property
    def ep_vendor(self) -> str:
        """
        The execution provider's vendor name.
        """
class OrtExecutionProviderDevicePolicy:
    """
    Members:
    
      DEFAULT
    
      PREFER_CPU
    
      PREFER_NPU
    
      PREFER_GPU
    
      MAX_PERFORMANCE
    
      MAX_EFFICIENCY
    
      MIN_OVERALL_POWER
    """
    DEFAULT: typing.ClassVar[OrtExecutionProviderDevicePolicy]  # value = <OrtExecutionProviderDevicePolicy.DEFAULT: 0>
    MAX_EFFICIENCY: typing.ClassVar[OrtExecutionProviderDevicePolicy]  # value = <OrtExecutionProviderDevicePolicy.MAX_EFFICIENCY: 5>
    MAX_PERFORMANCE: typing.ClassVar[OrtExecutionProviderDevicePolicy]  # value = <OrtExecutionProviderDevicePolicy.MAX_PERFORMANCE: 4>
    MIN_OVERALL_POWER: typing.ClassVar[OrtExecutionProviderDevicePolicy]  # value = <OrtExecutionProviderDevicePolicy.MIN_OVERALL_POWER: 6>
    PREFER_CPU: typing.ClassVar[OrtExecutionProviderDevicePolicy]  # value = <OrtExecutionProviderDevicePolicy.PREFER_CPU: 1>
    PREFER_GPU: typing.ClassVar[OrtExecutionProviderDevicePolicy]  # value = <OrtExecutionProviderDevicePolicy.PREFER_GPU: 3>
    PREFER_NPU: typing.ClassVar[OrtExecutionProviderDevicePolicy]  # value = <OrtExecutionProviderDevicePolicy.PREFER_NPU: 2>
    __members__: typing.ClassVar[dict[str, OrtExecutionProviderDevicePolicy]]  # value = {'DEFAULT': <OrtExecutionProviderDevicePolicy.DEFAULT: 0>, 'PREFER_CPU': <OrtExecutionProviderDevicePolicy.PREFER_CPU: 1>, 'PREFER_NPU': <OrtExecutionProviderDevicePolicy.PREFER_NPU: 2>, 'PREFER_GPU': <OrtExecutionProviderDevicePolicy.PREFER_GPU: 3>, 'MAX_PERFORMANCE': <OrtExecutionProviderDevicePolicy.MAX_PERFORMANCE: 4>, 'MAX_EFFICIENCY': <OrtExecutionProviderDevicePolicy.MAX_EFFICIENCY: 5>, 'MIN_OVERALL_POWER': <OrtExecutionProviderDevicePolicy.MIN_OVERALL_POWER: 6>}
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
class OrtExternalInitializerInfo:
    """
    Location information for initializer data stored in an external file
    """
    def __init__(self, arg0: str, arg1: typing.SupportsInt | typing.SupportsIndex, arg2: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def byte_size(self) -> int:
        """
        The byte size of the initializer data in the file.
        """
    @property
    def file_offset(self) -> int:
        """
        The file byte offset where the initializer data is stored.
        """
    @property
    def filepath(self) -> str:
        """
        The relative path to the file in which initializer data is stored.
        """
class OrtHardwareDevice:
    """
    ONNX Runtime hardware device information.
    """
    @property
    def device_id(self) -> int:
        """
        Hardware device's unique identifier.
        """
    @property
    def metadata(self) -> dict[str, str]:
        """
        Hardware device's metadata as string key/value pairs.
        """
    @property
    def type(self) -> OrtHardwareDeviceType:
        """
        Hardware device's type.
        """
    @property
    def vendor(self) -> str:
        """
        Hardware device's vendor name.
        """
    @property
    def vendor_id(self) -> int:
        """
        Hardware device's vendor identifier.
        """
class OrtHardwareDeviceType:
    """
    Members:
    
      CPU
    
      GPU
    
      NPU
    """
    CPU: typing.ClassVar[OrtHardwareDeviceType]  # value = <OrtHardwareDeviceType.CPU: 0>
    GPU: typing.ClassVar[OrtHardwareDeviceType]  # value = <OrtHardwareDeviceType.GPU: 1>
    NPU: typing.ClassVar[OrtHardwareDeviceType]  # value = <OrtHardwareDeviceType.NPU: 2>
    __members__: typing.ClassVar[dict[str, OrtHardwareDeviceType]]  # value = {'CPU': <OrtHardwareDeviceType.CPU: 0>, 'GPU': <OrtHardwareDeviceType.GPU: 1>, 'NPU': <OrtHardwareDeviceType.NPU: 2>}
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
class OrtMemType:
    """
    Members:
    
      CPU_INPUT
    
      CPU_OUTPUT
    
      CPU
    
      DEFAULT
    """
    CPU: typing.ClassVar[OrtMemType]  # value = <OrtMemType.CPU_OUTPUT: -1>
    CPU_INPUT: typing.ClassVar[OrtMemType]  # value = <OrtMemType.CPU_INPUT: -2>
    CPU_OUTPUT: typing.ClassVar[OrtMemType]  # value = <OrtMemType.CPU_OUTPUT: -1>
    DEFAULT: typing.ClassVar[OrtMemType]  # value = <OrtMemType.DEFAULT: 0>
    __members__: typing.ClassVar[dict[str, OrtMemType]]  # value = {'CPU_INPUT': <OrtMemType.CPU_INPUT: -2>, 'CPU_OUTPUT': <OrtMemType.CPU_OUTPUT: -1>, 'CPU': <OrtMemType.CPU_OUTPUT: -1>, 'DEFAULT': <OrtMemType.DEFAULT: 0>}
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
class OrtMemoryInfo:
    @staticmethod
    def create_v2(arg0: str, arg1: OrtMemoryInfoDeviceType, arg2: typing.SupportsInt | typing.SupportsIndex, arg3: typing.SupportsInt | typing.SupportsIndex, arg4: OrtDeviceMemoryType, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: OrtAllocatorType) -> OrtMemoryInfo:
        """
        Create an OrtMemoryInfo instance using CreateMemoryInfo_V2()
        """
    def __init__(self, arg0: str, arg1: OrtAllocatorType, arg2: typing.SupportsInt | typing.SupportsIndex, arg3: OrtMemType) -> None:
        ...
    @property
    def allocator_type(self) -> OrtAllocatorType:
        """
        Allocator type
        """
    @property
    def device_id(self) -> int:
        """
        Device Id.
        """
    @property
    def device_mem_type(self) -> OrtDeviceMemoryType:
        """
        Device memory type (Device or Host accessible).
        """
    @property
    def device_vendor_id(self) -> int:
        ...
    @property
    def mem_type(self) -> OrtMemType:
        """
        OrtMemoryInfo memory type.
        """
    @property
    def name(self) -> str:
        """
        Arbitrary name supplied by the user
        """
class OrtMemoryInfoDeviceType:
    """
    Members:
    
      CPU
    
      GPU
    
      NPU
    
      FPGA
    """
    CPU: typing.ClassVar[OrtMemoryInfoDeviceType]  # value = <OrtMemoryInfoDeviceType.CPU: 0>
    FPGA: typing.ClassVar[OrtMemoryInfoDeviceType]  # value = <OrtMemoryInfoDeviceType.FPGA: 2>
    GPU: typing.ClassVar[OrtMemoryInfoDeviceType]  # value = <OrtMemoryInfoDeviceType.GPU: 1>
    NPU: typing.ClassVar[OrtMemoryInfoDeviceType]  # value = <OrtMemoryInfoDeviceType.NPU: 3>
    __members__: typing.ClassVar[dict[str, OrtMemoryInfoDeviceType]]  # value = {'CPU': <OrtMemoryInfoDeviceType.CPU: 0>, 'GPU': <OrtMemoryInfoDeviceType.GPU: 1>, 'NPU': <OrtMemoryInfoDeviceType.NPU: 3>, 'FPGA': <OrtMemoryInfoDeviceType.FPGA: 2>}
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
class OrtSparseFormat:
    """
    Members:
    
      ORT_SPARSE_UNDEFINED
    
      ORT_SPARSE_COO
    
      ORT_SPARSE_CSRC
    
      ORT_SPARSE_BLOCK_SPARSE
    """
    ORT_SPARSE_BLOCK_SPARSE: typing.ClassVar[OrtSparseFormat]  # value = <OrtSparseFormat.ORT_SPARSE_BLOCK_SPARSE: 4>
    ORT_SPARSE_COO: typing.ClassVar[OrtSparseFormat]  # value = <OrtSparseFormat.ORT_SPARSE_COO: 1>
    ORT_SPARSE_CSRC: typing.ClassVar[OrtSparseFormat]  # value = <OrtSparseFormat.ORT_SPARSE_CSRC: 2>
    ORT_SPARSE_UNDEFINED: typing.ClassVar[OrtSparseFormat]  # value = <OrtSparseFormat.ORT_SPARSE_UNDEFINED: 0>
    __members__: typing.ClassVar[dict[str, OrtSparseFormat]]  # value = {'ORT_SPARSE_UNDEFINED': <OrtSparseFormat.ORT_SPARSE_UNDEFINED: 0>, 'ORT_SPARSE_COO': <OrtSparseFormat.ORT_SPARSE_COO: 1>, 'ORT_SPARSE_CSRC': <OrtSparseFormat.ORT_SPARSE_CSRC: 2>, 'ORT_SPARSE_BLOCK_SPARSE': <OrtSparseFormat.ORT_SPARSE_BLOCK_SPARSE: 4>}
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
class OrtSyncStream:
    """
    Represents a synchronization stream for model inference.
    """
    def get_handle(self) -> int:
        """
        SyncStream handle that can be converted to a string and added to SessionOptions
        """
class OrtValue:
    @staticmethod
    def from_dlpack(data: typing.Any, is_bool_tensor: bool = False) -> OrtValue:
        """
        Converts a tensor from a external library into an OrtValue by means of the __dlpack__ protocol.
        """
    @staticmethod
    def ort_value_from_sparse_tensor(arg0: ...) -> OrtValue:
        ...
    @staticmethod
    def ortvalue_from_numpy(arg0: typing.Any, arg1: OrtDevice) -> OrtValue:
        ...
    @staticmethod
    def ortvalue_from_numpy_with_onnx_type(arg0: numpy.ndarray[typing.Any, numpy.dtype[typing.Any]], arg1: typing.SupportsInt | typing.SupportsIndex) -> OrtValue:
        ...
    @staticmethod
    def ortvalue_from_shape_and_onnx_type(arg0: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg1: typing.SupportsInt | typing.SupportsIndex, arg2: OrtDevice) -> OrtValue:
        ...
    @staticmethod
    def ortvalue_from_shape_and_type(arg0: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg1: typing.Any, arg2: OrtDevice) -> OrtValue:
        ...
    def __dlpack__(self, stream: typing.Any = None) -> typing.Any:
        """
        Returns a DLPack representing the tensor (part of __dlpack__ protocol). This method does not copy the pointer shape, instead, it copies the pointer value. The OrtValue must persist until the dlpack structure is consumed.
        """
    def __dlpack_device__(self) -> tuple:
        """
        Returns a tuple of integers, (device, device index) (part of __dlpack__ protocol).
        """
    def as_sparse_tensor(self) -> ...:
        ...
    def data_ptr(self) -> int:
        ...
    def data_type(self) -> str:
        ...
    def device_name(self) -> str:
        ...
    def element_type(self) -> int:
        """
        Returns an integer equal to the ONNX tensor proto type of the tensor or sequence. This integer is one type defined by ONNX TensorProto_DataType (such as onnx.TensorProto.FLOAT).Raises an exception in any other case.
        """
    def has_value(self) -> bool:
        ...
    def is_sparse_tensor(self) -> bool:
        ...
    def is_tensor(self) -> bool:
        ...
    def is_tensor_sequence(self) -> bool:
        ...
    def numpy(self) -> typing.Any:
        ...
    def shape(self) -> list:
        ...
    def tensor_size_in_bytes(self) -> int:
        """
        Returns tensor size in bytes.
        """
    def to_dlpack(self) -> typing.Any:
        """
        Returns a DLPack representing the tensor. This method does not copy the pointer shape, instead, it copies the pointer value. The OrtValue must be persist until the dlpack structure is consumed.
        """
    def update_inplace(self, arg0: numpy.ndarray[typing.Any, numpy.dtype[typing.Any]]) -> None:
        ...
class OrtValueVector:
    def __getitem__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> OrtValue:
        ...
    def __init__(self) -> None:
        ...
    def __iter__(self) -> collections.abc.Iterator[OrtValue]:
        ...
    def __len__(self) -> int:
        ...
    def bool_tensor_indices(self) -> list[int]:
        """
        Returns the indices of every boolean tensor in this vector of OrtValue. In case of a boolean tensor, method to_dlpacks returns a uint8 tensor instead of a boolean tensor. If torch consumes the dlpack structure, `.to(torch.bool)` must be applied to the torch tensor to get a boolean tensor.
        """
    def dlpack_at(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> typing.Any:
        ...
    def element_type_at(self, idx: typing.SupportsInt | typing.SupportsIndex) -> int:
        """
        Returns an integer equal to the ONNX proto type of the tensor at position i. This integer is one type defined by ONNX TensorProto_DataType (such as onnx.TensorProto.FLOAT).Raises an exception in any other case.
        """
    @typing.overload
    def push_back(self, arg0: OrtValue) -> None:
        ...
    @typing.overload
    def push_back(self, dlpack_tensor: typing.Any, is_bool_tensor: bool = False) -> None:
        """
        Add a new OrtValue after being ownership was transferred from the DLPack structure.
        """
    def push_back_batch(self, arg0: collections.abc.Sequence[typing.Any], arg1: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg2: collections.abc.Sequence[typing.Any], arg3: collections.abc.Sequence[collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex]], arg4: collections.abc.Sequence[OrtDevice]) -> None:
        """
        Add a batch of OrtValue's by wrapping PyTorch tensors.
        """
    def reserve(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def shrink_to_fit(self) -> None:
        ...
    def to_dlpacks(self, to_tensor: typing.Any) -> list:
        """
        Converts all OrtValue into tensors through DLPack protocol, the method creates
        a DLPack structure for every tensors, then calls python function `to_tensor` to a new object
        consuming the DLPack structure or return a list of capsule if this function is None.
        
        :param to_tensor: this function takes a capsule holding a pointer onto a DLPack structure and returns
            a new tensor which becomes the new owner of the data. This function takes one python object and
            returns a new python object. It fits the same signature as `torch.utils.from_dlpack`,
            if None, the method returns a capsule for every new DLPack structure.
        :return: a list containing the new tensors or a the new capsules if *to_tensor* is None
        
        This method is used to replace `tuple(torch._C._from_dlpack(ov.to_dlpack()) for ov in ort_values)`
        by a faster instruction `tuple(ort_values.to_dlpack(torch._C._from_dlpack))`. This loop
        is difficult to parallelize as it goes through the GIL many times.
        It creates many tensors acquiring ownership of existing OrtValue.
        This method saves one object creation and an C++ allocation
        for every transferred tensor.
        """
class RunOptions:
    """
    Configuration information for a single Run.
    """
    def __init__(self) -> None:
        ...
    def add_active_adapter(self, arg0: ...) -> None:
        """
        Adds specified adapter as an active adapter
        """
    def add_run_config_entry(self, arg0: str, arg1: str) -> None:
        """
        Set a single run configuration entry as a pair of strings.
        """
    def get_run_config_entry(self, arg0: str) -> str:
        """
        Get a single run configuration value using the given configuration key.
        """
    @property
    def log_severity_level(self) -> int:
        """
        Log severity level for a particular Run() invocation. 0:Verbose, 1:Info, 2:Warning. 3:Error, 4:Fatal. Default is 2.
        """
    @log_severity_level.setter
    def log_severity_level(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def log_verbosity_level(self) -> int:
        """
        VLOG level if DEBUG build and run_log_severity_level is 0.
        Applies to a particular Run() invocation. Default is 0.
        """
    @log_verbosity_level.setter
    def log_verbosity_level(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def logid(self) -> str:
        """
        To identify logs generated by a particular Run() invocation.
        """
    @logid.setter
    def logid(self, arg0: str) -> None:
        ...
    @property
    def only_execute_path_to_fetches(self) -> bool:
        """
        Only execute the nodes needed by fetch list
        """
    @only_execute_path_to_fetches.setter
    def only_execute_path_to_fetches(self, arg0: bool) -> None:
        ...
    @property
    def terminate(self) -> bool:
        """
        Set to True to terminate any currently executing calls that are using this
        RunOptions instance. The individual calls will exit gracefully and return an error status.
        """
    @terminate.setter
    def terminate(self, arg0: bool) -> None:
        ...
class RuntimeException(Exception):
    pass
class SessionIOBinding:
    def __init__(self, arg0: InferenceSession) -> None:
        ...
    @typing.overload
    def bind_input(self, arg0: str, arg1: typing.Any) -> None:
        ...
    @typing.overload
    def bind_input(self, arg0: str, arg1: OrtDevice, arg2: typing.SupportsInt | typing.SupportsIndex, arg3: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg4: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def bind_input(self, arg0: str, arg1: OrtDevice, arg2: typing.Any, arg3: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg4: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def bind_ortvalue_input(self, arg0: str, arg1: OrtValue) -> None:
        ...
    def bind_ortvalue_output(self, arg0: str, arg1: OrtValue) -> None:
        ...
    @typing.overload
    def bind_output(self, arg0: str, arg1: OrtDevice, arg2: typing.SupportsInt | typing.SupportsIndex, arg3: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg4: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def bind_output(self, arg0: str, arg1: OrtDevice, arg2: typing.Any, arg3: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg4: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def bind_output(self, arg0: str, arg1: OrtDevice) -> None:
        ...
    def clear_binding_inputs(self) -> None:
        ...
    def clear_binding_outputs(self) -> None:
        ...
    def copy_outputs_to_cpu(self) -> list:
        ...
    def get_outputs(self) -> OrtValueVector:
        ...
    def synchronize_inputs(self) -> None:
        ...
    def synchronize_outputs(self) -> None:
        ...
class SessionObjectInitializer:
    pass
class SessionOptions:
    """
    Configuration information for a session.
    """
    def __init__(self) -> None:
        ...
    def add_external_initializers(self, arg0: list, arg1: list) -> None:
        ...
    def add_external_initializers_from_files_in_memory(self, arg0: collections.abc.Sequence[str], arg1: collections.abc.Sequence[collections.abc.Buffer], arg2: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex]) -> None:
        """
        Provide external initializer file contents from memory.
        
        Args:
          names: sequence[str] of external file names (as referenced by the model's external_data locations).
          buffers: sequence[bytes-like] objects exposing the buffer protocol (e.g., bytes, bytearray, memoryview, numpy uint8 array) containing the corresponding file contents.
          lengths: sequence[int] sizes in bytes for each buffer.
        
        Notes:
          - Keep the provided buffers alive until after session creation completes. ONNX Runtime copies needed data during session creation.
          - The bytestream must match the external file layout expected by the model (raw tensor bytes at the specified offsets).
        """
    def add_free_dimension_override_by_denotation(self, arg0: str, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Specify the dimension size for each denotation associated with an input's free dimension.
        """
    def add_free_dimension_override_by_name(self, arg0: str, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Specify values of named dimensions within model inputs.
        """
    def add_initializer(self, arg0: str, arg1: typing.Any) -> None:
        ...
    def add_provider(self, arg0: str, arg1: collections.abc.Mapping[str, str]) -> None:
        """
        Adds an explicit execution provider.
        """
    def add_provider_for_devices(self, arg0: collections.abc.Sequence[OrtEpDevice], arg1: collections.abc.Mapping[str, str]) -> None:
        """
        Adds the execution provider that is responsible for the selected OrtEpDevice instances. All OrtEpDevice instances
        must refer to the same execution provider.
        """
    def add_session_config_entry(self, arg0: str, arg1: str) -> None:
        """
        Set a single session configuration entry as a pair of strings.
        """
    def get_session_config_entry(self, arg0: str) -> str:
        """
        Get a single session configuration value using the given configuration key.
        """
    def has_providers(self) -> bool:
        """
        Returns true if the SessionOptions has been configured with providers, OrtEpDevices, or
        policies that will run the model.
        """
    def register_custom_ops_library(self, arg0: str) -> None:
        """
        Specify the path to the shared library containing the custom op kernels required to run a model.
        """
    def set_load_cancellation_flag(self, arg0: bool) -> None:
        """
        Request inference session load cancellation
        """
    def set_provider_selection_policy(self, arg0: OrtExecutionProviderDevicePolicy) -> None:
        """
        Sets the execution provider selection policy for the session. Allows users to specify a
        selection policy for automatic execution provider (EP) selection.
        """
    def set_provider_selection_policy_delegate(self, arg0: collections.abc.Callable[[collections.abc.Sequence[OrtEpDevice], collections.abc.Mapping[str, str], collections.abc.Mapping[str, str], typing.SupportsInt | typing.SupportsIndex], list[OrtEpDevice]]) -> None:
        """
        Sets the execution provider selection policy delegate for the session. Allows users to specify a
        custom selection policy function for automatic execution provider (EP) selection. The delegate must return a list of
        selected OrtEpDevice instances. The signature of the delegate is
        def custom_delegate(ep_devices: Sequence[OrtEpDevice], model_metadata: dict[str, str], runtime_metadata: dict[str, str],
        max_selections: int) -> Sequence[OrtEpDevice]
        """
    @property
    def enable_cpu_mem_arena(self) -> bool:
        """
        Enable memory arena on CPU. Default is true.
        """
    @enable_cpu_mem_arena.setter
    def enable_cpu_mem_arena(self, arg1: bool) -> None:
        ...
    @property
    def enable_mem_pattern(self) -> bool:
        """
        Enable the memory pattern optimization. Default is true.
        """
    @enable_mem_pattern.setter
    def enable_mem_pattern(self, arg1: bool) -> None:
        ...
    @property
    def enable_mem_reuse(self) -> bool:
        """
        Enable the memory reuse optimization. Default is true.
        """
    @enable_mem_reuse.setter
    def enable_mem_reuse(self, arg1: bool) -> None:
        ...
    @property
    def enable_profiling(self) -> bool:
        """
        Enable profiling for this session. Default is false.
        """
    @enable_profiling.setter
    def enable_profiling(self, arg1: bool) -> None:
        ...
    @property
    def execution_mode(self) -> ExecutionMode:
        """
        Sets the execution mode. Default is sequential.
        """
    @execution_mode.setter
    def execution_mode(self, arg1: ExecutionMode) -> None:
        ...
    @property
    def execution_order(self) -> ExecutionOrder:
        """
        Sets the execution order. Default is basic topological order.
        """
    @execution_order.setter
    def execution_order(self, arg1: ExecutionOrder) -> None:
        ...
    @property
    def graph_optimization_level(self) -> GraphOptimizationLevel:
        """
        Graph optimization level for this session.
        """
    @graph_optimization_level.setter
    def graph_optimization_level(self, arg1: GraphOptimizationLevel) -> None:
        ...
    @property
    def inter_op_num_threads(self) -> int:
        """
        Sets the number of threads used to parallelize the execution of the graph (across nodes). Default is 0 to let onnxruntime choose.
        """
    @inter_op_num_threads.setter
    def inter_op_num_threads(self, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def intra_op_num_threads(self) -> int:
        """
        Sets the number of threads used to parallelize the execution within nodes. Default is 0 to let onnxruntime choose.
        """
    @intra_op_num_threads.setter
    def intra_op_num_threads(self, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def log_severity_level(self) -> int:
        """
        Log severity level. Applies to session load, initialization, etc.
        0:Verbose, 1:Info, 2:Warning. 3:Error, 4:Fatal. Default is 2.
        """
    @log_severity_level.setter
    def log_severity_level(self, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def log_verbosity_level(self) -> int:
        """
        VLOG level if DEBUG build and session_log_severity_level is 0.
        Applies to session load, initialization, etc. Default is 0.
        """
    @log_verbosity_level.setter
    def log_verbosity_level(self, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @property
    def logid(self) -> str:
        """
        Logger id to use for session output.
        """
    @logid.setter
    def logid(self, arg1: str) -> None:
        ...
    @property
    def optimized_model_filepath(self) -> str:
        """
        File path to serialize optimized model to.
        Optimized model is not serialized unless optimized_model_filepath is set.
        Serialized model format will default to ONNX unless:
        - add_session_config_entry is used to set 'session.save_model_format' to 'ORT', or
        - there is no 'session.save_model_format' config entry and optimized_model_filepath ends in '.ort' (case insensitive)
        """
    @optimized_model_filepath.setter
    def optimized_model_filepath(self, arg1: str) -> None:
        ...
    @property
    def profile_file_prefix(self) -> str:
        """
        The prefix of the profile file. The current time will be appended to the file name.
        """
    @profile_file_prefix.setter
    def profile_file_prefix(self, arg1: str) -> None:
        ...
    @property
    def use_deterministic_compute(self) -> bool:
        """
        Whether to use deterministic compute. Default is false.
        """
    @use_deterministic_compute.setter
    def use_deterministic_compute(self, arg1: bool) -> None:
        ...
    @property
    def use_per_session_threads(self) -> bool:
        """
        Whether to use per-session thread pool. Default is True.
        """
    @use_per_session_threads.setter
    def use_per_session_threads(self, arg1: bool) -> None:
        ...
class SparseBlockSparseView:
    def indices(self) -> numpy.ndarray[typing.Any, numpy.dtype[typing.Any]]:
        ...
class SparseCooView:
    def indices(self) -> numpy.ndarray[typing.Any, numpy.dtype[typing.Any]]:
        ...
class SparseCsrView:
    def inner(self) -> numpy.ndarray[typing.Any, numpy.dtype[typing.Any]]:
        ...
    def outer(self) -> numpy.ndarray[typing.Any, numpy.dtype[typing.Any]]:
        ...
class SparseTensor:
    format: OrtSparseFormat
    @staticmethod
    def blocksparse_from_numpy(arg0: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg1: numpy.ndarray[typing.Any, numpy.dtype[typing.Any]], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.int32], arg3: OrtDevice) -> SparseTensor:
        ...
    @staticmethod
    def sparse_coo_from_numpy(arg0: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg1: numpy.ndarray[typing.Any, numpy.dtype[typing.Any]], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.int64], arg3: OrtDevice) -> SparseTensor:
        ...
    @staticmethod
    def sparse_csr_from_numpy(arg0: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], arg1: numpy.ndarray[typing.Any, numpy.dtype[typing.Any]], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.int64], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.int64], arg4: OrtDevice) -> SparseTensor:
        ...
    def data_type(self) -> str:
        ...
    def dense_shape(self) -> list:
        ...
    def device_name(self) -> str:
        ...
    def get_blocksparse_data(self) -> SparseBlockSparseView:
        ...
    def get_coo_data(self) -> SparseCooView:
        ...
    def get_csrc_data(self) -> SparseCsrView:
        ...
    def to_cuda(self, arg0: OrtDevice) -> SparseTensor:
        ...
    def values(self) -> numpy.ndarray[typing.Any, numpy.dtype[typing.Any]]:
        ...
def copy_tensors(arg0: collections.abc.Sequence[OrtValue], arg1: collections.abc.Sequence[OrtValue], arg2: typing.Any) -> None:
    """
    "Copy tensors from sources to destinations using specified stream handle (or None)
    """
def create_and_register_allocator(arg0: OrtMemoryInfo, arg1: OrtArenaCfg) -> None:
    ...
def create_and_register_allocator_v2(arg0: str, arg1: OrtMemoryInfo, arg2: collections.abc.Mapping[str, str], arg3: OrtArenaCfg) -> None:
    ...
def disable_telemetry_events() -> None:
    """
    Disables platform-specific telemetry collection.
    """
def enable_telemetry_events() -> None:
    """
    Enables platform-specific telemetry collection where applicable.
    """
def get_all_operator_schema() -> list[...]:
    """
    Return a vector of OpSchema all registered operators
    """
def get_all_opkernel_def() -> list[...]:
    """
    Return a vector of KernelDef for all registered OpKernels
    """
def get_all_providers() -> list[str]:
    """
    Return list of Execution Providers that this version of Onnxruntime can support. The order of elements represents the default priority order of Execution Providers from highest to lowest.
    """
def get_available_providers() -> list[str]:
    """
    Return list of available Execution Providers in this installed version of Onnxruntime. The order of elements represents the default priority order of Execution Providers from highest to lowest.
    """
def get_build_info() -> str:
    ...
def get_default_session_options() -> ...:
    """
    Return a default session_options instance.
    """
def get_device() -> str:
    """
    Return the device used to compute the prediction (CPU, MKL, ...)
    """
def get_ep_devices() -> list[OrtEpDevice]:
    """
    Get the list of available OrtEpDevice instances.
    """
def get_model_compatibility_for_ep_devices(arg0: collections.abc.Sequence[OrtEpDevice], arg1: str) -> OrtCompiledModelCompatibility:
    """
    "Validate a compiled model's compatibility information for one or more EP devices.
    """
def get_session_initializer() -> ...:
    """
    Return a default session object initializer.
    """
def get_version_string() -> str:
    ...
def has_collective_ops() -> bool:
    ...
def is_dlpack_uint8_tensor(arg0: typing_extensions.CapsuleType) -> bool:
    """
    Tells if a DLPack structure is a uint8 tensor.
    .. note::
        Boolean tensors are also uint8 tensor once converted with DLPack protocol.
    """
@typing.overload
def quantize_matmul_2bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> None:
    ...
@typing.overload
def quantize_matmul_2bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, float16], arg2: typing.Annotated[numpy.typing.ArrayLike, float16], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> None:
    ...
@typing.overload
def quantize_matmul_4bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> None:
    ...
@typing.overload
def quantize_matmul_4bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, float16], arg2: typing.Annotated[numpy.typing.ArrayLike, float16], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> None:
    ...
@typing.overload
def quantize_matmul_8bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> None:
    ...
@typing.overload
def quantize_matmul_8bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, float16], arg2: typing.Annotated[numpy.typing.ArrayLike, float16], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> None:
    ...
@typing.overload
def quantize_matmul_bnb4(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg3: typing.SupportsInt | typing.SupportsIndex, arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex) -> None:
    ...
@typing.overload
def quantize_matmul_bnb4(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, float16], arg2: typing.Annotated[numpy.typing.ArrayLike, float16], arg3: typing.SupportsInt | typing.SupportsIndex, arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex) -> None:
    ...
@typing.overload
def quantize_qdq_matmul_2bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> bool:
    ...
@typing.overload
def quantize_qdq_matmul_2bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, float16], arg2: typing.Annotated[numpy.typing.ArrayLike, float16], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> bool:
    ...
@typing.overload
def quantize_qdq_matmul_4bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg2: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> bool:
    ...
@typing.overload
def quantize_qdq_matmul_4bits(arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg1: typing.Annotated[numpy.typing.ArrayLike, float16], arg2: typing.Annotated[numpy.typing.ArrayLike, float16], arg3: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], arg4: typing.SupportsInt | typing.SupportsIndex, arg5: typing.SupportsInt | typing.SupportsIndex, arg6: typing.SupportsInt | typing.SupportsIndex, arg7: bool) -> bool:
    ...
def register_execution_provider_library(arg0: str, arg1: str) -> None:
    """
    Register an execution provider library with ONNX Runtime.
    """
def register_tensorrt_plugins_as_custom_ops(arg0: ..., arg1: collections.abc.Mapping[str, str]) -> None:
    """
    Register TensorRT plugins as custom ops.
    """
def set_arena_extend_strategy(arg0: ...) -> None:
    ...
def set_cuda_device_id(arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
    ...
def set_cudnn_conv_algo_search(arg0: OrtCudnnConvAlgoSearch) -> None:
    ...
def set_default_logger_severity(arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
    """
    Sets the default logging severity. 0:Verbose, 1:Info, 2:Warning, 3:Error, 4:Fatal
    """
def set_default_logger_verbosity(arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
    """
    Sets the default logging verbosity level. To activate the verbose log, you need to set the default logging severity to 0:Verbose level.
    """
def set_do_copy_in_default_stream(arg0: bool) -> None:
    ...
def set_global_thread_pool_sizes(intra_op_num_threads: typing.SupportsInt | typing.SupportsIndex = 0, inter_op_num_threads: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
    """
    Set the number of threads used by the global thread pools for intra and inter op parallelism.
    """
def set_gpu_mem_limit(arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
    ...
def set_seed(arg0: typing.SupportsInt | typing.SupportsIndex) -> None:
    """
    Sets the seed used for random number generation in Onnxruntime.
    """
def unregister_execution_provider_library(arg0: str) -> None:
    """
    Unregister an execution provider library from ONNX Runtime.
    """
kNextPowerOfTwo: ArenaExtendStrategy  # value = <ArenaExtendStrategy.kNextPowerOfTwo: 0>
kSameAsRequested: ArenaExtendStrategy  # value = <ArenaExtendStrategy.kSameAsRequested: 1>
