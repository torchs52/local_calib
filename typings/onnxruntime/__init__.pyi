"""

ONNX Runtime is a performance-focused scoring engine for Open Neural Network Exchange (ONNX) models.
For more information on ONNX Runtime, please see `aka.ms/onnxruntime <https://aka.ms/onnxruntime/>`_
or the `Github project <https://github.com/microsoft/onnxruntime/>`_.
"""
from __future__ import annotations
import contextlib as contextlib
from onnxruntime.capi.onnxruntime_inference_collection import AdapterFormat
from onnxruntime.capi.onnxruntime_inference_collection import IOBinding
from onnxruntime.capi.onnxruntime_inference_collection import InferenceSession
from onnxruntime.capi.onnxruntime_inference_collection import ModelCompiler
from onnxruntime.capi.onnxruntime_inference_collection import OrtDevice
from onnxruntime.capi.onnxruntime_inference_collection import OrtValue
from onnxruntime.capi.onnxruntime_inference_collection import SparseTensor
from onnxruntime.capi.onnxruntime_inference_collection import copy_tensors
from onnxruntime.capi.onnxruntime_pybind11_state import ExecutionMode
from onnxruntime.capi.onnxruntime_pybind11_state import ExecutionOrder
from onnxruntime.capi.onnxruntime_pybind11_state import GraphOptimizationLevel
from onnxruntime.capi.onnxruntime_pybind11_state import LoraAdapter
from onnxruntime.capi.onnxruntime_pybind11_state import ModelMetadata
from onnxruntime.capi.onnxruntime_pybind11_state import NodeArg
from onnxruntime.capi.onnxruntime_pybind11_state import OrtAllocatorType
from onnxruntime.capi.onnxruntime_pybind11_state import OrtArenaCfg
from onnxruntime.capi.onnxruntime_pybind11_state import OrtCompileApiFlags
from onnxruntime.capi.onnxruntime_pybind11_state import OrtDeviceMemoryType
from onnxruntime.capi.onnxruntime_pybind11_state import OrtEpAssignedNode
from onnxruntime.capi.onnxruntime_pybind11_state import OrtEpAssignedSubgraph
from onnxruntime.capi.onnxruntime_pybind11_state import OrtEpDevice
from onnxruntime.capi.onnxruntime_pybind11_state import OrtExecutionProviderDevicePolicy
from onnxruntime.capi.onnxruntime_pybind11_state import OrtExternalInitializerInfo
from onnxruntime.capi.onnxruntime_pybind11_state import OrtHardwareDevice
from onnxruntime.capi.onnxruntime_pybind11_state import OrtHardwareDeviceType
from onnxruntime.capi.onnxruntime_pybind11_state import OrtMemType
from onnxruntime.capi.onnxruntime_pybind11_state import OrtMemoryInfo
from onnxruntime.capi.onnxruntime_pybind11_state import OrtMemoryInfoDeviceType
from onnxruntime.capi.onnxruntime_pybind11_state import OrtSparseFormat
from onnxruntime.capi.onnxruntime_pybind11_state import OrtSyncStream
from onnxruntime.capi.onnxruntime_pybind11_state import RunOptions
from onnxruntime.capi.onnxruntime_pybind11_state import SessionIOBinding
from onnxruntime.capi.onnxruntime_pybind11_state import SessionOptions
from onnxruntime.capi.onnxruntime_pybind11_state import create_and_register_allocator
from onnxruntime.capi.onnxruntime_pybind11_state import create_and_register_allocator_v2
from onnxruntime.capi.onnxruntime_pybind11_state import disable_telemetry_events
from onnxruntime.capi.onnxruntime_pybind11_state import enable_telemetry_events
from onnxruntime.capi.onnxruntime_pybind11_state import get_all_providers
from onnxruntime.capi.onnxruntime_pybind11_state import get_available_providers
from onnxruntime.capi.onnxruntime_pybind11_state import get_build_info
from onnxruntime.capi.onnxruntime_pybind11_state import get_device
from onnxruntime.capi.onnxruntime_pybind11_state import get_ep_devices
from onnxruntime.capi.onnxruntime_pybind11_state import get_version_string
from onnxruntime.capi.onnxruntime_pybind11_state import has_collective_ops
from onnxruntime.capi.onnxruntime_pybind11_state import register_execution_provider_library
from onnxruntime.capi.onnxruntime_pybind11_state import set_default_logger_severity
from onnxruntime.capi.onnxruntime_pybind11_state import set_default_logger_verbosity
from onnxruntime.capi.onnxruntime_pybind11_state import set_global_thread_pool_sizes
from onnxruntime.capi.onnxruntime_pybind11_state import set_seed
from onnxruntime.capi.onnxruntime_pybind11_state import unregister_execution_provider_library
from onnxruntime.capi import onnxruntime_validation
from . import capi
__all__: list[str] = ['AdapterFormat', 'ExecutionMode', 'ExecutionOrder', 'GraphOptimizationLevel', 'IOBinding', 'InferenceSession', 'LoraAdapter', 'ModelCompiler', 'ModelMetadata', 'NodeArg', 'OrtAllocatorType', 'OrtArenaCfg', 'OrtCompileApiFlags', 'OrtDevice', 'OrtDeviceMemoryType', 'OrtEpAssignedNode', 'OrtEpAssignedSubgraph', 'OrtEpDevice', 'OrtExecutionProviderDevicePolicy', 'OrtExternalInitializerInfo', 'OrtHardwareDevice', 'OrtHardwareDeviceType', 'OrtMemType', 'OrtMemoryInfo', 'OrtMemoryInfoDeviceType', 'OrtSparseFormat', 'OrtSyncStream', 'OrtValue', 'RunOptions', 'SessionIOBinding', 'SessionOptions', 'SparseTensor', 'capi', 'contextlib', 'copy_tensors', 'create_and_register_allocator', 'create_and_register_allocator_v2', 'cuda_version', 'disable_telemetry_events', 'enable_telemetry_events', 'get_all_providers', 'get_available_providers', 'get_build_info', 'get_device', 'get_ep_devices', 'get_version_string', 'has_collective_ops', 'import_capi_exception', 'onnxruntime_validation', 'package_name', 'preload_dlls', 'print_debug_info', 'register_execution_provider_library', 'set_default_logger_severity', 'set_default_logger_verbosity', 'set_global_thread_pool_sizes', 'set_seed', 'unregister_execution_provider_library', 'version']
def _extract_cuda_major_version(version_str: str) -> str:
    """
    Extract CUDA major version from version string (e.g., '12.1' -> '12').
    
        Args:
            version_str: CUDA version string to parse
    
        Returns:
            Major version as string, or "12" if parsing fails
        
    """
def _get_cufft_version(cuda_major: str) -> str:
    """
    Get cufft library version based on CUDA major version.
    
        Args:
            cuda_major: CUDA major version as string (e.g., "12", "13")
    
        Returns:
            cufft version as string
        
    """
def _get_nvidia_dll_paths(is_windows: bool, cuda: bool = True, cudnn: bool = True):
    ...
def _get_package_root(package_name: str, directory_name: str | None = None):
    ...
def _get_package_version(package_name: str):
    ...
def preload_dlls(cuda: bool = True, cudnn: bool = True, msvc: bool = True, directory = None):
    """
    Preload CUDA 12.x+ and cuDNN 9.x DLLs in Windows or Linux, and MSVC runtime DLLs in Windows.
    
           When the installed PyTorch is compatible (using same major version of CUDA and cuDNN),
           there is no need to call this function if `import torch` is done before `import onnxruntime`.
    
        Args:
            cuda (bool, optional): enable loading CUDA DLLs. Defaults to True.
            cudnn (bool, optional): enable loading cuDNN DLLs. Defaults to True.
            msvc (bool, optional): enable loading MSVC DLLs in Windows. Defaults to True.
            directory(str, optional): a directory contains CUDA or cuDNN DLLs. It can be an absolute path,
               or a path relative to the directory of this file.
               If directory is None (default value), the search order: the lib directory of compatible PyTorch in Windows,
                nvidia site packages, default DLL loading paths.
               If directory is empty string (""), the search order: nvidia site packages, default DLL loading paths.
               If directory is a path, the search order: the directory, default DLL loading paths.
        
    """
def print_debug_info():
    """
    Print information to help debugging.
    """
__author__: str = 'Microsoft'
__version__: str = '1.24.4'
cuda_version: str = '12.8'
import_capi_exception = None
package_name: str = 'onnxruntime-gpu'
version: str = '1.24.4'
