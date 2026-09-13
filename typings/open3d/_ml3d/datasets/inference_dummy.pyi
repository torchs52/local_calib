from __future__ import annotations
import logging as logging
import open3d._ml3d.datasets.base_dataset
from open3d._ml3d.datasets.base_dataset import BaseDatasetSplit
from open3d._ml3d.utils.builder import get_module
import open3d._ml3d.utils.registry
import typing
__all__: list[str] = ['BaseDatasetSplit', 'DATASET', 'InferenceDummySplit', 'get_module', 'log', 'logging']
class InferenceDummySplit(open3d._ml3d.datasets.base_dataset.BaseDatasetSplit):
    __abstractmethods__: typing.ClassVar[frozenset]  # value = frozenset()
    _abc_impl: typing.ClassVar[_abc._abc_data]  # value = <_abc._abc_data object>
    def __init__(self, inference_data):
        ...
    def __len__(self):
        ...
    def get_attr(self, idx):
        ...
    def get_data(self, idx):
        ...
DATASET: open3d._ml3d.utils.registry.Registry  # value = <open3d._ml3d.utils.registry.Registry object>
log: logging.Logger  # value = <Logger open3d._ml3d.datasets.inference_dummy (INFO)>
