"""
共通の定数を置いておくモジュール
"""

import numpy as np
import pytest
from argus_synchro_lib.octotree import OctoTree
from directory_config_helper import dev_directory_config

from argus_synchro import shared_app_config
from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.config.app_config import AppConfig


@pytest.fixture()
def directory_config() -> paths.DirectoryConfig:
    return dev_directory_config()


@pytest.fixture()
def app_logger_factory() -> AppLoggerFactory:
    return AppLoggerFactory(to_console=False)


@pytest.fixture()
def app_config(directory_config: paths.DirectoryConfig) -> AppConfig:
    sac = shared_app_config.SharedAppConfig(directory_config)
    return sac.read()


@pytest.fixture()
def octotree_obj(app_config: AppConfig) -> OctoTree:
    octotree_conf = app_config.OctoTree
    return OctoTree(
        max_xyz=np.array(octotree_conf.max_xyz),
        min_xyz=np.array(octotree_conf.min_xyz),
        max_tree_depth=octotree_conf.max_tree_depth,
        use_node_stats=octotree_conf.use_node_stats,
        quantile=None,
        origin_w2oct=np.array([0, 0, 0]),
    )


@pytest.fixture()
def remove_dist_tuple(
    app_config: AppConfig,
    octotree_obj: OctoTree,
) -> tuple[float, float, float]:
    return tuple(octotree_obj.cell_interval * app_config.OctoTree.remove_dist)
