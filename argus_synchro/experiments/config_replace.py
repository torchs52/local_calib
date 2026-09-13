"""config.app_config.pyに影響を及ぼす関数のモジュール
基本的にapp_configのdataclassは不変なので変えられないので、メインのモジュールに入れないが、開発をする際にあると便利なので、関数を作っておく
"""

from dataclasses import asdict, dataclass

from argus_synchro.config import app_config as app_config_module


def with_frozen_app_config(
    conf: dataclass,
    **kwargs: dict,
) -> dataclass:
    updated_vars = {
        var_name: kwargs[var_name] if var_name in kwargs else var_val
        for var_name, var_val in asdict(conf).items()  # vars(conf).items()
    }
    return getattr(app_config_module, conf.__class__.__name__)(**updated_vars)
