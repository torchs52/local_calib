from __future__ import annotations

import argparse
import os
from configparser import ConfigParser, ExtendedInterpolation
from dataclasses import dataclass
from pathlib import Path

PathLike = str | Path

MACHINE_MODEL_INFO: dict[str, dict[str, str]] = {
    "SCX700-3": {
        "description": "not_adapted",
        "param_file": "./SCX700-3_settings.ini",
    },
    "SCX900-3": {
        "description": "default",
        "param_file": "./SCX900-3_settings.ini",
    },
    "SCX2000-3": {
        "description": "default",
        "param_file": "./SCX2000-3_settings.ini",
    },
    "SCX3500-3": {
        "description": "memo",
        "param_file": "./SCX3500-3_settings.ini",
    },
}

# 校正モード(calib_settings.ini)用の機種別パラメータファイル
MACHINE_MODEL_CALIB_INFO: dict[str, dict[str, str]] = {
    "SCX700-3": {
        "description": "not_adapted",
        "param_file": "./SCX700-3_calib_settings.ini",
    },
    "SCX900-3": {
        "description": "default",
        "param_file": "./SCX900-3_calib_settings.ini",
    },
    "SCX2000-3": {
        "description": "default",
        "param_file": "./SCX2000-3_calib_settings.ini",
    },
    "SCX3500-3": {
        "description": "memo",
        "param_file": "./SCX3500-3_calib_settings.ini",
    },
}


def normalize_path(
    value: PathLike,
    base_dir: Path,
    *,
    resolve: bool = True,
) -> Path:
    """
    設定等から来るパス(相対/絶対混在)を正規化してPathで返す。

    - 絶対パス: そのまま(必要なら正規化)
    - 相対パス: base_dir を基準に結合して絶対化
    - `~` 展開、環境変数展開に対応
    """
    s: str = os.path.expandvars(str(value))
    p = Path(s).expanduser()

    if not p.is_absolute():
        p = base_dir / p

    if resolve:
        return p.resolve(strict=False)
    return p.absolute()


def get_path_list(text: str, base_dir: Path) -> list[str]:
    t0: str = text.replace("[", "").replace("]", "")
    t1: str = t0.strip(",")
    t2: list[str] = t1.split(",")
    path_list = [item.strip() for item in t2 if item.strip()]
    return [str(normalize_path(item, base_dir, resolve=True)) for item in path_list]


DEFAULT_MMAP_DIR = Path("/dev/shm")


@dataclass(frozen=True, slots=True)
class DirectoryConfig:
    config_dir: Path
    log_dir: Path
    mmap_dir: Path


DEFAULT_DIRECTORY_CONFIG = DirectoryConfig(
    config_dir=Path("/opt/argus3d/config/core"),
    log_dir=Path("/var/log/argus3d"),
    mmap_dir=DEFAULT_MMAP_DIR,
)


def directory_config_from_args(args: argparse.Namespace) -> DirectoryConfig:
    """CLI引数からDirectoryConfigを構築する。"""
    repo_root = Path.cwd()
    return DirectoryConfig(
        config_dir=normalize_path(args.config_dir, repo_root),
        log_dir=normalize_path(args.log_dir, repo_root),
        mmap_dir=normalize_path(args.mmap_dir, repo_root),
    )


def parse_directory_config() -> DirectoryConfig:
    """CLI引数を解析しsettings.iniを読んでDirectoryConfigを返す。"""
    cli_args = parse_args()
    cli_directory_config = directory_config_from_args(cli_args)
    directory_config, _ = load_directory_config_from_ini(cli_directory_config)
    return directory_config


def resolve_ini_roots(
    ini: ConfigParser,
    directory_config: DirectoryConfig,
) -> DirectoryConfig:
    """DEFAULTのルートディレクトリを解決して書き戻す。"""
    ini["DEFAULT"]["config_dir"] = str(directory_config.config_dir)
    ini["DEFAULT"]["log_dir"] = str(directory_config.log_dir)
    ini["DEFAULT"]["mmap_dir"] = str(directory_config.mmap_dir)
    ini["DEFAULT"]["data_dir"] = str(
        normalize_path(ini["DEFAULT"]["data_dir"], Path.cwd())
    )
    return directory_config


def load_directory_config_from_ini(
    directory_config: DirectoryConfig,
) -> tuple[DirectoryConfig, ConfigParser]:
    """config_dir/settings.iniを読み、ルートディレクトリを解決する。"""
    from argus_synchro.config.settings_validation import validate_settings

    settings_ini_path = get_config_dir(directory_config, "settings.ini")
    ini = ConfigParser(interpolation=ExtendedInterpolation())
    ini.read(str(settings_ini_path), encoding="utf-8")
    validate_settings(ini)
    directory_config = resolve_ini_roots(ini, directory_config)
    return directory_config, ini


def get_config_dir(directory_config: DirectoryConfig, *parts: str) -> Path:
    """config_dirを基点にしたパスを返す。"""
    return directory_config.config_dir.joinpath(*parts)


def get_log_dir(directory_config: DirectoryConfig, *parts: str) -> Path:
    """log_dirを基点にしたパスを返す。"""
    return directory_config.log_dir.joinpath(*parts)


def get_mmap_dir(directory_config: DirectoryConfig, *parts: str) -> Path:
    """mmap_dirを基点にしたパスを返す。"""
    return directory_config.mmap_dir.joinpath(*parts)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    d: DirectoryConfig = DEFAULT_DIRECTORY_CONFIG
    p.add_argument("--config-dir", default=str(d.config_dir))
    p.add_argument("--log-dir", default=str(d.log_dir))
    p.add_argument("--mmap-dir", default=str(d.mmap_dir))
    args, _ = p.parse_known_args()
    return args
