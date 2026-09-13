from pathlib import Path

from argus_synchro.common import paths


def dev_directory_config() -> paths.DirectoryConfig:
    repo_root = Path.cwd()
    directory_config, _ = paths.load_directory_config_from_ini(
        paths.DirectoryConfig(
            config_dir=paths.normalize_path("./config", repo_root),
            log_dir=paths.normalize_path("./log", repo_root),
            mmap_dir=paths.DEFAULT_MMAP_DIR,
        )
    )
    return directory_config
