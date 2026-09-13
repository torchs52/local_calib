#!/usr/bin/env python3

import sys

from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLoggerFactory
from argus_synchro.SystemMonitor.status_mmap import StatusMMAP


def get_status() -> None:
    raw_mode = "--raw" in sys.argv
    directory_config = paths.parse_directory_config()

    try:
        logger = AppLoggerFactory.from_name("StatusMMAP", to_console=False)
        status = StatusMMAP(
            logger,
            create=False,
            directory_config=directory_config,
        )
        code = status.read_status()
        status.close()

        if raw_mode:
            print(code)
        else:
            name = StatusMMAP.get_status_name(code)
            print(f"{code} {name}")

    except Exception as e:
        if raw_mode:
            print(-1)
        else:
            print("-1 ERROR")
        print(f"ERROR: {e}", file=sys.stderr)


if __name__ == "__main__":
    get_status()
