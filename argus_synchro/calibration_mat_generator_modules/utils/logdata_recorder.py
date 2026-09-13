import datetime
import os
import pickle
from typing import Any


def logdata_recorder(obj: Any, filepath: str | None, append_mode: bool = True):
    if not filepath:
        dirname = "calib_logdata" + datetime.datetime.now().strftime("%y%m%d")
        filepath = os.path.join(dirname, f"process{os.getpid()}_log.pickle")
    else:
        dirname = os.path.dirname(filepath)

    if not os.path.isdir(dirname):
        os.makedirs(dirname)

    if append_mode:
        openmode = "ab"
    else:
        openmode = "wb"
    with open(filepath, openmode) as wbf:
        pickle.dump(obj, wbf)
