from enum import Flag, IntEnum, auto


class ProfMode(IntEnum):
    """プロファイルモード"""

    No = auto()
    VizTracereMain = auto()
    VizTracereTarget = auto()
    Fps = auto()


class ProfCategory(Flag):
    """プロファイルカテゴリ"""

    Process = auto()
    Message = auto()
    All = Process | Message
