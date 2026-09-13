class NotStartedError(RuntimeError): ...


class ArgusSeverityAError(Exception):
    """
    重要度Aの例外が発生した時に投げるクラス
    """


class ArgusSeverityBIconError(Exception):
    """
    重要度Bの例外でアイコン確認表示が必要なエラーが発生した時に投げるクラス
    """


class ArgusSeverityBTimeError(Exception):
    """
    重要度Bの例外で時間経過で消灯するエラーが発生した時に投げるクラス
    """


class ArgusSeverityCError(Exception):
    """
    重要度Cの例外発生した時に投げるクラス
    """


class ArgusSeverityDError(Exception):
    """
    重要度Dの例外が発生した時に投げるクラス
    """
