from configparser import ConfigParser
from pathlib import Path

from argus_synchro.common import paths
from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory
from argus_synchro.common.paths import MACHINE_MODEL_CALIB_INFO, MACHINE_MODEL_INFO
from argus_synchro.config.app_config import AppConfig
from argus_synchro.config.startup_reset_policy import StartupResetPolicy


class NoCaseConfigParser(ConfigParser):
    """ConfigParserのサブクラスで、option名の大小を区別する"""

    def optionxform(self, optionstr: str) -> str:
        return optionstr  # デフォルトの小文字変換を無効にする


class MachineProfileHandler:
    _class_logger: AppLogger = AppLoggerFactory.from_name("MachineProfileHandler")

    @classmethod
    def log_register(cls, app_logger_factory: AppLoggerFactory) -> None:
        cls._class_logger = app_logger_factory.register_from_name(cls.__name__)

    def __init__(
        self,
        app_logger_factory: AppLoggerFactory,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> None:
        self._directory_config = directory_config
        self._logger: AppLogger = app_logger_factory.register_from_type(self.__class__)
        self._app_logger_factory = app_logger_factory

    @classmethod
    def _get_crane_model_name(
        cls,
        directory_config: paths.DirectoryConfig,
    ) -> str | None:
        # crane_modelは常にsettings.iniで管理されるため、operation_modeによらず参照する
        base_app_ini: NoCaseConfigParser = NoCaseConfigParser(
            interpolation=None,  # プレースホルダーの展開を無効にする
        )
        settings_ini_path = paths.get_config_dir(directory_config, "settings.ini")
        base_app_ini.read(settings_ini_path, encoding="utf-8")

        # crane_model の取得
        model_name: str | None = base_app_ini.get("UI_IF", "crane_model", fallback=None)
        if model_name is None:
            cls._class_logger.critical(
                "crane_model が設定ファイルに見つかりません。",
            )
        return model_name

    @classmethod
    def _get_model_specific_file_path(
        cls,
        directory_config: paths.DirectoryConfig,
        model_info: dict[str, dict[str, str]],
    ) -> Path | None:
        # settings.iniのcrane_modelで指定したモデルの機種別設定ファイルのパスを取得
        model_name: str | None = cls._get_crane_model_name(directory_config)
        if model_name is None:
            return None

        if model_name in model_info:
            param_file: str = model_info[model_name]["param_file"]
            config_dir: Path = paths.get_config_dir(directory_config)
            param_file_path: Path = paths.normalize_path(param_file, config_dir)
            if param_file_path.exists():
                return param_file_path
            cls._class_logger.critical(f"{param_file} が存在しません。")
        else:
            cls._class_logger.critical(
                f"不明なモデル名: {model_name}, possible: {model_info.keys()}"
            )
        return None

    @classmethod
    def get_model_specific_config_file_path(
        cls,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> Path | None:
        # 周辺監視モード用(settings.ini)の機種別設定ファイルのパスを取得
        return cls._get_model_specific_file_path(directory_config, MACHINE_MODEL_INFO)

    @classmethod
    def get_model_specific_calib_config_file_path(
        cls,
        directory_config: paths.DirectoryConfig = paths.DEFAULT_DIRECTORY_CONFIG,
    ) -> Path | None:
        # 校正モード用(calib_settings.ini)の機種別設定ファイルのパスを取得
        return cls._get_model_specific_file_path(
            directory_config, MACHINE_MODEL_CALIB_INFO
        )

    def _apply_model_specific_config_to(
        self,
        target_ini_name: str,
        model_info: dict[str, dict[str, str]],
        base_app_ini: NoCaseConfigParser | None = None,
    ) -> None:
        """
        モデルに基づいて設定を適用するメソッド。
        設定ファイルにコメント行を保持しながら、指定された設定を上書きします。
        """
        target_ini_path = paths.get_config_dir(self._directory_config, target_ini_name)
        if base_app_ini is None:
            # 基本設定ファイルの読み込み
            base_app_ini = NoCaseConfigParser(
                interpolation=None,  # プレースホルダーの展開を無効にする
            )

            base_app_ini.read(target_ini_path, encoding="utf-8")

        param_file_path: Path | None = self._get_model_specific_file_path(
            self._directory_config, model_info
        )
        if param_file_path is None:
            return
        # モデルが辞書に登録されていれば、該当パラメータを読み込む

        model_config: NoCaseConfigParser = NoCaseConfigParser(
            interpolation=None,  # プレースホルダーの展開を無効にする
        )
        try:
            model_config.read(param_file_path, encoding="utf-8")
        except Exception as e:
            self._logger.critical(
                f"パラメータファイルの読み込みエラー: {e}",
            )
            return

        # 必要な設定項目だけを上書き(すでに存在するキーだけコピー)
        for section in model_config.sections():
            if not base_app_ini.has_section(section):
                continue
            for key, value in model_config.items(section):
                if base_app_ini.has_option(section, key):
                    base_app_ini.set(section, key, value)

        # 設定ファイルの元のテキストを読み込む
        # (for usersを設定していた.)
        with open(target_ini_path, encoding="utf-8") as f:
            lines: list[str] = f.readlines()

        # コメントをそのまま保持して、設定を更新
        output_path: Path = Path(target_ini_path)
        current_section: str | None = None  # 現在のセクションを保持

        with open(output_path, "w", encoding="utf-8") as configfile:
            for raw_line in lines:
                line = raw_line.rstrip()  # 行の末尾の空白を削除

                # セクションの開始([Section])を検出
                if line.startswith("[") and line.endswith("]"):
                    current_section = line[1:-1]  # セクション名を更新

                if line.startswith("#"):
                    configfile.write(
                        line + "\n",
                    )  # コメント行はそのまま書き込み
                elif "=" in line:  # 設定行は処理して書き換える
                    key_value: list[str] = line.split("=", 1)
                    if len(key_value) == 2:
                        key: str = key_value[0].strip()
                        value: str = key_value[1].strip()

                        # セクション内の設定項目を上書きする処理
                        if current_section and base_app_ini.has_option(
                            current_section,
                            key,
                        ):
                            new_value: str = base_app_ini.get(
                                current_section,
                                key,
                            )
                            line = f"{key} = {new_value}\n"  # 設定値を更新

                    configfile.write(line)  # 更新した設定行を書き込む
                else:
                    configfile.write(
                        line + "\n",
                    )  # その他の行(空白行など)もそのまま書き込み

        model_name: str | None = self._get_crane_model_name(self._directory_config)
        self._logger.info(
            f"{model_name} 用の設定で{target_ini_name}を更新しました。",
        )

        return

    def apply_model_specific_config(self) -> None:
        """周辺監視モード用の設定(settings.ini)にモデル別パラメータを適用する。"""
        base_app_ini: NoCaseConfigParser = NoCaseConfigParser(
            interpolation=None,  # プレースホルダーの展開を無効にする
        )
        settings_ini_path = paths.get_config_dir(
            self._directory_config, "settings.ini"
        )
        base_app_ini.read(settings_ini_path, encoding="utf-8")

        # 起動時リセット
        StartupResetPolicy(self._app_logger_factory).apply(base_app_ini)
        self.current_appconfig = AppConfig(base_app_ini, self._directory_config)
        self._apply_model_specific_config_to(
            "settings.ini", MACHINE_MODEL_INFO, base_app_ini=base_app_ini
        )

    def apply_model_specific_calib_config(self) -> None:
        """
        校正モード用の設定(calib_settings.ini)にモデル別パラメータを適用する。
        """
        self._apply_model_specific_config_to(
            "calib_settings.ini", MACHINE_MODEL_CALIB_INFO
        )
