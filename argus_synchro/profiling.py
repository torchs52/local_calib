import argparse
import datetime
import re
import subprocess
import sys
from configparser import ConfigParser, ExtendedInterpolation
from pathlib import Path

from argus_synchro.common import paths
from argus_synchro.config.app_config import AppConfig
from argus_synchro.profiler import ProfInfo, ProfMode, ProfSharedWriter
from argus_synchro.profiler.prof_mode import ProfCategory

PY_MODULE = "argus_synchro"
DEFAULT_TRACE_TARGET_PROCESSES: tuple[str, ...] = (
    "CameraProviderProcess*",
    "PointsProviderProcess*",
    "ImuProviderProcess*",
    "CanDataProviderProcess",
    "GetDataProcess",
    "PointsRefineProcess",
    "ObjectDetectProcess",
    "VisualProcess",
)


def prepare_cache() -> None:
    """実行時コンパイルのキャッシュ生成"""
    subprocess.run(
        [sys.executable, "-m", "compileall", PY_MODULE],
        check=False,
        stdout=subprocess.DEVNULL,
    )


def get_file_input(
    config_path: Path,
    directory_config: paths.DirectoryConfig,
) -> bool:
    """性能測定時、ファイル入力モード設定読み取り"""
    app_ini = ConfigParser(interpolation=ExtendedInterpolation())
    app_ini.read(config_path, "UTF-8")
    app_config = AppConfig(app_ini, directory_config)
    return app_config.DEFAULT.File_Input


def get_mode_name(file_input: bool) -> str:
    """モード名取得"""
    return "file" if file_input else "sensor"


def create_out_dir(base_dir: Path) -> Path:
    """出力フォルダ準備"""
    now = datetime.datetime.now()
    out_dir = base_dir / f"result/{now:%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def prof_target(
    shared: ProfSharedWriter,
    mode_name: str,
    out_dir: Path,
    config_dir: Path,
    extra_args: list[str],
    category: ProfCategory = ProfCategory.All,
    dur_limit_us: float = 0.0,
    target_processes: tuple[str, ...] = DEFAULT_TRACE_TARGET_PROCESSES,
    tracer_entries: int = 200000,
    max_stack_depth: int = 8,
) -> None:
    """指定したプロセスのみを対象とした性能測定"""
    shared.set(
        ProfInfo(
            mode=ProfMode.VizTracereTarget,
            out_dir=str(out_dir),
            category=category,
            dur_limit_us=dur_limit_us,
            target_processes=target_processes,
            tracer_entries=tracer_entries,
            max_stack_depth=max_stack_depth,
            ignore_c_function=False,
            minimize_memory=True,
        ),
    )
    # NOTE:
    # 親プロセスをviztracerで包むとspawn子プロセスも自動フックされてメモリを圧迫するため、
    # アプリは通常起動し、指定した子プロセスのみProcessBase内で個別トレースする。
    subprocess.run(
        [sys.executable, "-m", PY_MODULE, "--config-dir", str(config_dir), *extra_args],
        check=False,
    )

    output_file = out_dir / f"result_target_{mode_name}.json"
    trace_files = sorted(out_dir.glob("trace_*.json"))
    if len(trace_files) == 0:
        print(f"[WARN] trace file not found: {out_dir}")  # noqa: T201
        return

    subprocess.run(
        [
            sys.executable,
            "-m",
            "viztracer",
            "--combine",
            *[str(file) for file in trace_files],
            "-o",
            str(output_file),
        ],
        check=False,
    )


def prof_fps(
    shared: ProfSharedWriter,
    out_dir: Path,
    config_dir: Path,
    extra_args: list[str],
) -> None:
    shared.set(
        ProfInfo(mode=ProfMode.Fps, out_dir=str(out_dir)),
    )

    subprocess.run(
        [sys.executable, "-m", PY_MODULE, "--config-dir", str(config_dir), *extra_args],
        check=False,
    )


def update_frame_range(s_frame: int, e_frame: int, config_path: Path) -> None:
    """config/settings.iniのs_frameとe_frameを指定した数値に更新する

    Args:
        s_frame: 開始フレーム番号
        e_frame: 終了フレーム番号
        config_path: 設定ファイルのパス (デフォルト: "./config/settings.ini")
    """
    if not config_path.exists():
        raise FileNotFoundError(f"設定ファイルが見つかりません: {config_path}")

    # 設定ファイルを読み込む
    with open(config_path, encoding="UTF-8") as f:
        content = f.read()

    # s_frame = <数値> の行を置換 (コメントアウトされていない行のみ)
    content = re.sub(
        r"^s_frame\s*=\s*\d+", f"s_frame = {s_frame}", content, flags=re.MULTILINE
    )

    # e_frame = <数値> の行を置換 (コメントアウトされていない行のみ)
    content = re.sub(
        r"^e_frame\s*=\s*\d+", f"e_frame = {e_frame}", content, flags=re.MULTILINE
    )

    # 設定ファイルに書き込む
    with open(config_path, "w", encoding="UTF-8") as f:
        f.write(content)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="argus_synchro profiling runner")
    parser.add_argument(
        "--config-dir",
        default="./config",
        help="設定ファイル(config)ディレクトリのパス",
    )
    parser.add_argument("--log-dir", default=None)
    parser.add_argument("--mmap-dir", default=None)
    parser.add_argument(
        "--base-dir",
        default=".",
        help="計測結果の出力先ベースディレクトリのパス",
    )
    parser.add_argument(
        "--profile-scope",
        choices=["section1", "section2", "section3", "overall"],
        default=None,
        help=(
            "実行対象を選択 (section1/section2/section3/overall)。"
            "未指定の場合は全区間を実行"
        ),
    )
    parser.add_argument(
        "--prof-run",
        choices=["fps", "target", "all"],
        default="all",
        help=(
            "計測フェーズを選択。"
            "fps: prof_fpsのみ, target: prof_targetのみ, all: 両方 (従来どおり)"
        ),
    )
    args, extra_args = parser.parse_known_args()
    args.extra_args = extra_args
    return args


def build_argus_extra_args(
    directory_config: paths.DirectoryConfig,
    extra_args: list[str],
) -> list[str]:
    return [
        "--log-dir",
        str(directory_config.log_dir),
        "--mmap-dir",
        str(directory_config.mmap_dir),
        *extra_args,
    ]


def print_results(out_dirs: list[Path]) -> None:
    """計測結果ファイル一覧出力"""
    for out_dir in out_dirs:
        for file in out_dir.glob("*"):
            print(file)  # noqa: T201


def run_section_prof(
    shared: ProfSharedWriter,
    out_dir: Path,
    config_dir: Path,
    argus_extra_args: list[str],
    prof_run: str,
    mode_name: str,
    category: ProfCategory,
    target_processes: tuple[str, ...],
) -> None:
    if prof_run in ("fps", "all"):
        prof_fps(shared, out_dir, config_dir, argus_extra_args)
    if prof_run in ("target", "all"):
        prof_target(
            shared,
            mode_name,
            out_dir,
            config_dir=config_dir,
            extra_args=argus_extra_args,
            category=category,
            target_processes=target_processes,
        )


def main() -> None:
    """性能測定を実施する"""
    args = parse_args()
    directory_config, _ = paths.load_directory_config_from_ini(
        paths.directory_config_from_args(args)
    )
    settings_path = paths.get_config_dir(directory_config, "settings.ini")
    argus_extra_args = build_argus_extra_args(directory_config, args.extra_args)
    base_dir = Path(args.base_dir)
    file_input = get_file_input(settings_path, directory_config)
    mode_name = get_mode_name(file_input)
    category = ProfCategory.Process
    target_processes = DEFAULT_TRACE_TARGET_PROCESSES
    prof_run: str = args.prof_run
    out_dirs: list[Path] = []

    # 事前処理
    prepare_cache()

    # 性能測定実行
    with ProfSharedWriter() as shared:
        if args.profile_scope in (None, "section1"):
            out_dir = create_out_dir(base_dir)
            out_dirs.append(out_dir)
            update_frame_range(
                s_frame=9300 - 10, e_frame=9350, config_path=settings_path
            )
            run_section_prof(
                shared,
                out_dir,
                directory_config.config_dir,
                argus_extra_args,
                prof_run,
                mode_name,
                category,
                target_processes,
            )

        if args.profile_scope in (None, "section2"):
            out_dir = create_out_dir(base_dir)
            out_dirs.append(out_dir)
            update_frame_range(
                s_frame=14830 - 10, e_frame=14890, config_path=settings_path
            )
            run_section_prof(
                shared,
                out_dir,
                directory_config.config_dir,
                argus_extra_args,
                prof_run,
                mode_name,
                category,
                target_processes,
            )

        if args.profile_scope in (None, "section3"):
            out_dir = create_out_dir(base_dir)
            out_dirs.append(out_dir)
            update_frame_range(
                s_frame=15300 - 10, e_frame=15350, config_path=settings_path
            )
            run_section_prof(
                shared,
                out_dir,
                directory_config.config_dir,
                argus_extra_args,
                prof_run,
                mode_name,
                category,
                target_processes,
            )

        if args.profile_scope in (None, "overall"):
            out_dir = create_out_dir(base_dir)
            out_dirs.append(out_dir)
            update_frame_range(s_frame=0, e_frame=30000, config_path=settings_path)
            run_section_prof(
                shared,
                out_dir,
                directory_config.config_dir,
                argus_extra_args,
                prof_run if prof_run != "all" else "fps",
                mode_name,
                category,
                target_processes,
            )

    # 測定結果ファイルの変換
    print_results(out_dirs)


if __name__ == "__main__":
    main()
