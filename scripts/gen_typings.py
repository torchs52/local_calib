import logging
import shutil
from pathlib import Path

import pybind11_stubgen


def gen_argus_synchro_lib_stubs(typings_dir: Path) -> None:
    logging.info("gen_argus_synchro_lib_stubs")
    cmd = [
        "-o",
        str(typings_dir),
        "--enum-class-locations",
        "NodeEntity:argus_synchro_lib.octotree",
        "--enum-class-locations",
        "CoordMethod:argus_synchro_lib.collision_detector",
        "--enum-class-locations",
        "AggName:argus_synchro_lib.edge_det",
        "--enum-class-locations",
        "BevCoord:argus_synchro_lib.edge_det",
        "--enum-class-locations",
        "DediscretizeMethod:argus_synchro_lib.edge_det",
        "--numpy-array-use-type-var",
        "--ignore-all-error",
        "argus_synchro_lib",
    ]
    pybind11_stubgen.main(cmd)
    _replace_dict_typings(typings_dir / "argus_synchro_lib")


def _replace_dict_typings(stubs_root: Path) -> None:
    if not stubs_root.exists():
        return

    for path in stubs_root.rglob("*"):
        if path.suffix not in {".py", ".pyi"}:
            continue

        text = path.read_text(encoding="utf-8")
        replaced = text.replace(
            "def __getstate__(self) -> dict:",
            "def __getstate__(self) -> dict[str, typing.Any]:",
        )
        replaced = replaced.replace(
            "def __setstate__(self, arg0: dict) -> None:",
            "def __setstate__(self, arg0: dict[str, typing.Any]) -> None:",
        )
        if replaced != text:
            path.write_text(replaced, encoding="utf-8")


def gen_open3d_stubs(typings_dir: Path) -> None:
    logging.info("gen_open3d_stubs")
    cmd = [
        "-o",
        str(typings_dir),
        "--enum-class-locations",
        "VoxelPoolingMode:open3d.geometry.VoxelGrid",
        "--ignore-all-error",
        "--numpy-array-use-type-var",
        "open3d",
    ]
    pybind11_stubgen.main(cmd)
    fix_open3d_stubs(typings_dir)


def gen_hnswlib_stubs(typings_dir: Path) -> None:
    pybind11_stubgen.main(
        ["-o", str(typings_dir), "--numpy-array-use-type-var", "hnswlib"]
    )


def fix_open3d_stubs(typings_dir: Path) -> None:
    open3d_init_file = typings_dir / "open3d" / "__init__.pyi"

    if not open3d_init_file.exists():
        raise FileNotFoundError(open3d_init_file)

    with open3d_init_file.open(encoding="utf-8") as f:
        content = f.read()

    # 修正が既に適用されているかチェック
    if "# Fix for open3d modules import" in content:
        print("Open3Dのstub修正は既に適用済みです")
        return

    content = content.replace(
        "import platform as platform\nimport re as re\n",
        "import platform as platform\nimport re as re\n\n# Fix for open3d modules import\n",
        1,
    )
    if "from . import cpu\nfrom . import ml" not in content:
        raise ValueError("open3d __init__.pyiのcpu import部分を検出できませんでした")
    content = content.replace(
        "from . import cpu\nfrom . import ml",
        "from . import cpu\nfrom . import cuda\nfrom . import ml",
        1,
    )
    if "'cpu', 'data'" not in content:
        raise ValueError("open3d __init__.pyiの__all__定義にcpuを検出できませんでした")
    content = content.replace("'cpu', 'data'", "'cpu', 'cuda', 'data'", 1)

    with open3d_init_file.open("w", encoding="utf-8") as f:
        f.write(content)

    _sync_cuda_typings(open3d_init_file.parent)


def _sync_cuda_typings(open3d_typings_dir: Path) -> None:
    cpu_typings_dir = open3d_typings_dir / "cpu"
    cuda_typings_dir = open3d_typings_dir / "cuda"

    if not cpu_typings_dir.exists():
        raise FileNotFoundError(cpu_typings_dir)

    if cuda_typings_dir.exists():
        shutil.rmtree(cuda_typings_dir)

    shutil.copytree(cpu_typings_dir, cuda_typings_dir)
    for path in cuda_typings_dir.rglob("*.pyi"):
        text = path.read_text(encoding="utf-8")
        replaced = text.replace("open3d.cpu", "open3d.cuda")
        if replaced != text:
            path.write_text(replaced, encoding="utf-8")


def gen_onnxruntime_stubs(typings_dir: Path) -> None:
    logging.info("gen_onnxruntime_stubs")
    cmd = [
        "-o",
        str(typings_dir),
        "--ignore-all-error",
        "--numpy-array-use-type-var",
        "onnxruntime",
    ]
    pybind11_stubgen.main(cmd)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    typings_dir = Path("typings")
    gen_argus_synchro_lib_stubs(typings_dir)
    gen_open3d_stubs(typings_dir)
    gen_onnxruntime_stubs(typings_dir)
