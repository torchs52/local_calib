import subprocess
from pathlib import Path


def main() -> None:
    result_dir = Path("result")
    jsons = result_dir.glob("*.json")
    cvfs = list(result_dir.glob("*.cvf"))

    for json in jsons:
        cvf = json.with_suffix(".cvf")
        if cvf in cvfs:
            continue
        subprocess.run(["viztracer", "--compress", json, "-o", cvf], check=True)
        print(f"Compress: {cvf}")


if __name__ == "__main__":
    main()
