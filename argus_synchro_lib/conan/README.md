# argus_synchro_lib — Conan サイドカー

**argus_synchro_lib** 用の Conan 2 サイドカー。依存（**Open3D** 含む）を 1 回の `conan install` で解決し、`conan/CMakeLists.txt` から共有ライブラリと pybind モジュールをビルドする。

従来ビルド（[`../CMakeLists.txt`](../CMakeLists.txt) + `pip install -e`）は **変更なし**。Conan ビルドではルート CMake は使わない。

---

## レイアウト

```
argus_synchro_lib/
├── CMakeUserPresets.json
├── CMakeLists.txt                 # 従来ビルド専用
└── conan/
    ├── conanfile.py               # argus-synchro-lib（open3d オプション含む）
    ├── conan_common.py
    ├── build.py
    ├── profiles/linux-gcc-release # argus 所有 Conan プロファイル
    ├── generate_dependency_licenses.py
    ├── CMakeLists.txt             # Conan 専用（-S conan）
    ├── cmake/ArgusSynchroConanDeps.cmake
    ├── vendor/sunshine/
    ├── docs/CONAN_SBOM_LICENSE_SPEC.md
    └── reports/

3rdparty/open3d/                   # サブモジュール（open3d レシピ export 元）
build/conan/                       # conan install 出力（gitignore）
```

---

## 前提条件

| 項目 | 内容 |
|------|------|
| Conan | `>= 2.0` |
| CMake | `>= 3.24` |
| Python | 3.x |
| サブモジュール | `3rdparty/open3d`（`conan export` に使用） |
| プロファイル | `conan/profiles/linux-gcc-release` |
| ホスト | OpenMP、Python 開発ヘッダ（pybind11） |

`conan profile detect --force`（初回）。Conan: `conan/.venv/bin/conan` → `PATH`。

---

## クイックスタート

```bash
cd argus_synchro_lib
python -m pip install --upgrade "conan>=2.0"
conan profile detect --force
python conan/build.py
```

**成果物（例）:** `build/conan/libargus_synchro_lib.so`、pybind モジュール。

初回は `open3d` パッケージのビルドに時間がかかる。

---

## `build.py` の処理

1. **`export_open3d()`** — `3rdparty/open3d/conan/recipes/*` と `open3d` レシピを export
2. **`export_recipes()`** — `conan/recipes/*`（あれば）
3. **`conan install conan/`** — プロファイル + `--build=missing`（open3d オプションは `conanfile.py`）
4. **`cmake --preset conan-release -S conan`**（cwd: `argus_synchro_lib`）

---

## 依存グラフ（`conanfile.py`）

| 依存 | 備考 |
|------|------|
| `open3d/0.19.0` | export 後、install 時にビルド |
| `nlohmann_json`, `nanoflann`, `opencv`, `pybind11` | Conan Center |
| `libjpeg-turbo` | override（OpenCV） |

OpenCV は `configure()` で argus 向けに最小化。

---

## 依存解決ポリシー

argus の Conan グラフは **ルート `argus-synchro-lib`** と、その `requires("open3d/0.19.0")` から広がる **open3d 推移依存** で構成される。優先思想は Open3D サイドカーと同型（Center → ローカルレシピ → プリビルト → ホスト SDK）。

### 優先順位（4 ティア）

1. **Conan Center** — argus が直接 `requires()` するパッケージ、および open3d / opencv からの推移依存のうち Center で充足するもの
2. **ローカルレシピ** — `open3d` ビルドに必要な `3rdparty/open3d/conan/recipes/*`（export 後にグラフに入る）。将来 `argus_synchro_lib/conan/recipes/*` を追加した場合もここ
3. **プリビルト blob** — argus 標準オプション（GUI/WebRTC OFF）では **通常グラフに入らない**。open3d 側で該当オプションを ON にした場合のみ
4. **ホスト SDK（Conan 対象外）** — SBOM にも載せない。ビルドホストが提供

**取得経路:** Conan 2 の `conan install` / `graph info` が正規。open3d パッケージのビルド中に CMake が Conan 生成物を消費するのは想定内。ビルドシステム外の ad-hoc 取得は対象外。

### Tier 1 — argus 直接 + Center 推移

**`conanfile.py` で直接宣言（すべて Conan Center、バージョン固定）:**

| パッケージ | 用途 |
|------------|------|
| `nlohmann_json` | JSON |
| `nanoflann` | 近傍探索（override） |
| `opencv` | 画像・幾何（モジュールは `configure()` で最小化） |
| `pybind11` | Python バインディング |
| `libjpeg-turbo` | OpenCV 向け JPEG（override） |

**推移例（OpenCV 等から）:** `protobuf`（無効時は入らない）、`quirc`, `libpng`, `zlib`, `openblas` / `eigen` 等 — 実際のグラフは `conan graph info` / SBOM で確認。

`open3d` 本体は Center パッケージではなく **export 済みローカルレシピ**（下記 Tier 2 の入口）。

### Tier 2 — ローカルレシピ（open3d サブグラフ）

`export_open3d()` が `3rdparty/open3d/conan/recipes/*` と `open3d` レシピを export。install 時に `open3d` がビルドされ、例えば次がグラフに載る（**argus 標準の open3d オプション下**）:

`open3d-vtk`, `open3d-qhull`, `open3d-rply`, `open3d-tinygltf`, `open3d-poissonrecon`, `open3d-uvatlas`, `open3d-tinyfiledialogs`, `open3d-liblzf`, `open3d-glew-source`, `open3d-tomasakeninemoeller`, … および Tier 1 の `assimp`, `embree`, `fmt`, `onetbb`, …（open3d の `_CORE_DEPS`）。

**argus の `-o` によりグラフから外れる例（open3d `_OPTION_DEPS`）:**

| open3d オプション（argus） | 主な除外パッケージ |
|---------------------------|-------------------|
| `build_cuda_module=False` | `open3d-stdgpu`, `open3d-cutlass`, `open3d-cub` |
| `build_gui=False` | `imgui`, `open3d-filament` |
| `build_webrtc=False` | `open3d-webrtc`, `open3d-boringssl`, `open3d-civetweb` |
| `build_python_module=False` | open3d 用 `pybind11` 要求（argus 側 `pybind11` は別途残る） |

`use_blas=True`（open3d デフォルト）のため **openblas** が BLAS として入る想定。

### Tier 3 — プリビルト（argus 標準では非該当）

`build_gui=False` / `build_webrtc=False` のため、WebRTC / BoringSSL / Filament プリビルトは **標準グラフに含めない**。open3d オプションを変更した場合のみ検討。

### Tier 4 — ホスト SDK（Conan / SBOM 外）

| コンポーネント | argus Conan ビルド |
|----------------|-------------------|
| CUDA toolkit | **不要**（`build_cuda_module=False`） |
| Python dev ヘッダ | **必要**（pybind11） |
| OpenMP | 必要（システム + OpenCV オプション） |
| OpenGL / X11 / OSMesa | 不要（GUI OFF） |
| Intel MKL / IPP | 不要（標準は openblas 経路） |

### 従来ビルド（Conan ポリシー外）

`pip install -e` はルート `CMakeLists.txt` が **FetchContent / ExternalProject / 事前ビルド Open3D** を使う。上記 4 ティアは **Conan 経路専用**。SBOM（`conan/reports/`）も Conan グラフのみを反映。

### SBOM との対応

`generate_dependency_licenses.py` の CycloneDX は **上記グラフの host 依存**（推移含む）を列挙。build ツール（cmake, ninja）は含めない。Tier 4 は含めない。

---

## Open3D の使い方（argus 視点）

### Conan グラフ

- `requires("open3d/0.19.0")` でグラフに載せ、**同一の `conan install` でビルド**する。
- 事前に `export_open3d()` が必須（サブモジュール未 checkout だと失敗）。

### install 時の open3d オプション（`conanfile.py` `configure()` で固定）

| オプション | 値 | argus への意味 |
|------------|-----|----------------|
| `build_cuda_module` | False | CUDA toolkit 不要 |
| `build_gui` / `build_webrtc` | False | GUI 系依存を除外 |
| `build_python_module` | False | Open3D の Python モジュールは不要 |
| `build_shared_libs` | True | **`libOpen3D.so`** をリンク |

### プロファイル

`conan_common.PROFILE` = `conan/profiles/linux-gcc-release`（argus 所有）。

### CMake（argus）

[`cmake/ArgusSynchroConanDeps.cmake`](cmake/ArgusSynchroConanDeps.cmake) で `find_package(Open3D CONFIG REQUIRED)` し、`cpp/CMakeLists.txt` から **`Open3D::Open3D`** をリンクする。

### export / RREV

`3rdparty/open3d` のソースが変わると `open3d` の RREV が変わり、`Missing packages: open3d` になりうる。  
**対処:** `python conan/build.py` を `--build=missing` まで実行。

### 従来ビルドとの違い

`pip install -e` 経路はルート `CMakeLists.txt` が **FetchContent / 事前ビルド Open3D** を使う。Conan 経路とは別。

---

## SBOM / HTML

argus の **1 グラフ全体**（`open3d` 推移依存含む）を SBOM 化。CSV は出さない。

```bash
pip install -r conan/vendor/sunshine/requirements.txt   # 初回
python conan/generate_dependency_licenses.py --output-dir conan/reports
```

| 出力 | 内容 |
|------|------|
| `sbom-cyclonedx-1.6.json` | 正本 |
| `dependency_report.html` | Sunshine 可視化 |

§Open3D オプションは `conanfile.py` に集約済み。

---

## 手動 install

```bash
cd argus_synchro_lib
# export_open3d 相当を済ませたうえで:
conan install conan/ --output-folder build/conan \
  -pr:h conan/profiles/linux-gcc-release \
  -pr:b conan/profiles/linux-gcc-release \
  --build=missing
cmake --preset conan-release -S conan
cmake --build --preset conan-release --parallel "$(nproc)"
```

---

## 従来ビルド

```bash
pip install -e ./argus_synchro_lib
```

---

## トラブルシュート

| 症状 | 対処 |
|------|------|
| `Missing packages: open3d` | `build.py` 再実行。`3rdparty/open3d` 更新後は export + install |
| `Open3D Conan sidecar not found` | サブモジュール取得 |
| preset / install 失敗 | `conan profile detect --force`、`build/conan` 削除後に再試行 |

---

## コマンド早見表

| タスク | コマンド |
|--------|----------|
| フルビルド | `python conan/build.py` |
| SBOM + HTML | `python conan/generate_dependency_licenses.py --output-dir conan/reports` |
| 従来ビルド | `pip install -e .` |

詳細: [`docs/CONAN_SBOM_LICENSE_SPEC.md`](docs/CONAN_SBOM_LICENSE_SPEC.md)
