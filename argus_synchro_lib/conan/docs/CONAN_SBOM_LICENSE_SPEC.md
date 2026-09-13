# argus_synchro_lib — Conan ビルド / SBOM / ライセンス — 仕様書

**argus_synchro_lib** の Conan 2 サイドカー（`argus_synchro_lib/conan/`）の実装仕様。運用・監査用。

Open3D は **`requires("open3d/0.19.0")` で利用する Conan パッケージ** として記述する。Open3D サイドカーの内部設計は `3rdparty/open3d/conan/docs/` を参照すること。

---

## 1. 概要

### 1.1 目的

| 機能 | 目的 | エントリポイント |
|------|------|------------------|
| **Conan ビルド** | 依存（Open3D 含む）を Conan で解決し、Conan 専用 CMake で `libargus_synchro_lib` をビルド | `conan/build.py` |
| **SBOM 生成** | 依存グラフを CycloneDX 1.6 JSON で出力 | `conan/generate_dependency_licenses.py` |
| **ライセンス可視化** | SBOM を HTML で閲覧（CSV は生成しない） | 同上（Sunshine） |

### 1.2 設計思想

- **サイドカー**: ルート [`CMakeLists.txt`](../../CMakeLists.txt)（pip / ExternalProject）は Conan ビルドでは使わない。エントリは [`conan/CMakeLists.txt`](../CMakeLists.txt)。
- **単一 `conan install`**: `open3d` を含む host 依存を 1 グラフで解決（§2.5 の open3d オプションで Open3D 機能を絞る）。
- **Open3D**: サブモジュール `3rdparty/open3d` を `export_open3d()` でレシピ登録し、install 時にビルドする。
- **SBOM**: Conan 2 の `cyclone_1.6.py` deployer。正本は JSON、HTML は派生物。

### 1.3 ディレクトリ構成

```
argus_synchro_lib/
├── CMakeUserPresets.json
├── CMakeLists.txt                     # 従来ビルド
└── conan/
    ├── conanfile.py                   # argus-synchro-lib
    ├── conan_common.py
    ├── build.py
    ├── generate_dependency_licenses.py
    ├── CMakeLists.txt
    ├── cmake/ArgusSynchroConanDeps.cmake
    ├── vendor/sunshine/
    ├── profiles/linux-gcc-release     # argus 所有 Conan プロファイル
    ├── docs/CONAN_SBOM_LICENSE_SPEC.md
    └── reports/

3rdparty/open3d/                       # open3d レシピ export 元（サブモジュール）
build/conan/                           # conan install 出力（gitignore）
```

---

## 2. Conan ビルド仕様

### 2.1 前提条件

| 要件 | 備考 |
|------|------|
| Conan | `>= 2.0` |
| CMake | `>= 3.24` |
| Python | 3.x |
| サブモジュール | `3rdparty/open3d` |
| プロファイル初回 | `conan profile detect --force` |

Conan 実行ファイル: `conan/.venv/bin/conan` → `PATH`。

### 2.2 ビルドワークフロー

```
export_open3d()
export_recipes(argus)    # conan/recipes/* があれば
    ↓
conan install conan/  (-pr:h/-pr:b PROFILE, --build=missing)
    ↓
cmake --preset conan-release -S conan
cmake --build --preset conan-release --parallel N
```

```bash
cd argus_synchro_lib
python -m pip install --upgrade "conan>=2.0"
conan profile detect --force
python conan/build.py
```

成果物: `build/conan/libargus_synchro_lib.so`、pybind（出力先は preset 依存）。

### 2.3 `conan install` の詳細

| 生成物（`build/conan/`） | 役割 |
|--------------------------|------|
| `CMakePresets.json` | preset ビルド |
| `conanbuild.sh` | 環境変数 |
| `conan_toolchain.cmake` | ツールチェーン |
| `Open3D*.cmake`, `opencv*.cmake`, … | CMakeDeps |

`PROFILE` = `conan/profiles/linux-gcc-release`。  
CMake は **`-S conan`** のみ。preset は `argus_synchro_lib/CMakeUserPresets.json` から include。

### 2.4 `conanfile.py`（argus-synchro-lib）

| 区分 | 内容 |
|------|------|
| `requirements()` | `open3d/0.19.0`, `nlohmann_json`, `nanoflann`, `opencv`, `pybind11`, `libjpeg-turbo`（override） |
| `build_requirements()` | `cmake`, `ninja` |
| `configure()` | open3d オプション（§2.5）+ OpenCV モジュール最小化 |
| `generate()` | `CMakeToolchain`, `CMakeDeps`（`open3d` → `Open3D::Open3D`）, `PkgConfigDeps` |

### 2.5 open3d オプション（`conanfile.py` `configure()` で固定）

| オプション | 値 |
|------------|-----|
| `build_cuda_module` | False |
| `build_gui` / `build_webrtc` | False |
| `build_python_module` | False |
| `build_shared_libs` | True |

install / lock / `graph info` で `-o` を渡す必要はない。CUDA 有効化などは **argus 標準外**（グラフ成分が変わる）。

### 2.6 プロファイル

`conan/profiles/linux-gcc-release`（argus 所有。Open3D 側 profile を fork）。

argus 標準構成（§2.5）では **ホスト CUDA は不要**。

### 2.7 Open3D の利用（argus 視点）

| 段階 | argus の動き |
|------|----------------|
| export | `export_open3d()` が `3rdparty/open3d/conan/recipes/*` と `open3d` レシピを登録 |
| install | §2.5 の open3d オプション付きでパッケージをビルド |
| CMake | `ArgusSynchroConanDeps.cmake` → `find_package(Open3D)` → `Open3D::Open3D` を `cpp` がリンク |
| SBOM | `open3d` およびその推移依存が `components[]` に含まれる |
| 従来ビルド | ルート CMake が別経路で Open3D を取得（Conan 非使用） |

Open3D パッケージのビルド失敗・レシピ詳細はサブモジュール側で切り分ける。

### 2.8 依存解決ポリシー（argus グラフ）

グラフルート: **`argus-synchro-lib`**。`open3d` は export 済みローカルレシピとしてビルドされ、その `_CORE_DEPS` / 有効な `_OPTION_DEPS` が推移依存になる。思想は Open3D サイドカーと同型の 4 ティア。

#### 優先順位

1. **Conan Center** — argus 直接 `requires` + 推移の Center 分
2. **ローカルレシピ** — `export_open3d()` 経由の `3rdparty/open3d/conan/recipes/*`、将来 `argus_synchro_lib/conan/recipes/*`
3. **プリビルト** — argus 標準（GUI/WebRTC OFF）では通常除外
4. **ホスト SDK** — Conan / SBOM 外（CUDA 不要、Python dev 要、等）

#### Tier 1 — argus 直接

| パッケージ | 備考 |
|------------|------|
| `nlohmann_json/3.12.0` | 直接 |
| `nanoflann/1.9.0` | 直接（override） |
| `opencv/4.10.0` | 直接、`configure()` でモジュール最小化 |
| `pybind11/3.0.1` | 直接 |
| `libjpeg-turbo/3.0.4` | override（OpenCV） |

推移: OpenCV / open3d からの Center パッケージ（`eigen`, `zlib`, `libpng`, `assimp`, …）。確定一覧は `conan graph info` または SBOM。

#### Tier 2 — open3d サブグラフ（標準オプション）

`export_open3d()` 後、install でビルド。§2.5 により **CUDA / GUI / WebRTC / Open3D Python モジュール** 用依存はグラフに入れない。

**入る例:** `open3d-vtk`, `open3d-qhull`, `open3d-rply`, `open3d-tinygltf`, `open3d-poissonrecon`, `open3d-uvatlas`, … + `_CORE_DEPS` の Center 分。

**標準で除外:** `open3d-stdgpu`, `open3d-cutlass`, `open3d-cub`; `imgui`, `open3d-filament`; `open3d-webrtc`, `open3d-boringssl`, `open3d-civetweb`。

BLAS: open3d `use_blas=True` 既定 → **`openblas`** 推移を想定。

#### Tier 3 / 4

- Tier 3: §2.5 のままでは該当なし（open3d GUI/WebRTC 無効）
- Tier 4: CUDA toolkit（不要）、Python dev（要）、OpenMP（要）。SBOM に含めない

#### 従来ビルド

`pip install -e` は FetchContent / ExternalProject。本ポリシーは **Conan 経路のみ**。

#### SBOM

`generate_dependency_licenses.py` は上記 **host** 依存を CycloneDX 化（build ツール・Tier 4 除外）。

### 2.9 `export_open3d()` と RREV

毎回 `3rdparty/open3d` を export。ソース変更で RREV が変わると `Missing packages: open3d` になりうる。  
**対処:** `python conan/build.py` を `--build=missing` まで完走。

argus の `conan/reports/` のみの変更は export ソースに含まれない。

### 2.10 argus CMake

- [`conan/CMakeLists.txt`](../CMakeLists.txt): IPO/LTO、`add_subdirectory(cpp|python)`。
- [`conan/cmake/ArgusSynchroConanDeps.cmake`](../cmake/ArgusSynchroConanDeps.cmake): `OpenMP`, `Eigen3`, `Open3D`, `OpenCV`, `pybind11` 等。

### 2.11 ホスト依存（Conan 外）

OpenMP、Python 開発ヘッダ。OpenCV GUI 無効時は追加 pkg は通常不要。

---

## 3. SBOM / HTML

### 3.1 概要

| 項目 | 値 |
|------|-----|
| 形式 | CycloneDX 1.6 JSON |
| deployer | `cyclone_1.6.py` |
| 可視化 | Sunshine → `dependency_report.html` |
| グラフ | `argus-synchro-lib`（`open3d` 推移含む） |

### 3.2 ワークフロー

```
export_open3d() + export_recipes(argus)
    ↓
conan graph info conan/ ... --deployer cyclone_1.6.py -df <dir>
    ↓
sbom-cyclonedx-1.6.json → dependency_report.html
```

`graph info` の生 JSON は一時ファイルのみ。

### 3.3 スコープ

| 含む | 含まない |
|------|----------|
| host の Conan 依存（`open3d` 推移含む） | build ツール（cmake, ninja） |
| | test 依存（デフォルト） |
| | ホスト SDK（CUDA toolkit 等・§2.5 では未使用） |

### 3.4 ライセンス

Conan レシピの `license` → CycloneDX `components[].licenses[]` → HTML 表示。  
`open3d` 推移分も同一 SBOM に載る。CSV は出力しない。

### 3.5 スナップショット

`conan/reports/sbom-cyclonedx-1.6.json` 現行 **59 コンポーネント**（§2.5 構成）。再生成後に件数を確認。

---

## 4. `conan_common.py`

| 符号 | 役割 |
|------|------|
| `PROFILE` | `conan/profiles/linux-gcc-release` |
| `OPEN3D_CONAN_DIR` | `3rdparty/open3d/conan` |
| `OPEN3D_RECIPES_DIR` | `3rdparty/open3d/conan/recipes` |
| `export_open3d()` | vendor recipes + `open3d` レシピを export |
| `resolve_conan_executable()` | venv → PATH |
| `run_command()` | `conanbuild.sh` source 可 |

---

## 5. 従来ビルド

```bash
pip install -e ./argus_synchro_lib
```

Conan サイドカーと独立。

---

## 6. チェックリスト

### Conan ビルド

- [ ] Conan 2.x、`3rdparty/open3d` あり
- [ ] `python conan/build.py` 成功
- [ ] `pip install -e` 従来ビルドが維持

### SBOM

- [ ] `generate_dependency_licenses.py` 成功
- [ ] CycloneDX 1.6、`open3d` 推移が `components[]` にある
- [ ] cmake/ninja が SBOM にない

### レポート

- [ ] HTML に主要コンポーネントのライセンスがある

---

## 7. 制限・注意

1. SBOM は Conan 管理依存のみ。
2. CSV 非対応（JSON / HTML を使用）。
3. `Missing open3d` → §2.9。
4. `--sunshine-enrich` は外部ネットワーク要。

---

## 8. コマンド早見表

| タスク | コマンド |
|--------|----------|
| フルビルド | `python conan/build.py` |
| SBOM + HTML | `python conan/generate_dependency_licenses.py --output-dir conan/reports` |
| 従来ビルド | `pip install -e .` |

---

## 9. 参照（argus 側）

| ファイル | 役割 |
|----------|------|
| `conan/build.py` | ビルド |
| `conan/generate_dependency_licenses.py` | SBOM + HTML |
| `conan/conanfile.py` | 依存グラフ |
| `conan/conan_common.py` | export / オプション / プロファイルパス |
| `conan/CMakeLists.txt` | Conan CMake ルート |
| `conan/cmake/ArgusSynchroConanDeps.cmake` | `Open3D::Open3D` 等 |
| `CMakeUserPresets.json` | preset include |

---

*argus_synchro_lib `conan/` 実装に基づく。*
