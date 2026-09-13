# calibration2d3d main/dev 統合作業 引き継ぎ

更新日: 2026-09-11

## 1. この文書の目的

この文書は、`main` をソフトウェア構造・契約の土台としながら、
`calibration2d3d_dev_temp20260904` の2D-3D校正アルゴリズムを統合した作業の引き継ぎ資料である。

次の担当者またはCopilotは、まず本書の「作業場所」「比較基準」「統合方針」を確認すること。
過去の調査では複数のcloneを使用したため、異なるworking treeを誤って編集・比較しないことが重要である。

## 2. 作業場所とGit基準

### 現在の実装リポジトリ

- リポジトリ: `/home/nvidia/argus_pipe_filter`
- ブランチ: `matsuoka_calib_merge`
- 現在のHEAD: `b01cb50ac4261e73a48bebb73b7603a3e7a1e091`
- HEADの説明: `2d3d校正を松永開発版同等に戻して動作確認`

### 比較基準

- main: `97e5967b6180ddedbfc93eebd8e0fcc1b7d04375`
- 開発ブランチ: `9025444c912598b53f3fb15157e3aefea49e38c1`
- merge-base: `2283a0a68f512d7595fb81032925630f07f1b522`

### mainから現在HEADまでのcommit

```text
a01f6c5 Merge calibration changes
4e2bd91 古いバージョンを削除
b01cb50 2d3d校正を松永開発版同等に戻して動作確認
```

### 他のディレクトリの位置づけ

- `/mnt/nvme/repository/local_pipe2`
  - 今回の統合対象ではない。
  - Python仮想環境とthree-way HTML生成スクリプトを利用した。
  - 仮想環境: `/mnt/nvme/repository/local_pipe2/.venv/bin/python`
  - HTML生成: `/mnt/nvme/repository/local_pipe2/scripts/three_way_review.py`
- `/mnt/nvme/repository/argus_pipe_filter_calibration_merge_20260910`
  - 初期調査・統合作業用に作成したclone。
  - 最終成果物の編集先ではない。

## 3. 統合方針

1. 公開API、設定読み込み、データI/O、診断、ログ、UI状態、process lifecycleはmainを優先する。
2. 候補生成、sampling、フィルタ、評価母集団、分母、集計、合否式は開発ブランチを優先する。
3. 実質的に同じ処理は、可能な範囲で開発ブランチと表現も一致させる。
4. 開発ブランチ内の重複代入、二重定義、debug残骸は移植しない。
5. mainの既存設定ファイルに新項目がなくても起動できる後方互換性を維持する。
6. production invariantを壊すtest doubleだけのためのfallbackは残さない。

## 4. 実装内容

### 4.1 calibcheck2d3d評価

対象:

- `argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/__init__.py`

実装内容:

- `evaluate_2d3d()` を開発版の局所添字・sampling方式へ移植した。
- strict指標の分母を、sampling済みかつ可視な3Dログ要素数とした。
- 同一frameに複数の3Dログ要素がある場合も、それぞれを分母へ計上する。
- legacy-like指標の分母を、比較可能かつscoreを取得できた要素数とした。
- mainのreason配列、debug情報、logger、diagnosis契約は維持した。
- 3D bbox overlap merge後のbboxをcalibcheck履歴へ保存する。

主要な内部データ:

- `frame_ix_3d`
- `bbox3d_full_list`
- `visible_mask`
- `projected_bbox2d_list`
- `visible_frame_to_local_ix`
- `sampled_local_ixs`
- `hit_mask`
- `legacy_frame_best_score`

検証では開発版関数を動的compileし、3ケースで戻り値とdebug metricsの一致を確認した。

### 4.2 calibration2d3d全体制御

対象:

- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/__init__.py`

アルゴリズムとして維持した開発版の流れ:

1. bbox中心の2D/3D対応点を生成する。
2. 中心点を使って仮校正する。
3. 仮校正結果から頭・足候補を生成する。
4. `corrpoint_mode` で中心点・軸点を選択する。
5. 3D中心Zを補正する。
6. 中心点と軸点を結合する。
7. `corner_rangefilter_mode` を適用する。
8. 結合後の対応点で最終推定する。
9. 同じ最終対応点を `LOOCV_bytime()` へ渡す。

通常校正の合格条件:

```python
accvalue < 30 or not check_enable
```

mainに存在した `or debug_allow_calibcalc_flag` は通常合否式から除去した。
file-end auto-exitは通常校正とは別のdebug lifecycleとして維持している。

mainから維持した処理:

- `INVALID_DATA_INPUT` と `ARRAY_SHAPE_ERROR` の入力診断
- CANデータを含む4要素FIFO契約
- 不正入力時の早期中断
- CAN yawのUI反映
- 結果CSVとdebug pickleのI/O診断
- `CalibrationCommonStatus` とcamera calibration statusの更新
- file-end auto-exitの不足データ検査と二重実行防止
- 校正結果と参照初期行列との差分ログ

`_log_calibration_matrix_difference()` の `rvec_ok` / `tvec_ok` はログ専用であり、
通常校正の採否、保存、最適化、LOOCVには使用しない。

### 4.3 center3d Z-ratio LUT

開発版の点別LUT式:

```python
z_ratio = center3d_zratio_LUT.evaluate(x, y)
z = (head_z - foot_z) * z_ratio + foot_z
```

現在の動作:

- LUT設定があり、LUTファイルを正常に読める場合:
  - 開発版の点別LUT補正を使用する。
- LUT設定が空の場合:
  - mainの固定比率・範囲方式へfallbackする。
- LUTパスが設定されていてもファイルがない、または読み込めない場合:
  - `FileNotFoundError` を捕捉して警告する。
  - mainの固定比率・範囲方式へfallbackする。
- LUTが2次元でないなどの構造不正:
  - 設定異常を隠さず例外とする。

main fallback:

- 指定XY範囲内: `bbox_center3d_z_ratio`
- 指定XY範囲外: `0.5`
- Z計算式: `(head_z - foot_z) * ratio + foot_z`

LUT設定を追加したファイル:

- `config/calib_settings.ini`（SCX900値）
- `config/SCX2000-3_calib_settings.ini`
- `config/SCX3500-3_calib_settings.ini`
- `config/SCX900-3_calib_settings.ini`

追加した設定:

- `center3d_zratio_coord_A_X`
- `center3d_zratio_coord_B_X`
- `center3d_zratio_coord_A_Y`
- `center3d_zratio_coord_B_Y`
- `center3d_zratio_LUT_c0/c1/c2`
- `center3d_zratio_path`
- `center3d_zratio_val_DEFAULT = [0.5, 0.5, 0.5]`

注意事項:

- 開発commitが参照する9個の `.npy` LUTは、開発commitにも現在HEADにも含まれていない。
- 想定配置先は次の配下である。
  - `${config_dir}/calibration_mat_generator_modules/SCX2000/`
  - `${config_dir}/calibration_mat_generator_modules/SCX3500/`
  - `${config_dir}/calibration_mat_generator_modules/SCX900/`
- 各ディレクトリにleft/center/right camera用の3ファイルが必要。
- LUTが未配備の環境では警告後にmain fallbackが使われる。
- SCX700は開発版にもLUT設定がなかったため変更していない。

### 4.4 app_config_calibration

対象:

- `argus_synchro/config/app_config_calibration.py`

追加・維持したoptional設定:

- `enable_bbox3d_overlap_merge=False`
- `bbox3d_overlap_merge_threshold=0.7`
- `is_headpoint_grid_overwrite=False`
- `calc_headpoint_grid_axis=["x", "x", "x"]`
- center3d Z-ratio LUTの6項目

center3d LUT項目はdataclassとparserの両方へ追加済み。
古いINIに項目がない場合は空リストとなり、controller側がmain fallbackを選択する。

### 4.5 detect2D axis候補生成

対象:

- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/track_main/detect2D/__init__.py`
- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/track_main/detect2D/detect2d_axis_faster.py`

実装内容:

- camera別のX/Y grid rangeを設定から渡す。
- 1段目・2段目の補間methodを設定から渡す。
- `calc_crosspoints()` の `xrange` / `yrange` は開発版どおり必須。
- `extract_results()` は4設定引数を受け取り、coreへforwardする。
- `extract_results_core()` も4設定引数を必須とした。
- 旧形式call siteを全PythonファイルでAST監査し、0件であることを確認した。

`detect2D/__init__.py` は開発commitとbyte単位で一致する状態まで寄せた。
`detect2d_axis_faster.py` は開発版の結果へ影響しない重複代入ブロックのみ採用していない。

### 4.6 detect3D headpoint補正

対象:

- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/track_main/__init__.py`
- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/track_main/detect3D/__init__.py`
- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/track_main/detect3D/calc_headpoint_z.py`

実装内容:

- `camera_index` を `track_main_class` から `detect3d_class` へ渡す。
- camera別の `calc_headpoint_grid_axis` を使用する。
- `grid_axis` は `x` / `y` のみ許可する。
- `apply_per_point()` で1m gridごとのhead Z中央値を適用する。
- `is_headpoint_grid_overwrite` でgrid補正と従来一括補正を切り替える。
- 空の `cornerlist3d` に対するguardを追加した。

`detect3D/__init__.py` のアルゴリズムと実行順は開発版と同等。
残る差はmainのconstructor引数順・defaultとimport/整形であり、productionでは `camera_index` を明示している。

### 4.7 3D bbox overlap mergeと履歴

対象:

- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/track_main/detect3D/person_tracker_SORT_3d/__init__.py`

実装内容:

- `merge_overlapping_bbox3d_xy()` を追加した。
- XY領域のintersectionを小さいbbox面積で割った比率で連結・統合する。
- mergeはSORT trackingより前に実行する。
- `last_bbox_multi_minmax` にmerge後bboxを保持する。
- 通常calibrationとcalibcheckの履歴はmerge後bboxと対応点群を保存する。

production wrapperは `update()` で必ず `last_bbox_multi_minmax` を設定する。
そのためtest double互換だけの `None` fallbackは削除し、対象ファイルは開発commitとbyte単位で一致させた。

### 4.8 target selection

対象:

- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/track_main/target_selector/__init__.py`

3D target比較時に、走査中の各track自身のscoreを使用するよう開発版へ合わせた。
別trackのscoreを誤参照しない。

### 4.9 calibration matrix checker

対象:

- `argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/calc_accuracy/calib2d3d_matchecker.py`

追加した関数:

- `conv_rtvec_to_mat`
- `conv_4x4mat_to_rtvec`
- `conv_rtvec_to_camera_pose`
- `check_diff`

数値25ケースで開発版との同値を確認し、4関数のnormalized ASTも開発版と一致させた。

## 5. テストと検証

### 最新の集中回帰

```bash
cd /home/nvidia/argus_pipe_filter
/mnt/nvme/repository/local_pipe2/.venv/bin/python -m pytest -q \
  tests/test_calibration2d3d_algorithm_merge.py \
  tests/test_app_config_calibration.py \
  tests/test_calibration2d3d_file_io_error.py \
  tests/test_calibration_contracts.py
```

結果:

```text
43 passed in 4.60s
```

確認済みの代表ケース:

- LUTありの点別Z補正: `[2.5, 7.5]`
- LUTなしのmain fallback: `[5.0, 5.0]`
- matrix rtvec/matrix round-tripと閾値判定
- 参照行列ファイル欠損が校正を阻害しないこと
- 2D axisのrange/interpolation伝播
- grid単位headpoint補正
- 3D bbox overlap merge
- merge後bbox履歴
- 3D target選択
- file-end auto-exitの不足データ拒否、一度だけの保存
- 結果CSVと非同期pickleのI/O診断
- configの新旧互換とLUT設定parse

その他の作業中検証:

- calibcheck開発版runtime equivalence: PASS
- calibcheckを含む関連回帰: 89 passed, 1 deselected
- detect2D axis変更後: 9 passed
- tracker差分復元後: 13 passed
- LUT/config/history集中検証: 23 passed
- Python compile: PASS
- VS Code diagnostics: エラーなし
- `git diff --check`: PASS

CRLF注意:

- `SCX2000-3_calib_settings.ini`、`SCX3500-3_calib_settings.ini`、`SCX900-3_calib_settings.ini` はmain/devともCRLF。
- 改行をLFへ正規化すると全行差分になるためCRLFを維持した。
- 品質確認は次を使用した。

```bash
git -c core.whitespace=cr-at-eol diff --check
```

## 6. Three-wayレビューHTML

出力先:

```text
/home/nvidia/argus_pipe_filter/.merge_review/three-way/
```

比較列:

- Vendor/main: `97e5967b6180ddedbfc93eebd8e0fcc1b7d04375`
- SHI/dev: `9025444c912598b53f3fb15157e3aefea49e38c1`
- Integration: 現在のworking tree

命名規則:

- repository相対pathの `/` を `__` に置換する。
- 単一underscore形式の重複ファイルは作らない。

主なHTML:

- `argus_synchro__calibration_mat_generator_modules__ctrl__calibcheck2d3d____init__.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d____init__.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__calc_accuracy__calib2d3d_matchecker.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__track_main____init__.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__track_main__detect2D____init__.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__track_main__detect2D__detect2d_axis_faster.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__track_main__detect3D____init__.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__track_main__detect3D__calc_headpoint_z.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__track_main__detect3D__person_tracker_SORT_3d____init__.py.html`
- `argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d__track_main__target_selector____init__.py.html`
- `argus_synchro__config__app_config_calibration.py.html`
- `config__calib_settings.ini.html`
- `config__SCX2000-3_calib_settings.ini.html`
- `config__SCX3500-3_calib_settings.ini.html`
- `config__SCX900-3_calib_settings.ini.html`

再生成例:

```bash
cd /home/nvidia/argus_pipe_filter
python3 /mnt/nvme/repository/local_pipe2/scripts/three_way_review.py \
  argus_synchro/calibration_mat_generator_modules/ctrl/calibration2d3d/__init__.py \
  --vendor-repo /home/nvidia/argus_pipe_filter \
  --vendor-ref 97e5967b6180ddedbfc93eebd8e0fcc1b7d04375 \
  --shi-repo /home/nvidia/argus_pipe_filter \
  --shi-ref 9025444c912598b53f3fb15157e3aefea49e38c1 \
  --integration-repo /home/nvidia/argus_pipe_filter \
  --output .merge_review/three-way/argus_synchro__calibration_mat_generator_modules__ctrl__calibration2d3d____init__.py.html
```

## 7. 次の担当者が注意する点

1. 編集対象は `/home/nvidia/argus_pipe_filter` である。
2. `local_pipe2` を現在実装として比較しない。
3. 開発版を丸ごとcopyしない。mainの診断・I/O・lifecycleが失われる。
4. `calibcheck2d3d` は評価分母とsamplingを関数名だけで判断しない。
5. `last_bbox_multi_minmax` はproduction wrapperのinvariantであり、test doubleはこの契約へ合わせる。
6. LUTのINI設定とLUT `.npy` 資産は別物である。設定があっても資産がなければfallbackする。
7. LUT資産を配置した後は、3 cameraすべてで実ファイルのshape、値域、座標変換を確認する。
8. file-end auto-exitは通常校正の合否経路と分けて扱う。
9. matrix reference checkは現状ログ専用。合否へ組み込む場合は仕様変更として扱う。
10. 変更後はfocused test、`py_compile`、diagnostics、`git diff --check`、three-way HTML更新を行う。

## 8. 現在の残課題

- 開発版が参照していた9個のcenter3d Z-ratio LUT `.npy` を入手・実機配置する。
- LUT配置後、SCX2000、SCX3500、SCX900の各cameraでLUT経路が選択されることをログと数値で確認する。
- SCX700は開発版にもLUT定義がない。LUT対応が必要なら、値と資産を別途設計する。
- 実センサまたは記録データを使ったend-to-end校正で、main fallbackとLUT版の結果差を保存・比較する。

## 9. 現在の完了判定

- Python実装は「main契約を維持し、設定・資産がある場合は開発版アルゴリズムを使用する」構造になっている。
- LUT資産がない環境でもmain方式へfallbackして起動可能である。
- 対象の集中テストと静的検証は通過している。
- 実装変更はHEAD `b01cb50` に含まれている。
- 本引き継ぎ文書を追加した後は、この文書のみ未コミット差分になる。