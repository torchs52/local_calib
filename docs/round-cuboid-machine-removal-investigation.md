# Round-cuboid機体点群除去テスト 調査メモ

最終更新: 2026-09-15

## 概要

`MachineCollisionImmobileRoundCuboid` の境界判定テストは、現行native実装との不一致により7ケースすべてが `xfail` になっている。

この処理は校正モードではなく、主に周辺監視モードのLiDAR点群前処理で使用される。機体設定に `MachineCollisionImmobileRoundCuboid` を使用する部位がある場合、機体自身の点群を除外する結果に影響する可能性がある。

現時点では、テスト用形状の生成方法とnative実装のどちらが仕様に合っているか確定していない。アプリの通常起動を妨げる事象は確認されていないが、境界付近の回帰検知には未検証領域が残っている。

## 対象

- テスト: `tests/test_machine_remove.py::test_round_cuboid_immobile_simple_remove`
- native実装: `argus_synchro_lib/cpp/src/machine_collision/MachineCollisionImmobileRoundCuboid.cpp`
- ヘッダ: `argus_synchro_lib/cpp/include/octotree/MachineCollisionImmobileRoundCuboid.h`
- 共通除去処理: `argus_synchro_lib/cpp/src/controller/controller.cpp::remove_machine_points`
- 周辺監視側呼び出し: `argus_synchro/process/points_refine_process.py::PointsRefineProcess._update_remove`
- 機体形状生成: `argus_synchro/SubScrutinizer.py::create_machine_points`

## 実行経路

周辺監視モードでは、次の経路で毎フレーム機体点群除去を実行する。

1. `argus_synchro.__main__.start_scrut_pipeline`
2. `PointsRefineProcess`
3. `PointsRefineProcess._update_remove`
4. `argus_synchro_lib.controller.remove_machine_points`
5. 各機体部位の `check_pcd_on_self`
6. 対象部位がround-cuboidの場合は `MachineCollisionImmobileRoundCuboid::check_pcd_on_self`

校正モードの `start_calib_pipeline` は別のプロセス構成であり、調査時点では校正モジュールから `remove_machine_points` への直接呼び出しは確認されていない。

## 現在のテスト状態

テストには次のマーカーが付いている。

```python
@pytest.mark.xfail(
    reason="vendor round-cuboid test geometry does not match the current native implementation",
    strict=False,
)
```

- `xfail` 導入日: 2026-09-09
- 導入コミット: `576deaac9b4f4d1c0f27d1c60430fe70f0fa81cf`
- テスト自体のパラメータ導入コミット: `26a30bc4`
- 7件は独立した障害ではなく、1つのパラメータ化テストの7ケース
- `strict=False` のため、将来XPASSしてもテストスイートは失敗しない

## 再現結果

`.venv` で `xfail` を無効化して実行した。

```bash
source .venv/bin/activate
python -m pytest -q --runxfail \
  tests/test_machine_remove.py::test_round_cuboid_immobile_simple_remove
```

結果: `7 failed`

| yaw | eps | 期待 | 結果 |
|---:|---:|---|---|
| `0.0` | `0.9` | 全点除去 | 80点残存 |
| `pi / 6` | `0.9` | 全点除去 | 80点残存 |
| `2.1 * pi` | `0.9` | 全点除去 | 80点残存 |
| `2.1 * pi` | `1.0` | 全点除去 | 100点残存 |
| `0.0` | `1.1` | 全点残存 | 一部が除去され失敗 |
| `pi / 6` | `1.1` | 全点残存 | 一部が除去され失敗 |
| `2.1 * pi` | `1.1` | 全点残存 | 一部が除去され失敗 |

通常の全体テストでは、この7ケースは `xfail` として処理される。

```bash
source .venv/bin/activate
python -m pytest -q -rxX \
  --ignore=argus_synchro_lib/build \
  --ignore=argus_synchro_lib/install \
  --ignore=argus_synchro_lib/argus_synchro_lib.egg-info \
  -k 'not test_damoyolo_onnx_accepts_batch_on_tensorrt'
```

2026-09-15時点の結果: `566 passed, 7 xfailed, 3 deselected`

## 確認できている実装上の事実

- round-cuboidは、直方体部分と円弧柱部分の除去判定の論理和を返す。
- 円弧の中心と半径は、機体形状ファイルの点から `CircumcenterProcessor` で算出する。
- 円弧部分の膨張量は、XY方向の `remove_dist` のユークリッド距離を半径へ加算している。
- `MachineCollisionImmobileRoundCuboid` は非可動部を表す。
- `check_pcd_on_self` はyawを受け取るが、現実装では使用しない。非可動部であるため、これだけでは不具合と断定できない。
- テスト側は、読み込んだ機体点群から円弧の端点と中心を逆算して境界点を生成する。

## 未確定事項

1. 正式なround-cuboid形状仕様
   - `machine_form_points` の先頭2点と、それ以降の6点単位の意味。
   - 円弧柱と直方体の接続位置、対象とする円弧範囲。

2. `remove_dist` の仕様
   - 各軸の距離を独立して使うか、XY距離を半径へ合成するか。
   - 境界上の点を除去対象に含めるか。

3. テスト用点群の妥当性
   - `create_points_on_circumcenter` がnative実装と同じ形状定義を再現できているか。
   - `cuboid_max_x` に膨張後の値を渡すことが仕様どおりか。
   - 実データの離散点群から端点を推定する方法が安定しているか。

4. 実機への影響
   - 使用中の各機種設定にround-cuboid部位が含まれるか。
   - 実フレームで機体点が残存するか、または周辺点を過剰除去するか。
   - 後段の蓄積、3D物体検出、衝突判定へ影響する量か。

## 推奨調査手順

1. 機体形状ファイル仕様と、vendor版が期待する境界条件を確認する。
2. 1部位だけの最小合成形状を用意し、直方体、円弧柱、接続境界を個別に検証する。
3. native実装が算出した中心、半径、z範囲、最小xをテストから参照または診断出力し、テスト側の算出値と比較する。
4. `eps=0.9`, `1.0`, `1.1` について、点ごとの入力座標、期待、実判定をCSVなどで比較する。
5. 現行機種設定ごとに `MachineCollisionImmobileRoundCuboid` の使用有無を棚卸しする。
6. 対象機種の実LiDARフレームで、除去前後の点群を保存して可視確認する。
7. 仕様確定後、テスト生成方法またはnative実装の一方を修正する。

## 完了条件

- round-cuboidの形状と `remove_dist` の仕様が文書化されている。
- 最小合成形状の内側、境界、外側テストが通常のテストとして成功する。
- 現在の7ケースについて、期待値を維持するか変更するかの根拠が明確である。
- `xfail` を削除してテストが成功する。
- 解決まで `xfail` を維持する場合は、追跡課題番号を理由へ記載し `strict=True` にする。
- 対象機種について、実データで過少除去と過剰除去が許容範囲内であることを確認する。

## 当面の扱い

- 今回の校正関連設定・テスト更新とは分離して扱う。
- 周辺監視アプリの起動可否を判断する際は既知問題として扱う。
- 機体点群除去の品質保証では未解決項目として扱い、恒久的には放置しない。