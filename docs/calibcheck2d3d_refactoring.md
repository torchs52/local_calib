# calibcheck2d3d Refactoring Memo

## 2026-07-27 追記（新アルゴリズムへの根本置換メモ）

### 位置づけ

- 今回の変更は、既存の別アルゴリズム前提で組まれていた処理を、calibcheck2d3d の新アルゴリズムへ実質的に置き換える作業。
- 単純な import 差し替えではなく、検知経路、3D-2D対応評価、可視判定、デバッグ導線まで含めた構造再編。

### 現在の __init__.py の実装構造

1. 検知基盤（YOLO）
- YOLODamoBatchAdapter を導入し、複数カメラフレームを batch 推論。
- 返却形式は既存 downstream 互換の [boxes, scores, classes, valid_detects] を維持。

2. Scene 判定レイヤ
- Scene_CalibCheck2d3d が SceneDesc.Scene を継承。
- integrate2d3d_calibcheck で 3D bbox の投影、背面クラスタ除外、2D-3D対応付け、human size gate を実施。

3. フレーム処理レイヤ
- proc_lidar1f: 点群前処理、静止点除去、クラスタリング、3D bbox 化。
- proc_camera1f: 歪み補正後フレームを一括推論し、UI向け bbox 描画情報を整形。
- record_bbox1f: フレーム単位で 2D/3D ログを保持。

4. 評価レイヤ
- track_3dbbox / track_2dbbox で追跡ログを作成。
- evaluate_2d3d で投影重なりの事前ゲートを通した後、evaluate_bbox_overlap_scenedesc で最終判定。
- strict 指標と legacy-like 指標を切替可能。

5. アプリ制御レイヤ
- pre_app_loopmain / app_loopmain / post_app_loopmain で取得・表示・評価を段階実行。
- reason_camera_notvalid で早期リターン理由を管理。

### 今回の実装で注意した点

- 互換 I/F 維持:
  既存コードが期待する YOLO 出力レイアウトを崩さず、内部実装のみ置換する方針を徹底。

- 座標系とフォーマット差分:
  bbox3d の表現差（[minx,miny,minz,maxx,maxy,maxz] と [x_min,x_max,y_min,y_max,z_min,z_max]）を明示的に変換してから評価。

- 高コスト評価の前段ゲート:
  2D交差なしケースを先に落とし、不要な Scene ベース判定を減らして処理負荷と誤判定を抑制。

- 可視判定の厳格化:
  project_3dbbox_core(require_points_in_image=True) で「画像内に見えている 3D」のみ評価母集団に採用。

- 人判定の最終ゲート:
  use_human_gate と passes_human_size を最終段で適用し、2D一致のみで HUMAN を確定しない設計を維持。

- 欠損データ耐性:
  カメラ欠損フレーム時は空検出を返し、パイプラインを停止させずに評価可能性を保持。

- デバッグ再現性:
  評価トレース、動画ダンプ、pickle 出力を残し、置換後の判定理由を後追いできるようにした。

- 今後の統合前提:
  現在の SceneDesc は暫定。将来的に argus_synchro_lib.scene.Scene へ寄せる際は、
  「2D-3D対応判定ロジック」「human gate」「入力出力フォーマット」を優先的に整合させる。

- 作成日: 2026-07-27
- 対象: calibcheck2d3d の YOLO 検知経路リファクタリング

## 目的

- calibcheck2d3d から YOLOdetectors.py への依存を解消する。
- Detect2dDamoYoloOnnx を共通実装として利用し、推論経路を一本化する。
- Camera.count 枚の画像を 1 回のバッチ推論で処理する。

## 行った事

1. 未使用コードの整理
- calibcheck2d3d の __init__.py で、参照されていない import / メソッドを削除。
- 併せて未使用ローカル変数を整理。

2. 挙動差分の調査
- Detect2dDamoYoloOnnx と既存 YOLOdetector (DAMO-YOLO) を比較。
- 入出力 I/F、バッチ処理、セッション設定の差分を確認。
- 直接置換は不可と判断し、アダプタ方式を採用。

3. 新規アダプタ実装
- 追加: argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/YOLOadapter.py
- クラス: YOLODamoBatchAdapter
- 役割:
  - Detect2dDamoYoloOnnx を内部利用
  - Camera.count を batch_size に設定
  - calibcheck2d3d 側の既存期待形式
    [boxes, scores, classes, valid_detects]
    に変換して返却
  - 欠損カメラフレームは空検出を返す

4. calibcheck2d3d 側置換
- __init__.py の import を YOLOdetector から YOLODamoBatchAdapter に変更。
- YOLO インスタンス生成をアダプタ生成に変更。
- proc_camera1f を逐次推論からバッチ推論に変更。
- app_loopmain から sec を proc_camera1f に渡すよう変更。

5. 依存除去と検証
- calibcheck2d3d 配下で YOLOdetectors.py / YOLOdetector 参照がないことを確認。
- py_compile で __init__.py と YOLOadapter.py の構文エラーがないことを確認。

## 結果

- calibcheck2d3d は YOLOdetectors.py への依存を解消。
- Detect2dDamoYoloOnnx を用いたバッチ推論経路へ置換完了。
- Camera.count 枚を 1 回で推論する要件を満たした。
- 既存の downstream が期待する検知結果フォーマットを維持。

## 補足

- 欠損カメラがある場合は、バッチサイズ維持のためダミー画像を投入しつつ、当該カメラの結果は空検出として返す実装。
- YOLOdetectors.py は廃止予定前提で、calibcheck2d3d 側からの参照を外した状態。

## 2026-07-27 追加修正メモ（実行時エラー対応）

### 背景

- calibcheck2d3d 実行中に以下 2 系統のエラーが同時発生。
  - logging の `TypeError: not all arguments converted during string formatting`
  - `TypeError: internal_make_BB() got an unexpected keyword argument 'x_range'`

### 原因

1. logger 呼び出しミス
- `self._logger.info(self, "...")` のように `self` を第1引数として渡していた。
- `AppLogger.info(msg, *args)` の仕様上、`self` がメッセージ本体扱いになり、第2引数が文字列フォーマット用引数として解釈されて例外化。

2. internal_make_BB のシグネチャ不一致
- 呼び出し側は `x_range / y_range / z_range` を keyword 指定。
- 定義側 `internal_make_BB(pcd)` がこれら引数を受け取っていなかった。

### 対応内容

1. logger 呼び出し修正
- 対象: `argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/__init__.py`
- `self._logger.info(self, ...)` / `warning(self, ...)` / `error(self, ...)` / `debug(self, ...)` を
  `self._logger.info(...)` 形式へ統一。

2. internal_make_BB 引数拡張
- 対象: `argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/debuginfo_and_functions.py`
- 変更前: `def internal_make_BB(pcd)`
- 変更後: `def internal_make_BB(pcd, x_range=(-10, 10), y_range=(-10, 10), z_range=(-10, 10))`
- `set_xyz_range(...)` 呼び出しへ受け取った各 range を渡すよう修正。

### 影響と確認

- 既存呼び出し（`x_range` 指定あり）と整合し、unexpected keyword argument は解消。
- logger 側フォーマット例外の連鎖出力を防止。
- `py_compile` で対象ファイルの構文エラーがないことを確認済み。

### 再発防止メモ

- `AppLogger` 利用時は標準 logging と同様に「第1引数は常にメッセージ文字列（または format 文字列）」を徹底する。
- ユーティリティ関数の引数変更時は、呼び出し側 keyword と定義側シグネチャを同時に確認する。
