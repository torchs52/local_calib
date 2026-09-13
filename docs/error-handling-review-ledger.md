# Error Handling Review Ledger (Post-outsourcing sync)

最終更新: 2026-08-03 (CE011/CE015追加)
目的: 委託先ソース反映後のエラー処理設計を、実装前に論点固定して段階的に見直す。
- 委託先リソースを反映済み。
- Jetson実機で通常動作確認済み (通常系ではエラー落ちなし)。
- 現時点では具体的な追加実装は未反映 (設計見直しフェーズ)。

## 見直しスコープ
- 対象
  - app/core/argus_synchro/shared_errors.py
  - app/core/argus_synchro/shared_err_config.py
  - app/core/argus_synchro/diagnosis/error_diagnosis.py
  - app/core/argus_synchro/diagnosis/state_errors.py
  - app/core/argus_synchro/diagnosis/action_errors.py
  - app/core/argus_synchro/diagnosis/state_d_errors.py
  - app/core/argus_synchro/diagnosis/error_config.py
  - app/core/argus_synchro/diagnosis/reduced_load_mode.py
  - app/core/argus_synchro/__main__.py
  - app/core/argus_synchro/process/ 配下の連携実装
- 非対象
  - UI表示のみの見た目変更 (エラー判定/伝搬に影響しないもの)

## これまでの前提 (維持事項)
- StateErrorIndex は SE001-SE045 との整合を維持する。
- CE系/Action系を StateErrorIndex に混在させない。
- 既存の設定ファイル異常 (CE005) の診断経路は維持する。

## レビュー観点 (優先順)
1. Index/Enum整合
- エラー番号と診断クラスの対応が単調に追えるか。
- 追加・削除で配列/辞書アクセスが破綻しないか。

2. 診断インタフェース整合
- detect_error / detect_recovery_error / detect_recovery_fail_safe の引数契約がクラス間で一貫しているか。
- StateErrorDiagnosisA/B/C と action/state_d 側で ResultDiagnosis の扱いが矛盾しないか。

3. 復帰条件とフェールセーフ条件
- しきい値・継続時間・連続成功回数の意味が実装と設定で一致しているか。
- recovery 側で状態リセット漏れがないか。

4. 設定連携
- error_config.py の型定義と各 diagnosis.update() の参照名が一致しているか。
- 設定未読込/部分欠損時のデフォルト挙動が明確か。

5. ログと運用可観測性
- error/recover/failsafe-recover のログ粒度が運用要件を満たすか。
- reduced_load_mode への入力条件が過剰/過小でないか。

6. エントリポイント/プロセス伝搬
- process -> shared_errors -> ErrorMonitor -> main 制御ループの伝搬が一貫しているか。
- 設定異常時の再試行ループと action error 診断の組み合わせに破綻がないか。

## 進め方 (実装前)
1. 事実抽出
- 各診断クラスの「エラー条件」「復帰条件」「設定項目」を一覧化。

2. 衝突整理
- 旧方針との不一致を "keep / modify / drop" で分類。

3. 方針確定
- 影響度 (安全性・運用・実装工数) で優先度付け。

4. 実装チケット化
- 1論点1差分で小さく反映。
- 反映単位ごとに実機再確認観点を明記。

5. 異常系受け入れ基準の定義
- エラー検知時間、復帰条件、再起動要否、ログ出力をシナリオ別に合格条件化する。

## 論点トラッカー
| ID | 対象 | 論点 | 現状 | 方針候補 | 決定 | 備考 |
|---|---|---|---|---|---|---|
| E-001 | state_errors.py | LogOutputStoppedDiagnosis の仕様水準 (簡易版 vs 閾値付き実装) | 現在は簡易版 | keep/modify | in-review | 戻し操作での差分混入履歴あり |
| E-002 | shared_errors.py | Indexとdiagnosis実体の対応保証 | レビュー済み | modify | in-review | SE/CE混在防止を再確認 |
| E-003 | error_config.py | update先パラメータ名整合 | 未レビュー | modify | pending | 型定義と参照の突合が必要 |
| E-004 | __main__.py | 再起動ループとCE005診断伝搬の整合 | レビュー済み | modify | decided | load_config失敗時のexcepts_diagnosis経路は確認済み。retry/backoff・ログ間引き・CE005カウント抑制を実装反映済み。allow_exit優先は仕様メモとして保留 |
| E-005 | process/* + diagnosis/* | 引数契約の突合 (呼び出し側と診断側) | 一次棚卸し済み | modify | in-review | detect_* の引数型/順序/None許容を棚卸し |
| E-006 | __main__.py + app_manager_process.py + diagnosis/* | CE006(LiDAR校正マトリクス不正)の枠組み実装 | 実装済み(枠組み) | modify | decided | 起動時/設定再読込時に lidar2lidar・lidar2crane を検証。現段階はI/O・形状4x4・有限値のみ。参照データ差分アルゴは未実装 |
| E-007 | visual_process.py + diagnosis/* | CE007-CE010(カメラ0-3校正データ不正)の枠組み実装 | 実装済み(枠組み) | modify | decided | カメラ再構築時に fisheye(JSON) と camera-lidar最新CSV を検証。現段階はI/O・形状・有限値のみ。参照差分アルゴは未実装だがnon-blocking |
| E-018 | shared_errors.py + state_d_errors.py + error_config.py | 重要度Dエラーの登録・index・例外分類体系の再設計 | 一部実装済み | modify | deferred | 現在の通常診断・例外分類・module診断を分離し、未実装Dエラーは実装時に登録する。LiDARデータ欠落の直接ログ方式も vendor Dレベル設計を踏まえて再検討する |

## 2026-08-15 作業メモ
- action_errors.py で SharedLidarShiftMonitorExcept の import 漏れを修正。vendor 側の LiDAR shift 系診断実装を取り込んだ後の仕上げ漏れだった。
- state_d_errors.py で InvalidDataInput の再帰 tuple 判定と ProcessForcedTermination の list 検証に型注釈を補強し、Pylance 上の不明型エラーを解消。
- ErrorConfig 属性名と action/state/state_d の update() 参照名を確認し、現時点で E-003 の実害がある属性名不一致は見つからず。
- 検証: tests/test_camera_data_missing_diagnosis.py, tests/test_shared_err_config.py は pass (5件)。
- E-005 の高優先修正として command_daemon_tegrastats.py を更新。
  - SOC1_TEMP_RE の正規表現 typo を修正 (`\\bsoc1@...`)。
  - 温度診断の呼び出し引数を 5引数旧契約から 4引数現契約へ統一 (`now, tj, tc, tg`)。
  - tegrastatsの値は `tj=tj`, `tc=max(cpu,soc0)`, `tg=max(soc1,soc2)` で集約。
  - これにより `INTERNAL_TEMPERATURE_RISE` / `TEMPERATURE_SENSOR_ABNORMAL` / `TEMPERATURE_RISE_TREND_CONTINUES` の実行時引数不一致リスクを解消。
- E-005 の PointsRefine 処理で、vendor 側にのみ存在した蓄積モジュール例外の伝搬を反映。
  - `_update_accum()` とその入力診断を `ACCUMULATION_MODULE_ERROR` の例外分類で保護。
  - vendor 実装は例外処理後に未代入の `output_accum` を参照し得たため、現行ではログ記録後に次フレームへ進む制御に補正。
  - 検証: `python -m py_compile argus_synchro/process/points_refine_process.py` は pass。
- E-005 の CalibProcess 処理で、vendor 側にのみ存在した `CALIBRATION_MODULE_ERROR` の例外伝搬を反映。
  - 校正モードごとのアプリ呼び出しを囲み、Dレベル例外または校正モジュール例外ならログ記録後に次サイクルへ進む。
  - 未分類例外は従来どおり再送出する。
  - 検証: `python -m py_compile argus_synchro/process/calib_process.py` は pass。
- E-005 の校正入力経路で、vendor 側にのみ存在した `SharedErrors` の依存性伝搬を反映。
  - `CalibProcess -> calibration2d3d_manager_class -> boss ->` 各校正コントローラ -> `datacapture_class` の経路で `SharedErrors` を渡すよう統一。
  - 3D-3D 校正では `INVALID_DATA_INPUT` により LiDAR/CAN 入力を判定し、検知時は当該処理を継続しない。
  - 共通の `datacapture_class` ではカメラ・LiDAR・CAN・フレーム番号を判定し、検知時は既存の空データ結果を返して後続処理を止める。
  - 検証: 関連8モジュールの `python -m py_compile` は pass。`tests/test_camera_data_missing_diagnosis.py` と `tests/test_shared_err_config.py` は 5 passed。
  - 保留: `calibcheck2d3d` と `wait_app` にある vendor 固有の直接診断/UI連携は、大規模な校正機能再構成差分と混在するため別レビュー単位とする。
- CalibProcess の3D-3D校正正常終了経路に、vendor 側の `Lidar_SM_ex.write_has_not_calibrated(False)` を反映。
  - CE001（LiDAR位置ズレ検出）が設定する未校正フラグを、校正完了後に解除するCE002（要センサ校正）の復帰連携。
  - `post_calib3d3d()` 完了後にのみ実行し、外側の例外経路では解除しない。
  - 検証: `python -m py_compile argus_synchro/process/calib_process.py` は pass。
- E-005 の校正検証・待機モードで、vendor 側の `INVALID_DATA_INPUT` 診断を現行FIFO入力経路へ最小移植。
  - `calibcheck2d3d.app_loopmain()` は共通 `datacapture_class` を通らずFIFOを直接扱うため、カメラ・LiDAR・CANを診断し、検知時は処理を継続しない。
  - `wait_app.dataproc()` も同じ入力を診断し、検知時は当該フレームのUI更新を行わず次フレームを待つ。
  - vendorの大規模な校正検証アルゴリズム差分は取り込まず、mainのアルゴリズムを維持。
  - 検証: 両モジュールの `python -m py_compile` は pass。Pylanceの残存指摘は既存の型注釈・未使用importに限られる。
- `calibcheck2d3d/SceneDesc.py` は main 側で追加したファイルであり、vendor 側は非保有。
  - 現行mainの `calibcheck2d3d` はこのファイルの `Scene` を継承・利用するため、vendor方式へ寄せる対象ではなくmainの実装を維持。
  - `passes_human_size()` の包括例外は、寸法情報を分解できない場合にサイズゲートを通す既存アルゴリズム仕様であり、vendorから代替のエラー処理は提供されていないため維持。
- `calibration_mat_generator_modules` の残差分を確認。
  - `tools3Dcapture.py` の座標変換オプション、`debuginfo_and_functions.py` の点群範囲パラメータ、追跡層の `frame_ix_lastmove` はmain側のアルゴリズム差分として維持。vendor由来の例外処理はない。
  - `facade/__init__.py` はmain側の設定キー存在確認を維持。vendorの削除を採ると、yaw設定欠損が外側の包括例外まで伝播してダミーUI値の反映全体を中断し得る。
  - `calibration2d3d.pre_app_loopmain()` のvendor側早期returnは、戻り値が未使用でメソッド本体も空のため実行時挙動を変えず、不採用。
  - `lidar2crane_trans_mat_*.csv` と `temp/.gitignore` は改行コードのみの差分。実行権限のみの差分も不採用。
- `calibration_mat_generator.py` のvendor側 `Calib_Mat_Generator.allow_exit()` と、対応する `__main__.py` の再起動抑止は未反映でレビュー完了。
  - vendor方式は、`CalibProcess._shutdown()` の終了フラグファイル生成、`CalibProcess.check_allowexit()`、`allow_exit()`、トップレベル再起動ループの `is_restart=False` を組み合わせるデバッグ・自動試験向けの校正自動終了フロー。
  - 現行は別方式で同等の終了制御を実現済みの認識のため、同フローを重複導入しない。
  - 将来、校正のバッチ自動終了／再起動抑止を見直す際に、現行方式とvendorの4点セットの対応・差異を再確認する。
- E-005 の `lidar_shift_monitor_process.py` をレビュー。
  - vendor側の error config 更新、IMU入力の `INVALID_DATA_INPUT` 診断、正常入力時のheartbeat更新、CE001（LiDAR位置ズレ検出）・CE002（要センサ校正）診断の実行は現行へ反映済み。
  - CE001のvendor実装は `errors_diagnosis()` の戻り値を旧3要素として分解していたが、vendor共通の `StateErrorDiagnosisA` 契約は2要素の `ResultDiagnosis` タプルであり、同じvendorファイルのCE002も2要素で処理していた。CE001を `log_output(*result)` へ修正。未修正では診断実行時に `ValueError` となる。
  - 検証: `python -m py_compile argus_synchro/process/lidar_shift_monitor_process.py` は pass。Pylanceエラーなし。専用テストは見つからず、既存のプロセス応答テストは旧3要素の期待値不整合のため対象外。
- E-005 の `imu_process.py` をレビュー。
  - SE040/SE041（IMU接続エラー）の設定更新・診断実行は `AppManagerProcess` が担い、IMUプロセスは実データ受信時の共有heartbeat更新を担う構造を確認。
  - vendor側の `ErrorConfig` 読み込みと `SharedErrors` ロガー登録を反映。現時点でIMUプロセス自身が更新・実行する診断はないため、`_err_config_load()` は将来の診断追加用の共通拡張点として維持。
  - main側の `TimeoutError` 時にheartbeatを更新しない制御、実データ到達時だけ0.5秒間隔でheartbeatを更新する制御は維持。これにより接続断をSE040/SE041が検出できる。
  - 検証: `python -m py_compile argus_synchro/process/imu_process.py` は pass。Pylanceエラーなし。専用テストは見つからず。
- `message/input_message.py` をレビュー。
  - vendor差分は例外・診断フローではなく、`CameraMessage` の共有画像バッファを設定解像度固定から `1920x1080` 固定上限へ変え、高さ・幅を別共有スカラーで保持して実寸スライスを返す機能変更。
  - ユーザー方針によりvendor方式を採用。通常入力・Scrutinizer入力の `CameraMessage` 生成を引数なしへ変更し、不要になった `SyscamRes` importを削除。
  - 現行設定は通常系 `1280x720`、校正系 `1920x1080` でvendor上限内。1080pを超える入力を将来サポートする場合は、最大バッファ寸法の見直しが必要。
  - 検証: `python -m py_compile argus_synchro/message/input_message.py argus_synchro/__main__.py` は pass。`1280x720` 画像の共有スロット書込み・読戻しでメタデータ、形状、画素値の一致を確認。`input_message.py` のPylanceエラーなし。`__main__.py` の残存Pylance指摘は今回の変更箇所と無関係の既存指摘。
- `message/scrutinizer_message.py` をレビュー。
  - vendor差分は `AccumPointsData` に `yaw_angle_deg` を追加し、`AccumPointsDataMessage` の共有スロットで同値を書込み・読戻しするもの。現行mainにはすでにvendorと同じ実装が反映済み。
  - `PointsRefineProcess` がCAN入力のyawを `AccumPointsData` に格納し、`VisualProcess` が3D物体検出とUI表示に利用する経路を確認。
  - 検証: Open3Dを利用しない最小スタブ環境で、点群・frame・time・`yaw_angle_deg` の共有スロット往復を確認。通常仮想環境にはOpen3Dが未導入のため、モジュールの通常importを要する統合実行は実施していない。コード変更なし。
- E-005 の `image_process.py` をレビュー。
  - vendor側の camera communication quality（低下・エラー）、画像不正、Dレベルのカメラデータ欠落診断を取り込み。画像取得成功時は最終成功時刻を更新し、失敗時は連続失敗数を増やして診断へ渡す。
  - `AppManagerProcess` のカメラ接続診断に対し、カメラプロセスの進行を示すheartbeatを更新。接続断・取得品質・黒画像はそれぞれ既存の診断クラスで分離して評価する。
  - vendor側の `CameraProviderProcess.__slots__` には `_sec_cam`、`_consecutive_read_failure_count`、`_timestamp` が未登録で、同実装では初期化時に `AttributeError` となるため、vendorフローを維持したままスロット登録を補正。
  - Dレベル例外および `CAMERA_MODULE_ERROR` をプロセスループで分類してログ出力後に次サイクルへ進め、未分類例外は再送出するvendor方式を反映。
  - 検証: `python -m py_compile argus_synchro/process/image_process.py` は pass。Pylanceエラーなし。`tests/test_camera_data_missing_diagnosis.py` は 2 passed。
- E-005 の `object_detect_process.py` をレビュー。
  - vendor側の `INVALID_DATA_INPUT` 設定・入力診断、`CAMERA_HUMAN_DETECTION_MODULE_ERROR` とDレベル例外の外側分類、`SharedErrors` ロガー登録は現行mainに反映済み。
  - 推論実行だけを例外境界とし、実行中の例外は `AiInferenceResultError` を含むDレベル診断で記録した後、空検出へフォールバックして処理を継続するmainの安全動作を維持。歪み補正・入力検証など推論以外の例外は外側のvendor診断フローへ委譲する。
  - AIモデル読込失敗をCE013として検出して `NotAppliedObjDetection` へ切り替えるmainの初期化時フォールバック、歪み補正のバックエンド選択、画像リサイズはmainのアルゴリズムとして維持。
  - 検証: `python -m py_compile argus_synchro/process/object_detect_process.py` は pass。Pylanceのheartbeat 2件は `ProcessBase._spe` の既存基底型注釈に起因。`tests/test_detect2d.py` は `onnxruntime` 未導入のため収集時に停止。
- E-005 の `points_process.py` をレビュー。
  - vendor側の `SharedErrors` ロガー登録と、Dレベル例外および `LIDAR_MODULE_ERROR` をプロセスループで分類して次サイクルへ進める例外境界を復元。未分類例外はトレースバックを保って再送出する。
  - main側のheartbeat間隔、SE008品質低下イベント時刻、SE020不正データ比率、少量点群の直接ログ、入力なし時の空点群継続はLiDARの品質・接続監視アルゴリズムとして維持。
  - vendorの `_err_config_load()` と `_err_config` スロットを採用。起動時に共有エラー設定のスナップショットを読み込み、`_change_device()` の品質低下しきい値参照をこのプロセス保持値へ統一。`_apply_parameters()` はvendorどおり `pass` とし、アプリ設定更新と独立したエラー設定更新をこの経路で不完全に追従しない。
  - 検証: `python -m py_compile argus_synchro/process/points_process.py` は pass。専用テストは見つからず。Pylanceの残存指摘は実行権限、既存TODO、naive datetime、既存型注釈に限られる。
- E-005 の `process/process.py` をレビュー。
  - `ProcessBase` の `kill()`／`terminate()` にvendorの `unsubscribe` 制御と `exitcode` を反映。
  - `ProcessManager.graceful_stop_all()` は、強制終了シグナルを各プロセスへ並列送信する間は共有flowをunsubscribeせず、送信失敗をプロセス単位でログ化して停止シーケンスを継続するvendor方式へ統一。
  - `ProcessManager` のロガー注入はvendorと同じ必須 `AppLogger` 引数へ統一。段階停止のタイムアウト既定値は維持。
  - 検証: `python -m py_compile argus_synchro/process/process.py` は pass。最小スタブで `unsubscribe=False` の強制終了が待機イベントを解除せず、既定の `kill()` が解除することと `exitcode` 委譲を確認。`tests/test_monitor_process_not_responding_diagnosis.py` は現行の2要素 `ResultDiagnosis` 契約に対して旧3要素bool契約を期待する既存不整合のため 6 failed（今回の変更とは無関係）。
- `profiler/prof_fps.py` と `profiling.py` をレビュー。
  - ユーザー方針によりvendor実装をそのまま採用。`ProfFps` の既定サンプリング間隔を20へ戻し、`profiling.py` にvendorの `--prof-run` 選択と区間別計測ヘルパーを反映。
- `file_watch.py` をレビュー。
  - ユーザー方針によりvendor実装を採用。settings.iniと機種別設定の更新・close・moveイベントを共通デバウンス処理へ集約し、原子的なファイル置換でも設定再読込が起動するようにした。
- `common/app_logger.py`、`common/paths.py`、`device/camera/helper.py` をレビュー。
  - `paths.py` はvendorと完全一致させ、共有mmapの既定ディレクトリを `/dev/shm` へ統一。
  - `app_logger.py` はvendor構造を維持しつつ、`__main__.py` が登録するCE015（ログファイルI/Oエラー）診断コールバックだけを実行互換性のため保持。削除すると起動時に `AttributeError` となるため、`GZipRotatingFileHandler` と非圧縮ローテーションハンドラへのコールバック伝播を維持する。
  - `helper.py` と `provider/image.py` はvendor実装を維持し、`dst` 事前確保バッファへ歪み補正結果を直接書き込む契約を採用。`object_detect_process.py` の呼び出し側も同じvendor方式へ復元した。
  - 検証: 対象モジュールの `python -m py_compile` はpass。恒等remapを使うCPU providerの戻り値契約を確認。Pylanceの残存2件は `object_detect_process.py` の既知のheartbeat基底型注釈に限られる。
- `provider/point_cloud.py` をレビュー。
  - vendorとの差分はmain側のSE008/SE020品質メトリクス（品質低下時刻・原点点割合）のみであり、例外・診断フローとして取り込むべきvendor差分はない。mainの診断用メトリクスを維持した。
  - `CalibMid360PointCloudProvider.get_accum_points()` は全パケットが空の場合に空リストへ `np.concatenate()` を実行し得たため、連結前に `None` を返すよう補正。ほかの点群プロバイダと同じ空入力契約へ統一した。
  - 検証: `python -m py_compile argus_synchro/provider/point_cloud.py` はpass。Open3Dをスタブ化した最小テストで、空パケット蓄積時に `None` を返すことを確認。
- `device/lidar/mid360_points.py` をレビュー。
  - vendorとの差分はmain側のSE008（LiDAR通信品質低下）向けパケット品質監視のみであり、vendor由来で追加すべき例外・診断フローはない。`udp_cnt` の欠番と `dot_num` 低下の検出、品質低下時刻の記録を維持した。
  - 現在の未コミット変更は、固定値だった `dot_num` しきい値を `ErrorConfig` のLiDAR別設定から受け取るもの。通常・校正設定ともLiDAR台数は2で、`config_lidars.json` もMID360を2台定義するため、`lidar0`/`lidar1` の設定マッピングと整合する。
  - 検証: `python -m py_compile argus_synchro/device/lidar/mid360_points.py` はpass。最小パケットでフレーム境界、連番、欠番、低点数を検証し、期待どおり品質低下を判定することを確認。Pylanceエラーなし。
- `detect2d.py` をレビュー。
  - ユーザー方針により、TensorRT I/O binding・エンジン/タイミングキャッシュ・グラフ最適化を含むvendor実装を採用した。モデル初期化のCE013分類と推論時の空検出フォールバックは、既レビューの `object_detect_process.py` が担う。
  - 明示的なモデルパス存在確認は追加せず、ONNX Runtimeのセッション初期化失敗をモデルpath付きの `RuntimeError` として再送出する処理をvendor版へ加えた。これにより呼び出し側でモデル読込失敗をCE013として記録できる。
  - `Detect2dDamoYoloOnnx` と `create_onnx_inference_session` の呼び出し契約は維持され、ワークスペース内の呼び出し箇所に破綻はない。`trt_ep_options` はONNXファイルパスを追加で受け取るvendor署名へ変わったが、直接呼び出すワークスペース内利用者はない。vendor版は設定済みバッチ数と同じフレーム数を前提にするが、通常処理と校正アダプタはいずれも設定カメラ数の固定長バッチを渡すため整合する。
  - 検証: `python -m py_compile argus_synchro/detect2d.py` はpass。`tests/test_detect2d.py` はONNX Runtime未導入のため実行不可。Pylanceの残存指摘はvendor実装におけるONNX Runtime/OpenCVの外部戻り型と既存ローカル注釈に限られる。
- `AccumulatePoints.py` をレビュー。
  - vendorとの差分は例外・診断フローではなく点群蓄積アルゴリズムのみ。グリッド占有値を直接 `1` で設定するvendor実装を採用し、旧mainの入力点群Z列を上書きする処理と防御コピーを削除した。占有グリッドの意味を維持しつつ、後段で出力・蓄積する非地面点群の高さをコピーなしで保持する。
  - vendorの `is_reduced_load_mode` によるボクセルサイズ切替を採用。`PointsRefineProcess` の `reduced_load_mode.enabled` を `FilterAccumPointsInterface`、`SubScrutinizer.exe_accumulation()` を通じて `accumulate_point()` まで伝搬し、立体物・地面点群のダウンサンプルサイズを切り替える。
  - `AccumulationConf` と5つの稼働設定にreduced-load用の立体物・地面点群ボクセルサイズを追加し、vendor既定値 `0.12` を設定。vendor側にあるdeque長切替は現行の初期化・動的再構成に接続されていないため、この変更には含めない。
  - 検証: 関連5モジュールの `python -m py_compile` はpass。最小テストで、グリッド占有値の生成後も呼び出し元が持つ点群Z値が不変であることを確認。5つの稼働設定でreduced-load用2キーがともに `0.12` として読めることを確認。Pylanceの残存指摘は実行権限・既存モジュール名・変数命名のスタイル指摘に限られる。
- `interface/filter_accum_points.py` をレビュー。
  - `is_reduced_load_mode` のインターフェース・`StaticAccumPoints` から `SubScrutinizer.exe_accumulation()` への伝搬はvendor由来であり、現行の6要素蓄積結果契約と整合するため維持した。`AccumAxisPointsCloud` は蓄積を実行しないため、共通契約としてフラグを受け取るだけで使用しない。
  - vendorの `StaticAccumPoints` が保持する `init_source_to_target` は、`SubScrutinizer.exe_accumulation()`、`Registrate_LiDAR.registrate_two_pclouds()`、`lidar_registration/multi_scale_icp.py` を入出力とも変更し、CPU版マルチスケールICPの初期変換へ渡す一体のICP状態管理改修である。ICPアルゴリズムをvendorへ委任する方針に従い、同4モジュールをまとめてvendor実装へ移行した。前回のICP変換を次回初期値として更新する挙動を採用し、main側のCUDA自動選択・モジュール固定単位行列は採用しない。
  - main側に追加された例外処理はなかった。`SubScrutinizer` の空deque保護（空のとき `np.empty((0, 3))` を渡す）は実行時の防御として維持した。vendorの実装どおり「蓄積結果6要素」と「変換Tensor」の2要素を返すが、vendorの平坦な7要素型注釈だけは実際の戻り値に合わせてネストした型へ修正した。
  - 検証: `python -m py_compile argus_synchro/interface/filter_accum_points.py` はpass。Pylanceの残存指摘は、蓄積を実行しない `AccumAxisPointsCloud` の既存未使用引数に限られる。
- `interface/collision_detection.py` をレビュー。
  - vendorは衝突クラスタの変換を `argus_synchro_lib.controller.cluster_col_map_to_py()` に委任するが、現行ビルド済みライブラリにはこの関数が公開されていない。vendor実装を採用すると衝突判定結果の変換時に `AttributeError` となる。
  - mainの `{label: (*val.to_tuple(), None) ...}` を保持。C++で公開済みの `CollisionDetResult.to_tuple()` は5要素を返し、末尾へ `None` を加えることで `t_py_col_res` の6要素契約と一致する。例外処理の追加はない。
  - 検証: `.venv` の `argus_synchro_lib.controller` で `cluster_col_map_to_py` が未公開であることを確認。スタブによる変換結果が6要素契約を満たすこと、`python -m py_compile`、Pylance、`git diff --check` がpass。
- `SystemMonitor/get_status.py` をレビュー。
  - vendorは `paths.parse_directory_config()` を用いて共通の設定解析・`settings.ini` 読み込みまで行う。mainは `--raw` と `--mmap-dir` のみを解析し、指定されたmmapディレクトリを直接使う。
  - `argus_bootfig.sh` とJetson版は `get_status.py --mmap-dir "$MMAP_DIR"` を起動時の状態確認に使用している。main方式は設定ファイルに依存せず、起動直後でもこの指定先を読めるため維持する。例外時の表示・終了動作はvendorと同一。
  - 検証: 一時mmapへ `RUNNING` を書き、`--mmap-dir` 指定のCLIがrawで `3`、通常表示で `3 RUNNING` を返すことを確認。`tests/test_status_mmap.py` は1 pass / 2 failで、signal handlerの`SystemExit`と`StatusMMAP.close()`の冪等性について現行 `status_mmap.py` とテスト期待が不一致。対象外の既存問題として本レビューでは変更しない。
- `SystemMonitor/argus_synchro_query.py` をレビュー。
  - ソース内容はvendorと完全一致。差分はGitの実行ビットのみで、vendorは `100644`、main/currentは `100755`。
  - ファイルにshebangはなく、リポジトリ内に直接実行する呼び出し元もないため、実行ビットはCLI挙動に影響しない。現行モードを維持する。
  - 検証: 一時mmapへ接続数を書き、`version`、`cam_connect`、`lidar_connect`、`connect`、`json` が期待どおりの値を返すことを確認。下位16ビットへのマスクも `0x10002 -> 2`、`0x20003 -> 3` で確認。`python -m py_compile`、Pylance、`git diff --check` がpass。
- `SystemMonitor/info_mmap.py` をレビュー。
  - I/Oロジックとソース内容はvendorと完全一致。差分はGitの実行ビットだけで、vendorは `100644`、main/currentは `100755` だった。
  - `info_mmap.py` はimport用モジュールで、shebangと直接実行する呼び出し元がない。実行ビットは機能を提供せずPylance診断の原因にもなるため、vendorと同じ `100644` に戻した。
  - 検証: 一時mmapで8バイトレイアウトの作成、初期値、更新、各値の下位16ビット化、`create=False` で既存データが保持されることを確認。`python -m py_compile` と `git diff --check` がpass。
- `SystemMonitor/status_mmap.py` をレビュー。
  - vendorは `StatusMMAP.close()` を `_closed` フラグで冪等にし、シグナルhandlerの最後で `sys.exit(0)` する。main側のmmap閉鎖時 `ValueError` をログ・既定エラー値へ変換するread/write保護を追加した際、これら2つのライフサイクル保護が落ちていた。
  - mainのread/write例外処理を維持し、vendorの `_closed` フラグと `sys.exit(0)` を復元。これによりシグナル処理後にプロセスが正常終了し、複数経路からのcloseでもファイル記述子の二重closeを防ぐ。
  - `status_mmap.py` もimport用モジュールでshebang・直接起動箇所がないため、vendorと同じ非実行モード `100644` に戻した。
  - 検証: `tests/test_status_mmap.py` は3 passed。`python -m py_compile` と `git diff --check` がpass。実ファイルとGitのモードは `100644` だが、Pylanceは実行ビット診断を保持しているため権限情報の再読込みが必要。残存指摘は既存のシグナル互換処理と未使用handler引数に限られる。
- `SystemMonitor/MonitorArgus.py` をレビュー。
  - vendorの `monitor_argus_last_heartbeat` 更新を復元。全稼働設定の `AppManager.monitor_argus_last_heartbeat_path` と `AppManagerProcess._monitor_argus_healthy_check()` がこのファイルの `time.perf_counter()` 値を読み、MonitorArgus停止を診断するため、main側の削除を維持すると監視契約が切れる。
  - vendorと同じ一時ファイル書込み後の `os.replace()` による原子的更新を採用。例外時に未初期化の一時パスを参照するvendorの不具合だけ、`temp_name: str | None` の初期化とnull確認で修正し、元の例外を再送出する挙動は維持した。
  - main側のUIモード選択、設定・実行ファイル検証、status.mmapのファイル種別・サイズ検証、シグナル時のUI停止は保持。
  - 検証: スタブでRUNNINGからSHUTDOWNまでの監視ループを実行し、status.mmap作成、UI起動・停止、ハートビートファイルの生成とfloat値を確認。`python -m py_compile` と `git diff --check` がpass。Pylanceの残存指摘は既存のJSON設定値が `object` 型であることに起因する。

## 2026-08-16 CAN 診断・受信レビュー
- `diagnosis/state_errors.py` の CAN 診断（SE027/SE028/SE029）をレビュー。
  - SE028 は既存診断と同じ `self.param` による設定参照へ統一し、vendorの時刻辞書を直接減算する不具合を、最新の有効なCAN受信時刻を選ぶ処理で補正した。
  - vendorにあったSE028の検出・復帰・フェイルセーフ復帰ログを復元した。
- `process/can_process.py` をレビュー。
  - 起動時にSE027/SE028/SE029へ共有エラー設定を `update()` する経路を反映した。
  - vendorの固定角度CAN ID・レバーCAN IDのコンストラクタ注入は不採用。CSVはデコーダ選択、provider側のID分類は別責務であり、機種別ID設定化は別課題とする。
- `device/can/can_receiver.py` をレビュー。
  - mainのCSV + `DECODER_REGISTRY` によるデコーダ選択方式を維持した。vendorの `getattr()` 方式へ戻さない。
  - SE027/SE028は通信品質診断ヘルパー、SE029はCANデータ不正診断ヘルパーとして現行構造に統合済みであることを確認した。
  - SE029が `DETECTION` または `KEEPING` の間はデコーダを実行しないvendorのゲートを反映した。不正フレームは `recv_data=None` とし、`failedcount` を加算する。
  - `YAW_ANGLE_INFO_ERROR` は診断クラスと設定を持つが、receiverからは未接続のままとする。接続時は設定の `angle_can_id` を使い、固定IDを再導入しない。
  - UDP受信ペイロードの長さ・DLC事前検証はvendorにも存在しないため、今回の取り込み対象外とする。
- `provider/can_data.py` と `device/can/can_decoders.py` をレビュー。
  - main側のSCX2000角度CAN ID対応、`received_data` によるCAN heartbeat更新、モジュール分離済みデコーダ群を維持する。
  - レバー圧を更新しない既存のvendor/main挙動は意図的に維持する。`ShiLibCan` のSCX2000対応も使用予定がないため対象外とする。
  - vendorの旧・新CAN・レバーデコーダはmainの `can_decoders.py` に同じ計算式で移設済みであり、vendor由来の未統合デコーダ処理はない。
- 検証: `tests/test_can_diagnosis.py` は4件pass。変更したCAN関連PythonファイルのPylance診断エラーなし。

## E-004/E-005 初期観測メモ
- __main__.py の ErrorMonitor 起動は main 冒頭で常時起動される構成。
- load_config() は while True で設定読込を再試行し、例外時に CE005 診断を通す構成。
- process 生成時は SharedErrors を各プロセスへ注入するため、診断引数契約は process 側の入力仕様と不可分。
- 特に ObjectDetect/PointsRefine/Visual 系は mode切替時の再生成経路も含めて整合確認が必要。

## E-004 レビュー結果 (2026-07-26)
### 事実
- load_config() は例外発生時に CE005 (CONFIG_FILE_MISSING) の excepts_diagnosis を呼び、同一ループ内で再試行する。
- main ループは is_restart_required か例外で再起動フローへ入る。
- ErrorMonitor は main の外側で先行起動し、ループ終了まで維持される。

### リスク
1. 設定ファイル異常が持続した場合、load_config() の無限再試行に待機が無く、高頻度リトライになる。
2. CE005 のカウンタ増加が短時間で進み、ログ/監視ノイズが増える可能性がある。
3. finally 節の allow_exit 判定が restart 判定より後段で上書きされるため、異常時の終了条件を誤読しやすい。

### 方針
- modify を維持。
- 実装時は retry 間隔・上限・ログ間引きを設計対象に含める。
- restart と allow_exit の優先順位を仕様として明文化する。

### CE005 仕様補強メモ (2026-07-27)
#### 目的
- CE005 (CONFIG_FILE_MISSING) の「欠損/破損」判定スコープを明文化し、実装差分やリファクタで診断粒度が後退しないようにする。

#### 例外判定スコープ
- CE005としてカウント対象にする例外
  - FileNotFoundError
  - PermissionError
  - IsADirectoryError
  - NotADirectoryError
  - OSError
  - UnicodeDecodeError
  - UnicodeError
  - configparser.NoOptionError
  - configparser.NoSectionError
  - configparser.MissingSectionHeaderError
  - configparser.ParsingError
  - configparser.DuplicateOptionError
  - configparser.DuplicateSectionError
  - ValueError (getboolean/getint などの設定値変換エラー)
- 上記以外は CE005 非該当として False を返す。

#### SHI完成実装との照合追記 (2026-09-04)
- SHI担当の完成実装では、設定値の型変換失敗を表す ValueError もCE005対象に含まれる。owner-first方針に従い、上記スコープへ追加した。
- 同実装には、同じ例外型と先頭メッセージが1.0秒未満に反復した場合のcounter抑止がある。例外型またはメッセージが変われば即時に再計上する。
- SHIの最大3回後に設定再確認を永久停止する制御は、後述のvendor方針「無限再試行を維持する」と衝突するため採用しない。
- SHI `settings_validation.py` の型・上下限・許容値検証をM-027で接続した。strict時の `ConfigValidationError` はValueErrorとしてCE005へ入り、壊れたsection名は後続のAppConfig構築で `NoSectionError` / `NoOptionError` となる。normalize時だけ、定義済み規則に従って実行時補正する。

#### カウンタ方針
- CE005 該当時のみ increment_counter() を呼ぶ。
- CE005 非該当時はカウンタを変更しない。

#### 受け入れ基準
1. 設定ファイル欠損・権限異常・パス種別不整合で CE005 が True になり、カウンタが増加する。
2. INI 構文破損・セクション欠損・キー欠損・重複定義で CE005 が True になり、カウンタが増加する。
3. CE005 非該当例外では False を返し、カウンタ不変となる。
4. E-004 の retry 方針 (間隔・上限・ログ間引き) と矛盾しない。

### CE005 発生トリガー一覧 (2026-07-27)
#### A. 起動時トリガー (main 起動シーケンス)
- load_config() 内で machine_profile を設定反映する時
  - MachineProfileHandler.apply_model_specific_config() が settings.ini / モデル別 ini を読み書きする。
- load_config() 内で SharedAppConfig を生成・読込する時
  - SharedAppConfig(directory_config) の初期化と sac.read() 実行時に設定内容の整合が要求される。
- load_config() 内で SharedAppConfigCalibration を生成する時
  - calib_settings.ini の存在と構文整合が要求される。
- 上記で例外が発生した場合は ActionErrorIndex.CONFIG_FILE_MISSING の excepts_diagnosis(e) を経由して CE005 判定に入る。

#### B. 運用時トリガー (設定変更監視)
- DebouncedEventHandler.process_event() で settings.ini 更新イベントを受けた時
  - sac.write(sec) 実行中の設定再読込・反映で例外が発生し得る。
- ChangeModelEventHandler.process_event() でモデル設定変更イベントを受けた時
  - apply_model_specific_config() 実行中の設定読込/書込で例外が発生し得る。
- いずれも例外時は ActionErrorIndex.CONFIG_FILE_MISSING の excepts_diagnosis(e) を呼び CE005 判定に入る。

#### C. 直接のファイルI/Oトリガー (根本要因)
- machine_profile.py
  - settings.ini の read (base_app_ini.read)
  - モデル別 ini の read (model_config.read)
  - settings.ini の write (open(..., "w"))
- shared_app_config.py
  - load_directory_config_from_ini() 経由で settings.ini を読込
  - AppConfig / AppConfigCalibration 構築時の各種 ini 取得

#### D. 非トリガー (誤検知防止のための明記)
- sac.read() 単体は共有メモリ読込であり、直接の設定ファイルI/Oではない。
- ただし直前の write / apply_model_specific_config の失敗影響を受けるため、結果として CE005 事象の観測点にはなり得る。

#### 受け入れ基準
1. CE005 の発生契機を「起動時」「運用時」「直接I/O」「非トリガー」で分類して説明できる。
2. 新規に設定読込経路を追加する変更では、本一覧への追記を同一変更セットで実施する。

### E-004 実装前仕様 (retry/ノイズ抑制) (2026-07-27)
#### retry 方針
- load_config() の再試行は固定間隔ではなく段階的 backoff を採用する。
- 初期間隔は 0.5 秒、上限は 10.0 秒とする。
- 連続失敗時は retry_interval_sec = min(retry_interval_sec * 2.0, 10.0) で増加する。
- 読込成功時は retry_interval_sec を 0.5 秒へリセットする。

#### retry 上限と遷移方針
- 無限再試行は維持する (起動不能で即終了させない)。
- ただし運用可観測性のため、連続失敗回数 consecutive_failures は管理する。
- 連続失敗回数に応じてログ粒度を切り替える。

#### ログ間引き方針
- CE005 該当例外のログは次の条件で error 出力する。
  - 連続失敗 1 回目
  - 2, 4, 8, 16 ... の 2 のべき乗回目
  - 前回 error ログ出力から 60 秒以上経過した回
- 上記以外の回は warning もしくは debug 相当の軽量出力に落とす。
- ログ本文には最低限以下を含める。
  - consecutive_failures
  - retry_interval_sec
  - exception class 名
  - 代表メッセージ (先頭1行)

#### CE005 カウンタ増加制御方針
- retry ループ内で同一原因が短時間に連続発生する場合、毎回 increment_counter() しない。
- 既定では「前回カウントから 1.0 秒未満」の同種例外はカウント抑止する。
- 例外クラスまたはメッセージが変化した場合は即カウント対象とする。

#### restart / allow_exit 優先順位
- 異常時の制御優先順位は次で固定する。
  1. 明示的終了要求 (allow_exit)
  2. 安全停止要求 (fail-safe由来の停止)
  3. restart 要求
  4. 通常継続
- finally 節では上記優先順位を崩さない実装順序にする。

#### 受け入れ基準
1. 設定ファイル欠損が継続しても CPU スパイクを誘発する busy retry にならない。
2. 連続失敗中でもエラーログは間引かれ、運用上必要な情報は保持される。
3. 設定復旧後に load_config() が正常復帰し、retry_interval_sec が初期値へ戻る。
4. allow_exit/restart の優先順位がテストシナリオどおりに一意に解釈できる。

## E-005 一次棚卸し結果 (2026-07-26)
### 呼び出し元
- app_manager_process.py: LOG_OUTPUT_STOPPED / LIDAR*_CONNECTION_ERROR / SURROUND_MONITOR_MODULE_NOT_RESPONDING
- object_detect_process.py: CE013(AI_MODEL_LOAD_FAILED) の excepts_diagnosis

### 契約上の注意点
1. process 側から渡す引数順序と diagnosis 側 _parse_args の順序が一致しているかを個別に確認する必要がある。
2. 現在は一部診断が簡易実装 (常に False/True) のため顕在化していない契約不整合が将来顕在化しうる。

### 次の確認単位
- E-005-1: app_manager_process.py で呼ぶ各診断について引数順序と型を突合。
- E-005-3: CE013 例外種別とフォールバック動作の期待仕様を整理。

## E-005-1 レビュー結果 (2026-07-26)
### 事実
- app_manager_process では LOG_OUTPUT_STOPPED / LIDAR*_CONNECTION_ERROR / SURROUND_MONITOR_MODULE_NOT_RESPONDING に対して errors_diagnosis を呼んでいる。
- 現在の state_errors 実装では上記3診断はいずれも簡易実装 (detect_error=False, recover/failsafe=True) で、引数を実際には参照していない。

### リスク
1. 引数を参照しない簡易実装のため、呼び出し順序や型の不整合が潜在化したままになりやすい。
2. 将来これら3診断を実装化した時点で、_parse_args 導入により実行時例外化する可能性がある。

### 方針
- modify を維持。
- 本実装化時に _parse_args を必須化し、呼び出し側の引数契約を同時に固定する。

## E-005-3 レビュー結果 (2026-07-26)
### 事実
- object_detect_process では AIモデル初期化失敗時に CE013(AI_MODEL_LOAD_FAILED) の excepts_diagnosis(e) を呼ぶ。
- ただし ActionErrorDiagnosisB の excepts_diagnosis は抽象契約であり、AiModelLoadFailed では未実装。

### リスク
1. CE013 呼び出し経路で NotImplementedError が発生しうる。
2. 例外種別に応じたカウント条件が未定義のため、運用上の CE013 一貫性が担保できない。

### 方針
- modify を維持。
- AiModelLoadFailed.excepts_diagnosis を実装対象として、対象例外種別とカウンタ条件を仕様化する。

## E-005-2 取り扱い
- ユーザー合意により、tegrastats 経路は今回対象外とする。
- 必要になった時点で E-005-2 を再オープンする。

## E-006 実装メモ (2026-07-27)
### 目的
- CE006(SENSOR_CALIB_DATA_INVALID) を、LiDAR校正マトリクスの異常検知に使えるようにするための枠組みを先行実装する。

### 対象マトリクス
- lidar2lidar: calibration.BothLidars (例: lidar2lidar_trans_mat_*.csv)
- lidar2crane: calibration.Lidar0..Lidar5 (例: lidar2crane_trans_mat_*.csv)

### 現段階で実装した検証
1. CSVファイル存在確認
2. CSV読込可否確認
3. 形状が4x4であること (enforce_shape_4x4)
4. 有限値のみで構成されること (finite_value_only)

### 未実装(今後追加)
- 参照標準データとの差分判定アルゴリズム
  - パラメータ枠のみ追加済み:
    - enable_reference_diff_check
    - reference_data_path
    - diff_threshold

### 呼び出し経路
- 起動時: load_config() 内で検証
- 運用時: AppManagerProcess の常時監視では再検証しない（2026-07-28方針変更）

### 受け入れ観点(枠組み)
1. 対象CSVが欠損/破損/不正形状/非有限値の場合に CE006 カウンタが増加する。
2. 正常CSV時は CE006 カウンタが増加しない。
3. 今後、差分判定アルゴを validator に差し込める構造である。

## E-001 レビュー結果 (2026-07-26)
### 事実
- app_manager_process 側で LOG_OUTPUT_STOPPED は定期的に errors_diagnosis(last_mono, now_mono) が呼ばれている。
- 現在の LogOutputStoppedDiagnosis は簡易実装で、detect_error=False / recover=True 固定。

### リスク
1. ログ更新停止監視の判定が実質無効化され、検知イベントが上がらない。
2. app_manager_process の監視コードは存在しても、診断側実装不足により運用期待と乖離する。

### 方針
- modify を採用。
- 実装仕様は「単調時刻ベース差分判定」を標準とし、引数契約は (last_update_mono: float, now_mono: float) に固定する。
- error_config の log_output_stopped パラメータを利用して停止許容秒を可変化する。

## E-002 レビュー結果 (2026-07-26)
### 事実
- SharedErrors.state_errors_A_C は StateErrorIndex の先頭実エントリ群に対応している。
- StateErrorIndex には多数の RESERVED が含まれるが、state_errors_A_C 側に予約分の実体は未配置。

### リスク
1. 予約領域インデックスを直接参照する実装が将来入ると tuple 範囲外アクセスとなる。
2. enum と tuple の対応がコメント依存で、機械的な整合検証が無い。

### 方針
- modify を採用。
- 実装時に「参照可能インデックス上限」と「実体配列長」の整合チェックを初期化時に追加する。
- 新規エラー追加時は enum 追加と tuple 追加を同一変更セットで必須化する。

## 実装仕様ドラフト (次実装の入力)

### E-005-3: CE013 (AI_MODEL_LOAD_FAILED)
#### 実装対象
- diagnosis/action_errors.py の AiModelLoadFailed.excepts_diagnosis を実装する。

#### 例外判定方針
- CE013としてカウントする例外
  - FileNotFoundError
  - PermissionError
  - OSError
  - RuntimeError
  - ValueError
  - ImportError
  - ModuleNotFoundError
- それ以外の例外は CE013 非該当として False を返す。

#### カウンタ方針
- CE013該当時のみ increment_counter() を呼ぶ。
- 非該当時はカウンタを変更しない。

#### 受け入れ基準
1. object_detect_process から excepts_diagnosis(e) を呼んでも NotImplementedError が発生しない。
2. 該当例外で True + カウンタ増加、非該当例外で False + カウンタ不変となる。
3. フォールバック (NotAppliedObjDetection) 経路を阻害しない。

### E-001: LogOutputStoppedDiagnosis
#### 実装対象
- diagnosis/state_errors.py の LogOutputStoppedDiagnosis を簡易実装から実判定へ更新。

#### 引数契約
- errors_diagnosis の入力契約を次で固定
  - args[0]: last_update_mono (float)
  - args[1]: now_mono (float)

#### 判定方針
- update() で error_config.log_output_stopped のパラメータを反映する。
- detect_error
  - (now_mono - last_update_mono) >= error_threshold_sec で True。
- detect_recovery_error / detect_recovery_fail_safe
  - (now_mono - last_update_mono) < recovery_receive_interval_sec を基準に復帰判定。
- 負値差分は 0.0 とみなす。

#### 受け入れ基準
1. app_manager_process からの既存呼び出し (last_mono, now_mono) で ValueError が出ない。
2. 閾値超過で DETECTION、復帰条件充足で RECOVERY が返る。
3. ログ監視無効時の擬似回復経路と矛盾しない。

## E-008 回帰テスト観点 (2026-07-30)
対象: APPLICATION_MANAGER_NOT_RESPONDING の再発防止ロジック

### 目的
- 起動直後の誤検知防止、同一サイクルでの検出/復帰同時発火防止、異常後の確実な復帰を継続保証する。

### 前提設定
- AppManager heartbeat 更新周期: 0.5秒
- APPLICATION_MANAGER_NOT_RESPONDING.error_threshold_sec: 5.0秒
- ApplicationManagerNotRespondingDiagnosis._heartbeat_tolerance_sec: 0.01秒

### テストシナリオ
1. 起動順耐性
- 条件: ErrorMonitor を先行起動し、AppManager未起動状態を数秒維持。
- 期待: APPLICATION_MANAGER_NOT_RESPONDING は未検知、ErrorMonitor プロセスは継続。

2. 初回heartbeatブートストラップ
- 条件: AppManagerが最初のheartbeatを1回だけ更新。
- 期待: 初回有効heartbeatは基準値登録のみで、即時DETECTIONしない。

3. 正常継続(0.5秒周期)
- 条件: heartbeatを0.5秒前後で継続更新(30秒以上)。
- 期待: APPLICATION_MANAGER_NOT_RESPONDING は終始OFF、RuntimeError未発生。

4. 停止検知(長時間未更新)
- 条件: heartbeat更新を停止して5秒以上経過させる。
- 期待: APPLICATION_MANAGER_NOT_RESPONDING がDETECTIONし、state_error bit38がON。

5. 復帰検知
- 条件: 停止後にheartbeat更新を再開。
- 期待: APPLICATION_MANAGER_NOT_RESPONDING がRECOVERYし、state_error bit38がOFF。

6. 同時発火防止
- 条件: 検知境界付近(5.0秒ちょうど前後)でheartbeat再開タイミングを揺らす。
- 期待: 「エラー検出とエラー復帰が同時に発生」が再発しない。

7. 時刻後退異常
- 条件: last_heartbeat を人為的に大きく後退させる(>=5秒相当の負方向ジャンプ)。
- 期待: ジャンプ異常としてDETECTIONする。

8. MMAP継続性
- 条件: 上記1-7の全ケースでErrorMonitor動作を観測。
- 期待: state_error/action_error のMMAP書き込みループが停止しない。

### 合否判定のログ観測点
- process crash 有無: ErrorMonitor の traceback 出力有無。
- state_error 16byte 値: bit38 のON/OFF遷移。
- recovered ログ: APPLICATION_MANAGER_NOT_RESPONDING recovered の出力。

### E-002: Indexとdiagnosis実体の対応保証
#### 実装対象
- shared_errors.py 初期化時に enum/tuple 整合チェックを追加。

#### 整合チェック方針
- state_errors_A_C
  - len(state_errors_A_C) > StateErrorIndex.LOG_OUTPUT_STOPPED を保証。
- action_errors_A_C
  - len(action_errors_A_C) > ActionErrorIndex.LOG_FILE_IO_ERROR を保証。
- module_errors
  - len(module_errors) > ModuleErrorIndex.CALIBRATION_MODULE_ERROR を保証。
- 不整合時は起動時に ValueError を送出して即時検知する。

#### 受け入れ基準
1. 正常構成で起動時例外が出ない。
2. 任意に1要素欠落させた場合に ValueError で即時失敗する。
3. 将来の enum 追加漏れを起動時に検出できる。

## 合意ログ
- 2026-07-26: 委託先反映後に設計見直しへ移行。まず台帳化して議論のゼロリセットを防ぐ方針に合意。
- 2026-07-26: 実装フェーズでは __main__.py / process連携も対象に含める方針へ更新。
- 2026-07-26: E-004 をレビュー実施。E-005 は一次棚卸しまで完了。
- 2026-07-26: E-005-2 (tegrastats) は対象外化。E-005-1/E-005-3 を優先継続。
- 2026-07-26: E-005-3 の実装として AiModelLoadFailed.excepts_diagnosis を追加。
- 2026-07-26: 推論時異常は CE013 に混在させず、ObjectDetectProcess で app_logger に warning を出して継続する方針を反映。
- 2026-07-26: Dレベル異常として AiInferenceResultError を追加し、ObjectDetectProcess の推論例外を state_errors_D 経路に接続。
- 2026-07-30: LiDARエラー群 (SE001/SE002/SE008/SE009/SE014/SE015/SE020/SE021 + Dレベル欠落) の設計開始。論点 E-009〜E-013 を追加。
- 2026-07-31: SE001/SE002 引数逆転バグの修正方針を Camera0 方式統一に確定。重要度別後処理仕様 (UI挙動・システム制御) を台帳に追加。
- 2026-07-31: SE008 のアイコン点滅防止は診断クラス側 error_confirm_duration_sec で吸収を確定。複数エラー同時発生の UI 優先・複数B同時発生は UI チームと合意済み・バックエンド範囲外。
- 2026-07-31: SE008 詳細設計確定。udp_cnt欠番+dot_num低下を last_quality_degraded_mono パターンで伝搬。rx_missed_errors は Phase2。
- 2026-07-31: SE014 設計確定。SE008 is_error フラグを入力とする時刻ベース2段階格上げ方式。Camera 独立実装方式は不採用。
- 2026-07-31: SE020 設計確定。反射強度フィルタ前の原点点割合を主検出。Tag ベース低信頼度点は Phase2。
- 2026-07-31: Lidarデータ欠落(D)設計確定。PointsProviderProcess での直接ロギング方式。state_d_errors 機構は例外専用のため不採用。1フレーム欠落はシステム性能に影響しないとの合意を記録。
- 2026-08-02: SE040/SE041 IMU接続エラー(B)の設計追加。論点 E-014 を追加。SharedIMUExcept への last_heartbeat 追加・ImuProviderProcess heartbeat 更新・AppManagerProcess 監視追加が実装差分。Camera0/Lidar 方式と同一パターン。
- 2026-08-03: CE004(CRANE_MODEL_FILE_MISSING) 機体モデルファイル欠損/破損の設計追加。論点 E-015 を追加。excepts_diagnosis 実装・PointsRefineProcess/_startup_remove/_startup_collision_cliff + VisualProcess/_startup のトリガー追加が実装差分。
- 2026-08-03: CE011(MMAP_READ_WRITE_ERROR) MMAPファイル読み書きエラーの設計追加。論点 E-016 を追加。Phase1: error_monitor_process._update() 内 ErrorMMapWriter write 系呼び出しのラップ。classMMap/status_mmap はPhase2。
- 2026-08-03: CE015(LOG_FILE_IO_ERROR) ログファイルI/Oエラーの設計追加。論点 E-017 を追加。Python logging.handleError オーバーライドで検知。C++ は logfunc コールバック経由で Python ログシステムに書くため直接ファイルI/Oなし。damp ファイルは通常 damp_out=False のため対象外。
- 2026-08-14: Dレベルの基盤を vendor 形式へ統合。通常診断は `StateErrorDIndex` / `state_errors_D`、例外分類は `state_errors_D_ex`、モジュール例外は `ModuleErrorIndex` / `module_errors` として現状動作を維持する。Dレベル全体のindex再編・未実装診断の登録・設定と呼び出し経路の再設計は E-018 として保留し、この時点では動作中の経路を拡張しない。
- 2026-08-14: 2026-07-31 の LiDARデータ欠落を `PointsProviderProcess` の直接ログ方式とし、`state_d_errors` 機構を不採用とした判断は、vendor のDレベル設計を把握する前のものだったため確定扱いを解除する。E-018 で `LidarDataMissing` の登録先・判定条件・設定・ログ出力経路を再検討する。

---

## MID-360 仕様調査結果メモ (2026-07-30)
### 調査方法
- Livox SDK2 公式Wiki (HAP通信プロトコル英語版) および livox_ros_driver2 README・ソースをネットワーク上から直接確認。
- MID-360固有の仕様PDFは公式Wikiには公開されていないが、MID-360はHAPと同一のLivox SDK2通信プロトコルファミリーを使用する。
- 本プロジェクト `mid360_points.py` の実装と一致: 36バイトヘッダー + Data1 (14バイト×96点/パケット, Cartesian 32bit)。

### UDPパケットヘッダー (Data1, offset=0起点)
| フィールド | オフセット | サイズ | 説明 |
|---|---|---|---|
| version | 0 | 1B | プロトコルバージョン (現在 0) |
| length | 1 | 2B | UDPデータ長 (versionからの全バイト数) |
| time_interval | 3 | 2B | フレーム内の最初〜最後の点の時間差 [0.1µs] |
| dot_num | 5 | 2B | このパケット内の点数 (通常 96) |
| udp_cnt | 7 | 2B | フレーム内UDPパケット連番 (フレーム開始で0リセット、毎パケット+1) |
| frame_cnt | 9 | 1B | フレームカウンタ (MID-360では常に0の模様) |
| data_type | 10 | 1B | データ形式 (0x01=Cartesian32bit, 0x02=Cartesian16bit) |
| time_type | 11 | 1B | タイムスタンプ種別 (0=電源ON基準[ns], 1=gPTP[ns]) |
| pack_info | 12 | 1B | bit0-1: safety_info (0=全信頼, 1=全不信頼, 2=非ゼロ点のみ信頼) |
| reserved | 13 | 11B | 予約 |
| crc32 | 24 | 4B | timestamp+data セクションのCRC-32チェック |
| timestamp | 28 | 8B | 先頭点のタイムスタンプ [ns] (uint64, little-endian) |
| data | 36 | - | 点群データ (14B × dot_num) |

### 各点データ (Data1, 14バイト/点)
| フィールド | オフセット | 型 | 説明 |
|---|---|---|---|
| x | 0 | int32_t | X座標 [mm] |
| y | 4 | int32_t | Y座標 [mm] |
| z | 8 | int32_t | Z座標 [mm] |
| Reflectivity | 12 | uint8_t | 反射強度 (0〜255) |
| Tag | 13 | uint8_t | 信頼度フラグ (下記参照) |

### Tagフィールド信頼度ビット定義
| ビット | 意味 | 値 |
|---|---|---|
| bit0-1 | 空間位置に基づく信頼度 | 00=高(正常), 01=中, 10=低, 11=非常に低 |
| bit2-3 | エネルギー強度に基づく信頼度 | 00=高(正常), 01=中, 10=低, 11=非常に低 |
| bit4-5 | その他カテゴリに基づく信頼度 | 00=高(正常), 01=中, 10=低, 11=非常に低 (HAP系では未使用) |
| bit6-7 | 予約 |   |

→ tag == 0x00 が「全カテゴリ高信頼度」点。  
→ 各カテゴリのbit対が 11 (=3) の点は「非常に低信頼度」＝事実上の不良点。

### 現在の mid360_points.py の実装状況
- **実装済み**: x/y/z (mm→m変換), reflectivity, timestamp (ns→s変換)
- **未取得**: `dot_num`, `udp_cnt`, `data_type`, `pack_info.safety_info`, `Tag` フィールド
- `udp_cnt` 欠番・`dot_num` 低下・`Tag` 低信頼度率の観測には mid360_points.py への追加実装が必要。

### LiDARデバイス状態プッシュ (毎秒, CMD 0x0102 経由)
- `cur_work_state` (key 0x8006, 1B): 0x01=SAMPLING, 0x04=ERROR, 0x02=IDLE 等
- `lidar_diag_status` (key 0x800E, 2B): system/scan/ranging/communication モジュール別診断
  - 各4bitフィールド: 0=正常, 1=warning, 2=error, 3=safety_err
- `status_code` (key 0x800D, 32B): 各bitが個別異常フラグ
- **現時点の実装では上記プッシュ情報を受信・活用していない。** 本格的なデバイス側診断活用は将来課題。

---

## Lidarエラー群 設計セクション (2026-07-30)

### 対象エラー一覧
| エラー番号 | StateErrorIndex (0-based) | エラーレベル | 種別クラス | エラー名 |
|---|---|---|---|---|
| SE001 | LIDAR0_CONNECTION_ERROR (0) | 重要度A | StateErrorDiagnosisA | Lidar0接続エラー |
| SE002 | LIDAR1_CONNECTION_ERROR (1) | 重要度A | StateErrorDiagnosisA | Lidar1接続エラー |
| SE008 | LIDAR0_COMM_QUALITY_DEGRADED (7) | 重要度C | StateErrorDiagnosisC | Lidar0通信品質低下 |
| SE009 | LIDAR1_COMM_QUALITY_DEGRADED (8) | 重要度C | StateErrorDiagnosisC | Lidar1通信品質低下 |
| SE014 | LIDAR0_COMM_QUALITY_ERROR (13) | 重要度B | StateErrorDiagnosisB | Lidar0通信品質エラー |
| SE015 | LIDAR1_COMM_QUALITY_ERROR (14) | 重要度B | StateErrorDiagnosisB | Lidar1通信品質エラー |
| SE020 | LIDAR0_INVALID_DATA (19) | 重要度B | StateErrorDiagnosisB | Lidar0データ不正 |
| SE021 | LIDAR1_INVALID_DATA (20) | 重要度B | StateErrorDiagnosisB | Lidar1データ不正 |
| (D) | state_d_errors | 重要度D | StateErrorDiagnosisD | Lidarデータ欠落 |

---

## E-009: SE001/SE002 Lidar接続エラー 設計 (2026-07-30)

### 現状
- `Lidar0ConnectionErrorDiagnosis.detect_error()` は stub (常に False)。
- `app_manager_process.py` の呼び出し: `errors_diagnosis(last_heartbeat, now_mono)`
  - `args[0] = last_heartbeat.value`, `args[1] = now_mono`
- Camera0ConnectionErrorDiagnosis の `_parse_args` 引数順序は `args[0]=now, args[1]=last_heartbeat` と**逆順**。
- → **潜在的な引数逆転バグ**: Lidar診断実装化の際に必ず修正が必要。

### 引数契約の方針決定 (2026-07-31 確定)
- **Camera0方式に統一することを確定。**
- `_parse_args` 引数順序: `args[0]=now (float)`, `args[1]=last_heartbeat (float)` に固定する。
- 実装化時は **呼び出し側 (app_manager_process.py) を先に修正**し、その後診断クラスを実装する。
- 修正後の呼び出し: `errors_diagnosis(now_mono, last_heartbeat.value)`
- Camera0ConnectionErrorDiagnosis の実装コードを LidarN 向けに流用する形を基本とする。

### 検出ロジック (設計案)
Camera0ConnectionErrorDiagnosis と同一パターンを採用。

**クラス外 (PointsProviderProcess / LIDARプロセス側)**:
- ループごとに `get_points()` が成功したら `SharedLIDExcept[i].last_heartbeat` をその時刻 (`time.monotonic()`) で更新する。
- 接続直後または点群取得失敗時は last_heartbeat を更新しない（または初期値 0.0 を維持）。

**クラス内 (Lidar0ConnectionErrorDiagnosis)**:
- detect_error: `now - last_heartbeat >= error_threshold_sec` のとき True。
  - 初回 (last_heartbeat が初期値 0.0 や負値) はブートストラップ期間として扱い、エラー判定しない。
  - Camera0 と同様: `_previous_heartbeat is None or last_heartbeat < 0.0` → False返却・記録のみ。
- detect_recovery_error: 正常受信 (last_heartbeat が `recovery_receive_interval_sec` 以内に更新) が `error_recovery_confirm_duration_sec` 秒継続。
- detect_recovery_fail_safe: 同様の継続確認で failsafe フラグを解除。

### 設定パラメータ (LidarNConnectionErrorParameters に追加)
| パラメータ | 型 | 説明 | 候補デフォルト値 |
|---|---|---|---|
| is_enabled | bool | 診断有効フラグ | True |
| error_threshold_sec | float | heartbeat未更新でエラー判定する閾値 [s] | 5.0 |
| error_recovery_confirm_duration_sec | float | エラー復帰確認継続時間 [s] | 5.0 |
| failsafe_recovery_confirm_duration_sec | float | failsafe復帰確認継続時間 [s] | 5.0 |
| recovery_receive_interval_sec | float | 復帰判定中の許容受信間隔 [s] | 1.0 |

### 論点・議論ポイント (2026-07-30)
1. **error_threshold_sec 5秒の妥当性**:
   - MID-360は通常 10Hz / 20Hz / 50Hz 等のフレームレート。5秒で検出は保守的（安全側）。
   - 実運用でのUDP timeout / recv timeout の発生頻度に合わせて調整可能。
   - `socket.settimeout(0.1)` が設定されているため、接続断後は0.1秒後に timeout exception が発生する。つまり heartbeat 更新は100ms周期以上で停止する → 5秒 = 50回の timeout が必要。
   - 案: 2〜3秒への短縮も検討余地あり。

2. **復帰判定: 案①(連続成功カウンタ) vs 案②(時刻継続確認)**:
   - 案① (PointsProviderProcess が X回連続取得成功で復帰通知): PointsProviderProcess に復帰判定ロジックが入る。シンプルだが責任分散。
   - 案② (AppManagerProcess が last_heartbeat を X秒確認継続で復帰): Camera0と同一パターン。検出・復帰の対称性が高い。
   - **推奨: 案②** (Camera0との一貫性・AppManagerの監視責任の集約)。

3. **MID-360からの `recv timeout` 例外の扱い**:
   - 現在は `socket.settimeout(0.1)` による `TimeoutError` がスローされる可能性あり。
   - この例外を PointsProviderProcess がキャッチして last_heartbeat を更新しない（=接続断扱い）とするのが自然。
   - 既存実装でのハンドリング有無を実装時に確認する。

### 受け入れ基準
1. 接続断（`recv timeout` 連続）が `error_threshold_sec` 秒継続で DETECTION。
2. 再接続・データ受信再開が `error_recovery_confirm_duration_sec` 秒継続で RECOVERY。
3. ブートストラップ期間（起動直後の heartbeat 未設定）で誤検知しない。
4. 引数逆転修正後も既存の Camera0 等の呼び出し契約が破綻しない。

---

## E-010: SE008/SE009 Lidar通信品質低下 設計 (2026-07-31 詳細化)

### 概念
- LiDAR は物理的には接続中だが、UDP パケットが散発的に欠損・信頼性低下している状態を検知する。
- SE001（接続断）より軽度だが、継続すると点群品質が劣化し計測精度に影響する。

---

### 実装レイヤー構造 (現状確認)

```
PointsProviderProcess._update()
  └─ CalibMid360PointCloudProvider.get_accum_points()   [0.1秒分を蓄積]
       └─ MID360Points.get_points()  [1パケット=96点, 約 471 回 / 0.1秒]
```

- `PointsProviderProcess` は `provider.get_accum_points()` しか呼ばない。
- `MID360Points.get_points()` は `provider` 内部のループで呼ばれ、外部から直接アクセスされない。
- 品質メトリクスは `MID360Points` レベルで計算し、プロパティ経由でプロバイダ → プロセスと伝搬させるのが最小変更。

---

### 検出信号ソースの比較と方針確定

#### 信号① udp_cnt 欠番 (主検出)
| 項目 | 内容 |
|---|---|
| ヘッダー位置 | offset 7-8 (uint16, little-endian) |
| 意味 | フレーム開始で 0 リセット、パケット毎に +1。欠番 = ネットワーク経路でのパケットロス |
| 根本原因例 | ケーブル劣化・スイッチ輻輳・EMI干渉 |
| 実装コスト | 低 (2バイト読むだけ) |
| 注意点 | `udp_cnt == 0` はフレーム境界として扱い、カウントをリセットする |

**フレーム境界の扱い**:
- `udp_cnt == 0` → 新フレーム開始。`_prev_udp_cnt` をリセット（欠番カウントしない）。
- `udp_cnt != prev + 1` かつ `udp_cnt != 0` → ギャップ発生。

**偽陽性リスク**:
- フレーム先頭パケットを見逃した場合、前フレームの最終 `udp_cnt` と次の値がジャンプして見える。
  → ただし、`udp_cnt == 0` を受信できれば防げる。実運用では稀なので許容する。

#### 信号② dot_num 低下 (補助検出)
| 項目 | 内容 |
|---|---|
| ヘッダー位置 | offset 5-6 (uint16, little-endian) |
| 意味 | 1パケットの点数 (正常は 96)。低下 = LiDAR側の問題またはフィルタ動作 |
| 実装コスト | 低 (信号①と同じヘッダー読み取り内) |
| 注意点 | ブラインドスポット設定 (50cm) により正常でも若干低下する場合あり。閾値を保守的に設定 |

#### 信号③ pack_info.safety_info (将来検討)
| 項目 | 内容 |
|---|---|
| ヘッダー位置 | offset 12 の bit0-1 |
| 意味 | 0=パッケージ全体信頼, 1=全体不信頼, 2=非ゼロ点のみ信頼 |
| 根本原因例 | センサー自己診断による信頼性低下（干渉・内部エラー等） |
| 実装コスト | 低 (同ヘッダー読み取り内) |
| 特徴 | **デバイス側の自己診断**。udp_cnt 欠番はネットワーク経路の問題、こちらはデバイス側の問題 |

- 解釈としては「通信品質」より「データ信頼性」寄りであるため、Phase1 の SE008 判定基準からは外す。
- 将来的に SE020 の補助信号、または別のデータ信頼性指標として扱う候補。

#### 信号④ rx_missed_errors / rx_no_buffer (Phase2・除外)
| 項目 | 内容 |
|---|---|
| 取得先 | `ethtool -S <NIC名>` (OS/NICレベル) |
| 意味 | NICがパケットを受け取ったがホストバッファが満杯でドロップ → ホスト側受信能力の問題 |
| 根本原因例 | CPU過負荷・受信バッファ設定不足 |
| 実装コスト | 高 (`subprocess` + テキストパース + 非同期処理 + 権限問題) |
| Phase1 判断 | **除外**。udp_cnt 欠番でネットワーク側ロスはほぼ捕捉できる。LidarShiftMonitor 同様の別プロセスとして将来実装 |

#### 信号の組み合わせ方針
- **Phase1 では udp_cnt 欠番 または dot_num 低下が発生 → 品質低下イベント** とする。
- ユーザーへの通知は「Lidar通信品質低下」1種類でよい（原因の詳細はログに残す）。
- 原因別に別エラーコードを設けるのは UI/オペレーターへの複雑性が増すため採用しない。

---

### アーキテクチャ設計

品質低下の検出は「最後に品質低下イベントが発生した単調時刻」を共有メモリで伝搬させ、
SE001 の `last_heartbeat` パターンと対称的な設計にする。

```
[MID360Points]
  ・get_points() 内で以下を計算:
    ① udp_cnt ギャップ検出
    ② dot_num < threshold
  ・いずれか発生 → _last_quality_degraded_mono = time.monotonic() を更新
  ・プロパティ last_quality_degraded_mono を公開

[CalibMid360PointCloudProvider]
  ・プロパティ last_quality_degraded_mono をデバイスから転送

[PointsProviderProcess]
  ・get_accum_points() 後に provider.last_quality_degraded_mono を読み取り
  ・SharedLIDExcept.last_quality_degraded_mono.value に書き込む

[AppManagerProcess]
  ・監視ループで errors_diagnosis(now_mono, last_quality_degraded_mono.value) を呼ぶ

[Lidar0CommQualityDegradedDiagnosis]
  ・(now - last_degraded) < recent_event_threshold_sec → 現在低下中
  ・現在低下中が error_confirm_duration_sec 秒継続 → DETECTION
  ・現在低下中でない状態が recovery_confirm_duration_sec 秒継続 → RECOVERY
```

この設計の利点:
- heartbeat パターンと完全対称（「最後の正常時刻」vs「最後の異常時刻」）
- 診断クラスが `error_confirm_duration_sec` でアイコン点滅を吸収（2026-07-31 確定方針）
- 共有メモリに書き込む値は float 1つ（`Synchronized[float]`）で実装コスト低

---

### 実装差分一覧

#### MID360Points への追加
```python
# 追加する属性
_prev_udp_cnt: int | None = None
_last_quality_degraded_mono: float = 0.0

# 追加するメソッド
def _check_packet_quality(self, dst_byte: bytes) -> bool:
    dot_num = int.from_bytes(dst_byte[5:7], "little", signed=False)
    udp_cnt = int.from_bytes(dst_byte[7:9], "little", signed=False)

    has_issue = False

    if udp_cnt == 0:
        self._prev_udp_cnt = 0          # フレーム境界リセット
    elif self._prev_udp_cnt is not None and udp_cnt != self._prev_udp_cnt + 1:
        has_issue = True                # udp_cnt ギャップ
    self._prev_udp_cnt = udp_cnt

    if dot_num < DOT_NUM_LOW_THRESHOLD: # デフォルト 50
        has_issue = True

    return has_issue

# get_points() 末尾に追加
if self._check_packet_quality(dst_byte):
    self._last_quality_degraded_mono = time.monotonic()

@property
def last_quality_degraded_mono(self) -> float:
    return self._last_quality_degraded_mono
```

#### CalibMid360PointCloudProvider への追加
```python
@property
def last_quality_degraded_mono(self) -> float:
    return self._device.last_quality_degraded_mono
```

#### SharedLIDExcept への追加
```python
self.last_quality_degraded_mono: Synchronized[float] = create_shared_single_data(0.0)
```

#### PointsProviderProcess._update() への追加
```python
# get_accum_points() 呼び出し後
if hasattr(self._provider, "last_quality_degraded_mono"):
    self._sec_lid.last_quality_degraded_mono.value = \
        self._provider.last_quality_degraded_mono
```

#### AppManagerProcess の監視ループへの追加
```python
se008_index = StateErrorIndex.LIDAR0_COMM_QUALITY_DEGRADED + i
se008_diag = self._ser.state_errors_A_C[se008_index]
se008_diag.errors_diagnosis(
    now_mono,
    self._sec.LiDAR_ex[i].last_quality_degraded_mono.value,
)
```

---

### 設定パラメータ (LidarNCommQualityDegradedParameters に確定)

| パラメータ | 型 | 説明 | 候補デフォルト値 |
|---|---|---|---|
| is_enabled | bool | 診断有効フラグ | True |
| dot_num_low_threshold | int | dot_num がこれ未満で品質低下イベント | 50 |
| recent_event_threshold_sec | float | この秒以内に品質低下イベントがあれば「現在低下中」 | 1.0 |
| error_confirm_duration_sec | float | 低下状態がこの秒継続でDETECTION (点滅防止) | 3.0 |
| recovery_confirm_duration_sec | float | 正常状態がこの秒継続でRECOVERY | 5.0 |

### rx_missed_errors の将来対応 (E-010-B)
- Phase2 として、LidarShiftMonitor に類似した別プロセスで `ethtool -S` を定期実行する。
- ホスト側バッファ不足は udp_cnt 欠番とは独立した故障モードなので、検出信号は別途追加する。
- Phase1 では「udp_cnt ギャップ・dot_num 低下」の2信号で SE008 を構成する。

### 受け入れ基準
1. `udp_cnt` ギャップ・`dot_num` 低下のいずれかで品質低下イベントを生成する。
2. 低下イベントが `recent_event_threshold_sec` 以内に継続して発生し、`error_confirm_duration_sec` 秒持続でDETECTION。
3. 品質低下イベントが止まり `recovery_confirm_duration_sec` 秒継続でRECOVERY。
4. 接続断 (SE001) 中は SE008 を発火させない（AppManagerProcess の呼び出しループで上位エラーをスキップ）。
5. `udp_cnt == 0`（フレーム境界）で欠番と誤判定しない。

---

## E-011: SE014/SE015 Lidar通信品質エラー 設計 (2026-07-31 詳細化)

### 概念
- SE008 (通信品質低下C) が持続する状態を重要度B として格上げする。
- 「一時的な品質低下」と「持続的・深刻な品質問題」を分ける役割。

### 既存カメラB級実装の確認

`Camera0CommQualityErrorDiagnosis` は:
- 入力: `(now, timestamp, read_failure_count)` → SE008 とは独立して自前で品質劣化を判定
- `_read_history: deque[tuple[float, bool]]` で時刻ベースのスライディングウィンドウを保持
- 複数の判定軸（stale/連続失敗数/失敗率）を並行評価

Lidar SE014 は **Camera の独立実装方式は採用しない**。理由:
- SE008 が既に `error_confirm_duration_sec` で安定したC級フラグを出力している。
- SE014 は「SE008 フラグが長時間継続」という純粋な2段階格上げとして定義した方がシンプルかつ意図が明確。
- Camera は「連続失敗数」という直接的な品質信号があるが、Lidar は「最終品質低下イベント時刻」経由の間接的な信号のため、独立実装するとSE008と二重に閾値設計が必要になる。

---

### 設計方針: SE008 の is_error フラグを入力とする

```
AppManagerProcess 監視ループ (per cycle)
  ①  SE001 が OFF の場合のみ ②〜④ を評価
  ②  SE008.errors_diagnosis(now, last_quality_degraded_mono)   → SE008.is_error.value 更新
  ③  SE014.errors_diagnosis(now, SE008.is_error.value)          → SE014.is_error.value 更新
  ④  SE020.errors_diagnosis(now, invalid_data_ratio)
```

- ②の後に③を呼ぶことで、同サイクル内でSE008の最新状態をSE014に渡せる。
- SE001 ON の場合は SE008・SE014 とも評価をスキップ（排他ルール準拠）。

---

### 検出ロジック (Lidar0CommQualityErrorDiagnosis)

**引数契約**:
- `args[0]`: `now_mono (float)`
- `args[1]`: `se008_is_error (bool)` ← SE008.is_error.value

**detect_error**:
- `se008_is_error` が True の間、内部タイマーを積算。
- `se008_is_error` が False になった時点でタイマーをリセット。
- タイマーが `error_confirm_duration_sec` に達したら True (DETECTION)。

**detect_recovery_error**:
- `se008_is_error` が False の間、復帰タイマーを積算。
- `se008_is_error` が True になった時点でリセット。
- 復帰タイマーが `recovery_confirm_duration_sec` に達したら復帰。
- `is_error.value = False` をクリアして True を返す。

**detect_recovery_fail_safe**:
- 同様のパターンで `failsafe_recovery_confirm_duration_sec` を使用。

---

### 設定パラメータ (LidarNCommQualityErrorParameters に確定)

| パラメータ | 型 | 説明 | 候補デフォルト値 |
|---|---|---|---|
| is_enabled | bool | 診断有効フラグ | True |
| error_confirm_duration_sec | float | SE008 ON がこの秒継続で DETECTION | 30.0 |
| recovery_confirm_duration_sec | float | SE008 OFF がこの秒継続で RECOVERY | 30.0 |
| failsafe_recovery_confirm_duration_sec | float | failsafe 復帰確認時間 | 60.0 |

### デフォルト値の根拠
- `error_confirm_duration_sec = 30秒`: SE008（3秒でDETECT）の10倍。「一時的な劣化」と「継続的な問題」を分ける最小境界として設定。実機で調整可能。
- `recovery_confirm_duration_sec = 30秒`: 同程度の継続確認で対称性を確保。短くすると SE008↔SE014 の間でフリッカーが起きやすくなる。

### 受け入れ基準
1. SE008 ON が `error_confirm_duration_sec` 秒継続で DETECTION。
2. SE008 OFF が `recovery_confirm_duration_sec` 秒継続で RECOVERY。
3. SE008 が短時間ON→OFF→ONを繰り返した場合、タイマーが都度リセットされ誤DETECTION しない。
4. SE001 ON 時は評価をスキップし SE014 は変化しない。

---

## E-012: SE020/SE021 Lidarデータ不正 設計 (2026-07-31 詳細化)

### 概念
- 点群データが「存在はするが内容が信頼できない」状態を検知する。
- SE001（接続断）・SE008/SE014（通信品質）とは独立して機能する。
- 対象: LiDAR自体の問題（視野閉塞・レンズ汚れ・発光素子異常等）が継続している状態。

---

### 「不正な点」の定義 (Phase1)

**採用: 原点点 (x=y=z=0)**
- MID-360 は測距失敗時に `x=y=z=0 [mm]` を出力する（ブラインドスポット設定 50cm 以内は本来送出されない）。
- Python 変換後は `(0.0, 0.0, 0.0) [m]`。
- 視野遮蔽・レンズ汚れ・強烈な近距離反射で多発する。

**不採用 (Phase1): Tag フィールドによる低信頼度点**
- Tag は現在 `mid360_points.py` で取得していない。追加にはデータパイプライン全体の変更が必要。
- Phase2 として Tag フィールド追加と同時に組み込む。

**不採用 (Phase1): NaN/Inf 点**
- MID-360 のプロトコル (int32) レベルでは発生しない。
- 変換バグ・オーバーフロー由来なら SE020 でなく別途 safeguard 処理で対応（`PointsProviderProcess` 内の `np.isfinite` チェック）。

---

### データパイプライン上の検出位置

現在 `get_accum_points()` は以下の順で処理する:

```python
# CalibMid360PointCloudProvider.get_accum_points() 内 (抜粋)
frame = np.concatenate(packet_chunks, axis=0)  # shape: (N, 4) [x, y, z, reflectivity]
frame = frame[frame[:, 3] != 0]                 # 輝度ゼロの点を除外
xyz = _as_xyz(frame)                            # shape: (M, 3) [x, y, z] のみ
```

**問題**: 輝度ゼロフィルタの後の `xyz` では原点点が既に取り除かれている可能性がある。
`x=y=z=0` の点が `reflectivity=0` を持つ場合、フィルタで消える。

**解決策**: **反射強度フィルタ前**のデータで原点比率を計算してプロパティに保存する。

```python
# CalibMid360PointCloudProvider に追加
frame = np.concatenate(packet_chunks, axis=0)

# ★ フィルタ前に原点点比率を計算して保持
total = len(frame)
if total > 0:
    is_origin = (frame[:, 0] == 0.0) & (frame[:, 1] == 0.0) & (frame[:, 2] == 0.0)
    self._last_invalid_ratio = float(np.sum(is_origin)) / total
else:
    self._last_invalid_ratio = 0.0

frame = frame[frame[:, 3] != 0]   # 以降は既存通り
```

---

### アーキテクチャ設計

SE008 の `last_quality_degraded_mono` と対称な設計を採用。

```
[CalibMid360PointCloudProvider]
  ・get_accum_points() 内、反射強度フィルタ前に
    _last_invalid_ratio = (原点点数) / (全点数) を計算
  ・プロパティ last_invalid_ratio: float を公開

[PointsProviderProcess]
  ・get_accum_points() 後に provider.last_invalid_ratio を読み
  ・SharedLIDExcept.invalid_data_ratio.value に書き込む

[AppManagerProcess]
  ・errors_diagnosis(now_mono, invalid_data_ratio.value) を呼ぶ

[Lidar0InvalidDataDiagnosis]
  ・invalid_ratio >= threshold → 「今このフレームは不正」
  ・その状態が error_confirm_duration_sec 継続 → DETECTION
  ・invalid_ratio < recovery_threshold が recovery_confirm_duration_sec 継続 → RECOVERY
```

---

### 検出ロジック (Lidar0InvalidDataDiagnosis)

**引数契約**:
- `args[0]`: `now_mono (float)`
- `args[1]`: `invalid_ratio (float)` ← SharedLIDExcept.invalid_data_ratio.value

**detect_error**:
- `invalid_ratio >= invalid_ratio_threshold` の間、エラータイマーを積算。
- 閾値を下回った時点でリセット。
- タイマーが `error_confirm_duration_sec` に達したら True。

**detect_recovery_error**:
- `invalid_ratio < recovery_ratio_threshold` の間、復帰タイマーを積算。
- 閾値以上に戻った時点でリセット。
- 復帰タイマーが `recovery_confirm_duration_sec` に達したら復帰。

---

### 設定パラメータ (LidarNInvalidDataParameters に確定)

| パラメータ | 型 | 説明 | 候補デフォルト値 |
|---|---|---|---|
| is_enabled | bool | 診断有効フラグ | True |
| invalid_ratio_threshold | float | 原点点割合がこれ以上でエラー判定開始 (0.0〜1.0) | 0.7 |
| error_confirm_duration_sec | float | 閾値以上が継続してから DETECTION [s] | 3.0 |
| recovery_ratio_threshold | float | 原点点割合がこれ未満で復帰判定開始 (0.0〜1.0) | 0.3 |
| recovery_confirm_duration_sec | float | 復帰条件継続確認時間 [s] | 3.0 |
| failsafe_recovery_confirm_duration_sec | float | failsafe 復帰確認時間 [s] | 5.0 |

### デフォルト値の根拠
- `invalid_ratio_threshold = 0.7`: フレーム内の70%が原点点なら「視野がほぼ遮蔽されている」と判断。
  センサー位置や環境によって誤検知しやすい場合は引き上げる。
- `recovery_ratio_threshold = 0.3`: エラーしきい値(0.7)とヒステリシスを設けることでフリッカーを防ぐ。
- `error_confirm_duration_sec = 3.0`: SE008 と同じ3秒。一時的な遮蔽（人が通過等）と区別する最低ライン。

### 共有メモリへの追加 (SharedLIDExcept)
```python
self.invalid_data_ratio: Synchronized[float] = create_shared_single_data(0.0)
```

### Phase2 検討事項: Tag フィールドの活用
- `mid360_points.py` で Tag (byte 13 / point) を取得する変更を追加。
- Provider で蓄積フレームの低信頼度点割合を計算し、`SharedLIDExcept.low_confidence_ratio` に追加。
- SE020 の invalid_ratio に Tag ベースの低信頼度割合を合算する。

### 受け入れ基準
1. 原点点割合 >= `invalid_ratio_threshold` が `error_confirm_duration_sec` 秒継続で DETECTION。
2. 原点点割合 < `recovery_ratio_threshold` が `recovery_confirm_duration_sec` 秒継続で RECOVERY。
3. 正常フレーム（原点点なし）では誤検知しない。
4. SE001 ON 時は評価スキップ（AppManagerProcess の呼び出し制御）。
5. フィルタ前のデータで比率を計算しているため、輝度ゼロ原点点の見逃しがない。

---

## E-013: Lidarデータ欠落 (重要度D) 設計 (2026-07-31 詳細化)

### 概念
- 重要度D: **ログ記録のみ**。システム動作・UIへの影響なし。
- LiDAR 単体の蓄積フレーム点数が異常に少ない状態を記録する。障害発生時・寿命到達時のサービスマン分析用。

#### 重要度D の根拠 (2026-07-31 合意)
- LiDAR からのデータが一瞬途切れる（1フレーム欠落する）ことは実運用上あり得る。
- **1フレームの欠落は、安全装置としての最終的なシステム性能に影響しない**と判断している。
- むしろ、データ欠落を検知してシステムが停止・再起動することによるリスク（動作継続不能）の方が重大。
- したがって、D レベル（ログのみ）が適切な重要度であり、A〜C レベルへの格上げは行わない。
- ログの目的は「即時対応のトリガー」ではなく、**長期的な品質劣化・寿命の兆候把握**にある。

---

### 既存 D レベル実装パターンの確認 (2026-07-31)

```
SharedErrors.is_state_error_d_exception(e, logger)
  └─ for diag in state_errors_D:
       diag.excepts_diagnosis(e)  # 例外型の判定のみ
```

- `state_errors_D` は **例外ベース（`excepts_diagnosis(e)`）専用**。
- `is_state_error_d_exception()` からのみ呼ばれ、`detect_error()` を使うパスは現在存在しない。
- `AiInferenceResultError` / `NumericAnomalyException` はいずれも例外キャッチ後に呼ぶ設計。

→ 「データ欠落」は**例外ではなく条件判定**なので、既存 D レベル機構をそのまま流用できない。

---

### 設計方針の選択肢と決定

#### 選択肢① 既存 D 機構を条件判定にも対応させる
- `SharedErrors` に `check_condition_d(name, result, logger)` のような汎用メソッドを追加。
- `LidarDataMissingDiagnosis.detect_error(point_count, min_count)` を実装し、
  `state_d_errors` へ追加する。
- 将来の条件判定型 D レベルが増えたときに再利用できる。
- **工数: 中**（SharedErrors の拡張 + state_d_errors 追加 + process 呼び出し）

#### 選択肢② PointsProviderProcess に直接ロギング
- `PointsProviderProcess._update()` 内で点数チェックし、条件を満たしたら `self._logger.info(...)` で直接記録。
- フレームワーク変更ゼロ。実装コスト最低。
- D レベルの記録手段として「ファイルへの info ログ」は十分に満たせる。
- 設計一貫性の観点では state_errors_D に乗らないのが気になる点。
- **工数: 低**

#### 決定: **選択肢②を採用（直接ロギング）**

理由:
- D レベルの要件は「ログ記録のみ」であり、フレームワーク経由である必要はない。
- 現時点で条件判定型 D レベルは「データ欠落」だけであり、汎用化の動機が薄い。
- 将来的に条件判定型 D レベルが複数必要になった時点で、選択肢①を実装すればよい。
- state_d_errors 機構の変更は既存のモジュールエラー・例外経路に影響を与えるリスクがある。

---

### 検出スコープと責任範囲

| 検出対象 | 検出箇所 | 採用 |
|---|---|---|
| LiDAR 単体の蓄積フレーム点数不足 | `PointsProviderProcess._update()` | **Phase1 対象** |
| `dot_num == 0` の単発パケット | `MID360Points._check_packet_quality()` | SE008 の `dot_num < threshold` で吸収済み → 重複不要 |
| 複数 Lidar 統合後の空データ | `get_data_process.py` 等の下流プロセス | **Phase2 以降（別論点）** |

---

### 検出ロジック (Phase1 設計)

**検出条件**: `pcd is not None AND 0 < len(pcd) < min_point_count`

- `pcd is None` は `get_accum_points()` が TimeoutError を返したケース → SE001 の heartbeat 停止として処理されるので対象外。
- `len(pcd) == 0` は現実装で `if not packet_chunks: return None` に至るケース → 同様に None 扱い。
- `len(pcd) < min_point_count` (0 < 点数 < 閾値) が「データは来たが極端に少ない」状態。

**SE001 抑制**:
```python
se001_index = StateErrorIndex.LIDAR0_CONNECTION_ERROR + self._index
if self._ser.state_errors_A_C[se001_index].is_error.value:
    return  # 接続断中のデータ欠落は上位エラーで文脈がわかる
```

**ログ抑制（過剰出力防止）**:
- 1フレームの欠落はシステム性能に影響しない（上記合意）ため、毎フレームでログを出すのは目的に反する。
- `_last_data_missing_log_mono` を内部保持し、前回ログから `log_suppress_interval_sec` (例: 10秒) 以内は出力しない。
- 目的は「頻度の高い定常ノイズのフィルタ」ではなく「間欠的な欠落パターンの蓄積記録」。
- 必要であれば間引き中の発生回数をカウントし、次回ログ出力時にサマリとして付加することも検討できる。

**実装イメージ（PointsProviderProcess._update() 末尾）**:
```python
# Dレベル: Lidarデータ欠落チェック
if pcd is not None and 0 < len(pcd) < self._min_lidar_point_count:
    se001 = self._ser.state_errors_A_C[
        StateErrorIndex.LIDAR0_CONNECTION_ERROR + self._index
    ]
    if not se001.is_error.value:
        now = time.monotonic()
        if now - self._last_data_missing_log_mono > self._data_missing_log_suppress_sec:
            self._logger.info(
                "Lidar%d data missing: point_count=%d (threshold=%d)",
                self._index, len(pcd), self._min_lidar_point_count,
            )
            self._last_data_missing_log_mono = now
```

---

### 設定パラメータ

| パラメータ | 実装場所 | 説明 | 候補デフォルト値 |
|---|---|---|---|
| min_lidar_point_count | `PointsProviderProcess` (config 経由) | これ未満の点数でデータ欠落と判定 | 100 |
| data_missing_log_suppress_sec | 同上 | 連続欠落時のログ間引き間隔 [s] | 10.0 |

---

### 将来課題: 統合後データ欠落 (E-013-B)
- 複数 Lidar を統合した後の下流プロセス（`get_data_process.py` 等）で空/極少データが来た場合。
- この場合は「どの Lidar が原因か」が不明なため、個別の SE001/SE008 とは切り離して記録する。
- 実装時は別の D レベルエントリまたは別ロガー経路として追加する。

### 受け入れ基準
1. 蓄積フレームの点数が `0 < len(pcd) < min_lidar_point_count` の場合に info ログが出力される。
2. SE001 が ON の場合は出力しない。
3. 連続欠落時は `data_missing_log_suppress_sec` 秒間隔に間引かれ、ログが氾濫しない。
4. 正常フレームで誤検知しない。

---

## SE001〜SE020 エラー間の排他/共存ルール (2026-07-30)

| エラー | SE001存在時 | SE008存在時 | 備考 |
|---|---|---|---|
| SE001 (接続エラーA) | ー | 排他推奨 | 接続断中は SE008 を発火させない |
| SE008 (品質低下C) | 発火しない | ー | SE001解消後に有効化 |
| SE014 (品質エラーB) | 発火しない | SE008 を入力とする | SE008 が解消すれば SE014 も復帰への評価を始める |
| SE020 (データ不正B) | 発火しない | 共存可 | 品質低下 + データ不正は同時起こりうる |
| D (データ欠落) | ログ抑制 | 共存可 | SE001中はログを出力しない |

### 排他ルールの実装アプローチ
- 現在の設計では、各診断クラスが独立して `errors_diagnosis()` を呼ばれる。
- 排他制御は「呼び出し側 (AppManagerProcess)」で実施するのが自然:
  ```python
  if not se001_error.is_error.value:
      se008_diag.errors_diagnosis(...)
  ```
- または診断クラス内で `is_enabled` を動的に切り替える構成（依存が生まれるため要注意）。
- **推奨**: AppManagerProcess の診断呼び出しループで上位エラー状態を参照してスキップする方式。

---

## 論点トラッカー（Lidar エラー群追加分）

以下を既存論点トラッカーに追加する:

| ID | 対象 | 論点 | 現状 | 方針候補 | 決定 | 備考 |
|---|---|---|---|---|---|---|
| E-009 | app_manager_process.py + state_errors.py + shared_excepts.py | SE001/SE002 Lidar接続エラー 本実装 | stub (常にFalse) | modify | in-design | 引数順序は Camera0 方式に統一決定 (2026-07-31)。呼び出し側修正が先。 |
| E-010 | mid360_points.py + shared_excepts.py + state_errors.py | SE008/SE009 Lidar通信品質低下 設計・実装 | 未実装 | modify | in-design | udp_cnt欠番+dot_num低下が主検出。ethtool観測はPhase2。 |
| E-011 | state_errors.py + app_manager_process.py | SE014/SE015 Lidar通信品質エラー 設計・実装 | 未実装 | modify | in-design | SE008のslidingwindow格上げ方式。復帰条件の運用要件を確認。 |
| E-012 | mid360_points.py + state_errors.py | SE020/SE021 Lidarデータ不正 設計・実装 | 未実装 | modify | in-design | 原点点割合 + Tag低信頼度点割合。Tag取得の追加実装が必要。 |
| E-013 | state_d_errors.py + process | Lidarデータ欠落 (D) 設計・実装 | 未実装 | modify | in-design | dot_num==0 + 統合後点数閾値。SE001中はログ抑制。 |
| E-014 | imu_process.py + shared_excepts.py + state_errors.py + app_manager_process.py | SE040/SE041 IMU接続エラー 設計・実装 | stub (常にFalse) + heartbeat未実装 | modify | in-design | 重要度B。heartbeat方式はLidar/Camera0と同一。SharedIMUExceptへのlast_heartbeat追加が必要。 |
| E-015 | diagnosis/action_errors.py + diagnosis/error_config.py + process/points_refine_process.py + process/visual_process.py | CE004(CRANE_MODEL_FILE_MISSING) 機体モデルファイル欠損/破損 設計・実装 | stub (excepts_diagnosis未実装, detect_error 常にFalse) + トリガーなし | modify | in-design | 重要度A。起動時に col_machine_info.jsonc/.csv ロード失敗で検知。 |
| E-016 | diagnosis/action_errors.py + diagnosis/error_config.py + process/error_monitor_process.py | CE011(MMAP_READ_WRITE_ERROR) MMAPファイル読み書きエラー 設計・実装 | stub (excepts_diagnosis未実装, detect_error 常にFalse) + トリガーなし | modify | in-design | 重要度B。ErrorMMapWriter write系の例外をPhase1対象とする。classMMap/status_mmapはPhase2。 |
| E-017 | common/app_logger.py + diagnosis/action_errors.py + __main__.py | CE015(LOG_FILE_IO_ERROR) ログファイルI/Oエラー 設計・実装 | stub (excepts_diagnosis未実装) + トリガーなし | modify | in-design | 重要度B。Python logging.handleError オーバーライドで検知。C++ logfunc経由のため直接ファイル書き込みなし。 |

---

## E-014: SE040/SE041 IMU接続エラー 設計 (2026-08-02)

### 概念
- MID-360 内蔵 IMU からのデータ取得プロセスが停止していないかを監視する。
- 重要度 B: IMU が接続断でも Lidar の計測・判定は継続できる。ただし異常をユーザーに通知する。

### 重要度 B の根拠
- IMU データは補助的な用途（姿勢推定・振動情報等）であり、接続断で周辺監視の安全性が直ちに失われるわけではない。
- Lidar 接続断 (SE001, 重要度 A) との違いは「Lidar がなければ周辺監視自体が成立しない」のに対し、「IMU がなくても Lidar 計測は継続できる」点。
- プロセスが静かに停止することをユーザー・オペレーターが気づけるよう、B 級の黄枠警告として表示する。

### 現状確認 (2026-08-02)

| 項目 | 現状 |
|---|---|
| `Imu0ConnectionErrorDiagnosis` | `StateErrorDiagnosisB` のスタブ (detect_error 常に False) |
| `Imu1ConnectionErrorDiagnosis` | 同上 |
| `ImuNConnectionErrorParameters` | 空 (フィールドなし) |
| `SharedIMUExcept.last_heartbeat` | **存在しない** (SharedLIDExcept には存在する) |
| `ImuProviderProcess._update()` | heartbeat 更新コードなし |
| `AppManagerProcess` の IMU 監視 | **コードなし** |

→ `SharedLIDExcept` + `PointsProviderProcess` + `Lidar0ConnectionErrorDiagnosis` の構成を IMU 向けに複製する形が最小差分。

### SE001 (Lidar A) との差異

| 項目 | SE001 Lidar接続エラー (A) | SE040 IMU接続エラー (B) |
|---|---|---|
| 重要度 | A | B |
| 診断基底クラス | StateErrorDiagnosisA | StateErrorDiagnosisB |
| 検知後のシステム動作 | アイドル移行・関連プロセス停止 | 継続動作 (failsafe 移行あり) |
| UI | 赤枠・前面固定 | 黄枠・×で閉じ可 |
| heartbeat 更新元 | PointsProviderProcess | ImuProviderProcess |
| 共有メモリ | SharedLIDExcept.last_heartbeat | SharedIMUExcept.last_heartbeat (要追加) |

### 引数契約 (Camera0 方式に統一)
- `args[0]`: `now_mono (float)` — 現在の単調時刻
- `args[1]`: `last_heartbeat (float)` — IMU プロセスの最終正常受信時刻
- Camera0ConnectionErrorDiagnosis および SE001 の方針と同一。

### 実装差分一覧

#### SharedIMUExcept への追加
```python
class SharedIMUExcept(SharedProcessExcept):
    def __init__(self) -> None:
        super().__init__()
        # 追加
        self.last_heartbeat: Synchronized[float] = create_shared_single_data(0.0)
```

#### ImuProviderProcess への追加
`__slots__` に `"_heartbeat_interval"`, `"_last_heartbeat"` を追加し、
`_update()` 内で **実データが届いた場合のみ** heartbeat を更新する:
```python
@log_target("IMU入力I/F", ProfCategory.Process)
def _update(self) -> ImuData | None:
    imu_ring, t = self._provider.get_accum_point()

    if imu_ring:
        cube = np.stack(imu_ring, axis=0)
        flat = cube.reshape(cube.shape[0], -1)
        # ★ 実データ取得成功時のみ heartbeat を更新
        now: float = time.monotonic()
        if now - self._last_heartbeat > self._heartbeat_interval:
            self._sec_imu.last_heartbeat.value = now
            self._last_heartbeat = now
        return ImuData(0, t, flat)

    # imu_ring が空 = データなし → heartbeat を更新しない（接続断として扱う）
    return ImuData(0, t, np.zeros((2, 2), dtype=np.float64))
```
- `imu_ring` が空（zeros 返却）の場合は heartbeat を更新しない。
- TimeoutError 等の例外が発生した場合も同様に更新しない（例外は上位で処理される）。
- `_heartbeat_interval` のデフォルト: `0.5秒` (PointsProviderProcess と同じ)。

#### ImuNConnectionErrorParameters への追加
```python
@dataclass(frozen=False, slots=True)
class ImuNConnectionErrorParameters(ErrorParameterBase):
    """IMUN接続エラー用パラメータ"""
    error_threshold_sec: float = 5.0
    error_recovery_confirm_duration_sec: float = 5.0
    failsafe_recovery_confirm_duration_sec: float = 5.0
    recovery_receive_interval_sec: float = 1.0
```
- `CameraNConnectionErrorParameters` と同一構成を採用。

#### Imu0ConnectionErrorDiagnosis の本実装
- `Camera0ConnectionErrorDiagnosis` と同一の判定ロジックを流用。
- 入力: `(now, last_heartbeat)` — Camera0 方式。
- `update()` で `err_conf.imu0_connection_error` を参照。

#### AppManagerProcess への追加
```python
# 既存の Lidar heartbeat 監視ループの後に追加
if not self._app_config.DEFAULT.File_Input:
    for i in range(self._num_imus):
        imu_index = StateErrorIndex.IMU0_CONNECTION_ERROR + i
        diag = self._ser.state_errors_A_C[imu_index]
        diag.errors_diagnosis(
            now_mono,
            self._sec.IMU_ex[i].last_heartbeat.value,
        )
```

### 設定パラメータ確定版

| パラメータ | 型 | 説明 | デフォルト値 |
|---|---|---|---|
| is_enabled | bool | 診断有効フラグ | True |
| error_threshold_sec | float | heartbeat 未更新でエラー判定する閾値 [s] | 5.0 |
| error_recovery_confirm_duration_sec | float | エラー復帰確認継続時間 [s] | 5.0 |
| failsafe_recovery_confirm_duration_sec | float | failsafe 復帰確認継続時間 [s] | 5.0 |
| recovery_receive_interval_sec | float | 復帰判定中の許容受信間隔 [s] | 1.0 |

### 受け入れ基準
1. IMU データ取得が `error_threshold_sec` 秒停止した場合に DETECTION。
2. データ取得再開が `error_recovery_confirm_duration_sec` 秒継続した場合に RECOVERY。
3. ブートストラップ期間（起動直後の heartbeat 未設定）で誤検知しない。
4. SE001 (Lidar 接続断) の有無に関係なく SE040 は独立して動作する。
5. File_Input モード中は評価をスキップする（Lidar と同様）。

---



本セクションは、エラー検知後のシステム全体の挙動を重要度ごとに定義する。
診断レイヤーで `DETECTION` / `RECOVERY` が確定した後、ErrorMonitor → main制御ループ → UIへの伝搬で使用する。

### 重要度 A — 安全担保不可レベル

#### システム制御
- アイドルモードへ移行する。周辺監視に直接関与しないプロセスは停止する。
- システムとしての計測・判定動作は停止する（利用できない状態）。
- 再起動ボタン操作によりシステム再起動が可能。
- **状態エラーの場合**: 再起動前であっても、エラー復帰条件を満たした場合はエラーフラグが解除される（アイドルモードからの自動復帰は要件確認）。

#### UI表示
- 赤枠の大きなエラーメッセージボックスを全画面前面に表示。
- エラー内容と営業所への連絡画面へのリンクを表示。
- 再起動ボタンを表示。押下でシステム再起動。
- エラー復帰条件を満たした場合はメッセージボックスが消える（ボタン押下なしで解除可能）。

#### ログ
- `error` レベルでエラー発生・復帰を記録する。

---

### 重要度 B — 継続利用可能・要認識レベル

#### システム制御
- 計測・判定の継続動作は維持する（直ちに利用停止を強制しない）。
- フェールセーフ動作が定義されている場合は、failsafe 状態へ移行する（計測精度の低下等を上位に通知）。

#### UI表示
- 中程度サイズの黄枠エラーメッセージボックスを表示。
- 右上の「×」ボタンでメッセージを閉じて利用継続できる。
- 閉じた後も、エラー復帰まで左側に「！」アイコンが残る。
- 「！」アイコンをクリックすると詳細を再表示できる。
- エラー復帰条件を満たした場合、「！」アイコンが消える。

#### ログ
- `warning` レベルでエラー発生を記録。復帰は `info` レベル。

---

### 重要度 C — 継続利用可能・非認識許容レベル

#### システム制御
- 計測・判定の継続動作を維持する。
- システム制御への介入なし。

#### UI表示
- 重要度Bのメッセージボックスを閉じた後と同じ状態（メッセージボックスの初期表示なし）。
- 左側に「！」アイコンが表示される。
  - ユーザーが気づかない場合もある（許容）。
  - クリックするとエラー内容を確認できる。
- エラー復帰条件を満たした場合、「！」アイコンが自動的に消える。

#### ログ
- `info` レベルでエラー発生・復帰を記録。

---

### 重要度 D — ログのみ

#### システム制御
- システム制御への介入なし。

#### UI表示
- なし（ユーザーには一切通知しない）。

#### ログ
- `info` または `debug` レベルでログファイルに記録する。
- 障害発生時・寿命到来時にサービスマンが回収・分析する用途を想定。

---

### 重要度別挙動一覧表

| 重要度 | システム制御 | UIメッセージボックス | 「！」アイコン | 自動復帰表示 | ログレベル |
|---|---|---|---|---|---|
| A | アイドル移行・関連プロセス停止 | 赤枠・大・前面固定 | ー (メッセージボックス表示中) | あり (条件充足でボックス消) | error |
| B | 継続動作 (failsafe移行あり) | 黄枠・中・×で閉じ可 | あり (閉じた後) | あり (条件充足でアイコン消) | warning / info |
| C | 継続動作 | なし | あり (初期から) | あり (条件充足でアイコン消) | info |
| D | 継続動作 | なし | なし | ー | info / debug |

---

### 設計上の注意点 (Lidarエラー群との対応)

1. **SE001 (重要度A) のアイドルモード移行**:
   - Lidar接続断は「周辺監視の安全が担保できない」レベルのため、アイドル移行が必須。
   - アイドルモードへの移行は `reduced_load_mode` または `fail_safe` フラグの ON で制御することを想定。
   - 既存の `StateErrorDiagnosisA` の `is_fail_safe.value` を使う経路が対応する。

2. **SE008 (重要度C) の「！アイコン点滅」**:
   - SE008 は発生・消滅が繰り返す可能性がある（短時間のパケット欠損）。
   - **対応方針 (2026-07-31 確定)**: 診断クラス側の `error_confirm_duration_sec` で吸収する。
     - DETECTION 側: エラー条件が `error_confirm_duration_sec` 秒継続してから DETECTION。
     - RECOVERY 側: 正常条件が `recovery_confirm_duration_sec` 秒継続してから RECOVERY。
   - これによりUIはフラグ変化をそのまま反映するだけでよく、点滅抑制の責任はバックエンドが持つ。

3. **SE014/SE020 (重要度B) の「×で閉じた後の状態管理」**:
   - ユーザーが「×」で閉じた時点のエラー状態と、その後の診断結果が独立して管理されていることが必要。
   - UI側が `is_error` フラグを購読し続け、エラー復帰時に「！アイコン消去」を自動実行する。
   - バックエンド側は UI の表示状態に依存せず、独立して診断継続する設計であること。

4. **複数エラー同時発生時のUI優先順位・複数重要度Bの取り扱い**:
   - UIアプリ開発チームと合意済み (2026-07-31)。バックエンド設計の範囲外。
   - バックエンドは各エラーの `is_error` / `is_fail_safe` フラグを独立して管理し、UI側が読み取る構成を維持する。

---

## E-015: CE004 (CRANE_MODEL_FILE_MISSING) 機体モデルファイル欠損/破損 設計 (2026-08-03)

### 概念

衝突判定・機体除去に使用する機体モデルファイル (JSON設定ファイル / CSV点群ファイル) が欠損・破損・読み込み不可の状態を検知する。
これらのファイルはプロセス起動時に必ず読み込まれ、欠損があると `PointsRefineProcess` / `VisualProcess` の起動が失敗し周辺監視機能が成立しない。

### 重要度: A

- 機体モデルなし = 衝突判定・機体除去が機能しない = 周辺監視の安全担保不可。
- `ActionErrorDiagnosisA` として既に定義済み。

### 対象ファイル

| ファイル | 説明 | 読み込み箇所 |
|---|---|---|
| `col_machine_info.jsonc` | 機体衝突判定設定 JSONC | `SubScrutinizer.create_machine_points()` → `load_machine_info()` → `com.read_jsonc()` |
| `*.csv` 形式の機体点群ファイル (例: `interpolated_SCX900_01_upper_part_lightning.csv`) | 機体形状点群データ | `create_machine_collision_list()` (C++ 側) |

パスは `app_config.OctoTree.col_machine_dir` + `app_config.OctoTree.json_col_machine_file` で決まる。

### CE004 と CE005 の切り分け

| エラー | 対象ファイル種別 | 主な例外 |
|---|---|---|
| CE005 (CONFIG_FILE_MISSING) | INI/設定ファイル (settings.ini, calib_settings.ini 等) | configparser 系, FileNotFoundError |
| CE004 (CRANE_MODEL_FILE_MISSING) | 機体モデルファイル (col_machine_info.jsonc, *.csv) | json.JSONDecodeError, FileNotFoundError, ValueError, RuntimeError |

両エラーは対象ファイルスコープが異なり、干渉しない。

### 現状確認

| 項目 | 現状 |
|---|---|
| `CraneModelFileMissingDiagnosis` (action_errors.py) | `ActionErrorDiagnosisA` のスタブ。`detect_error` 常に False。`excepts_diagnosis` 未実装 (NotImplementedError) |
| `CraneModelFileMissingParameters` (error_config.py) | 空 dataclass (is_enabled のみ) |
| トリガー呼び出し | `points_refine_process.py` / `visual_process.py` ともに CE004 への接続なし |

### トリガー箇所 (実装差分)

CE004 は「起動時に機体モデルファイルを読み込む」タイミングに限定して発火させる。

| 場所 | メソッド | 操作 |
|---|---|---|
| `process/points_refine_process.py` | `_startup_remove()` | `create_machine_points()` を try/except で囲み、例外時に `excepts_diagnosis(e)` |
| `process/points_refine_process.py` | `_startup_collision_cliff()` | 同上 |
| `process/visual_process.py` | `_startup()` | 同上 |

#### 実装イメージ (points_refine_process.py `_startup_remove`)

```python
def _startup_remove(self) -> None:
    ce004_diag = self._ser.action_errors_A_C[ActionErrorIndex.CRANE_MODEL_FILE_MISSING]
    ce004_diag.update(self._ser.shared_err_conf.error_config)
    try:
        (
            self._l_machine_col,
            machine_mobile_points_measure,
            machine_immobile_points_measure,
        ) = SubScrutinizer.create_machine_points(
            self._app_config.OctoTree.col_machine_dir,
            self._app_config.LiDARPosition,
            self._app_config.OctoTree.json_col_machine_file,
        )
    except Exception as e:
        is_target = ce004_diag.excepts_diagnosis(e)
        self._logger.error(
            "CE004(CRANE_MODEL_FILE_MISSING): machine model file load failed: "
            f"is_target_exception={is_target} exception={type(e).__name__}: {e}"
        )
        raise   # _startup 失敗としてプロセスを起動不可にする
    # ... (以降は既存の処理)
```

同じパターンを `_startup_collision_cliff()` と `visual_process._startup()` にも適用する。

### excepts_diagnosis の例外スコープ

CE004 としてカウント対象にする例外:

| 例外クラス | 根本原因 |
|---|---|
| `FileNotFoundError` | JSON/CSV ファイルが存在しない |
| `PermissionError` | ファイル読み取り権限なし |
| `IsADirectoryError` | パス先がディレクトリだった |
| `NotADirectoryError` | パスの一部がディレクトリでなかった |
| `OSError` | デバイス・I/O エラー、パス長超過など |
| `UnicodeDecodeError` | ファイルのエンコーディング不正 |
| `json.JSONDecodeError` | JSON/JSONC 構文破損 |
| `ValueError` | JSON データに必須フィールド欠落、型不正 (MachineConf 構築失敗) |
| `KeyError` | JSON オブジェクトに必須キーが存在しない |
| `RuntimeError` | C++ 側 (`create_machine_collision_list`) の CSV 読み込み失敗 |

上記以外の例外は CE004 非該当として False を返す。

#### カウンタ方針

- CE004 該当時のみ `increment_counter()` を呼ぶ。
- 非該当時はカウンタを変更しない。
- プロセス起動時の1回性なので、CE005 のようなカウント抑制 (連続カウント throttle) は不要。

### CraneModelFileMissingParameters の追加パラメータ

現状は空 (is_enabled のみ)。CE005 に準じてシンプルに保つ。追加パラメータは不要。

```python
@dataclass(frozen=False, slots=True)
class CraneModelFileMissingParameters(ErrorParameterBase):
    """機体モデルファイル欠損/破損用パラメータ"""
    # is_enabled のみ (ErrorParameterBase から継承)
```

### detect_error の方針

CE004 は「起動時の例外ベース」でのみ検知する。`detect_error()` は基底スタブのまま (False) で維持する。
- `excepts_diagnosis(e)` が True → 基底の `ActionErrorDiagnosisA` 内部で `is_error.value = True` に遷移。
- 復帰: 次回のプロセス起動 (再起動) が成功すれば `is_error` が False に戻る想定。
  - `detect_recovery_error()` / `detect_recovery_fail_safe()` は基底の True 固定のまま維持。

### 起動失敗時のプロセス挙動

`create_machine_points()` が失敗した後は例外を `raise` し直すことで `_startup()` が例外終了する。
これによりプロセスが起動しないまま `ProcessBase` の異常終了フローに乗り、上位の ErrorMonitor が CE004 の `is_error` を検知してA級エラーとして処理する。

### プロセス再起動と CE004 の再評価

- A 級エラー復帰はプロセス再起動が前提となる。
- 再起動後の `_startup()` で再度 `create_machine_points()` が実行される。
  - 成功 → `excepts_diagnosis` が呼ばれない → `is_error` が False のまま → CE004 解消。
  - 失敗 → 再度 `excepts_diagnosis(e)` が呼ばれカウンタ増加。

### 受け入れ基準

1. `col_machine_info.jsonc` が存在しない場合に CE004 が True になり、カウンタが増加する。
2. `col_machine_info.jsonc` の JSON 構文が破損している場合に CE004 が True になる。
3. `col_machine_info.jsonc` に必須フィールドが欠落している (MachineConf 構築失敗) 場合に CE004 が True になる。
4. `.csv` ファイルが C++ 側で読み込めない (RuntimeError) 場合に CE004 が True になる。
5. CE004 非該当例外では False を返し、カウンタ不変となる。
6. CE005 の対象例外と重複しない (INI ファイル関連の configparser 例外は CE005 のみに判定させる)。
7. CE004 検知後のプロセス起動失敗が上位 A 級エラー処理フローに到達する。
8. 機体モデルファイルが正常な場合は CE004 が発火しない。

---

## E-016: CE011 (MMAP_READ_WRITE_ERROR) MMAPファイル読み書きエラー 設計 (2026-08-03)

### 概念

ファイルベースの MMAP (`err0.dat`, `err1.dat` 等) への読み書き操作が失敗した状態を検知する。
MMAP は Godot UI へのエラー状態・動作状態の伝達チャネルとして機能しており、
書き込み失敗が継続すると UI が最新のエラー状態を認識できなくなる。

### 重要度: B

- MMAP 書き込み失敗があっても、Python 内部のプロセス間通信 (`multiprocessing.sharedctypes`) は正常に機能し続ける。
- センサー計測・衝突判定・システム制御の安全性は直接影響を受けない。
- ただし、Godot UI へのエラー通知チャネルが失われるため、ユーザーへの警告表示が機能しなくなる可能性がある。
- `ActionErrorDiagnosisB` として既に定義済み。

### MMAP の2種類と CE011 の対象範囲

| 種別 | 実体 | 用途 | 書き込みプロセス | CE011 Phase |
|---|---|---|---|---|
| **ファイルベース MMAP** | `err0.dat`, `err1.dat` | ErrorMonitorProcess → Godot UI へのエラー状態伝達 | ErrorMonitorProcess | **Phase1** (主対象) |
| ファイルベース MMAP | `status.mmap` | システム状態コード (RUNNING/ERROR 等) | `__main__.py` (StatusMMAP) | Phase2 |
| ファイルベース MMAP | `*.mmap` (UI_mmap 等) | キャリブレーション UI 用データ | CalibMMapMaintainer (classMMap 経由) | Phase2 |
| Python プロセス間共有メモリ | `multiprocessing.sharedctypes` | SharedErrors / SharedExcepts | 各プロセス | 対象外 (ファイルシステム依存なし) |

Phase1 では `err0.dat`/`err1.dat` への書き込み (`ErrorMMapWriter.write_state_error`, `write_action_error`) のみを対象とする。

### 現状確認

| 項目 | 現状 |
|---|---|
| `MmapReadWriteErrorDiagnosis` (action_errors.py) | `ActionErrorDiagnosisB` のスタブ。`detect_error` 常に False。`excepts_diagnosis` 未実装 (NotImplementedError) |
| `MmapReadWriteErrorParameters` (error_config.py) | 空 dataclass (is_enabled のみ) |
| トリガー呼び出し | `error_monitor_process.py` に CE011 への接続なし |

### トリガー箇所 (Phase1 実装差分)

`error_monitor_process._update()` 内で `ErrorMMapWriter` 系の呼び出しを try/except で囲む。

```
error_monitor_process._update()
  ├─ self._mmap.start_write()              ┐
  ├─ self._mmap.write_state_error(...)     │ try/except で囲む
  ├─ self._mmap.write_action_error(...)    │ → 例外時に CE011.excepts_diagnosis(e)
  └─ self._mmap.rotate_if_busy()          ┘
```

CE004 と異なり、起動時の1回性ではなく **継続的な監視** である。
- 書き込み失敗 → `excepts_diagnosis(e)` でカウンタ増加 + ログ出力
- `_update()` ループは継続 (raise しない)。MMAP が壊れても ErrorMonitor 自体を止めると全エラー通知が失われる。

#### 実装イメージ (error_monitor_process._update)

```python
def _update(self) -> None:
    now_mono = time.monotonic()
    # ... (既存の APPLICATION_MANAGER_NOT_RESPONDING 診断)

    ce011_diag = self.ser.action_errors_A_C[ActionErrorIndex.MMAP_READ_WRITE_ERROR]
    try:
        self._mmap.start_write()
        state_err: bytes = self._make_state_error_bits(self.ser.state_errors)
        self._debug_log_state_errors(state_err)
        self._mmap.write_state_error(state_err)

        action_err: bytes = self._make_action_error_bits(self.ser.action_errors)
        self._debug_log_action_errors(action_err)
        self._mmap.write_action_error(action_err)
        self._mmap.rotate_if_busy()
    except Exception as e:
        is_target = ce011_diag.excepts_diagnosis(e)
        self._logger.error(
            "CE011(MMAP_READ_WRITE_ERROR): mmap write failed: "
            f"is_target_exception={is_target} exception={type(e).__name__}: {e}"
        )
```

### excepts_diagnosis の例外スコープ

CE011 としてカウント対象にする例外:

| 例外クラス | 根本原因 |
|---|---|
| `OSError` (含む `mmap.error`) | ファイルシステム障害、ディスク容量不足、デバイス I/O エラー、`mmap.error` は `OSError` のサブクラス |
| `ValueError` | mmap クローズ/無効状態でのアクセス、バッファ範囲外 |
| `BufferError` | バッファ競合 |
| `RuntimeError` | C++ 側 (`ErrorMMapWriter`) の内部エラー |

上記以外の例外は CE011 非該当として False を返す。

#### カウンタ方針

- CE011 該当時のみ `increment_counter()` を呼ぶ。
- 非該当時はカウンタを変更しない。
- 同一サイクル内での連続カウント抑制 (throttle) は不要 (0.2秒周期の `_update()` 単位で1回)。

### detect_error の方針

他の CE エラーと同様、`excepts_diagnosis(e)` のみで検知。
- `detect_error()` は基底スタブのまま (False)。
- `detect_recovery_error()` / `detect_recovery_fail_safe()` は基底スタブのまま (True)。

CE011 は「失敗を検知したらカウントする」ことが目的であり、「閾値超過でエラー/復帰を切り替える」状態管理は不要。MMAP 書き込みが失敗した事実をカウンタとして蓄積し、ログに残すことが運用上の主な価値。

### MmapReadWriteErrorParameters の追加パラメータ

現状は空 (is_enabled のみ)。追加パラメータは不要。

### Phase2 設計メモ (将来課題)

#### status.mmap (StatusMMAP)
- `__main__.py` の `status_mmap.write_status()` / `read_status()` に try/except を追加。
- `__main__.py` が `SharedErrors` を持つため、`ser.action_errors_A_C[ActionErrorIndex.MMAP_READ_WRITE_ERROR].excepts_diagnosis(e)` を呼べる。
- 対象例外: `OSError`, `ValueError`, `struct.error` (struct.pack/unpack 失敗)

#### classMMap (calib UI)
- `classMMap` は現在例外を内部で `logger.info()` に吸収している。
  - 対応案: コールバック引数 `on_error: Callable[[Exception], None] | None = None` を追加し、
    呼び出し側 (CalibMMapMaintainer / CalibProcess) が CE011 トリガーを渡せるようにする。
- `CalibProcess` / `CalibMMapMaintainer` が `SharedErrors` を持っていないため、
  SharedErrors アクセスを追加するか、別経路で伝達する必要がある。

### 受け入れ基準

1. `ErrorMMapWriter.write_state_error()` / `write_action_error()` が `OSError` を送出した場合に CE011 カウンタが増加する。
2. `ErrorMMapWriter` が `RuntimeError` を送出した場合に CE011 カウンタが増加する。
3. CE011 非該当例外 (例: `KeyboardInterrupt`) では False を返し、カウンタ不変となる。
4. MMAP 書き込み失敗後も `error_monitor_process._update()` ループは継続し、次サイクルで再試行される。
5. MMAP 書き込みが正常な場合は CE011 が発火しない。

---

## E-017: CE015 (LOG_FILE_IO_ERROR) ログファイルI/Oエラー 設計 (2026-08-03)

### 概念

ログファイル (`*.log`, `*.gz`) への書き込みが失敗した状態を検知する。
ディスク容量不足・権限エラー等によりログが記録できなくなると、障害発生時の事後分析が不可能になる。

### 重要度: B

- ログ書き込み失敗があっても、センサー計測・衝突判定・UI通知の安全機能は維持される。
- ただし、障害発生時の原因分析手段が失われるため、ユーザー・オペレーターに通知する価値がある。
- `ActionErrorDiagnosisB` として既に定義済み。

### C++ 側のログ書き込み調査結果

| 書き込み箇所 | C++ 直接ファイルI/O | CE015 関係 |
|---|---|---|
| `logger_.info/error(...)` (C++ 全般) | **なし** (全て `logfunc` コールバック → Python `AppLogger`) | Python 側と同一経路 |
| `damp_fp_list` (ui_interface.cpp) | あり (`std::ofstream`) | damp_out=True 時のみ。通常運用では False → **対象外** |

→ **CE015 は実質 Python 側のみの問題。** C++ 自体はログファイルに直接書かない。

### Python logging の例外飲み込み問題

Python `logging.RotatingFileHandler.emit()` がファイルI/Oエラーで失敗した場合:
- `logging.Handler.handleError(record)` が呼ばれる
- デフォルト動作: `sys.stderr` に "--- Logging error ---" を出力して**継続**
- 例外は raise されない → 通常の try/except では捕捉不可

→ CE015 をトリガーするには `handleError()` をオーバーライドする必要がある。

### 設計方針: handleError オーバーライド

`GZipRotatingFileHandler` に `on_io_error` コールバックを追加し、`handleError()` をオーバーライドする。

```
[RotatingFileHandler.emit() 失敗]
  └─ GZipRotatingFileHandler.handleError(record)
       └─ self.on_io_error(e) が設定されていれば呼ぶ
            └─ CE015.excepts_diagnosis(e) → increment_counter()
```

**コールバックの注入経路:**

```
__main__.py (main())
  ├─ ser = SharedErrors()
  └─ app_logger_factory = AppLoggerFactory(...)
       └─ factory.set_io_error_callback(
              lambda e: ser.action_errors_A_C[
                  ActionErrorIndex.LOG_FILE_IO_ERROR
              ].excepts_diagnosis(e)
          )
```

`AppLoggerFactory` はコールバックを全ての `AppLogger` に伝播させ、各 `GZipRotatingFileHandler` の `on_io_error` にセットする。

### 実装差分一覧

| ファイル | 変更内容 |
|---|---|
| `common/app_logger.py` | `GZipRotatingFileHandler` に `handleError()` オーバーライドと `on_io_error` コールバックを追加 |
| `common/app_logger.py` | `AppLogger._create_file_handler()` にコールバック引数を追加 |
| `common/app_logger.py` | `AppLoggerFactory` に `set_io_error_callback()` を追加 |
| `diagnosis/action_errors.py` | `LogFileIoErrorDiagnosis.excepts_diagnosis()` を実装 |
| `__main__.py` | `app_logger_factory` 生成後に CE015 コールバックを注入 |

### excepts_diagnosis の例外スコープ

| 例外クラス | 根本原因 |
|---|---|
| `OSError` | ディスク容量不足・権限エラー・デバイスI/Oエラー |
| `PermissionError` | ファイル書き込み権限なし (OSError サブクラスだが明示) |
| `FileNotFoundError` | ログディレクトリ削除 (OSError サブクラスだが明示) |

上記以外は CE015 非該当として False を返す。

#### カウンタ方針
- CE015 該当時のみ `increment_counter()` を呼ぶ。
- handleError は emit() 失敗の都度呼ばれるため、CE005 と同様に throttle を設ける。
  - 同一例外クラス+メッセージの組が `throttle_sec` (1.0秒) 以内に再発した場合はカウントしない。

### detect_error の方針

他の CE エラーと同様、`excepts_diagnosis(e)` のみで検知。
- `detect_error()` は基底スタブのまま (False)。
- `detect_recovery_error()` / `detect_recovery_fail_safe()` は基底スタブのまま (True)。

### damp ファイルの扱い (Phase2 以降検討)

- `ui_interface.cpp` の `damp_fp_list` (`std::ofstream`) はデバッグ用ダンプファイル。
- `damp_out=True` 時のコンストラクタ失敗は `RuntimeError` として Python に伝搬する。
- これが「ログエラー」か「出力ファイルエラー」かは議論の余地がある。
- 現時点では CE015 の対象外とし、`GodotUIVisualizer` の CE011 ラップ (`OSError, RuntimeError`) で代替検知する。

### 受け入れ基準

1. `RotatingFileHandler.emit()` が `OSError` で失敗した場合に CE015 カウンタが増加する。
2. CE015 非該当例外では False を返し、カウンタ不変となる。
3. ログ書き込み失敗後もアプリケーションは継続動作する (handleError の既存挙動を維持)。
4. throttle により、連続失敗時の 1秒内重複カウントが抑制される。
5. `AppLoggerFactory` にコールバックが未設定の場合 (ユニットテスト等) は CE015 が発火しない (安全な無効化)。
