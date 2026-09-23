# calibcheck2d3d 旧新対比表とクラス図

## 1. 何が変わったか

結論として、**処理の大筋は同じ**ですが、**クラス構成と責務分割は大きく変わっています**。

### 対比表

| 観点 | 当初 | 現在 |
|---|---|---|
| 入口 | 巨大な `__init__.py` に処理が集中 | [__init__.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/__init__.py) は facade |
| 初期化 | 設定・依存生成・処理本体が混在 | facade が各 helper を組み立てる |
| 1フレーム処理 | `__init__.py` 内で直接実行 | [processor.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/processor.py) へ委譲 |
| LiDAR処理 | 点群前処理、bbox化、UI反映が同居 | processor が担当 |
| Camera処理 | 画像補正、YOLO、描画、UI反映が同居 | processor が担当 |
| 追跡 | 2D/3D の追跡・選別・妥当性判定が混在 | [tracker.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/tracker.py) に分離 |
| 評価 | 投影、重なり評価、仮想 bbox、判定が混在 | [evaluator.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/evaluator.py) に分離 |
| Scene判定 | `SceneDesc.py` と本体が密結合 | [scene_calibcheck2d3d.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/scene_calibcheck2d3d.py) へ切り出し |
| デバッグ補助 | 描画・点群・rtvec が散在 | [calibcheck_utils.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/calibcheck_utils.py) に集約 |
| 追跡記録 | 2D/3D recorder が別々に重複 | [tracker_recorder.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/tracker_recorder.py) で共通基底化 |
| UI反映 | 直接呼び出しが多い | processor が集約して扱う |
| エラー・診断 | 本体に直書き | `diagnosis` / `shared_errors` に寄せて保持 |

## 2. 現在の処理フロー

```mermaid
flowchart LR
    CAP[data_capture]
    FAC[calibcheck2d3d facade]
    PROC[CalibCheck2d3dProcessor]
    TRK[CalibCheck2d3dTracker]
    EVA[CalibCheck2d3dEvaluator]
    SCN[Scene_CalibCheck2d3d]
    UTL[calibcheck_utils]
    REC[tracker_recorder]
    UI[CalibrationUIGodot]

    CAP --> FAC
    FAC --> PROC
    FAC --> TRK
    FAC --> EVA
    FAC --> SCN
    FAC --> UTL
    TRK --> REC
    PROC --> UI
    EVA --> UI
```

## 3. クラス図

```mermaid
classDiagram
    class calibcheck2d3d {
        +pre_app_loopmain()
        +app_loopmain()
        +post_app_loopmain()
        +data_evaluation_process()
        +track_3dbbox()
        +track_2dbbox()
        +evaluate_2d3d()
    }

    class CalibCheck2d3dProcessor {
        +input_settings()
        +pre_app_loopmain()
        +proc_lidar1f()
        +proc_camera1f()
        +record_bbox1f()
        +validate_3dbbox_log()
        +validate_2dbbox_log()
    }

    class CalibCheck2d3dTracker {
        +track_3dbbox()
        +track_2dbbox()
        +select_3dbbox_tracking_results()
        +select_2dbbox_tracking_results()
        +validate_tracked_bboxes()
    }

    class CalibCheck2d3dEvaluator {
        +project_3dbbox_core()
        +evaluate_bbox_overlap_scenedesc()
        +evaluate_2d3d()
        +judge_calibration_result()
    }

    class Scene_CalibCheck2d3d {
        +integrate2d3d_calibcheck()
    }

    class YOLODamoBatchAdapter {
        +predict_batch()
    }

    class _CalibCheckBBoxTrackerRecorderBase {
        +reset()
        +print_trackinfo()
        +is_person_detected()
        +draw_mot()
    }

    class calibcheck2d_bboxtracker_recorder
    class calibcheck3d_bboxtracker_recorder

    calibcheck2d3d --> CalibCheck2d3dProcessor
    calibcheck2d3d --> CalibCheck2d3dTracker
    calibcheck2d3d --> CalibCheck2d3dEvaluator
    calibcheck2d3d --> Scene_CalibCheck2d3d
    calibcheck2d3d --> YOLODamoBatchAdapter

    CalibCheck2d3dTracker --> calibcheck2d_bboxtracker_recorder
    CalibCheck2d3dTracker --> calibcheck3d_bboxtracker_recorder
    calibcheck2d_bboxtracker_recorder --|> _CalibCheckBBoxTrackerRecorderBase
    calibcheck3d_bboxtracker_recorder --|> _CalibCheckBBoxTrackerRecorderBase
```

## 4. 見方

- 旧構成は「`__init__.py` に全部入っている」形
- 現構成は「入口 + helper 群」の形
- そのため、**処理順は似ていても、クラスの役割分担は別物** と見てよい

## 5. 旧新の並列表現

| 処理段階 | 当初 | 現在 |
|---|---|---|
| 起動 | `calibcheck2d3d.__init__` が初期化と処理を兼ねる | `calibcheck2d3d` facade が各 helper を組み立てる |
| 設定読込 | 本体内で直接実施 | [processor.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/processor.py) の `input_settings()` |
| 1フレーム入力診断 | 本体内で直接実施 | [processor.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/processor.py) |
| LiDAR処理 | 点群整形・静止点除去・bbox生成・UI反映を直書き | [processor.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/processor.py) |
| Camera処理 | 歪み補正・YOLO・描画・UI反映を直書き | [processor.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/processor.py) |
| bbox記録 | 本体内でフレーム蓄積 | [processor.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/processor.py) の `record_bbox1f()` |
| 2D/3D追跡 | 本体内で追跡・選別・妥当性判定 | [tracker.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/tracker.py) |
| 3D-2D評価 | 本体内で投影・仮想bbox・重なり評価・判定 | [evaluator.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/evaluator.py) |
| Scene判定 | 本体と密結合 | [scene_calibcheck2d3d.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/scene_calibcheck2d3d.py) |
| デバッグ補助 | 描画・点群・rtvec が散在 | [calibcheck_utils.py](D:/argus-3d-vision/app/core/argus_synchro/calibration_mat_generator_modules/ctrl/calibcheck2d3d/calibcheck_utils.py) |
| 終了時評価 | 本体内で追跡→評価→判定→UI反映 | `__init__.py` が各 helper を順に呼び出し、結果を UI へ反映 |

### 横並びの見え方

```mermaid
flowchart LR
    subgraph OLD["当初"]
        O1[巨大な __init__.py]
        O2[設定読込]
        O3[1フレーム処理]
        O4[追跡]
        O5[評価]
        O6[UI反映]
        O1 --> O2 --> O3 --> O4 --> O5 --> O6
    end

    subgraph NEW["現在"]
        N1[calibcheck2d3d facade]
        N2[processor.py]
        N3[tracker.py]
        N4[evaluator.py]
        N5[scene_calibcheck2d3d.py]
        N6[calibcheck_utils.py]
        N7[UI反映]
        N1 --> N2 --> N3 --> N4 --> N7
        N1 --> N5
        N1 --> N6
    end
```

必要なら次に、この対比表を [calibcheck2d3d_current_flow.md](D:/argus-3d-vision/app/core/docs/calibcheck2d3d_current_flow.md) へリンクして、文書群を一本化できます。

## 6. 最終構成（2026-09-18時点）

第一次リファクタリング完了後の実装（`lifecycle.py`/`reason_codes.py`/`result_artifacts.py`
を `reporting.py` へ統合、`types.py` を `processor.py` へ統合、`debuginfo_and_functions.py`
を `debug_artifacts.py`/`calibration_utils.py` へ分割した状態）を反映する。
`calibration_utils.py` はその後 `wait_app` とも共用する汎用モジュールであるため、
`calibcheck2d3d` 配下から `calibration_mat_generator_modules/utils/` へ移設した。

### 6.1 ファイル構成（11ファイル、`calibration_utils.py` は上位の `utils/` に移設済み）

| ファイル | 責務 |
|---|---|
| `__init__.py` | facade。`pre_app_loopmain`/`app_loopmain`/`post_app_loopmain` などUI・設定ファイルとの外部プロトコルを担う |
| `processor.py` | 1フレーム処理（LiDAR/カメラ）、セッション状態、`VirtualBBoxDebugCounts`等のデバッグ用型定義 |
| `tracker.py` | bbox記録、ログ検証、2D/3D追跡のオーケストレーション |
| `tracker_recorder.py` | `calibcheck2d_bboxtracker_recorder` / `calibcheck3d_bboxtracker_recorder`（SORTベースの実追跡） |
| `evaluator.py` | 3D→2D投影、重なり評価、判定、`EvaluationRuntime`、評価ループ中心 |
| `SceneDesc.py` | `Scene` 基底クラス、IOU計算、人サイズゲート |
| `scene_calibcheck2d3d.py` | `Scene_CalibCheck2d3d`（`Scene` を継承し、fisheyeカメラ内部パラメータを保持） |
| `debug_artifacts.py` | トレース、動画/pickle出力、bbox描画、`vis_viewer`（Open3D可視化） |
| `reporting.py` | reason変換、MMAPステータス公開、結果ファイル出力（旧 lifecycle/reason_codes/result_artifacts統合） |
| `YOLOadapter.py` | `YOLODamoBatchAdapter`。Detect2dDamoYoloOnnxのバッチ推論ラッパー |
| `calibcheck_detection_2d3d.py` | 旧アルゴリズム(`evaluate2d3d`等)。現行の `pre/app/post_app_loopmain` からは呼ばれず、未使用の `dataproc()` レガシーメソッドからのみ参照される |
| `calibration_mat_generator_modules/utils/calibration_utils.py`（上位モジュール） | LiDAR統合(`conbine3d3d`)、キャリブレーション行列読込(`read_rtvec`)。`calibcheck2d3d` と `wait_app` の双方が対等に参照する共通モジュール |

### 6.2 処理フロー

```mermaid
flowchart TD
    CP[calib_process] --> FAC[calibcheck2d3d facade]

    FAC --> PRE[pre_app_loopmain]
    PRE --> LOAD["processor.load_calibration_settings\n(utils.calibration_utils.read_rtvec)"]
    PRE --> STATE[processor.create_calibcheck_session_state]
    PRE --> MMAP_RUN[MMAP: RUNNING を transmit]

    FAC --> LOOP["app_loopmain\n(start2D3DCheckCalc=False の間繰り返し)"]
    LOOP --> PCF[processor.process_calibcheck_frame]
    PCF --> PLF[processor.process_lidar_frame]
    PCF --> PCAM[processor.process_camera_frame]
    PLF --> MKBB["debug_artifacts.internal_make_BB\n(点群→3D bbox)"]
    PCAM --> YOLO[YOLOadapter.predict_batch]
    PCAM --> DRAW[debug_artifacts.draw_multibbox]
    PCF --> REC[tracker.record_bbox1f]
    REC --> FRAMEINFO[(frame_info 蓄積)]
    LOOP --> MMAP_FRAME[MMAP: フレーム毎の画像/点群/検出結果を transmit]

    FAC --> POST[post_app_loopmain]
    POST --> VALID[tracker.validate_recorded_bbox_logs]
    VALID --> TRK3[tracker.track_3dbbox]
    VALID --> TRK2[tracker.track_2dbbox]
    TRK3 --> REC3[tracker_recorder.calibcheck3d_bboxtracker_recorder]
    TRK2 --> REC2[tracker_recorder.calibcheck2d_bboxtracker_recorder]
    TRK3 --> VALIDTRK[tracker.validate_tracked_bboxes]
    TRK2 --> VALIDTRK
    VALIDTRK --> EVAL[evaluator.evaluate_2d3d]
    EVAL --> PROJ[evaluator.project_3dbbox_core]
    EVAL --> OVERLAP["evaluator.evaluate_bbox_overlap_scenedesc\n(scene_calibcheck2d3d / SceneDesc)"]
    EVAL --> JUDGE[evaluator.judge_calibration_result]
    JUDGE --> REPORT["reporting.publish_camera_calibcheck_statuses\nreporting.write_calibcheck_result_files"]
    REPORT --> MMAP_RESULT[MMAP: CALCULATING → カメラ別診断結果を transmit]
    POST --> ENDWAIT["end_wait / send_end_wait\n(reporting.publish_calibcheck_lifecycle_status)"]
    ENDWAIT --> MMAP_END[MMAP: COMPLETED → INACTIVE]
```

### 6.3 相互作用図（UI / MMAP を含む）

```mermaid
sequenceDiagram
    participant UI as UI application
    participant INI as settings.ini / SharedAppConfig
    participant CP as calib_process
    participant FAC as calibcheck2d3d facade
    participant PROC as processor.py
    participant TRK as tracker.py / tracker_recorder.py
    participant EVA as evaluator.py
    participant REP as reporting.py
    participant MMAP as CalibrationUIGodot / MMAP

    UI->>INI: isRunning2D3Dcheck = True
    CP->>FAC: pre_app_loopmain()
    FAC->>PROC: create_calibcheck_session_state()
    FAC->>MMAP: RUNNING と初回データを transmit
    MMAP-->>UI: 診断準備完了

    loop start2D3DCheckCalc is False
        CP->>FAC: app_loopmain(fifo frame)
        FAC->>PROC: process_calibcheck_frame()
        PROC->>TRK: record_bbox1f()
        FAC->>MMAP: フレーム情報(画像・点群・検出結果・yaw)を transmit
        MMAP-->>UI: 画面更新
    end

    UI->>INI: start2D3DCheckCalc = True
    CP->>FAC: post_app_loopmain()
    FAC->>TRK: track_3dbbox() / track_2dbbox()
    FAC->>EVA: evaluate_2d3d()
    EVA-->>FAC: 評価統計・reason
    FAC->>REP: publish_camera_calibcheck_statuses() / write_calibcheck_result_files()
    FAC->>MMAP: CALCULATING → カメラ別診断結果を transmit
    CP->>FAC: end_wait() / send_end_wait()
    FAC->>REP: publish_calibcheck_lifecycle_status()
    FAC->>MMAP: COMPLETED, then INACTIVE
```

### 6.4 クラス図

```mermaid
classDiagram
    class calibcheck2d3d {
        +pre_app_loopmain()
        +app_loopmain()
        +post_app_loopmain()
        +data_evaluation_process()
        +evaluate_2d3d()
        +judge_calibration_result()
        +end_wait()
        +send_end_wait()
    }

    class ProcessorModule {
        <<processor.py>>
        +create_calibcheck_session_state()
        +load_calibration_settings()
        +process_lidar_frame()
        +process_camera_frame()
        +process_calibcheck_frame()
        +VirtualBBoxDebugCounts
        +EvaluationMetricDebug
        +TrackProximityWarning
        +CalibCheckSessionState
    }

    class TrackerModule {
        <<tracker.py>>
        +record_bbox1f()
        +validate_recorded_bbox_logs()
        +track_3dbbox()
        +track_2dbbox()
        +validate_tracked_bboxes()
        +detect_close_3dbbox_tracks()
    }

    class calibcheck2d_bboxtracker_recorder
    class calibcheck3d_bboxtracker_recorder

    class EvaluatorModule {
        <<evaluator.py>>
        +EvaluationRuntime
        +project_3dbbox_core()
        +evaluate_bbox_overlap_scenedesc()
        +evaluate_2d3d()
        +judge_calibration_result()
    }

    class Scene {
        <<SceneDesc.py>>
        +calc_iou()
        +get_human_3bb()
        +passes_human_size()
    }

    class Scene_CalibCheck2d3d {
        <<scene_calibcheck2d3d.py>>
        +create_for_evaluation()
        +integrate2d3d_calibcheck()
    }

    class DebugArtifactsModule {
        <<debug_artifacts.py>>
        +internal_make_BB()
        +draw_multibbox()
        +draw_evaluation_bboxes()
        +vis_viewer
    }

    class CalibrationUtilsModule {
        <<calibration_mat_generator_modules/utils/calibration_utils.py>>
        +conbine3d3d()
        +read_rtvec()
    }

    class ReportingModule {
        <<reporting.py>>
        +error_reason_to_string()
        +publish_camera_calibcheck_statuses()
        +publish_calibcheck_lifecycle_status()
        +write_calibcheck_result_files()
    }

    class YOLODamoBatchAdapter {
        <<YOLOadapter.py>>
        +predict_batch()
    }

    class LegacyDetection2d3d {
        <<calibcheck_detection_2d3d.py, 現行フロー未使用>>
        +evaluate2d3d()
        +get_human_3bb_withscore()
    }

    calibcheck2d3d --> ProcessorModule
    calibcheck2d3d --> TrackerModule
    calibcheck2d3d --> EvaluatorModule
    calibcheck2d3d --> Scene_CalibCheck2d3d
    calibcheck2d3d --> DebugArtifactsModule
    calibcheck2d3d --> CalibrationUtilsModule
    calibcheck2d3d --> ReportingModule
    calibcheck2d3d --> YOLODamoBatchAdapter
    calibcheck2d3d ..> LegacyDetection2d3d : dataproc() のみ(未使用経路)

    ProcessorModule --> YOLODamoBatchAdapter
    ProcessorModule --> DebugArtifactsModule
    TrackerModule --> calibcheck2d_bboxtracker_recorder
    TrackerModule --> calibcheck3d_bboxtracker_recorder
    EvaluatorModule --> Scene_CalibCheck2d3d
    Scene_CalibCheck2d3d --|> Scene
```

### 6.5 補足

- `calibration_utils.py` は `calibcheck2d3d` 専用ではなく `wait_app` とも共用するため、
  パッケージ階層を1段上げて `calibration_mat_generator_modules/utils/` に配置している。
  これにより `wait_app` が `calibcheck2d3d` の内部モジュールを直接importするという
  不自然な依存方向を解消した。
- `calibcheck_detection_2d3d.py` は `pre_app_loopmain`/`app_loopmain`/`post_app_loopmain`
  のいずれからも呼ばれない。参照は未使用の `dataproc()` メソッド内のみで、実運用の校正診断
  フローには関与しない（削除・整理の候補だが、現時点では安全のため残置）。
- MMAPへの書き込みは引き続き `CalibrationUIGodot.transmit_setdata()` に集約されており、
  各モジュールは値を facade に返すのみで MMAP を直接操作しない。
- リファクタリング前後で `pre_app_loopmain`/`app_loopmain`/`post_app_loopmain` の外部シグネチャ・
  呼び出し回数・MMAP契約は変更していない（`docs/calibcheck2d3d_refactoring.md` の実測結果も参照）。
