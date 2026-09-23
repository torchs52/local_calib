# Jetson 実行テストによる修正内容の分析

## 概要

Jetson で実行テストを行った結果、以下の修正が加わりました。修正は **遅延初期化パターン**、**状態共有の明示化**、**エラーハンドリングの改善**、**設定パラメータ化** という 4 つの主要テーマで構成されています。

---

## 1. 修正内容の詳細分析

### 1.1 遅延初期化パターンの導入（`__init__.py`）

**変更内容:**
- `_get_evaluator()` メソッドの追加
- `_get_processor()` メソッドの追加
- `__dict__` を使用した動的属性の共有
- 複数の委譲メソッドの追加

**コード例:**
```python
def _get_evaluator(self) -> CalibCheck2d3dEvaluator:
    evaluator = getattr(self, "evaluator", None)
    if evaluator is not None:
        return evaluator

    # 必要な属性を設定してから初期化
    self.__dict__.setdefault("image_width", getattr(self, "width", 0))
    self.__dict__.setdefault("image_height", getattr(self, "height", 0))
    # ... その他の属性設定 ...
    
    evaluator = CalibCheck2d3dEvaluator.__new__(CalibCheck2d3dEvaluator)
    evaluator.__dict__ = self.__dict__  # __dict__ 共有
    self.evaluator = evaluator
    return evaluator
```

**問題:** 
- 元の設計では `evaluator` と `processor` を宣言的に初期化していたが、移植時に初期化タイミングの問題が発生
- Jetson での実行では、属性アクセスの順序が異なり、未初期化の属性参照が起きた

**修正の妥当性:** ✓ **妥当**
- 遅延初期化により必要な属性を先に準備できる
- `__dict__` 共有パターンは元の実装と一貫している


### 1.2 モジュール間の状態共有の明示化（`processor.py`）

**変更内容:**
```python
# processor.py の input_settings() メソッド
self.evaluator.rtvec_mat = self.rtvec_mat  # 明示的に設定
```

**問題:**
- 元の実装では `CalibCheck2d3dProcessor` が `owner.__dict__` を共有していたため、状態の流れが暗黙的
- 分割後、`evaluator` が生成される前に `processor` が初期化されると、`rtvec_mat` が設定されない
- `evaluator.rtvec_mat` が未初期化のまま使用される可能性

**修正の妥当性:** ✓ **妥当**
- 状態の流れを明示的にしリスクを低減
- `processor` が `evaluator` に必要なデータを供給する関係が明確になる


### 1.3 エラーハンドリングロジックの修正（`processor.py`）

**変更内容:**

#### 3-1: `any()` → `all()` への修正
```python
# 修正前（誤り）
if any(reason_camera_notvalid):
    return reason_camera_notvalid, [False for _ in range(...)]

# 修正後（正）
if all(reason_camera_notvalid):
    return reason_camera_notvalid, [False for _ in range(...)]
```

**問題:**
- `reason_camera_notvalid` は複数カメラのエラーコードを保持（正常時は 0）
- `any(reason_camera_notvalid)` だと「いずれかのカメラにエラーがあれば全体を失敗」になる（間違い）
- 本来は「全カメラが正常でなければ失敗」すべき

**修正の妥当性:** ✓ **正しい**
- エラーハンドリングロジックが意図通りに動作するようになった

#### 3-2: 複数のエラー理由の合流ロジック
```python
# 記録フェーズでのエラー
reason_camera_notvalid = ...

# トラッキングフェーズでのエラー
tracking_reasons = self.validate_tracked_bboxes(...)

# 両方を合流（どちらかにエラーがあれば失敗）
reason_camera_notvalid = [
    recorded_reason or tracking_reason
    for recorded_reason, tracking_reason in zip(
        reason_camera_notvalid,
        tracking_reasons,
        strict=True,
    )
]
```

**問題:**
- 元の実装では各段階のエラー理由が上書きされていた可能性
- 修正前は `tracking_reasons` を `reason_camera_notvalid` に直接代入
- 後段で評価エラーが発生すると、記録エラーの情報が失われる

**修正の妥当性:** ✓ **妥当**
- 複数の障害段階を正確に追跡できるようになった
- ユーザーに正確なエラー理由を報告可能に

#### 3-3: 評価結果への理由反映
```python
camera_evaluation_results = [
    result if reason == 0 else False
    for reason, result in zip(
        reason_camera_notvalid,
        camera_evaluation_results,
        strict=True,
    )
]
```

**問題:**
- 評価が成功しても、それまでのエラーが存在すれば失敗と判定すべき
- 修正前は評価結果のみで判定していた

**修正の妥当性:** ✓ **正しい**


### 1.4 デバッグ情報の充実（`processor.py`）

**変更内容:**
```python
if self.DEBUG_CALIBCHECK_ENABLED:
    debug_store("tracking_3d_data_interface", tracking_3d_data_interface)
    for camera_ix, tracking_2d_data_interface in enumerate(...):
        debug_store(f"tracking_2d_data_interface_{camera_ix}", ...)
    debug_force_snapshot()
```

**新メソッド:**
```python
def _dump_debug_eval_info(self) -> None:
    if not self.DEBUG_CALIBCHECK_ENABLED:
        return
    with open(self.DEBUG_EVAL_PICKLE_PATH, "wb") as output_file:
        pickle.dump(self._debug_eval_info, output_file)
```

**問題:**
- 元の実装ではデバッグ情報が適切に保存されていなかった可能性
- Jetson での実行時にデバッグが必要になったが、情報が不足していた

**修正の妥当性:** ✓ **妥当**
- デバッグ情報を構造化して保存
- 問題の事後分析が容易に


### 1.5 モジュール間インポート修正

**変更内容:**
- `argus_synchro.facade` → `argus_synchro.calibration_mat_generator_modules.facade`
- `argus_synchro.diagnosis.error_diagnosis` などの完全修飾パス化
- `debug_store`, `debug_force_snapshot` の明示的なインポート

**問題:**
- 修正前は相対パスやショートカットに依存していた
- Jetson での Python パス設定が異なり、インポートが失敗した可能性

**修正の妥当性:** ✓ **妥当**
- 完全修飾パスは環境依存性を減らす


### 1.6 `CalibCheck2d3dEvaluator` の初期化改善

**変更内容:**
```python
# evaluator.py
def __init__(self, ...):
    # ... 既存コード ...
    self._calibcheck_status_diagnosis = CameraCalibCheckStatusDiagnosis()
```

プロパティの追加:
```python
@property
def virtual_bbox_debug_counts(self) -> list[VirtualBBoxDebugCounts]:
    return self._virtual_bbox_debug_counts

@property
def debug_eval_info(self) -> dict[str, object]:
    return self._debug_eval_info
```

**問題:**
- 元は `evaluator` が `scene_calibcheck._calibcheck_status_diagnosis` を参照
- 分割後、`evaluator` が独立し、`_calibcheck_status_diagnosis` が初期化されていなかった

**修正の妥当性:** ✓ **妥当**
- `evaluator` が必要な診断オブジェクトを自身で保持
- 依存関係が明確になった


### 1.7 トラッキング選択ロジックのパラメータ化（`tracker.py`）

**変更内容:**
```python
def _select_tracking_results_by_thresholds(
    self,
    tracking_idmetadata: dict[int, _TrackingMetadataLike],
    label: str,
    min_frame_length: int,
    min_accum_track_length: float,
    min_workarea_count: int | None = None,
    min_workarea_ratio: float | None = None,
) -> None:
    # ハードコード値からパラメータに変更
    if frame_ix_length < min_frame_length:  # 30 → パラメータ
        metadata.is_alive = False
    if accum_track_length < min_accum_track_length:  # 3.0 → パラメータ
        metadata.is_alive = False
```

呼び出し側:
```python
# 3D用
self._select_tracking_results_by_thresholds(
    ...,
    min_frame_length=30,
    min_accum_track_length=3.0,
    min_workarea_count=30,
    min_workarea_ratio=0.1,
)

# 2D用
self._select_tracking_results_by_thresholds(
    ...,
    min_frame_length=10,
    min_accum_track_length=50.0,
)
```

**問題:**
- 元の実装ではハードコード値が固定
- Jetson でのテストで、トラッキング設定が環境に合わず調整が必要
- パラメータ化されていないため、変更するたびにコードを修正する必要

**修正の妥当性:** ✓ **妥当**
- 設定値の変更が容易に
- 今後、config ファイルから読み込む準備が整った


### 1.8 `DATAPROC_READ_INTERVAL` の移動

**変更内容:**
```python
# 修正前: __init__.py で定義
# DATAPROC_READ_INTERVAL: int = 3

# 修正後: processor.py で定義
# DATAPROC_READ_INTERVAL: int = 3
# __init__.py でインポート
from argus_synchro.calibration_mat_generator_modules.ctrl.calibcheck2d3d.processor import (
    ...,
    DATAPROC_READ_INTERVAL,
)
```

**問題:**
- 定数が各モジュールで使用される場合、集中管理が必要
- 元は `__init__.py` で定義されていたが、`processor` がこの定数を使用
- 移植時に定数の所在が不明確だった

**修正の妥当性:** ✓ **妥当**
- 定数を使用するモジュール内で定義する方が自然
- 依存関係が明確に

---

## 2. なぜ修正漏れが生じたのか？原因分析

### 2.1 静的解析では検出不可能な問題

| 問題 | 原因 | 検出手段 |
|------|------|--------|
| **初期化タイミング** | 属性アクセス順序が実行時に決まる | 静的解析不可、実行テスト必須 |
| **状態共有の暗黙性** | `__dict__` 共有パターンは意図が不明確 | コードレビューで発見可能、実行テストで確認 |
| **条件式の誤り（any/all）** | ロジックエラーは静的解析では意図を判定不可 | 単体テストで検出可能 |

### 2.2 環境依存の問題

| 問題 | 原因 | 検出手段 |
|------|------|--------|
| **インポートパス** | 環境による Python パス設定の差異 | Jetson でのインストール・実行テスト |
| **C++ 依存関係の欠落** | `argus_synchro_lib` が desk 環境に無い | pytest 実行時に発生、desk では発見不可 |

### 2.3 分割設計の不完全さ

| 問題 | 原因 | 検出手段 |
|------|------|--------|
| **モジュール境界での状態共有** | 分割時に暗黙的な依存関係を明示化できず | 統合テスト、実行時の動的トレース |
| **エラー処理フロー** | 複数モジュールにまたがるエラーハンドリングを統一していなかった | 統合テスト、実際のエラー条件でのテスト |
| **デバッグ情報の欠落** | 問題発生時に根本原因を追跡するための情報が不足 | Jetson で実行後、逆引きで必要性が判明 |

### 2.4 テスト戦略の限界

**desk 環境での検証:**
- ✓ py_compile: 構文チェック OK
- ✗ pytest: `argus_synchro_lib` が無いため conftest が失敗
- ✗ 統合テスト: C++ ビルドが必須（不可能）

**Jetson での検証:**
- ✓ 実際の環境で実行可能
- ✓ エラーの再現と修正の検証ができた
- ✓ 所定の動作確認も実施可能

---

## 3. 修正内容の妥当性評価

### 3.1 各修正の正当性

| 修正項目 | 妥当性 | 根拠 |
|---------|--------|------|
| **遅延初期化** | ✓ 正しい | 属性準備後に初期化するため合理的 |
| **状態共有の明示化** | ✓ 正しい | 暗黙的な流れを明確に |
| **any() → all()** | ✓ 正しい | 学習不可ロジックから必要条件へ |
| **エラー理由の合流** | ✓ 正しい | 複数段階の失敗を正確に追跡 |
| **デバッグ情報充実** | ✓ 正しい | 問題分析を支援 |
| **インポートパス完全修飾** | ✓ 正しい | 環境依存性を低減 |
| **トラッキング設定パラメータ化** | ✓ 正しい | 柔軟な設定変更を可能に |

### 3.2 修正内容の完全性

修正は **包括的** と判定します。理由：

1. **全層をカバー** - `__init__.py`, `processor.py`, `evaluator.py`, `tracker.py` を修正
2. **初期化から評価まで** - システムのライフサイクル全体に対応
3. **エラー処理の一貫性** - 複数モジュールのエラーハンドリングを統一

### 3.3 副作用のリスク

修正に伴う潜在的リスク：

- **遅延初期化** → リスク低（必要な属性を事前準備）
- **状態共有の明示化** → リスク低（既存の `__dict__` 共有パターン継続）
- **エラーロジック修正** → リスク低（正しいロジックへの修正）
- **デバッグ情報追加** → リスク低（デバッグ時のみ有効）

---

## 4. 推奨事項

### 4.1 今後の予防措置

1. **統合テスト環境の構築**
   - Jetson の試験環境で継続的にテスト
   - C++ 依存なしで Python 範囲をテスト可能な conftest 設計

2. **状態共有の明示化ガイド**
   - モジュール分割時には `__dict__` 共有パターンを避け、明示的なデータの受け渡しにする
   - 設計ドキュメントでモジュール間の依存関係を図示

3. **エラーハンドリングの統一**
   - エラー処理フローを設計段階で明確化
   - 複数モジュールにまたがるエラーは合流ロジックを統一

4. **デバッグ情報の標準化**
   - 問題時の根本原因追跡に必要な情報を定義
   - デバッグ出力を構造化（JSON/pickle 形式で保存可能に）

5. **環境別テストスイート**
   - Python 範囲: desk + Jetson
   - C++ ビルド: CI パイプライン
   - E2E テスト: Jetson 環境のみ

### 4.2 ドキュメント更新

- [x] 本分析ドキュメント作成
- [ ] 実装ガイドラインの更新（モジュール分割時の注意点）
- [ ] トラッキング設定パラメータのドキュメント化
- [ ] デバッグ情報形式の標準化

---

## 5. 結論

**修正内容は妥当で、Jetson 環境での運用に必要な改善が網羅的に行われています。**

主な改善点：
1. **遅延初期化** で属性準備のタイミング問題を解決
2. **エラーハンドリングロジック** を正確化
3. **デバッグ情報** を充実させ、問題分析を支援
4. **トラッキング設定** をパラメータ化し、環境適応性を向上

**推奨アクション:**
1. 修正を本体に統合
2. テスト環境を Jetson に拡張
3. 将来のモジュール分割時の設計ガイドラインを策定
