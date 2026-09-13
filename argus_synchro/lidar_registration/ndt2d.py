from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


def preprocess_xy(
    pts_xyz: NDArray[np.float64],
    r_min: float = 0.5,
    r_max: float = 80.0,
) -> NDArray[np.float64]:
    """Range + optional z-gate, then return XY (N,2)."""
    P = np.asarray(pts_xyz, dtype=np.float64)
    if P.ndim != 2 or P.shape[1] < 3:
        raise ValueError("pts_xyz must be (N,3+)")

    r = np.linalg.norm(P[:, :3], axis=1)
    m = (r >= r_min) & (r <= r_max)

    P = P[m]
    return P[:, :2].astype(np.float64)


def voxel_downsample_xy(Pxy: NDArray[np.float64], voxel: float) -> NDArray[np.float64]:
    """Cheap 2D voxel downsample: keep one point per 2D voxel (centroid)."""
    P = np.asarray(Pxy, dtype=np.float64)
    if voxel <= 0:
        return P
    keys = np.floor(P / voxel).astype(np.int32)
    # group by keys via sort
    order = np.lexsort((keys[:, 1], keys[:, 0]))
    keys_s = keys[order]
    P_s = P[order]

    # find group boundaries
    change = np.any(np.diff(keys_s, axis=0) != 0, axis=1)
    idx = np.flatnonzero(np.r_[True, change, True])
    out = []
    for a, b in zip(idx[:-1], idx[1:]):
        out.append(P_s[a:b].mean(axis=0))
    return np.asarray(out, dtype=np.float64)


def se2_from_params(tx: float, ty: float, yaw: float) -> NDArray[np.float64]:
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(3, dtype=np.float64)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    T[0, 2] = tx
    T[1, 2] = ty
    return T


def se2_apply(T: NDArray[np.float64], Pxy: NDArray[np.float64]) -> NDArray[np.float64]:
    P = np.asarray(Pxy, dtype=np.float64)
    R = T[:2, :2]
    t = T[:2, 2]
    return (P @ R.T) + t


def se2_to_se3(T2: NDArray[np.float64]) -> NDArray[np.float64]:
    """Embed SE2 into SE3 (rotation about z, xy translation)."""
    T3 = np.eye(4, dtype=np.float64)
    T3[:2, :2] = T2[:2, :2]
    T3[0, 3] = T2[0, 2]
    T3[1, 3] = T2[1, 2]
    return T3


@dataclass(frozen=True)
class NDT2DParams:
    # 2D格子の1辺 [m]。大きいほどロバストで局所解に落ちにくいが精度は落ちる
    voxel_size: float = 0.5

    # 1セルを「有効な正規分布」として採用するために必要な最小点数
    # 大きいほど統計が安定するが、有効セルが減って対応が取れなくなる（inliers減）
    min_points_per_voxel: int = 30

    # 共分散に足すI成分（Sigma += cov_regularization * I）
    # 退化（直線状/点が少ない）セルでも逆行列が破綻しないようにする
    cov_regularization: float = 1e-3

    # 共分散の固有値下限。固有値が小さすぎるセル（退化）を強制的に丸めて安定化する
    eig_floor: float = 1e-4

    # 9近傍(3x3)のうち、評価に使うセル数。小さいほど高速だが境界不連続が出やすい
    # 6〜9が無難
    neighbor_top_k: int = 6  # <=9

    # マハラノビス距離^2 のゲート。これより大きいセルは「外れ」として重み0にする
    # 小さすぎると inliers が減って動かない。大きすぎると外れに引っ張られる
    neighbor_maha2_gate: float = 200.0

    # 大きいほど重み分布が平坦になりロバスト（ただし重くなる）
    # 1.0〜3.0が目安
    weight_temperature: float = 1.5

    # 幾何重みのスケール。w_geom ∝ exp(-0.5*||p-center||^2 / geom_sigma^2)
    # voxel_sizeの0.7〜1.0倍が目安。小さすぎると境界で不連続、大きすぎると効果が薄い
    geom_sigma: float = 0.5

    # セル中心からの距離ゲート [m]（0なら無効）
    # 有効セルを近傍中心に限定して外れを切りたいときに使うが、厳しすぎるとinliersが落ちる
    # 0は無効
    geom_gate: float = 0.0

    # 最大反復回数。初期行列がある程度あっているなら10〜30で十分
    max_iters: int = 20

    # LMの初期ダンピング。小さいほどGauss-Newton寄りで速いが不安定になりやすい
    lm_lambda_init: float = 1e-3

    # λの上下限
    lm_lambda_min: float = 1e-6
    lm_lambda_max: float = 1e6

    # ステップがrejectされたら λ *= upacceptされたら λ *= down
    lm_lambda_up: float = 10.0
    lm_lambda_down: float = 0.3

    # 結果が吹き飛ぶのを防止するための策
    # 1反復あたりの並進更新量の上限 [m]（tx,tyの2次元ノルムでクリップ）
    step_clip_trans: float = 0.5

    # 1反復あたりのyaw更新量上限 [rad]
    step_clip_yaw: float = np.deg2rad(10)

    # 停止条件
    # ||Δx||（tx,ty,yawの更新量ノルム）がこれ未満なら収束とみなす
    convergence_eps: float = 1e-4

    # スコア改善量がこれ未満なら頭打ちとして停止
    improve_eps: float = 1e-6

    # 正規化項の有無
    # 多くの場合FalseでOK。Trueはセルの分散差を強く意識させたい場合
    # Trueにすると 0.5*log|Sigma| のような正規化項をスコアに含める。
    score_include_norm: bool = False


@dataclass
class IterLog2D:
    iter: int
    score: float
    inliers: int
    lm_lambda: float
    delta_norm: float
    accepted: bool


@dataclass
class NDT2DResult:
    T2: NDArray[np.float64]  # (3,3) src->tgt in XY
    converged: bool
    iters: int
    final_score: float
    inliers: int
    history: list[IterLog2D] = field(default_factory=list)

    @property
    def T3(self) -> NDArray[np.float64]:
        return se2_to_se3(self.T2)


_key2_dtype = np.dtype([("x", np.int32), ("y", np.int32)])


def _to_struct2(keys_xy: NDArray[np.int32]) -> NDArray:
    out = np.empty(keys_xy.shape[:-1], dtype=_key2_dtype)
    out["x"] = keys_xy[..., 0]
    out["y"] = keys_xy[..., 1]
    return out


@dataclass
class GaussianCell2D:
    mu: NDArray[np.float64]  # (2,)
    cov: NDArray[np.float64]  # (2,2)
    info: NDArray[np.float64]  # (2,2)
    logdet: float
    num_points: int


class VoxelGaussianMap2D:
    def __init__(self, params: NDT2DParams):
        self.p = params
        self._keys_sorted: NDArray | None = None
        self._mu_sorted: NDArray[np.float64] | None = None  # (M,2)
        self._info_sorted: NDArray[np.float64] | None = None  # (M,2,2)
        self._logdet_sorted: NDArray[np.float64] | None = None  # (M,)

    @property
    def num_cells(self) -> int:
        return 0 if self._keys_sorted is None else int(self._keys_sorted.shape[0])

    def build(self, tgt_xy: NDArray[np.float64]) -> None:
        vs = float(self.p.voxel_size)
        P = np.asarray(tgt_xy, dtype=np.float64)
        if P.ndim != 2 or P.shape[1] != 2:
            raise ValueError("tgt_xy must be (N,2)")

        keys = np.floor(P / vs).astype(np.int32)
        order = np.lexsort((keys[:, 1], keys[:, 0]))
        keys_s = keys[order]
        P_s = P[order]

        # group boundaries
        change = np.any(np.diff(keys_s, axis=0) != 0, axis=1)
        idx = np.flatnonzero(np.r_[True, change, True])

        keys_list = []
        mu_list = []
        info_list = []
        logdet_list = []

        for a, b in zip(idx[:-1], idx[1:]):
            if (b - a) < int(self.p.min_points_per_voxel):
                continue
            pts = P_s[a:b]
            mu = pts.mean(axis=0)
            X = pts - mu
            cov = (X.T @ X) / max(len(pts) - 1, 1)
            cov = self._regularize_cov(cov)
            info = np.linalg.inv(cov)
            sign, ld = np.linalg.slogdet(cov)
            logdet = float(ld) if sign > 0 else 0.0

            keys_list.append(keys_s[a])
            mu_list.append(mu)
            info_list.append(info)
            logdet_list.append(logdet)

        if not keys_list:
            self._keys_sorted = None
            self._mu_sorted = None
            self._info_sorted = None
            self._logdet_sorted = None
            return

        keys_xy = np.asarray(keys_list, dtype=np.int32)  # (M,2)
        s_keys = _to_struct2(keys_xy)
        ord2 = np.argsort(s_keys, kind="mergesort")

        self._keys_sorted = s_keys[ord2]
        self._mu_sorted = np.asarray(mu_list, dtype=np.float64)[ord2]
        self._info_sorted = np.asarray(info_list, dtype=np.float64)[ord2]
        self._logdet_sorted = np.asarray(logdet_list, dtype=np.float64)[ord2]

    def batch_lookup(self, keys_xy: NDArray[np.int32]) -> NDArray[np.int32]:
        if self._keys_sorted is None:
            return -np.ones((keys_xy.shape[0],), dtype=np.int32)
        q = _to_struct2(keys_xy)
        pos = np.searchsorted(self._keys_sorted, q)
        idx = -np.ones((q.shape[0],), dtype=np.int32)
        valid = (pos >= 0) & (pos < self._keys_sorted.shape[0])
        if not np.any(valid):
            return idx
        posv = pos[valid]
        eq = self._keys_sorted[posv] == q[valid]
        idx[valid] = np.where(eq, posv.astype(np.int32), -1).astype(np.int32)
        return idx

    def gather(
        self, idx: NDArray[np.int32]
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        if (
            self._mu_sorted is None
            or self._info_sorted is None
            or self._logdet_sorted is None
        ):
            raise RuntimeError("Map not built.")
        idx_clip = np.maximum(idx, 0)
        mu = self._mu_sorted[idx_clip]
        info = self._info_sorted[idx_clip]
        logdet = self._logdet_sorted[idx_clip]
        mask = idx >= 0
        mu = np.where(mask[..., None], mu, 0.0)
        info = np.where(mask[..., None, None], info, 0.0)
        logdet = np.where(mask, logdet, 0.0)
        return mu, info, logdet

    def _regularize_cov(self, cov: NDArray[np.float64]) -> NDArray[np.float64]:
        cov = np.asarray(cov, dtype=np.float64)
        cov = 0.5 * (cov + cov.T)
        cov = cov + float(self.p.cov_regularization) * np.eye(2, dtype=np.float64)
        w, V = np.linalg.eigh(cov)
        w = np.maximum(w, float(self.p.eig_floor))
        cov2 = (V * w) @ V.T
        return 0.5 * (cov2 + cov2.T)


class NDTObjective2D:
    def __init__(self, params: NDT2DParams, m: VoxelGaussianMap2D):
        self.p = params
        self.m = m
        # 9-neighborhood offsets
        off = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                off.append((dx, dy))
        self.off9 = np.asarray(off, dtype=np.int32)  # (9,2)

    def evaluate(
        self, T2: NDArray[np.float64], src_xy: NDArray[np.float64]
    ) -> tuple[float, NDArray[np.float64], NDArray[np.float64], int]:
        Psrc = np.asarray(src_xy, dtype=np.float64)
        if Psrc.ndim != 2 or Psrc.shape[1] != 2:
            raise ValueError("src_xy must be (N,2)")

        vs = float(self.p.voxel_size)

        # Apply SE2
        R = T2[:2, :2]
        t = T2[:2, 2]
        P = (Psrc @ R.T) + t  # (N,2)
        N = P.shape[0]

        # Base voxel key for each transformed point
        V = np.floor(P / vs).astype(np.int32)  # (N,2)
        K = V[:, None, :] + self.off9[None, :, :]  # (N,9,2)

        # Lookup neighbors in batch
        idx = self.m.batch_lookup(K.reshape(-1, 2)).reshape(N, 9)  # (N,9)
        mu, info, logdet = self.m.gather(idx)  # mu:(N,9,2), info:(N,9,2,2)

        # residuals
        e = P[:, None, :] - mu  # (N,9,2)
        maha2 = np.einsum("nki,nkij,nkj->nk", e, info, e)  # (N,9)

        valid = idx >= 0
        maha2 = np.where(valid, maha2, np.inf)
        maha2 = np.where(maha2 <= float(self.p.neighbor_maha2_gate), maha2, np.inf)

        # top-k (<=9)
        Ktop = int(self.p.neighbor_top_k)
        if 0 < Ktop < 9:
            sel = np.argpartition(maha2, Ktop - 1, axis=1)[:, :Ktop]
            maha2 = np.take_along_axis(maha2, sel, axis=1)
            idx2 = np.take_along_axis(idx, sel, axis=1)
            mu = np.take_along_axis(mu, sel[:, :, None], axis=1)
            info = np.take_along_axis(info, sel[:, :, None, None], axis=1)
            logdet = np.take_along_axis(logdet, sel, axis=1)
            K = np.take_along_axis(K, sel[:, :, None], axis=1)
            valid = (idx2 >= 0) & np.isfinite(maha2)
        else:
            valid = (idx >= 0) & np.isfinite(maha2)

        # geometric distance to voxel centers
        centers = (K.astype(np.float64) + 0.5) * vs  # (N,K,2)
        geom_d2 = np.sum((P[:, None, :] - centers) ** 2, axis=2)

        if float(self.p.geom_gate) > 0.0:
            gate2 = float(self.p.geom_gate) ** 2
            valid = valid & (geom_d2 <= gate2)

        # stable weight
        maha2_masked = np.where(valid, maha2, np.inf)
        maha2_min = np.min(maha2_masked, axis=1)  # (N,)
        has_any = np.isfinite(maha2_min)
        maha2_min = np.where(has_any, maha2_min, 0.0)

        temp = max(float(self.p.weight_temperature), 1e-6)
        sigma_g = max(float(self.p.geom_sigma), 1e-6)

        w1 = np.exp(-0.5 * (maha2 - maha2_min[:, None]) / temp)
        w2 = np.exp(-0.5 * geom_d2 / (sigma_g * sigma_g))
        w_raw = np.where(valid, w1 * w2, 0.0)

        w_sum = np.sum(w_raw, axis=1)  # (N,)
        w = np.zeros_like(w_raw)
        np.divide(w_raw, w_sum[:, None], out=w, where=(w_sum[:, None] > 0.0))

        # score
        score_terms = 0.5 * maha2
        if self.p.score_include_norm:
            score_terms = score_terms + 0.5 * logdet
        score = float(np.sum(w * np.where(np.isfinite(score_terms), score_terms, 0.0)))

        # Jacobian for params [tx, ty, yaw]
        # p = R * psrc + t
        # dp/dtx = [1,0], dp/dty=[0,1]
        # dp/dyaw = d(R*psrc)/dθ
        # using psrc (not transformed):
        # d/dθ [c -s; s c] [x;y] = [-s*x - c*y,  c*x - s*y]
        x = Psrc[:, 0]
        y = Psrc[:, 1]
        c = R[0, 0]
        s = R[1, 0]  # since R = [[c,-s],[s,c]]

        dtheta = np.stack([-s * x - c * y, c * x - s * y], axis=1)  # (N,2)

        J = np.zeros((N, 2, 3), dtype=np.float64)
        J[:, :, 0] = np.array([1.0, 0.0])  # tx
        J[:, :, 1] = np.array([0.0, 1.0])  # ty
        J[:, :, 2] = dtheta  # yaw

        # r = info @ e  -> (N,K,2)
        r = np.einsum("nkij,nkj->nki", info, (P[:, None, :] - mu))
        # Jr = J^T r -> (N,K,3)
        Jr = np.einsum("nij,nki->nkj", J, r)

        g = np.einsum("nk,nkj->j", w, Jr).astype(np.float64)  # (3,)

        # H = sum w * (J^T info J)
        B = np.einsum("nkij,njl->nkil", info, J)  # (N,K,2,3)
        H_nk = np.einsum("nij,nkil->nkjl", J, B)  # (N,K,3,3)
        H = np.einsum("nk,nkjl->jl", w, H_nk).astype(np.float64)

        inliers = int(np.sum(w_sum > 0.0))
        return score, g, H, inliers


class LM3:
    def __init__(self, p: NDT2DParams):
        self.p = p
        self.lmbd = float(p.lm_lambda_init)

    def solve(
        self, H: NDArray[np.float64], g: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        lam = float(np.clip(self.lmbd, self.p.lm_lambda_min, self.p.lm_lambda_max))
        A = H + lam * np.eye(3, dtype=np.float64)
        b = -np.asarray(g, dtype=np.float64).reshape(3)
        try:
            dx = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            dx = np.linalg.lstsq(A, b, rcond=None)[0]
        return dx.astype(np.float64)

    def update(self, accepted: bool) -> None:
        if accepted:
            self.lmbd = max(self.p.lm_lambda_min, self.lmbd * self.p.lm_lambda_down)
        else:
            self.lmbd = min(self.p.lm_lambda_max, self.lmbd * self.p.lm_lambda_up)


def clip_step(dx: NDArray[np.float64], p: NDT2DParams) -> NDArray[np.float64]:
    dx = np.asarray(dx, dtype=np.float64).reshape(3).copy()
    dt = dx[:2]
    dth = float(dx[2])

    nt = float(np.linalg.norm(dt))
    if nt > p.step_clip_trans and nt > 1e-12:
        dx[:2] = dt * (p.step_clip_trans / nt)
    if abs(dth) > p.step_clip_yaw:
        dx[2] = np.sign(dth) * p.step_clip_yaw
    return dx


class NDT2D:
    def __init__(self, params: NDT2DParams | None = None):
        self.p = params or NDT2DParams()
        self.map = VoxelGaussianMap2D(self.p)
        self.obj = NDTObjective2D(self.p, self.map)
        self.lm = LM3(self.p)

    def fit(self, tgt_xy: NDArray[np.float64]) -> None:
        self.map.build(tgt_xy)
        if self.map.num_cells == 0:
            raise RuntimeError(
                "2D NDT map has zero valid voxels. Check voxel_size / min_points_per_voxel / filtering."
            )

    def register(
        self, src_xy: NDArray[np.float64], init_T2: NDArray[np.float64]
    ) -> NDT2DResult:
        T = np.asarray(init_T2, dtype=np.float64).copy()
        if T.shape != (3, 3):
            raise ValueError("init_T2 must be (3,3)")

        history: list[IterLog2D] = []

        score, g, H, inliers = self.obj.evaluate(T, src_xy)
        prev_score = float(score)
        converged = False

        for it in range(1, self.p.max_iters + 1):
            dx = self.lm.solve(H, g)
            dx = clip_step(dx, self.p)
            delta_norm = float(np.linalg.norm(dx))

            # compose update in SE2: left-multiply Exp(dx) ≈ [R(dyaw), dt]
            dtx, dty, dyaw = float(dx[0]), float(dx[1]), float(dx[2])
            dT = se2_from_params(dtx, dty, dyaw)
            T_cand = dT @ T

            score_c, g_c, H_c, inliers_c = self.obj.evaluate(T_cand, src_xy)

            accepted = bool(score_c < score)
            self.lm.update(accepted)

            if accepted:
                T, score, g, H, inliers = T_cand, score_c, g_c, H_c, inliers_c

            history.append(
                IterLog2D(
                    iter=it,
                    score=float(score),
                    inliers=int(inliers),
                    lm_lambda=float(self.lm.lmbd),
                    delta_norm=delta_norm,
                    accepted=accepted,
                )
            )

            # if delta_norm < self.p.convergence_eps:
            #    converged = True
            #    break
            # improvement = float(prev_score - score)
            # if improvement >= 0.0 and improvement < self.p.improve_eps:
            #    converged = True
            #    break
            prev_score = float(score)

        return NDT2DResult(
            T2=T,
            converged=converged,
            iters=len(history),
            final_score=float(score),
            inliers=int(inliers),
            history=history,
        )
