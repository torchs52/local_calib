import pickle
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from argus_synchro.common.app_logger import AppLogger, AppLoggerFactory

# argus_synchro/SceneDesc.py からコピペ・変更 相談の上で統合した方が良い
_logger: AppLogger = AppLoggerFactory.from_name("")


def log_register(app_logger_factory: AppLoggerFactory) -> None:
    app_logger_factory.append_logger(_logger)


def get_human_3bb_withscore(
    box2d: NDArray[np.float32],
    width: int,
    height: int,
    box3ds: NDArray[Any],
    num_3d: int,
    method: str = "center",
    verbose=False,
    recordfile_debuginfo=None,
) -> tuple[int, float]:
    """
    box2d:YoloのBB、[image_h_min, image_w_min, image_h_max, image_w_max](0~1で正規化された位置)
    width:画像の幅
    height:画像の高さ
    box3ds:立体物数*[x, y]*8点が1列に並んでいる
    cat3ds:立体物のクラス、0:人, それ以外も判別はされている
    num_3d:3dbbの数
    返り値:box2dに最も近いbox3dのインデックス、条件に合うものがない場合は-1
    """
    if method == "center":
        return _correspondence_by_center_withscore(
            box2d, width, height, box3ds, num_3d, verbose, recordfile_debuginfo
        )
    if method == "iou":
        return _correspondence_by_iou_withscore(
            box2d, width, height, box3ds, num_3d, verbose, recordfile_debuginfo
        )
    if method == "endpoints":
        return _correspondence_by_endpoints_withscore(
            box2d, width, height, box3ds, num_3d, verbose, recordfile_debuginfo
        )
    raise ValueError(f"method should be 'center' or 'iou', current method = {method}")


def _correspondence_by_endpoints_withscore(
    box2d: NDArray[np.float32],
    width: int,
    height: int,
    box3ds: NDArray[Any],
    num_3d: int,
    verbose=False,
    recordfile_debuginfo=None,
) -> tuple[int, float]:
    """
    2dと3dのbounding boxの端点の近さで選ぶ, 端点は最小のxy, 最大のxyのそれぞれでの比較
    box2d:YoloのBB、[image_h_min, image_w_min, image_h_max, image_w_max](0~1で正規化された位置)
    width:画像の幅
    height:画像の高さ
    box3ds:立体物数*[x, y]*8点が1列に並んでいる
    cat3ds:立体物のクラス、0:人, それ以外も判別はされている
    num_3d:3dbbの数
    返り値:box2dに最も近いbox3dのインデックス、条件に合うものがない場合は-1
    """
    box2d_min: NDArray[np.float64] = np.array((box2d[1] * width, box2d[0] * height))
    box2d_max: NDArray[np.float64] = np.array((box2d[3] * width, box2d[2] * height))
    dists = []
    for j in range(num_3d):
        box_3d_in_2d = box3ds[(j * 8 + 4) : (j * 8 + 8)]
        box3d_max = box_3d_in_2d.max(axis=0)
        box3d_min = box_3d_in_2d.min(axis=0)
        dists.append(
            (
                np.linalg.norm(box3d_min - box2d_min)
                + np.linalg.norm(box3d_max - box2d_max)
            )
            / 2
        )
        if verbose:
            _logger.info(
                "_correspondence_by_endpoints_withscore",
                f"box_3d_in_2d:{box_3d_in_2d}, dists:{dists}",
            )

    index: int = dists.index(min(dists))
    threshould: int = max(width, height)  # しきい値、画像サイズ長辺の100%
    if recordfile_debuginfo is not None:
        pickle.dump(
            ("_correspondence_by_endpoints_withscore calc", dists, index, threshould),
            recordfile_debuginfo,
        )
    if dists[index] > threshould:
        return -1, -1
    return index, min(dists)


def calc_iou(
    ax_min: float,
    ay_min: float,
    ax_max: float,
    ay_max: float,
    bx_min: float,
    by_min: float,
    bx_max: float,
    by_max: float,
) -> tuple[float, float, float, float]:
    """
    aとbのiouを計算する関数
    """
    a_area = (ax_max - ax_min) * (ay_max - ay_min)
    b_area = (bx_max - bx_min) * (by_max - by_min)

    if (a_area == 0) and (b_area == 0):
        return 0, 0, 0, 0

    abx_min = max(ax_min, bx_min)
    aby_min = max(ay_min, by_min)
    abx_max = min(ax_max, bx_max)
    aby_max = min(ay_max, by_max)

    intersect = max(0, abx_max - abx_min) * max(0, aby_max - aby_min)
    iou = intersect / (a_area + b_area - intersect)

    return iou, intersect, a_area, b_area


def _correspondence_by_iou_withscore(
    box2d: NDArray[np.float32],
    width: int,
    height: int,
    box3ds: NDArray[Any],
    num_3d: int,
    verbose=False,
    recordfile_debuginfo=None,
) -> tuple[int, float]:
    """
    iouベースで2dと3dの対応付けを行う, iouが最も大きい対応のものを選ぶ
    box2d:YoloのBB、[image_h_min, image_w_min, image_h_max, image_w_max](0~1で正規化された位置)
    width:画像の幅
    height:画像の高さ
    box3ds:立体物数*[x, y]*8点が1列に並んでいる
    cat3ds:立体物のクラス、0:人, それ以外も判別はされている
    num_3d:3dbbの数
    返り値:box2dに最も近いbox3dのインデックス、条件に合うものがない場合は-1
    """
    box2d_x: tuple[float, float] = (box2d[1] * width, box2d[3] * width)
    box2d_y: tuple[float, float] = (box2d[0] * height, box2d[2] * height)
    box2d_xy = (box2d_x[0], box2d_y[0], box2d_x[1], box2d_y[1])

    ious = []
    for j in range(num_3d):
        box_3d_in_2d = box3ds[(j * 8 + 0) : (j * 8 + 8)]
        max_pos = box_3d_in_2d.max(axis=0)
        min_pos = box_3d_in_2d.min(axis=0)
        box3d_xy = (min_pos[0], min_pos[1], max_pos[0], max_pos[1])
        ious.append(calc_iou(*box2d_xy, *box3d_xy)[0])
        if verbose:
            _logger.info(
                f"box2d_xy:{box2d_xy}, box3d_xy:{box3d_xy}, calc_iou(*box2d_xy, *box3d_xy)[0]:{calc_iou(*box2d_xy, *box3d_xy)[0]}"
            )

    if recordfile_debuginfo is not None:
        pickle.dump(
            ("_correspondence_by_iou_withscore calc", ious), recordfile_debuginfo
        )

    if max(ious) == 0:
        # iouが全て0の場合は失敗とする
        return -1, -1

    index: int = ious.index(max(ious))
    return index, max(ious)


# 2DBBに最も近い3DBBを探索
def _correspondence_by_center_withscore(
    box2d: NDArray[np.float32],
    width: int,
    height: int,
    box3ds: NDArray[Any],
    num_3d: int,
    verbose=False,
    recordfile_debuginfo=None,
) -> tuple[int, float]:
    """
    重心ベースで2dと3dの対応付けを行う
    box2d:YoloのBB、[image_h_min, image_w_min, image_h_max, image_w_max](0~1で正規化された位置)
    width:画像の幅
    height:画像の高さ
    box3ds:立体物数*[x, y]*8点が1列に並んでいる
    cat3ds:立体物のクラス、0:人, それ以外も判別はされている
    num_3d:3dbbの数
    返り値:box2dに最も近いbox3dのインデックス、条件に合うものがない場合は-1
    """

    # bboxが無い場合はそのままリターン
    if num_3d == 0 or len(box3ds) == 0:
        return -1, -1

    # 両方のBBの地面位置重心距離が最も近いものを選ぶ
    dists: list[float] = []
    bottom_x_2d: float = (box2d[3] + box2d[1]) * width / 2.0
    bottom_y_2d: float = box2d[2] * height
    # for j in range(int(len(box3ds)/8)):
    for j in range(num_3d):
        bottom_x_3d = 0
        bottom_y_3d = 0
        for i in range(4, 8):  # 3dbb底面の画像上重心を計算
            bottom_x_3d += (
                box3ds[j * 8 + i][0] / 4
            )  # 射影失敗して大きな負値の場合もあるが、距離が遠くなるので候補からは除外される
            bottom_y_3d += box3ds[j * 8 + i][1] / 4
        dists.append(
            np.sqrt(
                pow(bottom_x_2d - bottom_x_3d, 2) + pow(bottom_y_2d - bottom_y_3d, 2),
            ),
        )  # 重心距離を格納
        if verbose:
            _logger.info(f"bottom_x_2d:{bottom_x_2d}, bottom_y_3d:{bottom_y_3d}")
        if recordfile_debuginfo is not None:
            pickle.dump(
                (
                    "_correspondence_by_center_withscore calc",
                    j,
                    i,
                    bottom_x_2d,
                    bottom_y_2d,
                    bottom_x_3d,
                    bottom_y_3d,
                    dists[-1],
                ),
                recordfile_debuginfo,
            )

    # 最小の3bbが遠すぎる場合は除外する
    # index = dists.index(min(dists))
    # threshould = max(width, height)*0.05 #しきい値、画像サイズ長辺の5%
    # if dists[index]>threshould:
    #     return -1
    # else:
    #     return index

    # 画面外の3D bboxに

    # 安全のため、人は必ずどれかのBBに紐付ける（->前提は再度検討必要）
    index: int = dists.index(min(dists))
    threshould: int = max(width, height)  # しきい値、画像サイズ長辺の100%
    if dists[index] > threshould:
        return -1, -1
    return index, dists[index]


# --- コピペここまで


def project_3dbbox_to2d(
    box3ds, rvec, tvec, ncm1, width, height, recordfile_debuginfo=None
):
    """
    box3ds: [x,y,z]のバウンディングボックス頂点座標が8×bbox個数分
    """
    if (box3ds is None) or (box3ds.size == 0):
        return np.zeros((0, 2))
    assert box3ds.shape[0] > 0
    if recordfile_debuginfo is not None:
        pickle.dump(("project_3dbbox_to2d - box3ds", box3ds), recordfile_debuginfo)

    box3ds_reproj: NDArray[Any] = cv2.projectPoints(
        np.array([box3ds]),
        rvec,
        tvec,
        ncm1,
        np.zeros((1, 5)),
    )[0].squeeze(1)  # 2次元座標に変換
    assert (
        np.array([box3ds]).shape[0] == 1
    )  # 形状として(1,N,3)を想定。この形が崩れると後の処理で意図しない動作をする可能性がある (reshape時に順序が崩れる、次元が合わない等)

    if recordfile_debuginfo is not None:
        pickle.dump(
            ("project_3dbbox_to2d - box3ds_reproj - first", box3ds_reproj),
            recordfile_debuginfo,
        )

    extrmat = np.eye(4)
    extrmat[:3, 3] = tvec.reshape(3)
    extrmat[:3, :3] = cv2.Rodrigues(rvec)[0]

    homogeneous_points = np.hstack([box3ds, np.ones((box3ds.shape[0], 1))]).T
    camera_coordinate_pts = extrmat @ homogeneous_points
    camera_coordin_z = camera_coordinate_pts[2]

    if recordfile_debuginfo is not None:
        pickle.dump(("project_3dbbox_to2d - extrmat", extrmat), recordfile_debuginfo)
        pickle.dump(
            ("project_3dbbox_to2d - camera_coordinate_pts", camera_coordinate_pts),
            recordfile_debuginfo,
        )

    BBOX_VERTEX_POINTS = 8

    # box3ds_reproj: bbox8点分ずつ格納。前からn_clusters*8点分のみ有効（n_clusters*8以降は不定？）対応するz座標を8個ずつ見て1つでも<0なら8点全て除去する必要がある
    # 本当に除去してしまうとintegrated_retults_2d3d反映時のインデックスと整合が取れなくなるので-1e6に飛ばすことで対応。
    camera_coordin_z_bboxset = camera_coordin_z.reshape(-1, BBOX_VERTEX_POINTS)
    camera_coordin_z_bboxset_flag = np.all(camera_coordin_z_bboxset > 0.5, axis=1)
    box3ds_zfilter = np.repeat(camera_coordin_z_bboxset_flag, BBOX_VERTEX_POINTS)

    if recordfile_debuginfo is not None:
        pickle.dump(
            (
                "project_3dbbox_to2d - camera_coordin_z_bboxset_flag",
                camera_coordin_z_bboxset_flag,
            ),
            recordfile_debuginfo,
        )
        pickle.dump(
            ("project_3dbbox_to2d - box3ds_zfilter", box3ds_zfilter),
            recordfile_debuginfo,
        )

    box3ds_reproj[box3ds_zfilter == 0] = -1e6

    if recordfile_debuginfo is not None:
        pickle.dump(
            ("project_3dbbox_to2d - box3ds_reproj - final", box3ds_reproj),
            recordfile_debuginfo,
        )

    return box3ds_reproj


def integrate_results2d3d(
    yoloresult_whole,
    box3ds_reproj,
    width,
    height,
    n_clusters,
    defaultval,
    linkmethod,
    comparemode,
    recordfile_debuginfo=None,
    integrated_retults_2d3d=None,
):
    if integrated_retults_2d3d is None:
        integrated_retults_2d3d = [
            ("other", -1, defaultval, np.zeros((0, 2)), np.zeros((0, 2)))
            for x in range(n_clusters)
        ]  # 3DBBの属性リスト

    if recordfile_debuginfo is not None:
        pickle.dump(
            ("integrate_results2d3d box3ds_reproj", box3ds_reproj), recordfile_debuginfo
        )

    for i in range(int(yoloresult_whole[3])):  # 2dbbを順番にチェック
        if yoloresult_whole[2].reshape((-1, 1))[i] == 0:  # 人である場合
            box2d: NDArray[np.float32] = yoloresult_whole[0].reshape((-1, 4))[i]
            index, score = get_human_3bb_withscore(
                box2d=box2d,
                width=int(width),
                height=int(height),
                box3ds=box3ds_reproj,
                num_3d=n_clusters,
                method=linkmethod,
                recordfile_debuginfo=recordfile_debuginfo,
            )
            if recordfile_debuginfo is not None:
                pickle.dump(
                    ("get_human_3bb_withscore result", i, index, score),
                    recordfile_debuginfo,
                )

            if index >= 0:
                if comparemode == "min":
                    eval_result = score < integrated_retults_2d3d[index][1]
                else:
                    eval_result = score > integrated_retults_2d3d[index][1]
                if eval_result:
                    # 該当3dBBの情報を書き換え
                    integrated_retults_2d3d[index] = (
                        "human",
                        i,
                        score,
                        box3ds_reproj[int(index * 8) : int(index * 8 + 8)],
                        box2d,
                    )
    if recordfile_debuginfo is not None:
        pickle.dump(
            (
                "integrated_retults_2d3d result",
                [[ix, *X] for ix, X in enumerate(integrated_retults_2d3d)],
            ),
            recordfile_debuginfo,
        )
    return integrated_retults_2d3d


def evaluate2d3d(
    width,
    height,
    multi_points,
    linkmethod,
    yoloresult_whole,
    rvec,
    tvec,
    ncm1,
    recordfile_debuginfo=None,
    integrated_retults_2d3d_old=None,
):
    """
    method, comparemode:
        "center"  minが最良 "min"
        "iou": maxが最良 "max"
        "endpoints": minが最良 "min"
    """
    comparemode = {"iou": "max", "center": "min", "endpoints": "min"}[linkmethod]
    if comparemode == "min":
        defaultval = 1e6
    else:
        defaultval = -1.0

    if len(multi_points) == 0:
        return [("none", -1, defaultval, np.zeros((0, 2)), np.zeros((0, 2)))], np.zeros(
            (0, 2)
        )

    box3ds_reproj = project_3dbbox_to2d(
        box3ds=multi_points,
        rvec=rvec,
        tvec=tvec,
        ncm1=ncm1,
        width=width,
        height=height,
        recordfile_debuginfo=recordfile_debuginfo,
    )

    n_clusters = int(len(multi_points) / 8)
    integrated_retults_2d3d = integrate_results2d3d(
        yoloresult_whole=yoloresult_whole,
        box3ds_reproj=box3ds_reproj,
        width=width,
        height=height,
        n_clusters=n_clusters,
        defaultval=defaultval,
        linkmethod=linkmethod,
        comparemode=comparemode,
        recordfile_debuginfo=recordfile_debuginfo,
        integrated_retults_2d3d=integrated_retults_2d3d_old,
    )
    return integrated_retults_2d3d, box3ds_reproj
