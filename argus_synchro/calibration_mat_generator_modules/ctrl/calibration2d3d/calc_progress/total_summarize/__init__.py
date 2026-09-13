# ブロック集計ファイル

import numpy as np
from numpy.typing import NDArray

dtype_totalprogress_result = tuple[
    float, dict[str, NDArray]
]  # 全体進捗, dict("id_count"→ブロックごとの進捗NDArray[np.float32], "progress_idmap"→ブロックごとの閾値適用済みNDArray[np.float32])


class total_summarize:
    def __init__(self):
        pass

    def setup(self, block_groupid, id_threshold):  # block_threshold, :不要？
        """
        block_groupid: blockの属するID -1で集計から除外する
        id_threshold: idの中で何個満たしたらOKとするか。
        idを持つブロック：全グループで条件達成ブロック数が閾値を超えなければＯＫとは言わない
        """
        # assert( len(block_threshold) == len(block_groupid))
        # data_import関連
        # self.block_threshold = block_threshold
        self.block_groupid = block_groupid
        self.id_threshold = id_threshold

    def calc_progress(self, block_progressval) -> dtype_totalprogress_result:
        """
        block_progressval: blockごとの進捗（満たした・満たしていないをboolで）
        """
        id_pvalues: NDArray[np.float32] = np.zeros(
            len(self.id_threshold), dtype=np.float32
        )
        assert len(self.block_groupid) == len(block_progressval)

        # id毎に進捗計算
        for bid, blockpval in zip(self.block_groupid, block_progressval, strict=False):
            if bid < 0:
                continue
            # if blockflag >= 1.0:
            #    id_count[bid] += 1
            id_pvalues[bid] += blockpval

        # id毎に閾値判定
        self.progress_idmap = np.zeros(len(self.id_threshold))
        for bid, (id_pval, thresh) in enumerate(
            zip(id_pvalues, self.id_threshold, strict=False)
        ):
            # if idval >= thresh:
            #    self.progress_idmap[bid] = 1
            self.progress_idmap[bid] = min(1, id_pval / thresh)

        block_details = {}
        block_details["id_count"] = id_pvalues
        block_details["progress_idmap"] = self.progress_idmap

        return float(np.sum(self.progress_idmap)) / len(
            self.progress_idmap
        ), block_details
