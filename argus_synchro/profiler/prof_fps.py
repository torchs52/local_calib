import random
import time
from collections import deque
from pathlib import Path

import numpy as np
from argus_synchro_lib.octotree import OctoTree
from numpy.typing import NDArray

from argus_synchro.message.input_message import PointCloudData
from argus_synchro.message.scrutinizer_message import (
    AccumGroundPointsData,
    AccumPointsData,
    CameraDetectionsData,
    RemovePointsData,
)
from argus_synchro.profiler import (
    ProfInfo,
    ProfMode,
    ProfSharedReader,
    ProfSharedWriter,
)


class ProfFps:
    def __init__(
        self, name: str, maxlen: int = 30000, skip: int = 20, console: bool = False
    ) -> None:
        reader = ProfSharedReader()
        self._info: ProfInfo = reader.get()
        self._name: str = name
        self._skip: int = skip
        self._console: bool = console
        self._pcd_frames: deque[int] = deque(maxlen=maxlen)
        self._camera_frames: deque[int] = deque(maxlen=maxlen)
        self._fps: deque[float] = deque(maxlen=maxlen)
        self._proc: deque[float] = deque(maxlen=maxlen)
        self._pcd_delay: deque[float] = deque(maxlen=maxlen)
        self._camera_delay: deque[float] = deque(maxlen=maxlen)
        self._pcd_sizes: deque[int] = deque(maxlen=maxlen)
        self._remove_points_sizes: deque[int] = deque(maxlen=maxlen)
        self._accum_ground_points_sizes: deque[int] = deque(maxlen=maxlen)
        self._camera_detection_sizes: deque[int] = deque(maxlen=maxlen)
        self._points_sizes: deque[int] = deque(maxlen=maxlen)
        self._downsample_sizes: deque[int] = deque(maxlen=maxlen)
        self._octotree_sizes: deque[int] = deque(maxlen=maxlen)
        if self._info.mode == ProfMode.Fps:
            self.start = self._start
            self.enter = self._enter
            self.prof = self._prof
            self.export = self._export
        self._enter_time: float = 0
        self._prev: float = 0

    def start(self) -> None:
        pass

    def enter(self) -> None:
        pass

    def prof(
        self,
        pcd_frame: int | None = None,
        camera_frame: int | None = None,
        pcd_s_time: float | None = None,
        camera_s_time: float | None = None,
        pcd: PointCloudData | None = None,
        remove_points: RemovePointsData | None = None,
        accum_ground_points: AccumGroundPointsData | None = None,
        camera_detection: CameraDetectionsData | None = None,
        points: AccumPointsData | None = None,
        downsample: NDArray[np.float64] | None = None,
        octotree: OctoTree | None = None,
    ) -> None:
        pass

    def export(self) -> None:
        pass

    def _start(self) -> None:
        self._prev = time.perf_counter()

    def _enter(self) -> None:
        self._enter_time = time.perf_counter()

    def _prof(
        self,
        pcd_frame: int | None = None,
        camera_frame: int | None = None,
        pcd_s_time: float | None = None,
        camera_s_time: float | None = None,
        pcd: PointCloudData | None = None,
        remove_points: RemovePointsData | None = None,
        accum_ground_points: AccumGroundPointsData | None = None,
        camera_detection: CameraDetectionsData | None = None,
        points: AccumPointsData | None = None,
        downsample: NDArray[np.float64] | None = None,
        octotree: OctoTree | None = None,
    ) -> None:
        now: float = time.perf_counter()
        fps = 1 / (now - self._prev)
        proc = now - self._enter_time
        self._fps.append(fps)
        self._proc.append(proc)

        if pcd_frame is not None:
            self._pcd_frames.append(pcd_frame)
        if camera_frame is not None:
            self._camera_frames.append(camera_frame)
        if pcd_s_time is not None:
            self._pcd_delay.append(now - pcd_s_time)
        if camera_s_time is not None:
            self._camera_delay.append(now - camera_s_time)

        if pcd is not None:
            self._pcd_sizes.append(pcd.point_cloud.shape[0])
        if remove_points is not None:
            self._remove_points_sizes.append(remove_points.point_cloud.shape[0])
        if accum_ground_points is not None:
            self._accum_ground_points_sizes.append(
                accum_ground_points.point_cloud.shape[0]
            )
        if camera_detection is not None:
            self._camera_detection_sizes.append(
                sum(c for c in camera_detection.valid_detects)
            )
        if points is not None:
            self._points_sizes.append(points.point_cloud.shape[0])
        if downsample is not None:
            self._downsample_sizes.append(downsample.shape[0])
        if octotree is not None:
            if octotree.labeled_octo_nodes is None:
                self._octotree_sizes.append(0)
            else:
                self._octotree_sizes.append(
                    sum(
                        len(octonodes)
                        for octonodes in octotree.labeled_octo_nodes.values()
                    ),
                )

        if self._console:
            text: list[str] = []
            if pcd_frame is not None and pcd_s_time is not None:
                pcd_delay_str = (
                    f"{self._pcd_delay[-1]:.3f}" if len(self._pcd_delay) > 0 else "N/A"
                )
                text.append(f"PCD: {pcd_frame}, {pcd_delay_str}")
            if camera_frame is not None and camera_s_time is not None:
                camera_delay_str = (
                    f"{self._camera_delay[-1]:.3f}"
                    if len(self._camera_delay) > 0
                    else "N/A"
                )
                text.append(f"Camera: {camera_frame}, {camera_delay_str}")
            text.append(f"Fps: {fps:.3f}")
            text.append(f"Proc: {proc:0.3f}")
            print(" ,".join(text))

        self._prev = now

    def _export_graph(
        self,
        path: Path,
        fps_values: list[float],
        proc_values: list[float],
        pcd_delay_values: list[float],
        camera_delay_values: list[float],
    ) -> None:
        if (
            len(fps_values) == 0
            and len(proc_values) == 0
            and len(pcd_delay_values) == 0
            and len(camera_delay_values) == 0
        ):
            return

        try:
            import matplotlib.pyplot as plt
        except Exception as e:
            if self._console:
                print(f"Skip graph export: matplotlib import failed ({e})")
            return

        line_kwargs = {"linewidth": 0.9, "alpha": 0.95}

        fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(12, 8), sharex=True)

        fps_index = np.arange(len(fps_values))
        axes[0].plot(
            fps_index,
            fps_values,
            label="fps",
            color="tab:blue",
            **line_kwargs,
        )
        axes[0].set_ylabel("FPS")
        axes[0].set_title(f"FPS Summary ({self._name})")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="best")

        proc_index = np.arange(len(proc_values))
        axes[1].plot(
            proc_index,
            proc_values,
            label="proc[s]",
            color="tab:orange",
            **line_kwargs,
        )
        if len(pcd_delay_values) > 0:
            pcd_delay_index = np.arange(len(pcd_delay_values))
            axes[1].plot(
                pcd_delay_index,
                pcd_delay_values,
                label="pcd_delay[s]",
                color="tab:green",
                **line_kwargs,
            )
        if len(camera_delay_values) > 0:
            camera_delay_index = np.arange(len(camera_delay_values))
            axes[1].plot(
                camera_delay_index,
                camera_delay_values,
                label="camera_delay[s]",
                color="tab:red",
                **line_kwargs,
            )
        axes[1].set_xlabel("sample index")
        axes[1].set_ylabel("seconds")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc="best")

        fig.tight_layout()
        fig.savefig(path, dpi=200)
        plt.close(fig)

    def _export(self) -> None:
        if self._info.mode != ProfMode.Fps:
            return
        out_dir = Path(self._info.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f"fps_{self._name}.csv"
        graph_path = out_dir / f"fps_{self._name}.png"

        # 各データが存在するかチェック
        has_pcd_frame = len(self._pcd_frames) > 0
        has_camera_frame = len(self._camera_frames) > 0
        has_pcd_delay = len(self._pcd_delay) > 0
        has_camera_delay = len(self._camera_delay) > 0
        has_pcd = len(self._pcd_sizes) > 0
        has_remove_points = len(self._remove_points_sizes) > 0
        has_accum_ground_points = len(self._accum_ground_points_sizes) > 0
        has_camera_detection = len(self._camera_detection_sizes) > 0
        has_points = len(self._points_sizes) > 0
        has_downsample = len(self._downsample_sizes) > 0
        has_octotree = len(self._octotree_sizes) > 0

        # データ数の最大値を計算
        max_len = max(
            len(self._pcd_frames) if has_pcd_frame else 0,
            len(self._camera_frames) if has_camera_frame else 0,
            len(self._fps),
        )

        with path.open("wt") as f:
            # ヘッダー作成
            headers: list[str] = []
            if has_pcd_frame:
                headers.append("pcd_frame")
            if has_camera_frame:
                headers.append("camera_frame")
            headers.append("fps")
            headers.append("proc[s]")
            if has_pcd_delay:
                headers.append("pcd_delay[s]")
            if has_camera_delay:
                headers.append("camera_delay[s]")
            if has_pcd:
                headers.append("pcd")
            if has_remove_points:
                headers.append("remove_points")
            if has_accum_ground_points:
                headers.append("accum_ground_points")
            if has_camera_detection:
                headers.append("camera_detection")
            if has_points:
                headers.append("points")
            if has_downsample:
                headers.append("downsample")
            if has_octotree:
                headers.append("octotree")
            f.write(",".join(headers) + "\n")

            # データ行作成
            for i in range(max_len):
                row: list[str] = []
                if has_pcd_frame:
                    row.append(
                        str(self._pcd_frames[i]) if i < len(self._pcd_frames) else ""
                    )
                if has_camera_frame:
                    row.append(
                        str(self._camera_frames[i])
                        if i < len(self._camera_frames)
                        else ""
                    )
                row.append(str(self._fps[i]) if i < len(self._fps) else "")
                row.append(str(self._proc[i]) if i < len(self._proc) else "")
                if has_pcd_delay:
                    row.append(
                        str(self._pcd_delay[i]) if i < len(self._pcd_delay) else ""
                    )
                if has_camera_delay:
                    row.append(
                        str(self._camera_delay[i])
                        if i < len(self._camera_delay)
                        else ""
                    )
                if has_pcd:
                    row.append(
                        str(self._pcd_sizes[i]) if i < len(self._pcd_sizes) else ""
                    )
                if has_remove_points:
                    row.append(
                        str(self._remove_points_sizes[i])
                        if i < len(self._remove_points_sizes)
                        else ""
                    )
                if has_accum_ground_points:
                    row.append(
                        str(self._accum_ground_points_sizes[i])
                        if i < len(self._accum_ground_points_sizes)
                        else ""
                    )
                if has_camera_detection:
                    row.append(
                        str(self._camera_detection_sizes[i])
                        if i < len(self._camera_detection_sizes)
                        else ""
                    )
                if has_points:
                    row.append(
                        str(self._points_sizes[i])
                        if i < len(self._points_sizes)
                        else ""
                    )
                if has_downsample:
                    row.append(
                        str(self._downsample_sizes[i])
                        if i < len(self._downsample_sizes)
                        else ""
                    )
                if has_octotree:
                    row.append(
                        str(self._octotree_sizes[i])
                        if i < len(self._octotree_sizes)
                        else ""
                    )
                f.write(",".join(row) + "\n")

        fps_all = list(self._fps)
        proc_all = list(self._proc)
        pcd_delays_all = list(self._pcd_delay)
        camera_delays_all = list(self._camera_delay)

        pcd_delays_after_skip = pcd_delays_all[self._skip :]
        camera_delays_after_skip = camera_delays_all[self._skip :]

        self._export_graph(
            graph_path,
            fps_all,
            proc_all,
            pcd_delays_all,
            camera_delays_all,
        )

        if self._console:
            if len(pcd_delays_after_skip) > 0:
                print(
                    f"PCD: avg {sum(pcd_delays_after_skip) / len(pcd_delays_after_skip):.3f}, min {min(pcd_delays_after_skip):.3f}, max {max(pcd_delays_after_skip):.3f}"
                )
            if len(camera_delays_after_skip) > 0:
                print(
                    f"Camera: avg {sum(camera_delays_after_skip) / len(camera_delays_after_skip):.3f}, min {min(camera_delays_after_skip):.3f}, max {max(camera_delays_after_skip):.3f}"
                )


if __name__ == "__main__":
    with ProfSharedWriter() as writer:
        writer.set(ProfInfo(ProfMode.Fps, out_dir="result"))
        prof = ProfFps("")
        prof.start()
        for i in range(100):
            t = time.perf_counter()
            time.sleep(random.randrange(1, 5) * 0.02)
            prof.prof(pcd_frame=i, pcd_s_time=t)
        prof.export()
