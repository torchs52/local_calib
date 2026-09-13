import threading

import cv2


class MultiCameraReceiver:
    def __init__(self, rtp_urls):
        assert len(rtp_urls) == 3, "3つのRTP URLが必要です"
        self.rtp_urls = rtp_urls
        self.frames = [None] * 3
        self.prev_frames = [None] * 3
        self.locks = [threading.Lock() for _ in range(3)]
        self.captures = [cv2.VideoCapture(url, cv2.CAP_FFMPEG) for url in rtp_urls]
        self.threads = []
        self.running = True  # スレッド制御用フラグ

        for i in range(3):
            t = threading.Thread(target=self._update_frame, args=(i,), daemon=True)
            t.start()
            self.threads.append(t)

    def _update_frame(self, index):
        cap = self.captures[index]
        while self.running:
            ret, frame = cap.read()
            with self.locks[index]:
                if ret:
                    self.frames[index] = frame

    def read(self):
        result = []
        for i in range(3):
            with self.locks[i]:
                frame = self.frames[i]
                if frame is not None:
                    self.prev_frames[i] = frame
                    result.append(frame)
                else:
                    result.append(self.prev_frames[i])
        if all(f is None for f in result):
            return None
        return result

    def stop(self):
        """安全に終了処理を行う"""
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)  # スレッドが終了するのを待つ（タイムアウト付き）
        for cap in self.captures:
            cap.release()


def test_main():
    rtp_urls = [
        "rtp://192.168.1.75:10750",
        "rtp://192.168.1.76:10760",
        "rtp://192.168.1.77:10770",
    ]
    receiver = MultiCameraReceiver(rtp_urls)

    try:
        while True:
            frames = receiver.read()
            if frames is not None:
                for i, frame in enumerate(frames):
                    if frame is not None:
                        cv2.imshow(f"Camera {i + 1}", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        receiver.stop()
        cv2.destroyAllWindows()
