from __future__ import annotations

import logging
import threading
import time
from typing import Any

from ..interfaces import ICameraSource


class D435Camera(ICameraSource):
    """RealSense D435 color/depth capture with a shared latest-frame buffer."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        jpeg_quality: int = 80,
        serial: str | None = None,
        startup_timeout_s: float = 8.0,
    ) -> None:
        try:
            import cv2
            import numpy as np
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "D435 dependencies missing; run: pip install -r requirements.txt"
            ) from exc

        self._cv2, self._np, self._rs = cv2, np, rs
        self._jpeg_quality = max(1, min(100, int(jpeg_quality)))
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._jpeg: bytes | None = None
        self._depth: Any | None = None
        self._last_error: Exception | None = None
        self._log = logging.getLogger(__name__)

        self._pipeline = rs.pipeline()
        rs_config = rs.config()
        if serial:
            rs_config.enable_device(serial)
        rs_config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        rs_config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self._align = rs.align(rs.stream.color)
        try:
            self._pipeline.start(rs_config)
        except Exception as exc:
            raise RuntimeError(f"failed to start RealSense D435: {exc}") from exc

        self._thread = threading.Thread(target=self._capture_loop, name="d435-capture", daemon=True)
        self._thread.start()
        if not self._ready.wait(startup_timeout_s):
            self.close()
            detail = f": {self._last_error}" if self._last_error else ""
            raise RuntimeError(f"D435 did not produce a color/depth frame{detail}")

    def _capture_loop(self) -> None:
        encode_args = [self._cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        while not self._stop.is_set():
            try:
                frames = self._align.process(self._pipeline.wait_for_frames(1000))
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame:
                    continue
                color = self._np.asanyarray(color_frame.get_data())
                ok, encoded = self._cv2.imencode(".jpg", color, encode_args)
                if not ok:
                    raise RuntimeError("OpenCV failed to encode a D435 frame")
                depth = (
                    self._np.asanyarray(depth_frame.get_data()).copy()
                    if depth_frame else None
                )
                with self._lock:
                    self._jpeg = encoded.tobytes()
                    self._depth = depth
                self._ready.set()
            except Exception as exc:
                self._last_error = exc
                if not self._stop.is_set():
                    self._log.warning("D435 capture error: %s", exc)
                    time.sleep(0.1)

    def get_jpeg_frame(self) -> bytes:
        if not self._ready.wait(2.0):
            raise RuntimeError(f"D435 frame unavailable: {self._last_error or 'timeout'}")
        with self._lock:
            if self._jpeg is None:
                raise RuntimeError("D435 JPEG buffer is empty")
            return self._jpeg

    def get_depth_frame(self) -> Any | None:
        with self._lock:
            return None if self._depth is None else self._depth.copy()

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        try:
            self._pipeline.stop()
        except Exception:
            pass
        thread = getattr(self, "_thread", None)
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)

