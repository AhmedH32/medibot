"""
Non-blocking threaded UVC Camera Hardware Abstraction Layer with 30-frame circular ring buffer.
Allows retroactive high-FPS replay when a target disappears during low-FPS (2 FPS) dialogue mode.
"""

from collections import deque
import threading
import time
import logging
from typing import Deque, List, Optional, Tuple, Union
import cv2
import numpy as np

logger = logging.getLogger("MediBot.HAL.Camera")


class CameraStream:
    def __init__(
        self,
        camera_id: Union[int, str] = 0,
        width: int = 640,
        height: int = 480,
        target_fps: int = 30,
        ring_buffer_size: int = 30,
    ):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.ring_buffer_size = ring_buffer_size

        self._cap: Optional[cv2.VideoCapture] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Circular buffer of (frame, monotonic_timestamp)
        self._ring_buffer: Deque[Tuple[np.ndarray, float]] = deque(maxlen=ring_buffer_size)

    def start(self) -> None:
        if self._running:
            return

        logger.info("Opening camera source: %s", self.camera_id)
        self._cap = cv2.VideoCapture(self.camera_id)
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open camera source: {self.camera_id}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, name="CameraGrabber", daemon=True)
        self._thread.start()

    def _capture_loop(self) -> None:
        while self._running and self._cap is not None:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                now = time.perf_counter()
                with self._lock:
                    self._ring_buffer.append((frame, now))
            else:
                time.sleep(0.005)

    def inject_synthetic_frame(self, frame: np.ndarray, timestamp: float) -> None:
        """Used by unit tests to populate the ring buffer without physical webcam hardware."""
        with self._lock:
            self._ring_buffer.append((frame.copy(), timestamp))

    def read_latest(self) -> Tuple[Optional[np.ndarray], float]:
        """Returns the most recent (frame, monotonic_timestamp) without blocking."""
        with self._lock:
            if not self._ring_buffer:
                return None, 0.0
            frame, ts = self._ring_buffer[-1]
            return frame.copy(), ts

    def get_frames_since(self, since_timestamp: float, max_samples: int = 5) -> List[Tuple[np.ndarray, float]]:
        """
        Retrieves evenly spaced buffered frames captured after `since_timestamp`.
        Used to retroactively inspect what happened between low-FPS (2 FPS) YOLO ticks.
        """
        with self._lock:
            candidates = [(f.copy(), ts) for (f, ts) in self._ring_buffer if ts > since_timestamp]

        if len(candidates) <= max_samples:
            return candidates

        indices = np.linspace(0, len(candidates) - 1, max_samples, dtype=int)
        return [candidates[i] for i in indices]

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        logger.info("Camera stream closed.")