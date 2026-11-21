"""Hand tracking with the MediaPipe Tasks HandLandmarker.

Input: a BGR frame and a timestamp. Output: the hand point (palm centre) in
pixels and normalised coordinates, or None. No knowledge of frame sources.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions, vision

from iris.config import REPO_ROOT

DEFAULT_MODEL = REPO_ROOT / "models" / "hand_landmarker.task"
# Wrist + the four finger-base knuckles: stable when fingers curl.
PALM_IDS = (0, 5, 9, 13, 17)


@dataclass
class Hand:
    point_px: Tuple[float, float]
    point_norm: Tuple[float, float]
    landmarks_px: List[Tuple[float, float]]
    handedness: str
    score: float


class HandTracker:
    def __init__(self, model_path: Path = DEFAULT_MODEL, num_hands: int = 1,
                 min_detection: float = 0.5, min_presence: float = 0.5,
                 min_tracking: float = 0.5) -> None:
        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"{model_path} missing; see README for the download command")
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_buffer=model_path.read_bytes()),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=num_hands,
            min_hand_detection_confidence=min_detection,
            min_hand_presence_confidence=min_presence,
            min_tracking_confidence=min_tracking,
        )
        self._lm = vision.HandLandmarker.create_from_options(options)
        self._last_ts = -1

    def detect(self, frame_bgr, timestamp_ms: int) -> Optional[Hand]:
        ts = max(int(timestamp_ms), self._last_ts + 1)  # must strictly increase
        self._last_ts = ts
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = self._lm.detect_for_video(image, ts)
        if not res.hand_landmarks:
            return None
        scores = [h[0].score if h else 0.0 for h in res.handedness]
        i = max(range(len(res.hand_landmarks)), key=lambda k: scores[k] if k < len(scores) else 0)
        lms = res.hand_landmarks[i]
        h, w = frame_bgr.shape[:2]
        nx = sum(lms[k].x for k in PALM_IDS) / len(PALM_IDS)
        ny = sum(lms[k].y for k in PALM_IDS) / len(PALM_IDS)
        label = res.handedness[i][0].category_name if i < len(res.handedness) else "?"
        return Hand(
            point_px=(nx * w, ny * h),
            point_norm=(nx, ny),
            landmarks_px=[(p.x * w, p.y * h) for p in lms],
            handedness=label,
            score=scores[i] if i < len(scores) else 0.0,
        )

    def close(self) -> None:
        self._lm.close()
