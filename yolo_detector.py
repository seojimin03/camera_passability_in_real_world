from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from ultralytics import YOLO

from .config import YOLO_CONF_THRESH, TARGET_CLASSES


@dataclass(frozen=True)
class Detection:
    # bbox (pixel space)
    cx: float
    cy: float
    bw: float
    bh: float
    cls: int
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float


class YoloDetector:
    """
    Step 2: YOLOv8 추론 → person/bicycle만 남김.
    """

    def __init__(self, model_path: str = "yolov8n.pt", conf_thresh: float = YOLO_CONF_THRESH):
        self._model = YOLO(model_path)
        self._conf_thresh = conf_thresh

    def detect(self, color_img: np.ndarray) -> List[Detection]:
        results = self._model.predict(
            color_img,
            conf=self._conf_thresh,
            verbose=False,
        )

        dets: List[Detection] = []
        if not results:
            return dets

        boxes = results[0].boxes
        if boxes is None:
            return dets

        for box in boxes:
            cls = int(box.cls[0])
            if cls not in TARGET_CLASSES:
                continue

            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = x2 - x1
            bh = y2 - y1
            dets.append(Detection(cx=cx, cy=cy, bw=bw, bh=bh, cls=cls, conf=conf, x1=x1, y1=y1, x2=x2, y2=y2))

        return dets

