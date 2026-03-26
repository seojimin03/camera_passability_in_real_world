from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from sensor_msgs.msg import CameraInfo

from .config import DEPTH_MEDIAN_RATIO, DEPTH_MAX_M, DEPTH_MIN_M
from .yolo_detector import Detection


@dataclass(frozen=True)
class CamPoint:
    # camera-frame 3D point (m)
    X: float
    Y: float
    Z: float
    cls: int


class PinholeProjector:
    """
    Step 3: 핀홀 역투영 (pixel -> camera-frame 3D)
    """

    def robust_depth_from_bbox(
        self,
        depth_img: np.ndarray,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> Optional[float]:
        """
        bbox ROI 내 유효 depth 값들의 하위 percentile을 반환합니다.
        (배경 depth 혼입 방지: 하위 percentile=전경 우선)
        """
        if depth_img is None:
            return None

        h, w = depth_img.shape[:2]
        x1i = max(0, int(x1))
        y1i = max(0, int(y1))
        x2i = min(w, int(x2))
        y2i = min(h, int(y2))
        if x2i <= x1i or y2i <= y1i:
            return None

        roi = depth_img[y1i:y2i, x1i:x2i].astype(np.float32)

        # ROS2 코드와 동일 가정: compressedDepth는 mm 단위(16UC1)로 들어오는 경우가 많음.
        # 만약 이미 meter 단위로 들어오면 결과가 작아질 수 있으나,
        # 현재 요구사항은 기존 로직 유지이므로 동일 처리합니다.
        roi_m = roi / 1000.0

        valid = roi_m[(roi_m >= DEPTH_MIN_M) & (roi_m <= DEPTH_MAX_M)]
        if valid.size == 0:
            return None

        percentile_val = float(np.percentile(valid, DEPTH_MEDIAN_RATIO * 100.0))
        return percentile_val

    def project_to_3d(
        self,
        detections: List[Detection],
        depth_img: np.ndarray,
        info_msg: CameraInfo,
    ) -> List[CamPoint]:
        fx = float(info_msg.K[0])
        fy = float(info_msg.K[4])
        cx = float(info_msg.K[2])
        cy = float(info_msg.K[5])

        cam_points: List[CamPoint] = []
        for det in detections:
            Z = self.robust_depth_from_bbox(
                depth_img,
                det.x1,
                det.y1,
                det.x2,
                det.y2,
            )
            if Z is None:
                continue

            X = (det.cx - cx) * Z / fx
            Y = (det.cy - cy) * Z / fy
            cam_points.append(CamPoint(X=X, Y=Y, Z=Z, cls=det.cls))

        return cam_points

