"""
pose_recognizer.py  (ROS1)
==========================
MediaPipe Pose Landmarker 기반 몸 제스처 인증 모듈.

hand_recognizer 와 짝을 이루는 "몸 제스처" 인식기. 둘 다 MediaPipe Tasks API 로
통일하여 vision_auth FSM 의 2차 인증에서 병렬 사용한다.

설계(= hand_recognizer 와 동일 패턴):
  - MediaPipe 지연 import → 미설치 환경에서도 모듈 import 가능
  - 분류(classify_pose_gesture)는 순수 함수 → 카메라/MediaPipe 없이 단위테스트
  - recognize()/reset() 인터페이스 동일 → FSM 드롭인

MediaPipe Pose landmark 인덱스 (BlazePose 33점, 주요만):
  0  nose
  7  left_ear         8  right_ear
  11 left_shoulder    12 right_shoulder
  13 left_elbow       14 right_elbow
  15 left_wrist       16 right_wrist
  23 left_hip         24 right_hip
"""

from __future__ import annotations

import os
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
from sensor_msgs.msg import CameraInfo
import rospy


# ── 파라미터 상수 ──────────────────────────────────────────────────────── #

POSE_DETECTION_CONF = 0.6
POSE_PRESENCE_CONF  = 0.6
POSE_TRACKING_CONF  = 0.5
NUM_POSES           = 1

DEPTH_VALID_MAX_M   = 0.5
DEPTH_VALID_MIN_M   = 0.1
DEPTH_PERCENTILE    = 20
CONFIRM_FRAMES      = 5

# 인증으로 인정할 몸 제스처
VALID_GESTURES: set = {'hands_on_head'}

# landmark 인덱스
NOSE        = 0
L_EAR       = 7
R_EAR       = 8
L_SHOULDER  = 11
R_SHOULDER  = 12
L_WRIST     = 15
R_WRIST     = 16

_DEFAULT_TASK = os.path.join(
    os.path.dirname(__file__), '..', '..', 'models', 'pose_landmarker.task'
)


@dataclass
class PoseResult:
    detected:   bool
    gesture:    str   = ''
    confidence: float = 0.0
    depth_m:    float = 0.0
    num_poses:  int   = 0
    reason:     str   = ''


# ═══════════════════════════════════════════════════════════════════════ #
#  순수 함수: landmark → 제스처  (ROS/MediaPipe 비의존, 단위테스트 대상)
# ═══════════════════════════════════════════════════════════════════════ #

def classify_pose_gesture(lm: np.ndarray) -> Tuple[str, float]:
    """
    lm: (33, 3) 정규화 landmark 배열 (x, y, z), y는 위로 갈수록 작음.
    반환: (gesture, confidence[0..1]). 미분류는 ('', 0.0).

    hands_on_head(머리에 손): 양 손목이
      (1) 귀보다 위(y 작음)로 올라가 있고,
      (2) 수평(x)으로 머리 중심(코) 근처에 있어야 함.
    → 손을 높이·바깥으로 벌리는 '만세'와 구분됨.
    """
    nose_x = lm[NOSE, 0]
    ear_y  = float(lm[L_EAR, 1] + lm[R_EAR, 1]) / 2.0
    head_w = abs(float(lm[R_EAR, 0] - lm[L_EAR, 0]))
    if head_w < 1e-6:
        head_w = 0.08                     # 정규화 좌표 기준 머리 폭 근사치
    x_tol = head_w * 1.5                   # 머리 중심에서 허용하는 좌우 편차

    lw_up   = lm[L_WRIST, 1] < ear_y
    rw_up   = lm[R_WRIST, 1] < ear_y
    lw_near = abs(lm[L_WRIST, 0] - nose_x) < x_tol
    rw_near = abs(lm[R_WRIST, 0] - nose_x) < x_tol

    if lw_up and rw_up and lw_near and rw_near:
        return 'hands_on_head', 0.85

    return '', 0.0


# ═══════════════════════════════════════════════════════════════════════ #
#  N-프레임 확정 버퍼
# ═══════════════════════════════════════════════════════════════════════ #

@dataclass
class _FrameBuffer:
    maxlen: int
    buf: deque = field(default_factory=deque)

    def push(self, gesture: str) -> None:
        self.buf.append(gesture)
        if len(self.buf) > self.maxlen:
            self.buf.popleft()

    def is_confirmed(self, gesture: str) -> bool:
        return len(self.buf) == self.maxlen and all(g == gesture for g in self.buf)

    def reset(self) -> None:
        self.buf.clear()


# ═══════════════════════════════════════════════════════════════════════ #
#  MediaPipe 래퍼
# ═══════════════════════════════════════════════════════════════════════ #

class PoseRecognizer:

    def __init__(self) -> None:
        self._pose   = self._init_mediapipe()
        self._buffer = _FrameBuffer(maxlen=CONFIRM_FRAMES)
        rospy.loginfo('PoseRecognizer: MediaPipe Pose ready')

    @staticmethod
    def _init_mediapipe():
        """MediaPipe Tasks API(PoseLandmarker, IMAGE 모드) 초기화."""
        try:
            from mediapipe.tasks import python as mpp
            from mediapipe.tasks.python import vision
        except ImportError as e:  # noqa: BLE001
            raise RuntimeError(
                'mediapipe 미설치 — `pip install mediapipe` 후 사용하세요'
            ) from e

        task_path = os.environ.get('POSE_LANDMARKER_TASK', _DEFAULT_TASK)
        if not os.path.exists(task_path):
            raise RuntimeError(
                f'pose_landmarker.task 없음: {task_path}\n'
                '  → models/README.md 의 URL 에서 받아 두세요'
            )

        options = vision.PoseLandmarkerOptions(
            base_options=mpp.BaseOptions(model_asset_path=task_path),
            num_poses=NUM_POSES,
            min_pose_detection_confidence=POSE_DETECTION_CONF,
            min_pose_presence_confidence=POSE_PRESENCE_CONF,
            min_tracking_confidence=POSE_TRACKING_CONF,
            running_mode=vision.RunningMode.IMAGE,
        )
        return vision.PoseLandmarker.create_from_options(options)

    def recognize(
        self,
        color_img:   np.ndarray,
        depth_img:   np.ndarray,
        camera_info: CameraInfo,
    ) -> PoseResult:
        import cv2  # 지연 import
        import mediapipe as mp

        rgb = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res = self._pose.detect(mp_image)

        if not res.pose_landmarks:
            self._buffer.push('')
            return PoseResult(detected=False, reason='no_pose')

        h, w = color_img.shape[:2]
        best = None  # (gesture, conf, depth_m)

        for lms in res.pose_landmarks:
            lm = np.array([[p.x, p.y, p.z] for p in lms])

            gesture, conf = classify_pose_gesture(lm)
            if gesture not in VALID_GESTURES:
                continue

            depth_m = self._measure_depth(depth_img, lm, w, h)
            if depth_m is None or not (DEPTH_VALID_MIN_M <= depth_m <= DEPTH_VALID_MAX_M):
                continue

            if best is None or conf > best[1]:
                best = (gesture, conf, depth_m)

        if best is None:
            self._buffer.push('')
            return PoseResult(
                detected=False, num_poses=len(res.pose_landmarks),
                reason='no_valid_gesture',
            )

        gesture, conf, depth_m = best
        self._buffer.push(gesture)

        if self._buffer.is_confirmed(gesture):
            return PoseResult(
                detected=True, gesture=gesture, confidence=conf,
                depth_m=depth_m, num_poses=len(res.pose_landmarks),
            )

        done = len(self._buffer.buf)
        return PoseResult(
            detected=False, gesture=gesture, confidence=conf, depth_m=depth_m,
            num_poses=len(res.pose_landmarks),
            reason=f'confirming:{done}/{CONFIRM_FRAMES}',
        )

    def reset(self) -> None:
        self._buffer.reset()

    # ── depth: 상반신 landmark(어깨~손목) bbox percentile ──────────────── #

    @staticmethod
    def _measure_depth(depth_img, lm, w, h) -> Optional[float]:
        # 몸 전체 대신 상반신 핵심점만 사용 (배경 depth 혼입 최소화)
        idx = [NOSE, L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST]
        xs  = (lm[idx, 0] * w).astype(int)
        ys  = (lm[idx, 1] * h).astype(int)
        x1, x2 = max(0, xs.min()), min(w, xs.max())
        y1, y2 = max(0, ys.min()), min(h, ys.max())
        if x2 <= x1 or y2 <= y1:
            return None
        roi   = depth_img[y1:y2, x1:x2]
        valid = roi[roi > 0]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, DEPTH_PERCENTILE)) / 1000.0
