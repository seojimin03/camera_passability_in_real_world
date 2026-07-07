#!/usr/bin/env python3
"""
test_pose_recognizer_offline.py
===============================
MediaPipe/카메라/ROS 없이 몸 제스처 분류 순수 로직만 검증.
합성 landmark(33×3) 로 classify_pose_gesture 를 직접 호출.

실행:  python3 test/test_pose_recognizer_offline.py
"""

import os
import sys
import types

import numpy as np

# ── ROS 모듈 stub ──────────────────────────────────────────────────────── #
_rospy = types.ModuleType('rospy')
_rospy.loginfo = _rospy.logwarn = _rospy.logdebug = lambda *a, **k: None
sys.modules['rospy'] = _rospy
_sm     = types.ModuleType('sensor_msgs')
_sm_msg = types.ModuleType('sensor_msgs.msg')
_sm_msg.CameraInfo = object
_sm.msg = _sm_msg
sys.modules['sensor_msgs'] = _sm
sys.modules['sensor_msgs.msg'] = _sm_msg

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from campus_delivery_auth.pose_recognizer import (  # noqa: E402
    classify_pose_gesture, NOSE, L_EAR, R_EAR, L_WRIST, R_WRIST,
)


def _pose(lw, rw) -> np.ndarray:
    """머리(코·귀) 고정 + 손목 좌표만 지정한 (33,3) 배열."""
    lm = np.zeros((33, 3), dtype=float)
    lm[NOSE]  = (0.50, 0.30, 0.0)
    lm[L_EAR] = (0.45, 0.28, 0.0)     # 머리 폭 ≈ 0.10, 귀선 y ≈ 0.28
    lm[R_EAR] = (0.55, 0.28, 0.0)
    lm[L_WRIST] = (lw[0], lw[1], 0.0)
    lm[R_WRIST] = (rw[0], rw[1], 0.0)
    return lm


# 손목이 귀보다 위(y<0.28) + 머리 중심(x≈0.5) 근처
HANDS_ON_HEAD = _pose(lw=(0.46, 0.20), rw=(0.54, 0.20))
# 손을 위로 들되 좌우로 넓게 벌린 '만세' — hands_on_head 아님
MANSE_WIDE    = _pose(lw=(0.20, 0.15), rw=(0.80, 0.15))
# 양손 아래
HANDS_DOWN    = _pose(lw=(0.42, 0.70), rw=(0.58, 0.70))


def test_hands_on_head():
    g, c = classify_pose_gesture(HANDS_ON_HEAD)
    assert g == 'hands_on_head', f'got {g}'
    assert c >= 0.6
    print('  ✓ hands_on_head (머리에 손) 인식')


def test_manse_not_accepted():
    g, _ = classify_pose_gesture(MANSE_WIDE)
    assert g == '', f'벌린 만세는 미인정이어야 함, got {g}'
    print('  ✓ 넓게 벌린 만세는 미인정 (머리에 손과 구분)')


def test_hands_down_not_accepted():
    g, _ = classify_pose_gesture(HANDS_DOWN)
    assert g == '', f'양손 아래는 미인정이어야 함, got {g}'
    print('  ✓ 양손 내린 경우 미인정')


if __name__ == '__main__':
    print('pose_recognizer 오프라인 분류 테스트 (hands_on_head)')
    test_hands_on_head()
    test_manse_not_accepted()
    test_hands_down_not_accepted()
    print('\n✅ 전체 통과')
