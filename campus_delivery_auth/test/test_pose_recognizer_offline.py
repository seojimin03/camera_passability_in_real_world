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
    classify_pose_gesture, NOSE, L_WRIST, R_WRIST,
)


def _pose(nose_y, lw_y, rw_y) -> np.ndarray:
    lm = np.zeros((33, 3), dtype=float)
    lm[NOSE]    = (0.50, nose_y, 0.0)
    lm[L_WRIST] = (0.40, lw_y,   0.0)
    lm[R_WRIST] = (0.60, rw_y,   0.0)
    return lm


# y는 위로 갈수록 작음: 손목 y < 코 y 이면 "손을 든" 것
BOTH_UP   = _pose(nose_y=0.40, lw_y=0.20, rw_y=0.22)   # 양손 위
ONE_UP    = _pose(nose_y=0.40, lw_y=0.20, rw_y=0.60)   # 한 손만
BOTH_DOWN = _pose(nose_y=0.40, lw_y=0.70, rw_y=0.72)   # 양손 아래


def test_both_hands_up():
    g, c = classify_pose_gesture(BOTH_UP)
    assert g == 'both_hands_up', f'got {g}'
    assert c >= 0.6
    print('  ✓ both_hands_up (양손 위) 인식')


def test_one_hand_not_accepted():
    g, _ = classify_pose_gesture(ONE_UP)
    assert g == '', f'한 손은 미인정이어야 함, got {g}'
    print('  ✓ 한 손만 든 경우 미인정')


def test_both_down_not_accepted():
    g, _ = classify_pose_gesture(BOTH_DOWN)
    assert g == '', f'양손 아래는 미인정이어야 함, got {g}'
    print('  ✓ 양손 내린 경우 미인정')


if __name__ == '__main__':
    print('pose_recognizer 오프라인 분류 테스트')
    test_both_hands_up()
    test_one_hand_not_accepted()
    test_both_down_not_accepted()
    print('\n✅ 전체 통과')
