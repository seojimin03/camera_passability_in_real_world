#!/usr/bin/env python3
"""
test_pose_recognizer_mediapipe.py
=================================
실제 MediaPipe 로 PoseRecognizer 전체 경로가 카메라 없이도 crash 없이 도는지
검증하는 헤드리스 스모크테스트. mediapipe 미설치 시 SKIP.

실행:  python3 test/test_pose_recognizer_mediapipe.py
"""

import os
import sys
import types

import numpy as np

try:
    import mediapipe  # noqa: F401
except ImportError:
    print('SKIP: mediapipe 미설치 (pip install mediapipe)')
    sys.exit(0)

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
from campus_delivery_auth.pose_recognizer import PoseRecognizer  # noqa: E402


def test_blank_frame_no_crash():
    rec = PoseRecognizer()  # 실제 MediaPipe Pose 초기화
    color = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.zeros((480, 640), dtype=np.uint16)

    res = rec.recognize(color, depth, camera_info=None)

    assert res.detected is False
    assert res.reason == 'no_pose'
    print(f'  ✓ blank 프레임 처리 정상 (detected={res.detected}, '
          f'reason={res.reason!r})')
    rec.reset()
    print('  ✓ reset() 정상')


if __name__ == '__main__':
    print('pose_recognizer MediaPipe 스모크테스트')
    test_blank_frame_no_crash()
    print('\n✅ 전체 통과 — MediaPipe Pose 경로 정상 동작')
