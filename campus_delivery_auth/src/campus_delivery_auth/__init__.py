"""
campus_delivery_auth
====================
1:5 스케일 배송 로봇용 비전 기반 수령인 인증 + 적재함 개방 패키지.

라이브러리 모듈:
  - camera_sync     : Color/Depth 동기화 on/off 래퍼
  - qr_scanner      : WeChat QR 1차 인증
  - hand_recognizer : MediaPipe 손 21점 2차 인증
  - pose_recognizer : MediaPipe 몸 33점 2차 인증
  - serial_driver   : 적재함 잠금장치 시리얼 제어

실행 노드는 scripts/ 참고 (vision_auth_node.py, cargo_actuator_node.py).
"""
