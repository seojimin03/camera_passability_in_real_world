"""
공통 파라미터/토픽/프레임 계약.

ROS1에서도 기존 ROS2 노드와 동일한 출력 토픽/프레임을 유지하기 위해
상수는 이 모듈에서 일괄 관리합니다.
"""

# 프레임/토픽 계약
BASE_LINK_FRAME = "base_link"

TOPIC_COLOR_COMPRESSED = "/camera/color/image_raw/compressed"
TOPIC_DEPTH_COMPRESSED = "/camera/aligned_depth_to_color/image_raw/compressedDepth"
TOPIC_CAMERA_INFO = "/camera/color/camera_info"

TOPIC_DYNAMIC_OBSTACLES = "/dynamic_obstacles/pointcloud"
TOPIC_PASSABLE = "/dynamic_obstacles/passable"


# 통과성 판단 파라미터
ROBOT_WIDTH_M = 0.4
SAFETY_MARGIN_M = 0.4
PASS_THRESHOLD_M = ROBOT_WIDTH_M + SAFETY_MARGIN_M  # 0.8m

FOV_DEG = 120.0  # base_link yaw 기준 전방 부채꼴(±60deg)
MAX_RANGE_M = 5.0

# 가상 벽 점 간격
VIRTUAL_WALL_STEP_M = 0.05


# YOLO 파라미터
YOLO_CONF_THRESH = 0.5
TARGET_CLASSES = {0, 1}  # person=0, bicycle=1 (COCO)


# Depth 파라미터
DEPTH_MIN_M = 0.3
DEPTH_MAX_M = 6.0
DEPTH_MEDIAN_RATIO = 0.2  # bbox 내 하위 percentile 사용(전경 우선)


# Step 1: 시간 동기화
SYNC_SLOP_SEC = 0.05

