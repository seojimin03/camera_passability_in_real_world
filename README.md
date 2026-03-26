# camera_passability_in_real_world
카메라 기반 동적 장애물 인지/통과성 판단 파이프라인 모듈 패키지(ROS1).


# Camera Passability (Pseudo-BEV) - ROS1

## Overview
본 레포지토리는 1:5 Scale 차량 환경에서 **카메라 기반 동적&정적 객체(사람/자전거) 인지 → 3D 역투영 → base_link BEV 좌표 변환 → 통과성(Passability) 판단 → PointCloud2 + 가상 벽 발행**을 수행합니다.

각 로직을 **모듈 단위로 분리**하여 관리합니다.

## Output Contract (필수)
다음 토픽/프레임 계약은 “판단/제어팀”과의 인터페이스로 유지합니다.

- `/dynamic_obstacles/pointcloud`  
  - 타입: `sensor_msgs/PointCloud2`  
  - `frame_id`: `base_link`  
  - 내용: 동적 객체 점 + (필요 시) 두 객체 사이 가상 벽 점

- `/dynamic_obstacles/passable`  
  - 타입: `std_msgs/Bool`  
  - 내용: 통과 가능 여부 (가상 벽 생성 로직과 함께 의미가 정합적)

## Pipeline Steps (Step ↔ 파일 매핑)
- Step 1. Time Synchronization + Decode  
  - `camera_passability/camera_sync.py`  
  - `/camera/color/image_raw/compressed`  
  - `/camera/aligned_depth_to_color/image_raw/compressedDepth`  
  - `/camera/color/camera_info`

- Step 2. 2D Detection (YOLOv8)  
  - `camera_passability/yolo_detector.py`  
  - COCO 기준 `person(0)`, `bicycle(1)`등 동적 객체만 필터링해서 정적 객체와 별도로 관리, 추후에 develop할 예정

- Step 3. 3D Projection (Pinhole Inverse Projection)  
  - `camera_passability/depth_projector.py`  
  - bbox 중심 픽셀 `(u,v)` + depth 기반 `Z`  
  - `camera_info`의 `fx, fy, cx, cy`로 `X,Y` 계산

- Step 4. TF Transform + FOV Filtering  
  - `camera_passability/tf_transformer.py`  
  - `camera_passability/fov_filter.py`

- Step 4-c. Passability Judgement + Virtual Wall  
  - `camera_passability/passability_judger.py`

- Step 5. Publish Results  
  - `camera_passability/obstacle_publisher.py`
  - `/dynamic_obstacles/pointcloud` + `/dynamic_obstacles/passable` 동시 발행

## Key Parameters
- 기본 임계값/상수는 `camera_passability/config.py`에 정의되어 있습니다.
- 특히:
  - `PASS_THRESHOLD_M` : 통과성 기준 거리(현재 `0.8m`)
  - `VIRTUAL_WALL_STEP_M` : 가상 벽 점 간격
  - `FOV_DEG`, `MAX_RANGE_M` : base_link 전방 부채꼴 필터

## Node Entrypoint (ROS1)
- `dynamic_passability_detector_node_ros1.py`

### 실행 예시
(패키지/경로는 ROS 워크스페이스 구성에 맞춰 조정)

- 일반 실행:
```bash
rosrun <your_pkg> dynamic_passability_detector_node_ros1.py _model_path:=yolov8n.pt _conf_thresh:=0.5
