# camera_passability_in_real_world
카메라 기반 동적 장애물 인지/통과성 판단 파이프라인 모듈 패키지(ROS1) + 비전 인증 기반 적재함 개방 


# Camera Passability (Pseudo-BEV) - ROS1

## Overview
본 레포지토리는 1:5 Scale 차량 환경에서 **카메라 기반 동적&정적 객체(사람/자전거) 인지 → 3D 역투영 → base_link BEV 좌표 변환 → 통과성(Passability) 판단 → PointCloud2 + 가상 벽 발행**을 수행합니다.\n
또한 배송 로봇이 목적지에 도착한 후, 수령인을 안전하게 확인하고 적재함을 개방(Unlock)하는 역할을 수행합니다. 엣지 디바이스의 연산 자원을 최적화하기 위해 **상태 기반(State-based) 상태 머신**으로 작동합니다.

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
- 일반 실행:

## camera_passability_Pipeline Steps (Step ↔ 파일 매핑)
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

 
## 2. 비전 기반 수령인 인증 및 적재함 제어 시스템 (Vision Authentication System)

### 주요 기능 (Key Features)
1. **이중 인증 시스템 (Dual Auth):** - **1차 (QR):** WeChat QR 엔진을 활용한 빠르고 왜곡 없는 수령인 암호 해독.
   - **2차 (Gesture):** 스마트폰 부재 시, 수화 및 특정 제스처(YOLOv8)를 통한 백업 인증.
2. **공간 필터링 (Depth Constraint):** - 혼잡한 캠퍼스 환경에서의 오인식을 막기 위해, `aligned_depth`를 활용하여 **카메라 앞 0.5m 이내**의 객체만 유효한 인증으로 판별합니다.
3. **자원 최적화 (Resource Saving):**
   - 주행 중(`NAVIGATING`)에는 카메라 토픽 구독을 완전히 해제하여, 자율주행 제어부 및 로컬 플래너에 모든 컴퓨팅 자원을 양보합니다.

---

### 판단/제어팀 연동 가이드 (Interface for Control Team)

이 시스템은 제어팀의 **[로봇 상태 신호]**에 따라 켜지고, 인증이 완료되면 제어팀으로 **[개방 신호]**를 보냅니다.

#### 입력 (Subscribe)
* **`/robot_state`** (`std_msgs/String`)
  * 제어팀에서 로봇이 목적지에 도착하면 `"ARRIVED"`를 발행해 주세요. (이때 비전 인증 모듈이 켜집니다.)
  * 주행을 시작할 때는 `"NAVIGATING"`을 발행해 주세요. (이때 비전 모듈은 완전히 꺼집니다.)

#### 출력 (Publish)
* **`/cargo_unlock`** (`std_msgs/Bool`)
  * 인증이 성공하면 `True` 신호를 1초 간격으로 3회 연속 발행합니다.
  * **[제어팀 Action]:** 이 토픽이 `True`로 들어오면 CAN 통신이나 아두이노를 통해 적재함의 잠금장치를 해제해 주세요.

---

### 패키지 구조 (Package Structure)
`campus_delivery_auth` 패키지는 다음과 같이 모듈화되어 있습니다.
- `vision_auth_node.py`: 상태 머신 관리 및 제어부 통신 담당 (Main)
- `camera_sync.py`: Color와 Depth 이미지 시간 동기화
- `qr_scanner.py`: OpenCV WeChat QR 기반 1차 인증
- `gesture_recognizer.py`: YOLOv8 기반 2차 제스처 인증

## 실행 방법 (Usage)

### 동적 통과성 판단 노드 실행 & 비전 인증 시스템 노드 실행 (Launch)
```bash
rosrun camera_passability_in_real_world dynamic_passability_detector_node_ros1.py _model_path:=yolov8n.pt _conf_thresh:=0.5

rosrun <your_pkg> dynamic_passability_detector_node_ros1.py _model_path:=yolov8n.pt _conf_thresh:=0.5
