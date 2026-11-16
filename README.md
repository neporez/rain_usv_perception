# rain_usv_perception

ROS2 기반 라이다 포인트클라우드 객체 인식 및 추적 패키지입니다.

---

## Installation

설치 방법은 세 가지입니다.

| 방식 | 문서 |
|---|---|
| Ubuntu 26.04 로컬, conda / Docker 없이 (venv) — **배포 기본** | [docs/Installation_venv.md](./docs/Installation_venv.md) |
| conda (RoboStack) | [docs/Installation.md](./docs/Installation.md) |
| Docker | [docker/Dockerfile](./docker/Dockerfile), `docker compose -f docker/docker-compose.yml up -d` |

---

## Architecture

```
PointCloud2 (LiDAR, 10Hz)  ◄── (input:=packets) ouster_packet_node: /dev/lidar/lidar_packets 디코드 (ouster-sdk)
        │
        ▼
┌────────────────────┐
│   detection node   │  VoxelNeXt 9-DoF (OpenPCDet, KRISO fork)
└────────────────────┘
        │  Object3DArray
        ▼
┌────────────────────┐      GNSS / INS Pose (10 Hz hemisphere, 또는 100 Hz Spatial FOG)
│   tracking node    │◄─────────────────────
└────────────────────┘  (라이다 프레임 시각으로 pose 보간)
        │  TrackedObject3DArray
        ▼
┌────────────────────┐
│  marker publisher  │  RViz 시각화
└────────────────────┘
```

## Package Overview

| 패키지 | 역할 |
|---|---|
| `lidar_perception_interfaces` | `Object3D`, `TrackedObject3D` 등 커스텀 메시지 정의 |
| `lidar_perception_detection` | PointCloud2 → VoxelNeXt 추론 → `Object3DArray` 발행; `ouster_packet_node`(원시 패킷 → PointCloud2), `extract_ouster_metadata` |
| `lidar_perception_tracking` | Detection + GNSS pose → 다중 객체 추적 → `TrackedObject3DArray` 발행 |
| `lidar_perception_viz` | Detection / Tracking 결과를 RViz `MarkerArray`로 시각화 (`detection_marker_node`, `marker_publisher_node`) |
| `lidar_perception_bringup` | launch 파일 및 파라미터 config 모음 |

모델(OpenPCDet) 및 tracker 라이브러리는 `third_party/`의 git submodule로 들어오며(`git clone --recursive`),
각 노드는 이를 라이브러리로 import하여 사용합니다.

---

## Usage

```bash
perception_env            # venv 설치 기준 (conda 설치는 lidar_perception_ros_lyrical)
source ~/ros2_ws/install/setup.bash

# Detection 노드만 실행 (입력: /dev/lidar/points PointCloud2)
ros2 launch lidar_perception_bringup detection_only.launch.py

# 입력이 Ouster 원시 패킷(/dev/lidar/lidar_packets)일 때: 모든 launch 에 input:=packets
ros2 launch lidar_perception_bringup perception.launch.py input:=packets

# bag 재생 (패킷 bag, GNSS 포함): 노드 시계를 bag 시각에 맞춘다
ros2 bag play <bag> --clock
ros2 launch lidar_perception_bringup perception_viz.launch.py input:=packets use_sim_time:=true
#   --start-offset 로 중간부터 재생하면 latched metadata 를 못 받으므로 JSON 을 뽑아 넘긴다:
#   ros2 run lidar_perception_detection extract_ouster_metadata <bag> -o lidar_metadata.json
#   ros2 launch ... input:=packets use_sim_time:=true ouster_metadata_path:=$PWD/lidar_metadata.json

# Detection + RViz 시각화 (추적 없음, GNSS 불필요)
ros2 launch lidar_perception_bringup detection_viz.launch.py

# Detection + Tracking
ros2 launch lidar_perception_bringup perception.launch.py

# Detection + Tracking + RViz 시각화
ros2 launch lidar_perception_bringup perception_viz.launch.py
```

### 주요 파라미터

파라미터는 `lidar_perception_bringup/config/*.yaml`에서 수정합니다.

| 파일 | 주요 파라미터 |
|---|---|
| `ouster_packets.yaml` | `packets_topic`, `metadata_topic`, `metadata_path`, `stamp_mode`(driver = 드라이버와 같은 시각 규약) |
| `detection.yaml` | `pointcloud_topic`, `pointcloud_qos_reliability`, `model_config_path`(기본 `cfgs/kriso_models/kriso_voxelnext_deploy.yaml`), `ckpt_path`(기본 `cfgs/ckpt/voxelnext_kriso_real_epoch_80.pth`, git 미포함), `score_threshold` |
| `tracker.yaml` | `detection_topic`, `gnss_topic`(기본 `/dev/gnss/hemisphere/pose`, 10 Hz), `pose_time_source`, `pose_max_age_sec`, `heading_convention`, `tracker_config_path` |
| `viz.yaml` | `marker_publisher_node`: `tracking_viz_topic`, `output_topic` / `detection_marker_node`: `detection_topic`, `output_topic`, `min_score`, `line_width`, `color_rgb`, `show_heading`, `show_label` |

---

## Third-Party Code & Licenses

이 저장소 자체 코드는 [LICENSE](./LICENSE)(Apache-2.0)를 따릅니다.
다음 서드파티 코드가 일부 포함되어 있으며, 각각 원 라이선스를 따릅니다.

| 위치 | 원본 | 라이선스 | 비고 |
|---|---|---|---|
| `third_party/tracker_task/` (submodule; `lidar_perception_tracking/.../tracker`는 이 안의 `tracker/`로 가는 심볼릭 링크) | [taekim26-eng/tracker_task](https://github.com/taekim26-eng/tracker_task) — CasTrack 기반 ([hailanyi/3D-Multi-Object-Tracker](https://github.com/hailanyi/3D-Multi-Object-Tracker)) + 해상용 Mahalanobis 연관 + 선박 CTRV-EKF | Apache-2.0 | |
| `third_party/OpenPCDet/` (submodule) | [neporez/OpenPCDet](https://github.com/neporez/OpenPCDet) ← [open-mmlab/OpenPCDet](https://github.com/open-mmlab/OpenPCDet) | Apache-2.0 | KRISO 데이터셋 / 9-DoF VoxelNeXt 코드 추가 |

각 디렉토리 내 `LICENSE` 파일을 참조하세요.

---
