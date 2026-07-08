# rain_usv_perception

ROS2 기반 라이다 포인트클라우드 객체 인식 및 추적 패키지입니다.

---

## Installation

자세한 설치 과정은 [docs/Installation.md](./docs/Installation.md)를 참고하세요.

---

## Architecture

```
PointCloud2 (LiDAR, 10Hz)
        │
        ▼
┌────────────────────┐
│   detection node   │  VoxelNeXt (OpenPCDet)
└────────────────────┘
        │  Object3DArray
        ▼
┌────────────────────┐      GNSS Pose (1Hz)
│   tracking node    │◄─────────────────────
└────────────────────┘  (등속도 외삽으로 10Hz 보간)
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
| `lidar_perception_detection` | PointCloud2 → VoxelNeXt 추론 → `Object3DArray` 발행 |
| `lidar_perception_tracking` | Detection + GNSS pose → 다중 객체 추적 → `TrackedObject3DArray` 발행 |
| `lidar_perception_viz` | Tracking 결과를 RViz `MarkerArray`로 시각화 |
| `lidar_perception_bringup` | launch 파일 및 파라미터 config 모음 |

모델(VoxelNeXt) 및 tracker 라이브러리는 ROS2 노드 코드와 분리되어 있으며,
각 노드는 이를 라이브러리로 import하여 사용합니다.

---

## Usage

```bash
conda_jazzy
source ~/ros2_ws/install/setup.bash

# Detection 노드만 실행
ros2 launch lidar_perception_bringup detection_only.launch.py

# Detection + Tracking
ros2 launch lidar_perception_bringup perception.launch.py

# Detection + Tracking + RViz 시각화
ros2 launch lidar_perception_bringup perception_viz.launch.py
```

### 주요 파라미터

파라미터는 `lidar_perception_bringup/config/*.yaml`에서 수정합니다.

| 파일 | 주요 파라미터 |
|---|---|
| `detection.yaml` | `pointcloud_topic`, `pointcloud_qos_reliability`, `model_config_path`, `ckpt_path`, `score_threshold` |
| `tracking.yaml` | `detection_topic`, `gnss_topic`, `tracker_config_path` |
| `viz.yaml` | `tracking_topic`, `detection_topic` |

---

## Third-Party Code & Licenses

이 저장소 자체 코드는 [LICENSE](./LICENSE)(Apache-2.0)를 따릅니다.
다음 서드파티 코드가 일부 포함되어 있으며, 각각 원 라이선스를 따릅니다.

| 위치 | 원본 | 라이선스 | 비고 |
|---|---|---|---|
| `lidar_perception_tracking/lidar_perception_tracking/tracker/` | [hailanyi/3D-Multi-Object-Tracker](https://github.com/hailanyi/3D-Multi-Object-Tracker) | Apache-2.0 | NumPy 2.0 호환을 위한 최소 수정 |
| [neporez/OpenPCDet](https://github.com/neporez/OpenPCDet) (별도 저장소) | [open-mmlab/OpenPCDet](https://github.com/open-mmlab/OpenPCDet) | Apache-2.0 | KRISO 데이터셋용 코드 추가 |

각 디렉토리 내 `LICENSE` 파일을 참조하세요.

---
