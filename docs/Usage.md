# 실행 가이드 (Usage)

설치가 끝난 PC에서 라이다 탐지·추적 패키지를 실행하는 방법을 상황별로 정리한다. 모든 명령은 **터미널마다 먼저**
환경 파일을 읽은 뒤 실행한다 (저장소 위치는 설치한 곳에 따라 다르다).

```bash
source <저장소 경로>/env.sh        # 예: source ~/ros2_ws/src/lidar_detection/rain_usv_perception/env.sh
```
`env.sh`는 ROS2, venv, 워크스페이스, CUDA 환경과 큰 점군 메시지 유실을 막는 `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`를 설정한다.
이 파일을 읽지 않은 터미널에서 띄운 노드(라이다 드라이버 포함)는 점군을 제대로 받지 못할 수 있다.

---

## 1. 입력이 무엇인지 먼저 확인한다

라이다 드라이버나 bag 재생이 돌고 있는 상태에서:

```bash
ros2 topic list | grep -E "lidar|gnss|fog"
```

| 보이는 토픽 | 입력 종류 | launch 인자 |
|---|---|---|
| `/dev/lidar/points` (sensor_msgs/PointCloud2) | 점군 | (없음, 기본값) |
| `/dev/lidar/lidar_packets` (ouster_sensor_msgs/PacketMsg) 만 있음 | 원시 패킷 | `input:=packets` |
| 패킷 + `/dev/lidar/metadata` 있음 | 원시 패킷, metadata 자동 수신 | `input:=packets` |
| 패킷만 있고 `/dev/lidar/metadata` 없음 (중간을 잘라낸 bag 등) | 원시 패킷, metadata 파일 필요 | `input:=packets ouster_metadata_path:=<json 절대경로>` |
| `/dev/gnss/hemisphere/pose` 또는 `/system/gnss/pose` (10 Hz) | 추적용 자선 pose (기본값) | - |
| `/dev/spatial_fog_dual/pose` (100 Hz) 만 있음 | 추적용 pose 를 이 토픽으로 | `tracker.yaml` 의 `gnss_topic` 수정 |

토픽 이름이 위와 다르면 `lidar_perception_bringup/config/*.yaml` 의 토픽 파라미터를 고친다 (3절).

**bag 재생인지 실센서인지**도 다르다. bag 이면 노드 시계를 bag 시각에 맞춘다: `ros2 bag play <bag> --clock` 으로 재생하고
launch 에 `use_sim_time:=true` 를 붙인다. 실센서면 둘 다 쓰지 않는다.

## 2. 실행 명령

| 상황 | 터미널 1 (노드) | 터미널 2 |
|---|---|---|
| 실센서, 드라이버가 PointCloud2 발행 | `ros2 launch lidar_perception_bringup perception_viz.launch.py` | - |
| 실센서, 드라이버가 패킷만 발행 | `ros2 launch lidar_perception_bringup perception_viz.launch.py input:=packets` | - |
| bag (PointCloud2 기록) | `ros2 launch lidar_perception_bringup perception_viz.launch.py use_sim_time:=true` | `ros2 bag play <bag> --clock` |
| bag (패킷 기록, 처음부터 녹화돼 metadata 포함) | `... perception_viz.launch.py input:=packets use_sim_time:=true` | `ros2 bag play <bag> --clock` |
| bag (패킷 기록, metadata 없음) | `... perception_viz.launch.py input:=packets use_sim_time:=true ouster_metadata_path:=<json>` | `ros2 bag play <bag> --clock` |

- `perception_viz` = 탐지 + 추적 + RViz. 추적 없이 탐지만 보려면 `detection_viz.launch.py`(초록 상자 + 라벨), 화면 없이 돌리려면
  `perception.launch.py` / `detection_only.launch.py` 를 같은 인자로 쓴다.
- bag 은 `--loop` 로 반복 재생할 수 있다. 노드가 먼저 떠서 "Model loaded." 가 찍힌 뒤 재생을 시작하는 편이 깔끔하다 (모델 로드 5~10 s,
  설치 직후 첫 실행은 spconv JIT 캐시 때문에 1~2 분).
- 종료: 터미널 1 에서 Ctrl-C (launch 가 노드를 모두 끈다. 로그에 "process has died ... exit code -2" 가 찍히는 것은 Ctrl-C 로 끝났다는 뜻이다).

metadata JSON 이 필요한 bag 이면 원본 bag 에서 한 번 뽑아 둔다:
```bash
ros2 run lidar_perception_detection extract_ouster_metadata <원본 bag 폴더> -o lidar_metadata.json
```

## 3. 토픽·파라미터 바꾸기

`lidar_perception_bringup/config/` 의 yaml 을 고치면 된다 (symlink 설치라 재빌드 없이 다음 launch 부터 적용).

| 파일 | 자주 바꾸는 것 |
|---|---|
| `detection.yaml` | `pointcloud_topic`(기본 `/dev/lidar/points`), `pointcloud_qos_reliability`(드라이버가 best_effort 면 best_effort), `ckpt_path`(모델 교체), `score_threshold`(기본 0.3; 오탐이 많으면 0.4~0.5) |
| `ouster_packets.yaml` | `packets_topic`, `metadata_topic`, `metadata_path`(launch 인자 대신 고정하려면 여기) |
| `tracker.yaml` | `gnss_topic`(10 Hz 이상 pose 토픽), `pose_max_age_sec`, `heading_convention`(pose yaw 가 진북 기준 시계방향이면 `ned`) |
| `viz.yaml` | 마커 색·선 두께·라벨 |

출력 토픽: 탐지 `/detection/objects` (Object3DArray, 9자유도 상자), 추적 `/alg1/dl_pst_detection` (detect_msgs/ObjectsDetected),
시각화 `/tracking/objects_viz`, `/viz/tracks`, `/viz/detections`.

## 4. 동작 확인

```bash
ros2 topic hz /dev/lidar/points              # 10 Hz 전후. 1 Hz 안팎이면 5절의 LARGE_DATA 항목
ros2 topic hz /detection/objects             # 10 Hz 전후 (GPU 가 따라오는지)
ros2 topic echo /detection/objects --once    # center / size / yaw / roll / pitch / score
python3 <저장소>/tools/replay_check.py --seconds 60 --expect-tracking   # 60 초 수신 통계 요약, RESULT: OK 면 정상
```
RViz 에서는 Fixed Frame 이 `os_lidar` 인지, PointCloud2 표시의 Topic 이 `/dev/lidar/points` 이고 Reliability 가 발행자와 맞는지
(패킷 노드 / Ouster 드라이버 = Best Effort) 확인한다. launch 가 같이 띄우는 RViz 는 이 설정이 들어 있다.

## 5. 안 될 때

| 증상 | 원인 / 조치 |
|---|---|
| 아무것도 안 나오고 터미널에 `no sensor metadata yet ... dropping packets` 반복 | 패킷 입력인데 metadata 가 없다. 처음부터 녹화된 bag/실센서가 아니면 `ouster_metadata_path:=` 로 JSON 을 넘긴다 |
| `No GNSS pose within 0.5s of the LiDAR stamp ... skipping` 반복 | bag 재생인데 `use_sim_time:=true` 또는 `--clock` 이 빠졌거나, pose 토픽이 없다/1 Hz 뿐이다. 탐지는 되고 추적만 멈춘다 |
| 점군이 1 Hz 안팎, 탐지도 드물게 | `env.sh` 를 읽지 않은 터미널에서 띄움 (Fast DDS 3 의 큰 메시지 유실). 모든 노드를 `env.sh` 읽은 터미널에서 다시 띄운다 |
| `Model loaded.` 전에 수 분 걸림 | 설치 후 첫 실행의 spconv JIT 컴파일. 한 번만 기다리면 된다 |
| RViz 에 점군이 없고 오른쪽 상태가 빨간색 | PointCloud2 display 의 Topic/Reliability 또는 Fixed Frame (`os_lidar`) 확인 |
| `Not updated weight ...` 로그 뒤 이상한 상자 | 설정과 체크포인트 불일치. `kriso_voxelnext_deploy.yaml` + Release 체크포인트 조합인지 확인 |
| `no kernel image is available` | 이 PC 의 GPU 로 빌드되지 않음 (venv 를 다른 PC 에서 복사해 옴). 이 PC 에서 `docs/installation.sh` 를 다시 실행 |
