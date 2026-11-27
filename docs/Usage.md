# 실행 가이드 (Usage)

설치가 끝난 PC에서 라이다 탐지·추적 패키지를 실행하는 방법을 상황별로 정리한다. 모든 명령은 **터미널마다 먼저**
환경 파일을 읽은 뒤 실행한다 (저장소 위치는 설치한 곳에 따라 다르다).

```bash
source <저장소 경로>/env.sh        # 예: source ~/ros2_ws/src/lidar_detection/rain_usv_perception/env.sh
```
`env.sh`는 ROS2, venv, 워크스페이스, CUDA 환경과 Fast DDS 프로파일(`FASTRTPS_DEFAULT_PROFILES_FILE`, 아래 "큰 점군 메시지" 참고)을
설정한다. transport 종류와 `ROS_DOMAIN_ID`는 ROS2 기본값 그대로이므로 다른 모듈(라이다 드라이버, GNSS, 통합 소프트웨어)과 추가 설정
없이 토픽이 오가고, `env.sh`를 읽지 않은 터미널에서도 토픽이 보인다. `FASTDDS_BUILTIN_TRANSPORTS` 같은 transport 변경은 한쪽에만
주면 서로 발견조차 못 하므로 쓰지 않는다.

---

## 0. 터미널별 순서 (테스트 bag 예시)

**터미널 1 — 노드 (탐지 + 추적 + RViz)**
```bash
source <저장소>/env.sh
ros2 launch lidar_perception_bringup perception_viz.launch.py input:=packets
```
`[lidar_detection_node-2] ... Model loaded.` 가 찍히고 RViz 가 뜨면 다음으로 (첫 실행 1~2 분, 이후 10 s).

**터미널 2 — bag 재생**
```bash
source <저장소>/env.sh
ros2 bag play <bag 폴더> --loop
```
몇 초 뒤 터미널 1 에 `frame 1: ... points` 가 찍히고 RViz 에 점군·상자가 나온다.

**터미널 3 — 확인 (선택)**
```bash
source <저장소>/env.sh
ros2 topic hz /dev/lidar/points          # 10 Hz 전후
ros2 topic hz /detection/objects         # 10 Hz 전후
python3 <저장소>/tools/replay_check.py --seconds 60 --expect-tracking    # RESULT: OK
```
종료: 터미널 2 Ctrl-C → 터미널 1 Ctrl-C (`process has died ... exit code -2` 는 Ctrl-C 로 끝났다는 정상 메시지).

실센서에 붙일 때는 터미널 2 가 없고(드라이버가 그 역할), 터미널 1 은 드라이버가 PointCloud2 를 주면 인자 없이,
패킷만 주면 `input:=packets` 로 띄운다. 어떤 인자가 필요한지는 1절로 판단한다.

## 1. 입력이 무엇인지 먼저 확인한다

라이다 드라이버나 bag 재생이 돌고 있는 상태에서:

```bash
ros2 topic list | grep -E "lidar|gnss|fog"
```

| 보이는 토픽 | 입력 종류 | launch 인자 |
|---|---|---|
| `/dev/lidar/points` (sensor_msgs/PointCloud2) | 점군 | (없음, 기본값) |
| `/dev/lidar/lidar_packets` (ouster_sensor_msgs/PacketMsg) 만 있음 | 원시 패킷 | `input:=packets` |
| 패킷만 있고 `/dev/lidar/metadata` 없음 (중간부터 재생한 bag 등) | 원시 패킷, 패키지에 저장된 metadata 사용 | `input:=packets` |
| `/dev/gnss/hemisphere/pose` 또는 `/system/gnss/pose` (10 Hz) | 추적용 자선 pose (기본값) | - |
| `/dev/spatial_fog_dual/pose` (100 Hz) 만 있음 | 추적용 pose 를 이 토픽으로 | `tracker.yaml` 의 `gnss_topic` 수정 |

토픽 이름이 위와 다르면 `lidar_perception_bringup/config/*.yaml` 의 토픽 파라미터를 고친다 (3절).

**bag 재생**은 `ros2 bag play <bag>` 그대로 재생하면 된다. 추적 노드가 자선 pose 의 시각 기준을 자동으로 맞추므로(`pose_time_source: auto`)
`--clock` / `use_sim_time:=true` 는 없어도 동작한다. 붙이면 모든 노드가 bag 시각으로 움직여 시각이 더 정확해지므로, 선택 사항이다.

## 2. 실행 명령

| 상황 | 터미널 1 (노드) | 터미널 2 |
|---|---|---|
| 실센서, 드라이버가 PointCloud2 발행 | `ros2 launch lidar_perception_bringup perception_viz.launch.py` | - |
| 실센서, 드라이버가 패킷만 발행 | `ros2 launch lidar_perception_bringup perception_viz.launch.py input:=packets` | - |
| bag (PointCloud2 기록) | `ros2 launch lidar_perception_bringup perception_viz.launch.py` | `ros2 bag play <bag>` |
| bag (패킷 기록) | `... perception_viz.launch.py input:=packets` | `ros2 bag play <bag> [--start-offset <초>]` |

(선택) bag 시각으로 정확히 맞추려면 launch 에 `use_sim_time:=true`, 재생에 `--clock` 을 함께 붙인다.

- `perception_viz` = 탐지 + 추적 + RViz. 추적 없이 탐지만 보려면 `detection_viz.launch.py`(초록 상자 + 라벨), 화면 없이 돌리려면
  `perception.launch.py` / `detection_only.launch.py` 를 같은 인자로 쓴다.
- bag 은 `--loop` 로 반복 재생할 수 있다. 노드가 먼저 떠서 "Model loaded." 가 찍힌 뒤 재생을 시작하는 편이 깔끔하다 (모델 로드 5~10 s,
  설치 직후 첫 실행은 spconv JIT 캐시 때문에 1~2 분).
- 종료: 터미널 1 에서 Ctrl-C (launch 가 노드를 모두 끈다. 로그에 "process has died ... exit code -2" 가 찍히는 것은 Ctrl-C 로 끝났다는 뜻이다).

패킷 입력의 센서 metadata(빔 각도, 모드 등)는 `lidar_perception_detection/config/ouster_metadata.json` (OS-2-128 SN 992203000509, 2048x10)
에 들어 있어 bag 을 중간부터 재생해도(latched `/dev/lidar/metadata` 를 못 받아도) 동작한다. 드라이버의 `/dev/lidar/metadata` 가 오면 그
내용이 파일보다 우선한다. 센서 본체나 모드(1024x20 등)를 바꿨는데 토픽이 없는 bag 이면 그 bag 에서 JSON 을 뽑아 파일을 교체한다:
```bash
ros2 run lidar_perception_detection extract_ouster_metadata <bag 폴더> -o lidar_perception_detection/config/ouster_metadata.json
```
(한 번만 쓰려면 `ouster_metadata_path:=<json 절대경로>` 로 넘겨도 된다.)

**bag 을 중간부터 재생.** 1 GB 단위로 나뉜 bag(`_0`, `_1`, ...)은 폴더를 주면 조각을 이어서 재생한다. 몇 초 지점인지 모를 때는
조각 번호나 시각으로 시작점을 정한다 (`--` 뒤의 인자는 `ros2 bag play` 에 그대로 전달):
```bash
ros2 run lidar_perception_detection play_bag <bag 폴더> --list            # 조각별 offset / 시각 표
ros2 run lidar_perception_detection play_bag <bag 폴더> --file 2          # 조각 _2 부터 끝까지
ros2 run lidar_perception_detection play_bag <bag 폴더> --time 12:52:30   # 그 시각부터 (PC 로컬 시간)
ros2 run lidar_perception_detection play_bag <bag 폴더> --file 2 -- --loop
```

## 3. 토픽·파라미터 바꾸기

`lidar_perception_bringup/config/` 의 yaml 을 고치면 된다 (symlink 설치라 재빌드 없이 다음 launch 부터 적용).

| 파일 | 자주 바꾸는 것 |
|---|---|
| `detection.yaml` | `pointcloud_topic`(기본 `/dev/lidar/points`), `pointcloud_qos_reliability`(드라이버가 best_effort 면 best_effort), `ckpt_path`(모델 교체), `score_threshold`(기본 0.3; 오탐이 많으면 0.4~0.5) |
| `ouster_packets.yaml` | `packets_topic`, `metadata_topic`, `metadata_path`(비우면 패키지의 `config/ouster_metadata.json`) |
| `tracker.yaml` | `gnss_topic`(10 Hz 이상 pose 토픽), `pose_max_age_sec`, `heading_convention`(pose yaw 가 진북 기준 시계방향이면 `ned`), `output_rate_hz`(ObjectsDetected 발행 주기 상한, 0 = 프레임마다) |
| `viz.yaml` | 마커 색·선 두께·라벨 |

출력 토픽: 탐지 `/detection/objects` (Object3DArray, 9자유도 상자), 추적 `/alg1/dl_pst_detection` (detect_msgs/ObjectsDetected),
시각화 `/tracking/objects_viz`, `/viz/tracks`, `/viz/detections`.

**출력 주기 제한.** 추적 결과 `/alg1/dl_pst_detection`(ObjectsDetected)는 기본으로 입력 프레임마다(10 Hz) 나간다. 받는 쪽이 더 느린 주기를
원하면 launch 인자 `output_rate_hz` 로 상한을 준다 (추적은 그대로 매 프레임 수행되고 발행만 솎아내므로 ID·속도는 연속; 시각화 토픽은 영향 없음):
```bash
ros2 launch lidar_perception_bringup perception.launch.py input:=packets output_rate_hz:=2.0       # 0.5 s 간격
ros2 launch lidar_perception_bringup perception_viz.launch.py input:=packets output_rate_hz:=1.0   # 1 Hz
```
`tracker.yaml` 의 `output_rate_hz` 로 기본값을 바꿀 수도 있다. 간격은 LiDAR 스탬프 기준이라 bag 재생(`--rate`)에서도 같은 비율로 동작한다.

**메시지 패키지 중복.** `ros2_message_packages/` 안의 메시지 패키지(`detect_msgs`, `gnss_msgs`, `ouster-sensor-msgs` 등)는 KRISO 통합
워크스페이스에 이미 같은 이름으로 있을 수 있다. `installation.sh` 는 워크스페이스에 같은 이름의 패키지가 따로 있으면 이 저장소의 복사본에
`COLCON_IGNORE` 를 만들어 건너뛰므로 충돌 없이 빌드된다. 손으로 빌드할 때 "duplicate package" 오류가 나면 같은 식으로 `touch
ros2_message_packages/<pkg>/COLCON_IGNORE` 하면 된다.

## 4. 동작 확인

```bash
ros2 topic hz /dev/lidar/points              # 10 Hz 전후 (아래 "점군 QoS" 참고)
ros2 topic hz /detection/objects             # 10 Hz 전후 (GPU 가 따라오는지)
ros2 topic echo /detection/objects --once    # center / size / yaw / roll / pitch / score
python3 <저장소>/tools/replay_check.py --seconds 60 --expect-tracking   # 60 초 수신 통계 요약, RESULT: OK 면 정상
```
RViz 에서는 Fixed Frame 이 `os_lidar` 인지, PointCloud2 표시의 Topic 이 `/dev/lidar/points` 이고 Reliability 가 발행자와 맞는지
확인한다. launch 가 같이 띄우는 RViz 는 입력에 맞는 설정(packets = Reliable, PointCloud2 = Best Effort)을 자동으로 고른다.

**큰 점군 메시지 (Fast DDS 3 공유 메모리 한계).** ROS2 Lyrical 의 Fast DDS 3 는 공유 메모리 세그먼트가 기본 512 KB 라서 그보다 큰
PointCloud2(유효 점만 담은 unorganized 점군도 2~5만 점이면 1~2 MB)는 UDP 조각 전송으로 넘어가고, best-effort 에서는 같은 PC 안에서도
98 % 가 유실된다(측정, 2 MB 메시지: 기본 1/100). 해결은 세그먼트를 키운 프로파일 `lidar_perception_bringup/config/fastdds_large_messages.xml`
(16 MB, 6 MB 점군까지 확인) 이고 `env.sh` 가 `FASTRTPS_DEFAULT_PROFILES_FILE` 로 읽어 들인다. 이 변수는 **프로세스 단위**라 `env.sh` 를
읽은 셸에서 띄운 프로세스(= 이 파이프라인의 노드)에만 적용되고, 같은 PC 의 다른 모듈은 자기 설정 그대로 돈다.
- 프로파일은 **큰 메시지를 발행하는 쪽**에만 있으면 된다(측정: 발행자만 프로파일 100/100, 구독자만 1/100). 구독 쪽(탐지 노드, RViz, 다른
  모듈, `ros2 topic hz`)은 기본 설정 그대로 받는다. 프로파일이 있는 프로세스와 없는 프로세스는 서로 발견·통신한다.
- **권장: `input:=packets`.** 드라이버는 작은 패킷(약 25 KB)만 발행하고 큰 점군은 우리 `ouster_packet_node` 가 발행하므로, 프로파일은
  우리 프로세스에만 있으면 되고 다른 모듈이나 드라이버는 아무것도 바꾸지 않는다(추가로 reliable QoS 로 발행하고, 탐지 노드와 launch 가 띄우는
  RViz 는 `pointcloud_qos:=auto` 로 reliable 구독).
- 드라이버가 직접 PointCloud2 를 발행하는 구성을 써야 할 때만 **드라이버 프로세스**에 이 변수가 필요하다(유실은 발행 측에서 생기므로 구독 측
  설정으로는 못 막는다). 드라이버를 띄우는 셸이나 서비스 유닛에 변수 한 줄을 주면 되고(그 프로세스에만 적용), 또는 드라이버를
  `use_system_default_qos:=true`(reliable) 로 띄우고 `pointcloud_qos:=reliable` 을 주는 방법도 있다. 두 방법 모두 다른 모듈에는 영향이 없다.
- bag 재생(`ros2 bag play`)도 발행자이므로 `env.sh` 를 읽은 터미널에서 돌린다. organized / unorganized 점군 모두 x, y, z 필드만 쓰므로
  그대로 호환된다.

**다른 터미널에서 토픽이 안 보일 때.** `ros2 topic list` 는 백그라운드의 ros2 daemon 을 거친다. daemon 이 다른 환경(다른
`ROS_DOMAIN_ID`, 예전 transport 환경변수)에서 먼저 떠 있으면 목록이 비거나 어긋난다. `ros2 daemon stop` 한 번 뒤 다시 실행한다.
