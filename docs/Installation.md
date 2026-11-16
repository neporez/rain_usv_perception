# Installation

설치 방법은 세 가지 중 하나를 선택한다. 배포 PC(Ubuntu 26.04, ROS2 Lyrical 설치됨, NVIDIA 드라이버 580 이상)에는 **venv 방식**을 기본으로 한다.

| 방식 | 문서 | 비고 |
|---|---|---|
| **venv (conda / Docker 없음) — 배포 기본** | [Installation_venv.md](Installation_venv.md) | `git clone --recursive` → `bash docs/installation.sh` → `source env.sh` |
| Local Installation (Conda, RoboStack) | [Installation_Local.md](Installation_Local.md) | ROS2 와 딥러닝 스택을 하나의 conda 환경에 설치 |
| Docker | [Installation_Docker.md](Installation_Docker.md) | 컨테이너 환경에서 빌드/실행 (가이드라인 PDF) |

공통 사항
- 저장소는 submodule(`third_party/OpenPCDet`, `third_party/tracker_task`)을 포함하므로 **`git clone --recursive`** 로 받는다.
- 학습된 체크포인트(`.pth`)는 git 에 없고 GitHub Release(v1.0.0)에서 받는다. 각 문서의 체크포인트 절 참조.
- 입력은 PointCloud2(`/dev/lidar/points`)와 Ouster 원시 패킷(`/dev/lidar/lidar_packets`, launch 인자 `input:=packets`) 둘 다 지원한다.
