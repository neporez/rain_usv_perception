# Docker Installation

`docker/` 폴더의 `Dockerfile`, `docker-compose.yml`을 사용해 컨테이너 환경에서 빌드/실행한다.
Dockerfile 은 `git clone --recursive` 로 저장소(submodule 포함)를 받고 `third_party/OpenPCDet` 에서 OpenPCDet 을 설치한다.

GPU 아키텍처: Dockerfile 은 기본으로 sm 7.5 / 8.6 / 8.9 (RTX 20·30·40 계열) 커널을 빌드한다. 다른 GPU 는
`--build-arg CUDA_ARCHS="7.5;8.6;8.9;<compute_cap>"` 으로 추가한다 (`nvidia-smi --query-gpu=compute_cap --format=csv`).
cubin 은 같은 major 안에서만 상위 호환이므로(8.6 → 8.9 가능, 8.9 → 8.6 불가) 대상 GPU 의 값이 목록에 있어야 한다.

```bash
cd docker
sudo docker build -t musv_lidar .                                  # 필요 시 --build-arg CUDA_ARCHS="..."
sudo docker compose run --rm musv_lidar bash
# 컨테이너 안에서: 체크포인트 다운로드 (GitHub Release) 후 실행
cd /ros2_ws/src/rain_usv_perception/lidar_perception_detection/lidar_perception_detection/cfgs/ckpt
wget https://github.com/neporez/rain_usv_perception/releases/download/v1.0.0/voxelnext_kriso_real_epoch_80.pth
ros2 launch lidar_perception_bringup perception_viz.launch.py          # 첫 실행 시 spconv JIT 빌드 (10~20 분)
```

## 가이드라인 문서

Docker 빌드 가이드라인(PDF, 스크린샷 포함)은 별도 납품 문서(Doxygen 문서 패키지)에 포함되어 있으며 이 저장소에는 넣지 않는다.
