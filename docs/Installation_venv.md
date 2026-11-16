# Installation (Ubuntu 26.04, venv only — no conda, no Docker)

`docs/Installation.md`는 RoboStack(conda) 기준, `docker/Dockerfile`은 컨테이너 기준 설치 절차입니다.
이 문서는 **배포 PC에 conda와 Docker 없이** 설치하는 절차입니다. 구성은 다음과 같습니다.

- ROS2 Lyrical: Ubuntu 26.04 공식 apt 패키지 (시스템 Python 3.14)
- 딥러닝 스택(PyTorch, spconv, OpenPCDet): 시스템 ROS 패키지를 볼 수 있는 **venv** (`--system-site-packages`)
- colcon: **venv 안에** pip로 설치 (apt colcon을 쓰면 노드가 시스템 python으로 빌드되어 torch를 찾지 못합니다)

절차의 CUDA / PyTorch / spconv 부분은 `docker/Dockerfile`(Ubuntu 26.04 + CUDA 13.1 + PyTorch 2.12.0에서 검증)과 같고,
`pip3 install --break-system-packages` 자리를 venv의 `pip`로 바꾼 것입니다.

> 검증 상태: 이 문서의 venv 절차는 Dockerfile의 검증된 명령을 옮긴 것입니다. 배포 PC에서 처음 수행할 때
> 문제가 생기면 Dockerfile의 해당 단계와 비교하세요.

---

## 0. 사전 조건

- Ubuntu 26.04 x86_64, NVIDIA GPU, **NVIDIA 드라이버 580 이상**(CUDA 13 지원). 확인:
  ```bash
  nvidia-smi                      # Driver Version >= 580, CUDA Version 13.x
  nvidia-smi --query-gpu=name,compute_cap --format=csv   # 뒤에서 ARCH 지정에 사용 (예: 8.9)
  ```
- 기본 도구:
  ```bash
  sudo apt update
  sudo apt install -y git curl gnupg lsb-release ca-certificates build-essential cmake ninja-build \
      python3-venv python3-dev python3-pip
  ```

## 1. ROS2 Lyrical (apt)

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-lyrical-desktop ros-dev-tools
```

`ros-dev-tools`는 rosdep 등 보조 도구용입니다. **apt의 `python3-colcon-*`는 설치하지 않습니다**(venv에 설치).

## 2. venv 생성

```bash
python3 -m venv --system-site-packages ~/venv/perception
source ~/venv/perception/bin/activate
pip install -U pip setuptools wheel
pip install colcon-common-extensions
```

`--system-site-packages`가 있어야 venv 안에서 `rclpy`, `sensor_msgs` 등 apt로 설치된 ROS 파이썬 패키지가 보입니다.

`~/.bashrc`에 활성화 함수를 추가해 두면 편합니다.

```bash
perception_env() {
    source /opt/ros/lyrical/setup.bash
    source ~/venv/perception/bin/activate
    [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
    export CUDA_HOME=/usr/local/cuda-13.1
    export PATH=$CUDA_HOME/bin:$PATH
    export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
    echo "ROS2 Lyrical + perception venv activated"
}
```

이후 모든 `pip install` / `colcon build` / `ros2 launch`는 `perception_env`를 실행한 셸에서 수행합니다.

## 3. CUDA Toolkit 13.1

```bash
curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2604/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb
sudo dpkg -i /tmp/cuda-keyring.deb
sudo apt update
sudo apt install -y cuda-toolkit-13-1
```

glibc 2.43과 nvcc 13.1의 `rsqrt` 예외 명세 충돌 패치(Dockerfile과 동일):

```bash
sudo sed -i \
  -e 's/double *rsqrt(double x);/double rsqrt(double x) noexcept (true);/' \
  -e 's/float *rsqrtf(float x);/float rsqrtf(float x) noexcept (true);/' \
  /usr/local/cuda-13.1/targets/x86_64-linux/include/crt/math_functions.h
```

환경 변수는 2절의 `perception_env` 함수에 포함되어 있습니다. 새 셸에서 `perception_env` 후 `nvcc --version`으로 확인합니다.

## 4. PyTorch 및 파이썬 의존성 (venv)

```bash
perception_env
pip install torch==2.12.0 torchvision==0.27.0 --index-url https://download.pytorch.org/whl/cu130
pip install numpy==2.4.6 llvmlite==0.48.0 numba==0.66.0 tensorboardX==2.6.5 \
    easydict==1.13 scikit-image tqdm==4.67.3 SharedArray==3.2.4 \
    opencv-python pyquaternion pyyaml
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"   # True 이어야 함
```

## 5. GPU 아키텍처 지정

빌드 머신에 GPU가 있으면 자동 감지되지만, 명시해 두면 빌드가 재현됩니다. 0절에서 확인한 `compute_cap`을 넣습니다
(예: RTX 4090 = 8.9, RTX 3050 = 8.6, RTX 2080 / Quadro RTX 8000 = 7.5). 여러 개는 `;`로 구분.

```bash
export TORCH_CUDA_ARCH_LIST="8.9"
export CUMM_CUDA_ARCH_LIST="8.9"
```

(`perception_env` 함수에 넣어 두면 재빌드 때도 같은 값이 쓰입니다.)

## 6. cumm / spconv 소스 빌드

```bash
cd ~
git clone https://github.com/FindDefinition/cumm && cd cumm && git checkout v0.7.11 && pip install -e . && cd ~

git clone https://github.com/traveller59/spconv
sed -i 's/"pccm>=0.4.16", "cumm>=0.7.11"/"pccm>=0.4.16"/' ~/spconv/pyproject.toml
cd ~/spconv && pip install -e . && cd ~

# pccm 기본 C++ 표준 c++14 -> c++17 (CUDA 13.x libcu++ 요구)
PCCM_PATH=$(python -c "import pccm, os; print(os.path.dirname(pccm.__file__))")
sed -i 's/std="c++14",/std="c++17",/' "$PCCM_PATH/builder/pybind.py"
sed -i 's/std: Optional\[str\] = "c++14",/std: Optional[str] = "c++17",/' "$PCCM_PATH/extension.py"

# JIT 캐시 초기화 (c++17 패치와 ARCH 설정 반영)
rm -rf ~/spconv/spconv/build ~/cumm/cumm/build
python -c "import spconv.pytorch as s; print('spconv ok')"   # 첫 import 때 JIT 컴파일 (수 분 소요)
```

## 7. OpenPCDet (KRISO fork)

```bash
cd ~
git clone https://github.com/neporez/OpenPCDet.git
cd OpenPCDet
pip install -e . --no-build-isolation            # CUDA ops 컴파일 (수 분 소요)
python -c "import pcdet; from pcdet.ops.iou3d_nms import iou3d_nms_cuda; print('pcdet ok')"
```

## 8. 워크스페이스 빌드

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b ros-lyrical https://github.com/neporez/rain_usv_perception.git
cd ~/ros2_ws
perception_env
colcon build --symlink-install
source install/setup.bash
```

주의: `gnss_msgs` 등 메시지 패키지는 저장소의 `ros2_message_packages/`에 포함되어 있습니다. 같은 이름의 패키지가
워크스페이스에 또 있으면 colcon이 중복 오류를 내므로 한쪽에 `COLCON_IGNORE` 파일을 둡니다.
venv 폴더를 워크스페이스 안에 만들었다면 그 폴더에도 `COLCON_IGNORE`를 둡니다.

## 9. 모델 설정과 체크포인트

- 설정: `lidar_perception_detection/lidar_perception_detection/cfgs/kriso_models/kriso_voxelnext_deploy.yaml`
  (추론 전용, 저장소에 포함). 학습 설정(OpenPCDet `tools/cfgs/kriso_models/kriso_voxelnext.yaml`)과 입력 범위·복셀
  크기·모델 구조가 같아야 하며, 체크포인트를 바꿀 때 함께 확인합니다.
- 체크포인트(`.pth`)는 git에 없고 **GitHub Release**([v1.0.0](https://github.com/neporez/rain_usv_perception/releases/tag/v1.0.0))에 있습니다.
  두 모델이 제공되며 설정 파일은 둘 다 `kriso_voxelnext_deploy.yaml`입니다.

  | 파일 | 학습 데이터 | 용도 |
  |---|---|---|
  | `voxelnext_kriso_real_epoch_80.pth` | 실측 2,551 프레임 | **기본값** |
  | `voxelnext_kriso_real_sim2000_epoch_80.pth` | 실측 2,551 + 가상 2,000 프레임 | 대형 선박(40 m 초과) 재현율 향상 버전 |

  ```bash
  CKPT_DIR=~/ros2_ws/src/rain_usv_perception/lidar_perception_detection/lidar_perception_detection/cfgs/ckpt
  cd $CKPT_DIR
  wget https://github.com/neporez/rain_usv_perception/releases/download/v1.0.0/voxelnext_kriso_real_epoch_80.pth
  wget https://github.com/neporez/rain_usv_perception/releases/download/v1.0.0/voxelnext_kriso_real_sim2000_epoch_80.pth   # 선택
  wget https://github.com/neporez/rain_usv_perception/releases/download/v1.0.0/SHA256SUMS
  sha256sum -c SHA256SUMS --ignore-missing                 # OK 가 나와야 함
  ```
  다른 모델을 쓰려면 `lidar_perception_bringup/config/detection.yaml`의 `ckpt_path`를 바꿉니다.
  (파일은 model_state만 담은 30 MB 추론용이며, 학습 재개용 optimizer 상태는 없습니다.)
- 입력 PointCloud2에는 `x`, `y`, `z`, `intensity` 필드가 있어야 하며(Ouster 기본 출력), 모델은 x, y, z만 사용합니다.
- 출력 `Object3D`는 9자유도(yaw, roll, pitch)입니다.

## 10. 실행 및 확인

```bash
perception_env
ros2 launch lidar_perception_bringup detection_only.launch.py      # 탐지만
ros2 launch lidar_perception_bringup detection_viz.launch.py       # 탐지 + RViz (GNSS 불필요)
ros2 launch lidar_perception_bringup perception.launch.py          # 탐지 + 추적
ros2 launch lidar_perception_bringup perception_viz.launch.py      # + RViz
```

다른 터미널에서:

```bash
perception_env
ros2 topic hz /dev/lidar/points          # 입력 10 Hz
ros2 topic echo /detection/objects --once
```

## 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `ModuleNotFoundError: torch` (노드 실행 시) | 노드가 시스템 python으로 빌드됨. venv 활성화 후 `pip install colcon-common-extensions`, `rm -rf build install`, 다시 `colcon build` |
| `No module named 'rclpy'` (venv 안) | venv를 `--system-site-packages` 없이 만들었음. venv를 지우고 다시 생성 |
| spconv import 시 nvcc 오류 `rsqrt` | 3절 `math_functions.h` 패치 누락 |
| `no kernel image is available` | `TORCH_CUDA_ARCH_LIST` / `CUMM_CUDA_ARCH_LIST`가 GPU와 다름. 5절 값 수정 후 6절 캐시 삭제·재빌드 |
| `Not updated weight ...` 로그 후 쓰레기 박스 | 설정과 체크포인트 불일치(드롭아웃 등). `kriso_voxelnext_deploy.yaml`과 체크포인트 조합 확인 |
| 점군 수신 안 됨 | QoS. `detection.yaml`의 `pointcloud_qos_reliability`를 발행자에 맞춤 (Ouster 드라이버 기본 best_effort) |
