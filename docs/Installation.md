# Installation

## 1. mamba 환경 설정

이 패키지는 [RoboStack](https://robostack.github.io/)을 통해 ROS2와
PyTorch 기반 딥러닝 스택을 하나의 conda 환경 안에서 함께 사용합니다.

### Environment

- Ubuntu 24.04
- Anaconda(또는 Miniconda)가 이미 설치되어 있다고 가정합니다.

### 1-1. mamba 설치

`base` 환경에 패키지 설치/관리 속도를 높여주는 `mamba`를 설치합니다.

```bash
conda install -n base -c conda-forge mamba
```

### 1-2. conda 채널 설정

RoboStack에서 제공하는 ROS2 패키지를 받을 수 있도록 채널을 등록하고,
채널 우선순위를 `strict`로 설정합니다.

```bash
conda config --add channels conda-forge
conda config --add channels robostack-jazzy
conda config --set channel_priority strict
```

### 1-3. ROS2 Jazzy 환경 생성

`voxelnext_ros_jazzy`라는 이름의 conda 환경을 생성하고, 이 환경 안에
ROS2 Jazzy(Desktop)와 Python 3.12를 함께 설치합니다.

```bash
mamba create -n voxelnext_ros_jazzy ros-jazzy-desktop python=3.12
```

### 1-4. colcon 설치

생성한 환경을 활성화한 뒤, 워크스페이스 빌드에 필요한 colcon을
**같은 conda 환경 안에** 설치합니다.

```bash
conda activate voxelnext_ros_jazzy
mamba install -c conda-forge colcon-core colcon-common-extensions
```

**주의:** 이후 모든 `pip install` / `colcon build` / `ros2 run` / `ros2 launch` 명령은
`voxelnext_ros_jazzy` 환경이 활성화된 상태에서 실행합니다.

### 1-5. 활성화 함수 등록

매번 `conda activate` 및 ROS2 환경 설정을 반복하지 않도록, `~/.bashrc`에
아래 함수를 추가해두면 편리합니다.

```bash
conda_jazzy() {
	conda activate voxelnext_ros_jazzy
	source $CONDA_PREFIX/setup.bash

	autoload -U +X compinit && compinit
	source $CONDA_PREFIX/share/ros2cli/environment/ros2-argcomplete.bash

	echo "ROS2 Jazzy activated via Conda"
}
```

```bash
source ~/.bashrc
```

이후로는 `conda activate voxelnext_ros_jazzy` 대신 아래 명령으로 환경을
활성화합니다.

```bash
conda_jazzy
```

---

## 2. OpenPCDet 설정

이 프로젝트는 KRISO 데이터셋용 코드가 추가된 [OpenPCDet](https://github.com/neporez/OpenPCDet)를
사용합니다. 아래는 **CUDA 12.6 / PyTorch 2.6.0** 환경에서 검증된 설치 순서입니다.

### 2-1. 저장소 clone

```bash
cd ~
git clone https://github.com/neporez/OpenPCDet.git
```

### 2-2. PyTorch 및 의존성 설치

```bash
pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 \
  --index-url https://download.pytorch.org/whl/cu126
pip install spconv-cu126
pip install numpy==2.1.3 llvmlite==0.44.0 numba==0.61.0 tensorboardX==2.6.5 \
  easydict==1.13 scikit-image==0.25.2 tqdm==4.67.3 SharedArray==3.2.4 \
  opencv-python==4.13.0 pyquaternion==0.9.9 open3d==0.19.0
```

**주의:** 다른 CUDA 버전을 사용하는 경우 `torch`/`spconv`는 사용 환경에 맞는 빌드로 교체해야 합니다.
상세한 설치 옵션 및 문제 해결은 [OpenPCDet 공식 Installation 문서](https://github.com/open-mmlab/OpenPCDet/blob/master/docs/INSTALL.md)를 참고하세요.

### 2-3. OpenPCDet 설치

```bash
cd ~/OpenPCDet
pip install -e . --no-build-isolation
```

---

## 3. 워크스페이스 빌드

**주의:** `rain_usv_perception`은 `gnss_msgs/msg/Pose` 메시지 타입에
의존합니다. 워크스페이스에 `gnss_msgs`를 포함한 패키지가 미리 빌드되어 있어야 합니다.

### 3-1. 워크스페이스 생성

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3-2. rain_usv_perception 배치

```bash
git clone https://github.com/neporez/rain_usv_perception.git
```

### 3-3. 빌드

```bash
cd ~/ros2_ws
conda_jazzy
colcon build --symlink-install
source install/setup.bash
```
---

## 4. Model checkpoints

이 저장소는 학습된 가중치 파일(`.pth`)을 포함하지 않습니다.

- 다운로드: **[링크 추가 예정]**
- 저장 경로: `rain_usv_perception/lidar_perception_detection/lidar_perception_detection/cfgs/ckpt/`

빌드 및 환경 설정이 끝난 뒤, 이 파일들을 위 경로에 위치시켜야
`detection_node`가 정상 동작합니다.

---

<!-- 이후 섹션: 패키지 설명(bringup / detection / tracking / interfaces / viz),
     tracker 라이브러리 설정, ckpt 다운로드 등은 추후 작성 -->