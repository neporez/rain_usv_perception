# Installation

## 1. Conda 환경 설정

이 패키지는 [RoboStack](https://robostack.github.io/)을 통해 ROS2와
PyTorch 기반 딥러닝 스택을 하나의 conda 환경 안에서 함께 사용합니다.

### Environment

- Ubuntu 26.04
- Anaconda(또는 Miniconda)가 이미 설치되어 있다고 가정합니다.

### 1-1. conda 채널 설정

RoboStack에서 제공하는 ROS2 패키지를 받을 수 있도록 채널을 등록하고,
채널 우선순위를 `strict`로 설정합니다.

```bash
conda config --add channels https://prefix.dev/robostack-lyrical
conda config --add channels https://prefix.dev/conda-forge
conda config --remove channels defaults
conda config --set channel_priority strict
```

### 1-2. ROS2 Lyrical 환경 생성

`lidar_perception_ros_lyrical`라는 이름의 conda 환경을 생성하고, 이 환경 안에
ROS2 Lyrical(Desktop)와 Python 3.14를 함께 설치합니다.

```bash
conda create -n lidar_perception_ros_lyrical ros-lyrical-desktop python=3.14
```

### 1-3. colcon 설치

생성한 환경을 활성화한 뒤, 워크스페이스 빌드에 필요한 colcon을
**같은 conda 환경 안에** 설치합니다.

```bash
conda activate lidar_perception_ros_lyrical
conda install -c conda-forge colcon-core colcon-common-extensions
```

**주의:** 이후 모든 `pip install` / `colcon build` / `ros2 run` / `ros2 launch` / `source install/setup.bash` 명령은
`lidar_perception_ros_lyrical` 환경이 활성화된 상태에서 실행합니다.

### 1-4. 활성화 함수 등록

매번 `conda activate` 및 ROS2 환경 설정을 반복하지 않도록, `~/.bashrc`에
아래 함수를 추가해두면 편리합니다.

```bash
lidar_perception_ros_lyrical() {
	conda activate lidar_perception_ros_lyrical
	source $CONDA_PREFIX/setup.bash

	autoload -U +X compinit && compinit
	source $CONDA_PREFIX/share/ros2cli/environment/ros2-argcomplete.bash

	echo "ROS2 Lyrical activated via Conda"
}
```

```bash
source ~/.bashrc
```

이후로는 `conda activate lidar_perception_ros_lyrical` 대신 아래 명령으로 환경을
활성화합니다.

```bash
lidar_perception_ros_lyrical
```

---

## 2. OpenPCDet 설정

이 프로젝트는 KRISO 데이터셋용 코드가 추가된 [OpenPCDet](https://github.com/neporez/OpenPCDet)를
사용합니다. 아래는 **CUDA 13.0 / PyTorch 2.12.0** 환경에서 검증된 설치 순서입니다.

### 2-1. 저장소 clone

```bash
cd ~
git clone https://github.com/neporez/OpenPCDet.git
```

### 2-2. CUDA Toolkit 설치

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-13-1
```

### 2-3. CUDA 환경 설정

~/.bashrc에 다음 명령어를 추가하여 CUDA 환경 변수를 설정합니다.

```bash
export CUDA_HOME=/usr/local/cuda-13.1
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
```

OpenPCDet 빌드 시 문제가 되는 파일을 수정합니다.
```bash
sudo sed -i \
  -e 's/double *rsqrt(double x);/double rsqrt(double x) noexcept (true);/' \
  -e 's/float *rsqrtf(float x);/float rsqrtf(float x) noexcept (true);/' \
  /usr/local/cuda-13.1/targets/x86_64-linux/include/crt/math_functions.h
```
### 2-4. PyTorch 및 의존성 설치

```bash
pip install torch==2.12.0 torchvision==0.27.0 --index-url https://download.pytorch.org/whl/cu130

pip install numpy llvmlite numba tensorboardX \
  easydict scikit-image tqdm SharedArray \
  opencv-python pyquaternion
```

### 2-5. Spconv 설치
```bash
cd ~
git clone https://github.com/FindDefinition/cumm
cd ./cumm
git checkout v0.7.11
pip install -e .
cd ~

git clone https://github.com/traveller59/spconv
sed -i 's/"pccm>=0.4.16", "cumm>=0.7.11"/"pccm>=0.4.16"/' ~/spconv/pyproject.toml
cd ./spconv
pip install -e .

sed -i 's/std="c++14",/std="c++17",/' \
  "$HOME/anaconda3/envs/lidar_perception_ros_lyrical/lib/python3.14/site-packages/pccm/builder/pybind.py"

sed -i 's/std: Optional\[str\] = "c++14",/std: Optional[str] = "c++17",/' \
  "$HOME/anaconda3/envs/lidar_perception_ros_lyrical/lib/python3.14/site-packages/pccm/extension.py"

rm -rf "$HOME/spconv/spconv/build"
rm -rf "$HOME/cumm/cumm/build"
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
git clone -b ros-lyrical https://github.com/neporez/rain_usv_perception.git
```

### 3-3. 빌드

```bash
cd ~/ros2_ws
lidar_perception_ros_lyrical
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