# =========================================================
# rain_usv_perception - ROS2 Lyrical + CUDA 13.1 + PyTorch
# =========================================================
FROM osrf/ros:lyrical-desktop-full

# -----------------------------------------------------------
# 1. 기본 도구 및 CUDA 저장소 등록
# -----------------------------------------------------------
RUN apt update && \
    apt install -y nano curl gnupg lsb-release ca-certificates python3-pip

RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2604/x86_64/cuda-keyring_1.1-1_all.deb \
    -o /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && \
    rm /tmp/cuda-keyring.deb && \
    apt update

# -----------------------------------------------------------
# 2. CUDA 13.1 툴킷 설치
# -----------------------------------------------------------
RUN apt install -y cuda-toolkit-13-1

# glibc 2.43 rsqrt/rsqrtf 예외 명세 충돌 패치
# 참고: https://forums.developer.nvidia.com/t/fedora-43-and-nvcc-cuda13-1-error-exception-specification-is-incompatible-rsqrt-rsqrtf/354510
RUN sed -i \
    -e 's/double *rsqrt(double x);/double rsqrt(double x) noexcept (true);/' \
    -e 's/float *rsqrtf(float x);/float rsqrtf(float x) noexcept (true);/' \
    /usr/local/cuda-13.1/targets/x86_64-linux/include/crt/math_functions.h

ENV CUDA_HOME=/usr/local/cuda-13.1
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# -----------------------------------------------------------
# 3. FastDDS host-container 통신 문제 해결
# SHM transport가 discovery 시 "같은 머신" 판별을 오작동 -> UDP 전용으로 강제
# 참고: https://github.com/eProsima/Fast-DDS/issues/2624
# -----------------------------------------------------------
COPY disable_shm.xml /root/disable_shm.xml
ENV FASTDDS_DEFAULT_PROFILES_FILE=/root/disable_shm.xml

# -----------------------------------------------------------
# 4. PyTorch 설치 (CUDA 13.0 wheel, 13.1 nvcc와 마이너 버전 호환)
# -----------------------------------------------------------
RUN pip3 install torch==2.12.0 torchvision==0.27.0 \
    --index-url https://download.pytorch.org/whl/cu130 \
    --break-system-packages

# -----------------------------------------------------------
# 5. pcdet 의존 파이썬 패키지
# -----------------------------------------------------------
RUN pip3 install \
    numpy==2.4.6 llvmlite==0.48.0 numba==0.66.0 tensorboardX==2.6.5 \
    easydict==1.13 scikit-image==0.25.2 tqdm==4.67.3 SharedArray==3.2.4 \
    opencv-python pyquaternion \
    --break-system-packages --ignore-installed

# -----------------------------------------------------------
# 6. GPU 아키텍처 타겟 명시 (필수)
# docker build 시점엔 GPU 미접근 -> 자동감지 불가, 반드시 명시해야
# 어떤 머신에서 빌드하든 동일한 결과(모든 대상 GPU 지원) 보장됨
#   sm_75 = RTX 2080 / Quadro RTX 8000 (Turing)
#   sm_89 = RTX 4090 (Ada Lovelace)
# -----------------------------------------------------------
ENV TORCH_CUDA_ARCH_LIST="7.5;8.9"
ENV CUMM_CUDA_ARCH_LIST="7.5;8.9"

# -----------------------------------------------------------
# 7. cumm 빌드
# -----------------------------------------------------------
WORKDIR /
RUN git clone https://github.com/FindDefinition/cumm && \
    cd cumm && \
    git checkout v0.7.11 && \
    pip3 install -e . --break-system-packages

# -----------------------------------------------------------
# 8. spconv 빌드
# -----------------------------------------------------------
RUN git clone https://github.com/traveller59/spconv && \
    sed -i 's/"pccm>=0.4.16", "cumm>=0.7.11"/"pccm>=0.4.16"/' spconv/pyproject.toml && \
    cd spconv && \
    pip3 install -e . --break-system-packages

# pccm 기본 C++ 표준 c++14 -> c++17 패치 (CUDA 13.x libcu++가 C++17 이상 요구)
RUN PCCM_PATH=$(python3 -c "import pccm, os; print(os.path.dirname(pccm.__file__))") && \
    sed -i 's/std="c++14",/std="c++17",/' "$PCCM_PATH/builder/pybind.py" && \
    sed -i 's/std: Optional\[str\] = "c++14",/std: Optional[str] = "c++17",/' "$PCCM_PATH/extension.py"

# JIT 빌드 캐시 초기화 (c++17 패치 및 TORCH_CUDA_ARCH_LIST 반영을 위해 필수)
RUN rm -rf /spconv/spconv/build /cumm/cumm/build

# -----------------------------------------------------------
# 9. OpenPCDet 설치 (KRISO 커스텀 fork)
# -----------------------------------------------------------
RUN git clone https://github.com/neporez/OpenPCDet && \
    cd OpenPCDet && \
    pip3 install -e . --no-build-isolation --break-system-packages

# -----------------------------------------------------------
# 10. rain_usv_perception ROS2 워크스페이스
# -----------------------------------------------------------
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone -b ros-lyrical https://github.com/neporez/rain_usv_perception.git

# 체크포인트(.pth)는 git에 없음 - 아래 중 하나로 반드시 추가할 것
#   (a) docker/checkpoints/ 에 .pth 넣고 COPY로 포함 (아래 주석 해제)
#   (b) 컨테이너 실행 후 docker cp로 주입
# COPY checkpoints/voxelnext04_epoch_160.pth \
#      /ros2_ws/src/rain_usv_perception/lidar_perception_detection/lidar_perception_detection/cfgs/ckpt/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/lyrical/setup.bash && colcon build --symlink-install"

# -----------------------------------------------------------
# 11. 대화형 셸 진입 시 환경 자동 로드
# -----------------------------------------------------------
RUN cat >> /root/.bashrc << 'EOF'
source /opt/ros/lyrical/setup.bash
source /ros2_ws/install/setup.bash
EOF

WORKDIR /ros2_ws
CMD ["bash"]
