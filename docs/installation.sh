#!/usr/bin/env bash
# =====================================================================================================================
# rain_usv_perception -- Ubuntu 26.04 / ROS2 Lyrical / venv-only installer (no conda, no Docker)
#
#   git clone --recursive -b ros-lyrical https://github.com/neporez/rain_usv_perception.git   # anywhere under <ws>/src/
#   bash <repo>/docs/installation.sh                 # asks for sudo once (CUDA apt packages); everything else is user-level
#   source <repo>/env.sh                        # in every terminal that runs the package
#
# The script is idempotent: every stage checks whether its result is already there and skips it. It never edits
# ~/.bashrc; it writes an env file (see the end) that you source when you use the package. Nothing is installed into
# the system Python: PyTorch, spconv, OpenPCDet, ouster-sdk and colcon live in the venv. The repository may live in
# any sub-folder of the workspace's src/ (e.g. <ws>/src/lidar_detection/rain_usv_perception); the workspace is the
# directory that contains that src/.
#
# Environment overrides (all optional):
#   VENV=$HOME/venv/lidar_detection     venv location
#   ROS_DISTRO=lyrical             ROS 2 distribution under /opt/ros
#   CUDA_VER=13-1                  CUDA apt package suffix; FULL_CUDA=1 installs cuda-toolkit-${CUDA_VER} instead of the minimal set
#   ARCH=8.9                       GPU compute capability for the CUDA builds (default: read from nvidia-smi)
#   SRC_DIR=$HOME/.local/src       where cumm / spconv are cloned and built (outside the workspace)
#   CKPT=all | real | sim2000 | none   which release checkpoints to download (default all = both models; detection.yaml uses real)
#   SKIP_APT=1                     skip the sudo/apt stage (CUDA already installed)
#   SKIP_WS_BUILD=1                skip colcon build
# =====================================================================================================================
set -euo pipefail
# any failing command aborts (set -e); say where, instead of exiting silently
trap 'printf "\n\033[1;31m[error] installation.sh stopped at line %s: %s\033[0m\n" "$LINENO" "$BASH_COMMAND" >&2' ERR

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${VENV:-$HOME/venv/lidar_detection}"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"
CUDA_VER="${CUDA_VER:-13-1}"
CUDA_HOME_DIR="/usr/local/cuda-${CUDA_VER/-/.}"
SRC_DIR="${SRC_DIR:-$HOME/.local/src}"
CKPT="${CKPT:-all}"
TORCH_SPEC="${TORCH_SPEC:-torch==2.12.0 torchvision==0.27.0}"
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cu130}"
PIP_DEPS="numpy==2.4.6 llvmlite==0.48.0 numba==0.66.0 tensorboardX==2.6.5 easydict==1.13 scikit-image tqdm==4.67.3 SharedArray==3.2.4 opencv-python pyquaternion pyyaml ouster-sdk"
CUDA_MIN_PKGS="cuda-nvcc-${CUDA_VER} cuda-cudart-dev-${CUDA_VER} cuda-crt-${CUDA_VER} cuda-nvrtc-dev-${CUDA_VER} cuda-profiler-api-${CUDA_VER} cuda-nvtx-${CUDA_VER} libcublas-dev-${CUDA_VER} libcusparse-dev-${CUDA_VER} libcusolver-dev-${CUDA_VER} libcurand-dev-${CUDA_VER} libcufft-dev-${CUDA_VER}"
RELEASE_URL="https://github.com/neporez/rain_usv_perception/releases/download/v1.0.0"
CKPT_DIR="$REPO/lidar_perception_detection/lidar_perception_detection/cfgs/ckpt"

log()  { printf '\n\033[1;34m[install] %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m[ok] %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31m[error] %s\033[0m\n' "$*" >&2; exit 1; }

# workspace = the directory that owns the "src" component of the repo path
WS="${WS:-}"
if [ -z "$WS" ]; then
  p="$REPO"
  while [ "$p" != "/" ]; do
    if [ "$(basename "$p")" = "src" ]; then WS="$(dirname "$p")"; break; fi
    p="$(dirname "$p")"
  done
fi
[ -n "$WS" ] || die "the repository must be inside a colcon workspace (<ws>/src/...), or set WS=<workspace>"

# ------------------------------------------------------------------------------------------------- 0. checks
log "0/8 checks  (repo $REPO, workspace $WS)"
. /etc/os-release; [ "${VERSION_ID}" = "26.04" ] || echo "[warn] tested on Ubuntu 26.04, this is ${PRETTY_NAME}"
[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] || die "ROS 2 $ROS_DISTRO not found under /opt/ros (install ros-${ROS_DISTRO}-desktop first)"
command -v nvidia-smi >/dev/null || die "nvidia-smi not found: install the NVIDIA driver (>= 580 for CUDA 13) first"
DRV="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || true)"
[ -n "$DRV" ] || die "nvidia-smi gives no driver version (output: $(nvidia-smi 2>&1 | head -3 | tr '\n' ' '))"
[ "${DRV%%.*}" -ge 580 ] 2>/dev/null || die "NVIDIA driver $DRV is too old for CUDA 13 (need >= 580)"
# compute capability: nvidia-smi 'compute_cap' (driver >= 510); otherwise from the GPU name; otherwise ARCH=<x.y> on the command line
if [ -z "${ARCH:-}" ]; then
  ARCH="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 || true)"
  case "$ARCH" in [0-9].[0-9]|[0-9][0-9].[0-9]) ;; *) ARCH="" ;; esac
fi
if [ -z "$ARCH" ]; then
  GPU_NAME="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || true)"
  case "$GPU_NAME" in
    *"RTX 20"*|*"GTX 16"*|*"T4"*|*"Quadro RTX"*|*"RTX 8000"*|*"RTX 6000"*) ARCH="7.5" ;;
    *"RTX 30"*|*"A10"*|*"A40"*|*"A2"*) ARCH="8.6" ;;
    *"RTX 40"*|*"L4"*|*"L40"*) ARCH="8.9" ;;
    *"A100"*|*"A30"*) ARCH="8.0" ;;
    *"H100"*) ARCH="9.0" ;;
  esac
  [ -n "$ARCH" ] || die "cannot detect the GPU compute capability for '$GPU_NAME': run  ARCH=8.6 bash docs/installation.sh  (RTX 20: 7.5, RTX 30: 8.6, RTX 40: 8.9)"
fi
export TORCH_CUDA_ARCH_LIST="$ARCH" CUMM_CUDA_ARCH_LIST="$ARCH"
command -v python3 >/dev/null || die "python3 missing"
python3 -c 'import venv' 2>/dev/null || die "python3-venv missing: sudo apt install python3-venv python3-dev"
ok "driver $DRV, GPU $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1), compute capability $ARCH, python $(python3 --version | cut -d' ' -f2)"
if [ -z "$(ls -A "$REPO/third_party/OpenPCDet" 2>/dev/null)" ] || [ -z "$(ls -A "$REPO/third_party/tracker_task" 2>/dev/null)" ]; then
  log "submodules missing -> git submodule update --init --recursive"; git -C "$REPO" submodule update --init --recursive
fi

# ------------------------------------------------------------------------------------------------- 1. CUDA (apt)
if [ "${SKIP_APT:-0}" = "1" ]; then
  log "1/8 CUDA apt stage skipped (SKIP_APT=1)"
elif [ -x "$CUDA_HOME_DIR/bin/nvcc" ]; then
  ok "1/8 nvcc already present: $("$CUDA_HOME_DIR/bin/nvcc" --version | tail -1)"
else
  log "1/8 CUDA ${CUDA_VER} via apt (sudo)"
  export DEBIAN_FRONTEND=noninteractive
  # unattended-upgrades may hold the dpkg lock right after boot: wait for it (up to 15 min) instead of failing
  for i in $(seq 1 90); do
    sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 || break
    [ "$i" = "1" ] && echo "[wait] dpkg lock is held (unattended-upgrades?) -- waiting"; sleep 10
  done
  # (no "grep -q" in a pipeline under pipefail: SIGPIPE to apt-cache would make the test fail)
  CAND="$(apt-cache policy "cuda-nvcc-${CUDA_VER}" 2>/dev/null | grep Candidate | grep -v '(none)' || true)"
  if [ -z "$CAND" ]; then
    log "adding the NVIDIA CUDA apt repository"
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2604/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb
    sudo apt-get -o DPkg::Lock::Timeout=600 install -y /tmp/cuda-keyring.deb; sudo apt-get -o DPkg::Lock::Timeout=600 update
  fi
  # DPkg::Lock::Timeout: wait for unattended-upgrades instead of failing on the dpkg lock
  if [ "${FULL_CUDA:-0}" = "1" ]; then sudo apt-get -o DPkg::Lock::Timeout=600 install -y "cuda-toolkit-${CUDA_VER}"
  else sudo apt-get -o DPkg::Lock::Timeout=600 install -y --no-install-recommends $CUDA_MIN_PKGS; fi
  ok "CUDA installed: $("$CUDA_HOME_DIR/bin/nvcc" --version | tail -1)"
fi
# glibc 2.43 / nvcc 13.x: rsqrt exception-specification conflict (harmless if already patched)
MF="$CUDA_HOME_DIR/targets/x86_64-linux/include/crt/math_functions.h"
if [ -f "$MF" ] && grep -qE 'double *rsqrt\(double x\);' "$MF"; then
  log "patching $MF (rsqrt noexcept)"
  sudo sed -i -e 's/double *rsqrt(double x);/double rsqrt(double x) noexcept (true);/' -e 's/float *rsqrtf(float x);/float rsqrtf(float x) noexcept (true);/' "$MF"
fi
export CUDA_HOME="$CUDA_HOME_DIR" PATH="$CUDA_HOME_DIR/bin:$PATH" LD_LIBRARY_PATH="$CUDA_HOME_DIR/lib64:${LD_LIBRARY_PATH:-}"

# ------------------------------------------------------------------------------------------------- 2. venv + colcon
log "2/8 venv $VENV"
[ -x "$VENV/bin/python" ] || python3 -m venv --system-site-packages "$VENV"
# shellcheck disable=SC1091
set +u   # setup files reference unset variables
source "$VENV/bin/activate"
set -u
# setuptools < 80: colcon-core 0.21 (symlink-install uses "setup.py develop") and torch 2.12 (< 82) require it
python -m pip install -q -U pip wheel "setuptools<80"
# with --system-site-packages pip would consider an apt colcon "already installed" -> force it into the venv so that
# colcon runs under the venv interpreter and builds our ament_python packages for it
if [ "$(command -v colcon)" != "$VENV/bin/colcon" ]; then
  pip install -q --no-cache-dir --ignore-installed colcon-common-extensions
fi
ok "colcon: $(command -v colcon)"

# ------------------------------------------------------------------------------------------------- 3. PyTorch + deps
log "3/8 PyTorch ($TORCH_SPEC, $TORCH_INDEX) + python deps"
if ! python -c "import torch, sys; sys.exit(0 if torch.__version__.startswith('${TORCH_SPEC#torch==}'.split()[0]) else 1)" 2>/dev/null; then
  pip install --no-cache-dir $TORCH_SPEC --index-url "$TORCH_INDEX"
fi
pip install -q --no-cache-dir $PIP_DEPS
python -c "import torch; assert torch.cuda.is_available(), 'torch.cuda.is_available() is False'; print('torch', torch.__version__, torch.cuda.get_device_name(0))"
ok "torch ready"

# ------------------------------------------------------------------------------------------------- 4. cumm / spconv
log "4/8 cumm + spconv from source (ARCH $ARCH) in $SRC_DIR"
mkdir -p "$SRC_DIR"
if ! python -c "import cumm" 2>/dev/null; then
  [ -d "$SRC_DIR/cumm" ] || git clone -q https://github.com/FindDefinition/cumm "$SRC_DIR/cumm"
  git -C "$SRC_DIR/cumm" checkout -q v0.7.11
  pip install --no-cache-dir -e "$SRC_DIR/cumm"
fi
if ! python -c "import spconv" 2>/dev/null; then
  [ -d "$SRC_DIR/spconv" ] || git clone -q https://github.com/traveller59/spconv "$SRC_DIR/spconv"
  sed -i 's/"pccm>=0.4.16", "cumm>=0.7.11"/"pccm>=0.4.16"/' "$SRC_DIR/spconv/pyproject.toml"
  pip install --no-cache-dir -e "$SRC_DIR/spconv"
fi
PCCM_PATH="$(python -c 'import pccm, os; print(os.path.dirname(pccm.__file__))')"
if grep -q 'std="c++14",' "$PCCM_PATH/builder/pybind.py" 2>/dev/null; then
  log "pccm: default C++ standard c++14 -> c++17 (CUDA 13 libcu++ requires it), clearing JIT caches"
  sed -i 's/std="c++14",/std="c++17",/' "$PCCM_PATH/builder/pybind.py"
  sed -i 's/std: Optional\[str\] = "c++14",/std: Optional[str] = "c++17",/' "$PCCM_PATH/extension.py"
  rm -rf "$SRC_DIR/spconv/spconv/build" "$SRC_DIR/cumm/cumm/build"
fi
python -c "import spconv.pytorch as s; print('spconv', s.__version__ if hasattr(s, '__version__') else 'ok')"
ok "spconv ready"

# ------------------------------------------------------------------------------------------------- 5. OpenPCDet
log "5/8 OpenPCDet (submodule) -> venv"
# (import torch first: the CUDA op links against torch's libc10.so, which only resolves once torch is loaded)
if ! python -c "import torch, pcdet; from pcdet.ops.iou3d_nms import iou3d_nms_cuda" 2>/dev/null; then
  pip install --no-cache-dir -e "$REPO/third_party/OpenPCDet" --no-build-isolation
fi
python -c "import torch, pcdet; from pcdet.ops.iou3d_nms import iou3d_nms_cuda; print('pcdet ok', pcdet.__file__)"
ok "pcdet ready"

# ------------------------------------------------------------------------------------------------- 6. checkpoints
log "6/8 checkpoints ($CKPT) from the GitHub release"
case "$CKPT" in
  all)     FILES="voxelnext_kriso_real_epoch_80.pth voxelnext_kriso_real_sim2000_epoch_80.pth" ;;
  real)    FILES="voxelnext_kriso_real_epoch_80.pth" ;;
  sim2000) FILES="voxelnext_kriso_real_sim2000_epoch_80.pth" ;;
  none)    FILES="" ;;
  *) die "CKPT must be all | real | sim2000 | none" ;;
esac
if [ -n "$FILES" ]; then
  mkdir -p "$CKPT_DIR"
  for F in $FILES; do
    [ -f "$CKPT_DIR/$F" ] || curl -fL --retry 3 -o "$CKPT_DIR/$F" "$RELEASE_URL/$F"
  done
  curl -fsL -o "$CKPT_DIR/SHA256SUMS" "$RELEASE_URL/SHA256SUMS"
  (cd "$CKPT_DIR" && sha256sum -c SHA256SUMS --ignore-missing)
  echo "[note] detection.yaml uses voxelnext_kriso_real_epoch_80.pth (real data only); set ckpt_path to voxelnext_kriso_real_sim2000_epoch_80.pth for the real+sim model"
fi

# ------------------------------------------------------------------------------------------------- 7. workspace build
if [ "${SKIP_WS_BUILD:-0}" = "1" ]; then
  log "7/8 colcon build skipped (SKIP_WS_BUILD=1)"
else
  log "7/8 colcon build in $WS (only this repository's packages)"
  # other packages in the workspace may already provide gnss_msgs / common_msgs / ... -> ignore our vendored copies
  for d in "$REPO"/ros2_message_packages/*/; do
    [ -f "$d/package.xml" ] || continue
    name="$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$d/package.xml" | head -1)"
    other="$(find "$WS/src" -name package.xml -not -path "$REPO/*" -not -path "*/third_party/*" -exec grep -l "<name>$name</name>" {} + 2>/dev/null | head -1)"
    if [ -n "$other" ] && [ ! -f "$d/COLCON_IGNORE" ]; then
      echo "[note] package '$name' also provided by $(dirname "$other") -> COLCON_IGNORE on our copy $d"; touch "$d/COLCON_IGNORE"
    fi
  done
  # shellcheck disable=SC1091
  set +u   # setup files reference unset variables
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  set -u
  (cd "$WS" && colcon build --symlink-install --packages-up-to lidar_perception_bringup lidar_perception_detection lidar_perception_tracking lidar_perception_viz)
  ok "workspace built"
fi

# ------------------------------------------------------------------------------------------------- 8. env file
ENVF="$REPO/env.sh"            # generated, git-ignored; a copy is kept next to the venv
cat > "$ENVF" <<EOT
# source this file in every shell that runs or builds rain_usv_perception (generated by docs/installation.sh)
source /opt/ros/$ROS_DISTRO/setup.bash
source $VENV/bin/activate
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
export CUDA_HOME=$CUDA_HOME_DIR
export PATH=\$CUDA_HOME/bin:\$PATH
export LD_LIBRARY_PATH=\$CUDA_HOME/lib64:\${LD_LIBRARY_PATH:-}
export TORCH_CUDA_ARCH_LIST="$ARCH" CUMM_CUDA_ARCH_LIST="$ARCH"
# Large point clouds on Fast DDS 3: the default 512 KB shared-memory segment makes best-effort PointCloud2 messages above
# that size fall back to fragmented UDP, where ~98 % are lost even on one PC. The profile below raises the segment to 16 MB.
# The variable is per process (only shells that source env.sh): other modules on the PC are not affected. It must be loaded
# by the PUBLISHER of the clouds -- with input:=packets that is our own ouster_packet_node; only a driver that publishes
# PointCloud2 itself needs it too. Processes without it still discover and talk to processes with it. FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA is NOT used: processes with and
# without that variable cannot discover each other. (Measured 2026-11-18, 2 MB messages: default 1/100, profile 100/100.)
export FASTRTPS_DEFAULT_PROFILES_FILE=$REPO/lidar_perception_bringup/config/fastdds_large_messages.xml
EOT
cp "$ENVF" "$VENV/perception_env.sh"
log "8/8 done. Environment file: $ENVF"
cat <<EOT

  source $ENVF                                                                  # per terminal
  ros2 launch lidar_perception_bringup perception_viz.launch.py                 # PointCloud2 input
  ros2 launch lidar_perception_bringup perception_viz.launch.py input:=packets  # Ouster raw packets

  (optional) add to ~/.bashrc:   alias perception_env='source $ENVF'
EOT
