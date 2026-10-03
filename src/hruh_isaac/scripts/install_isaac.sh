#!/usr/bin/env bash
# =============================================================================
#  Isaac Sim 6.1 + Isaac Lab 3.0 (+ rsl_rl, CUDA PyTorch 2.11) for the whole OS
#
#      sudo ./install_isaac.sh                 # interactive (asks for the EULA)
#      sudo ./install_isaac.sh --accept-eula   # non-interactive
#
#  Options
#      --prefix DIR     install location                    (default /opt/isaac)
#      --accept-eula    accept the NVIDIA Isaac Sim EULA without prompting
#      --remove-old     delete an old Isaac Sim 5.1 install in DIR (venv/, IsaacLab/) to free ~20 GB
#      --skip-apt       do not install apt packages
#      --skip-test      skip the final self-test
#
#  Why 6.1: Isaac Sim 5.1's RTX renderer crashes at start-up with NVIDIA
#  R590/R595 drivers on RTX 50-series (Blackwell) GPUs.  Isaac Sim 6.1 supports
#  those drivers, and uses Python 3.12 - the same Python as ROS 2 Jazzy.
#
#  What it does
#   1. checks Ubuntu / GLIBC / NVIDIA driver / free disk
#   2. apt: build tools, python3.12-venv, the ROS 2 Jazzy packages the HRUH
#      workspace uses; installs `uv` (fast Python package manager) to /usr/local/bin
#   3. group "isaac" (you are added) owns DIR, group-writable
#   4. DIR/venv-6.1      Python 3.12 venv: Isaac Sim 6.1.0.0 + torch 2.11.0 (cu128)
#      DIR/IsaacLab-3.0  Isaac Lab v3.0.0-EA (editable, all frameworks incl. rsl_rl)
#      hruh_lab          the HRUH Isaac Lab tasks from this workspace (editable)
#   5. /etc/profile.d/isaac.sh + commands in /usr/local/bin:
#        isaacsim       Isaac Sim GUI
#        isaac-python   Isaac Sim's Python, ROS 2 bridge environment set up
#        isaaclab       Isaac Lab's isaaclab.sh
#   6. self-test: headless Isaac Sim start (low CPU priority, capped threads)
#
#  Your system python3 and ROS 2 Jazzy stay untouched.  Re-running is safe:
#  finished steps are skipped and downloads come from uv's cache.
# =============================================================================
set -euo pipefail

PREFIX=/opt/isaac
ACCEPT_EULA=0
SKIP_APT=0
SKIP_TEST=0
REMOVE_OLD=0
ISAACSIM_VERSION=6.1.0.0
ISAACLAB_TAG=v3.0.0-EA
TORCH_SPEC="torch==2.11.0 torchvision==0.26.0"
TORCH_INDEX=https://download.pytorch.org/whl/cu128
EULA_URL=https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix) PREFIX="$2"; shift 2 ;;
    --accept-eula) ACCEPT_EULA=1; shift ;;
    --remove-old) REMOVE_OLD=1; shift ;;
    --skip-apt) SKIP_APT=1; shift ;;
    --skip-test) SKIP_TEST=1; shift ;;
    -h|--help) sed -n '2,38p' "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[warn] %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31m[error] %s\033[0m\n' "$*"; exit 1; }

[[ $EUID -eq 0 ]] || die "run with sudo:  sudo $0 $*"
USER_NAME="${SUDO_USER:-}"
[[ -n "$USER_NAME" && "$USER_NAME" != root ]] || die "run through sudo from your normal user account (SUDO_USER is not set)"
USER_HOME=$(getent passwd "$USER_NAME" | cut -d: -f6)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
HRUH_LAB_DIR=$(cd "$SCRIPT_DIR/.." && pwd)/hruh_lab
VENV="$PREFIX/venv-6.1"
LAB="$PREFIX/IsaacLab-3.0"
PY="$VENV/bin/python"
# run as the real user: their uv cache (~/.cache/uv) is reused, files stay group-writable
as_user() { sudo -u "$USER_NAME" -H env HOME="$USER_HOME" TERM=xterm OMNI_KIT_ACCEPT_EULA=YES \
            PATH="/usr/local/bin:/usr/bin:/bin" bash -c "umask 002; $*"; }

# ----------------------------------------------------------------------------- 1. checks
say "Checking the system"
. /etc/os-release
[[ "${VERSION_ID:-}" == "24.04" ]] || warn "tested on Ubuntu 24.04, this is ${PRETTY_NAME:-unknown}"
GLIBC=$(ldd --version | head -1 | grep -oE '[0-9]+\.[0-9]+$')
python3 - "$GLIBC" <<'PY' || die "Isaac Sim pip packages need GLIBC >= 2.35 (found $GLIBC)"
import sys; a, b = map(int, sys.argv[1].split(".")); sys.exit(0 if (a, b) >= (2, 35) else 1)
PY
command -v nvidia-smi >/dev/null || die "nvidia-smi not found: install the NVIDIA driver first"
GPU=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
DRV=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
VRAM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -1)
echo "GPU: $GPU (${VRAM} MiB)   driver: $DRV   GLIBC: $GLIBC"
[[ ${DRV%%.*} -ge 580 ]] || warn "driver $DRV is older than 580: Isaac Sim 6.1 recommends a recent production driver"
[[ ${VRAM:-0} -ge 12000 ]] || warn "less than 12 GB of GPU memory: train with fewer environments (--num_envs 512)"
mkdir -p "$PREFIX"
FREE_GB=$(df -BG --output=avail "$PREFIX" | tail -1 | tr -dc 0-9)
[[ $FREE_GB -ge 30 ]] || die "need ~30 GB free on $(df --output=target "$PREFIX" | tail -1), only ${FREE_GB} GB"

# ----------------------------------------------------------------------------- EULA
if [[ $ACCEPT_EULA -ne 1 ]]; then
  echo
  echo "Isaac Sim is licensed under the NVIDIA Omniverse License Agreement:"
  echo "    $EULA_URL"
  read -r -p "Do you accept the NVIDIA Isaac Sim EULA? [yes/no] " ans
  [[ "$ans" == "yes" ]] || die "EULA not accepted, nothing installed"
fi

# ----------------------------------------------------------------------------- 2. apt + uv
if [[ $SKIP_APT -ne 1 ]]; then
  say "Installing apt packages"
  apt-get update
  apt-get install -y curl git git-lfs build-essential cmake python3.12-venv python3.12-dev \
                     libglu1-mesa libxrandr2 libxinerama1 libxcursor1 libxi6 libvulkan1
  if [[ -d /opt/ros/jazzy ]]; then
    apt-get install -y \
      ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
      ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-planners-ompl ros-jazzy-moveit-kinematics \
      ros-jazzy-moveit-simple-controller-manager ros-jazzy-moveit-ros-visualization \
      ros-jazzy-moveit-configs-utils ros-jazzy-moveit-setup-assistant \
      ros-jazzy-joy ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher-gui ros-jazzy-stereo-image-proc ros-jazzy-tf2-ros \
      ros-jazzy-teleop-twist-keyboard || warn "some ROS packages failed to install"
  else
    warn "/opt/ros/jazzy not found: skipping ROS packages"
  fi
fi
if ! command -v uv >/dev/null; then
  say "Installing uv"
  curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin INSTALLER_NO_MODIFY_PATH=1 sh
fi

# ----------------------------------------------------------------------------- 3. group + prefix
say "Preparing $PREFIX (group 'isaac')"
groupadd -f isaac
usermod -aG isaac "$USER_NAME"
chown "$USER_NAME":isaac "$PREFIX"
chmod 2775 "$PREFIX"
if [[ $REMOVE_OLD -eq 1 ]]; then
  say "Removing the old Isaac Sim 5.1 install"
  rm -rf "$PREFIX/venv" "$PREFIX/IsaacLab"
fi

# ----------------------------------------------------------------------------- 4. Isaac Sim + Isaac Lab
if [[ ! -x "$PY" ]]; then
  say "Creating the Python 3.12 environment"
  as_user "uv venv --python /usr/bin/python3.12 --seed '$VENV'"
fi
ACT="source '$VENV/bin/activate'"

say "Installing Isaac Sim $ISAACSIM_VERSION (large download, ~15 GB)"
as_user "$ACT && uv pip install --upgrade pip && uv pip install 'isaacsim[all,extscache]==$ISAACSIM_VERSION' \
         --extra-index-url https://pypi.nvidia.com --index-strategy unsafe-best-match --prerelease=allow"

say "Installing PyTorch ($TORCH_SPEC, CUDA 12.8)"
as_user "$ACT && uv pip install -U $TORCH_SPEC --index-url $TORCH_INDEX"

say "Installing Isaac Lab $ISAACLAB_TAG"
if [[ ! -d "$LAB/.git" ]]; then
  as_user "git clone --branch $ISAACLAB_TAG --depth 1 https://github.com/isaac-sim/IsaacLab.git '$LAB'"
fi
as_user "$ACT && cd '$LAB' && ./isaaclab.sh -i" || warn "isaaclab.sh -i reported errors (see above)"
as_user "$PY -c 'import isaaclab, isaaclab_rl, isaaclab_tasks, rsl_rl'" \
  || die "Isaac Lab packages are not importable; scroll up for the first error"
# isaaclab.sh may have pulled another torch build: keep the CUDA 12.8 one (needed by RTX 50xx)
as_user "$ACT && uv pip install -U $TORCH_SPEC --index-url $TORCH_INDEX"

if [[ -f "$HRUH_LAB_DIR/setup.py" ]]; then
  say "Installing the HRUH Isaac Lab tasks (hruh_lab, editable)"
  as_user "$ACT && uv pip install -e '$HRUH_LAB_DIR'"
else
  warn "hruh_lab not found at $HRUH_LAB_DIR (install later: isaac-python -m pip install -e <path>)"
fi

# ----------------------------------------------------------------------------- 5. environment + commands
say "Writing /etc/profile.d/isaac.sh and /usr/local/bin commands"
cat > /etc/profile.d/isaac.sh <<EOF
# Isaac Sim / Isaac Lab (installed by install_isaac.sh)
export ISAAC_PREFIX="$PREFIX"
export ISAACSIM_PYTHON="$PY"
export ISAACLAB_PATH="$LAB"
export OMNI_KIT_ACCEPT_EULA=YES
EOF

cat > "$PREFIX/isaac_env.sh" <<EOF
# sourced by the isaacsim / isaac-python / isaaclab wrappers (install_isaac.sh)
export OMNI_KIT_ACCEPT_EULA=YES
export ISAACLAB_PATH="$LAB"
case "\${TERM:-}" in *+*|"") export TERM=xterm ;; esac
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
# Prefer Isaac's bundled ROS 2 Jazzy libraries; fall back to the system install
# (both are Python 3.12 builds with Isaac Sim 6.x).
SITE=\$("$PY" -c 'import site; print(site.getsitepackages()[0])')
ISAAC_ROS_LIB=\$(ls -d "\$SITE"/isaacsim/exts/isaacsim.ros2.core/jazzy/lib "\$SITE"/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib 2>/dev/null | head -1)
if [ -n "\$ISAAC_ROS_LIB" ]; then
  for v in PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; do unset \$v; done
  LD_LIBRARY_PATH=\$(printf '%s' "\${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/ros/' | paste -sd: -)
  export LD_LIBRARY_PATH="\$ISAAC_ROS_LIB\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
source "$VENV/bin/activate"
EOF
for cmd in isaacsim isaac-python isaaclab; do
  case $cmd in
    isaacsim)     run='exec isaacsim "$@"' ;;
    isaac-python) run='exec python "$@"' ;;
    isaaclab)     run='exec "$ISAACLAB_PATH/isaaclab.sh" "$@"' ;;
  esac
  cat > /usr/local/bin/$cmd <<EOF
#!/usr/bin/env bash
# $cmd: Isaac Sim $ISAACSIM_VERSION / Isaac Lab $ISAACLAB_TAG from $PREFIX (install_isaac.sh)
source "$PREFIX/isaac_env.sh"
$run
EOF
  chmod 755 /usr/local/bin/$cmd
done
chgrp -R isaac "$PREFIX" 2>/dev/null || true
chmod -R g+rwX "$PREFIX" 2>/dev/null || true

# ----------------------------------------------------------------------------- 6. self-test
if [[ $SKIP_TEST -ne 1 ]]; then
  say "Self-test: headless Isaac Sim start (first start compiles shaders: a few minutes)"
  as_user "timeout 600 nice -n 15 /usr/local/bin/isaac-python -c \"
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'extra_args': ['--/plugins/carb.tasking.plugin/threadCount=8']})
import isaaclab, torch
print('ISAAC_SELFTEST_OK', 'torch', torch.__version__, 'cuda', torch.cuda.is_available())
app.close()
\"" | grep -E "ISAAC_SELFTEST_OK|Error|error" || warn "self-test did not print ISAAC_SELFTEST_OK, see above"
fi

say "Done"
cat <<EOF
Installed Isaac Sim $ISAACSIM_VERSION + Isaac Lab $ISAACLAB_TAG in $PREFIX for group 'isaac'.
(If '$USER_NAME' was just added to the group: log out and back in once.)

  isaacsim                    Isaac Sim GUI
  isaac-python <script.py>    run a standalone Isaac Sim / Isaac Lab script
  isaaclab -p <script.py>     Isaac Lab's launcher
$( [[ -d "$PREFIX/venv" ]] && echo "
The old Isaac Sim 5.1 environment is still in $PREFIX/venv and $PREFIX/IsaacLab
(it does not run with driver $DRV). Free ~20 GB with:  sudo $0 --remove-old --skip-apt --skip-test --accept-eula" )
EOF
