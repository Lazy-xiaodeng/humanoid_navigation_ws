#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_URL="https://github.com/Lazy-xiaodeng/humanoid_navigation_ws.git"
WORKSPACE="${WORKSPACE:-$HOME/humanoid_ws}"
REPO_URL="${REPO_URL:-$DEFAULT_REPO_URL}"
BRANCH="${BRANCH:-main}"
INSTALL_ONEAPI=1
RUN_BUILD=1
RUN_NAVIGATION=0
PARALLEL_WORKERS="${PARALLEL_WORKERS:-4}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
APT_DEP_FILE="$SCRIPT_DIR/apt_dependencies_ubuntu24_jazzy.txt"
FASTDDS_TEMPLATE="$SCRIPT_DIR/fastdds_shm.xml"
ROSDEP_SKIP_KEYS=(
  ament_python
  apr
  elevation_mapping
  elevation_mapping_cupy
  grid_map_cmake_helpers
  lidar_localization
  lidar_localization_ros2
  open3d_loc
  ros2bag
  websockets
)

log() {
  printf '\n[%s] %s\n' "$(date '+%F %T')" "$*"
}

die() {
  printf '\nERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<USAGE
Usage: $0 [options]

Options:
  --workspace PATH       Target workspace path. Default: $HOME/humanoid_ws
  --repo URL             Git repository URL. Default: $DEFAULT_REPO_URL
  --branch NAME          Git branch to checkout or update. Default: main
  --skip-oneapi          Do not install Intel oneAPI packages.
  --no-build             Install and clone only; skip colcon build.
  --run-navigation       Run ./start_navigation.sh after a successful build.
  -h, --help             Show this help.

Examples:
  bash tools/bootstrap_ubuntu24_jazzy.sh
  bash tools/bootstrap_ubuntu24_jazzy.sh --workspace /home/robot/humanoid_ws --branch main
  bash tools/bootstrap_ubuntu24_jazzy.sh --skip-oneapi --run-navigation
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --workspace)
      WORKSPACE="${2:?--workspace requires a path}"
      shift 2
      ;;
    --repo)
      REPO_URL="${2:?--repo requires a URL}"
      shift 2
      ;;
    --branch)
      BRANCH="${2:?--branch requires a branch name}"
      shift 2
      ;;
    --skip-oneapi)
      INSTALL_ONEAPI=0
      shift
      ;;
    --no-build)
      RUN_BUILD=0
      shift
      ;;
    --run-navigation)
      RUN_NAVIGATION=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1"
      ;;
  esac
done

require_ubuntu_2404() {
  [ -r /etc/os-release ] || die "/etc/os-release not found; this script supports Ubuntu 24.04 only."
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${ID:-}" = "ubuntu" ] || die "Unsupported OS ID: ${ID:-unknown}. Install Ubuntu 24.04 first."
  [ "${VERSION_ID:-}" = "24.04" ] || die "Unsupported Ubuntu version: ${VERSION_ID:-unknown}. This script targets Ubuntu 24.04."
}

require_sudo() {
  if [ "$(id -u)" -eq 0 ]; then
    die "Run this script as a normal user with sudo access, not as root."
  fi
  sudo -v
}

install_ros_apt_source() {
  log "Configuring ROS 2 apt repository"
  sudo apt update
  sudo apt install -y curl gnupg lsb-release software-properties-common
  sudo add-apt-repository -y universe

  if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  fi
}

read_apt_packages() {
  [ -f "$APT_DEP_FILE" ] || die "Missing apt dependency file: $APT_DEP_FILE"
  grep -Ev '^\s*($|#)' "$APT_DEP_FILE"
}

install_apt_dependencies() {
  log "Installing apt dependencies"
  sudo apt update
  mapfile -t apt_packages < <(read_apt_packages)
  sudo apt install -y "${apt_packages[@]}"
}

install_oneapi_optional() {
  [ "$INSTALL_ONEAPI" -eq 1 ] || {
    log "Skipping Intel oneAPI installation"
    return 0
  }

  log "Installing Intel oneAPI runtime/compiler packages"
  wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
    | gpg --dearmor \
    | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null

  echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
    | sudo tee /etc/apt/sources.list.d/oneAPI.list > /dev/null

  sudo apt update
  sudo apt install -y intel-oneapi-basekit intel-oneapi-dpcpp-cpp intel-oneapi-opencl-rt intel-opencl-icd
}

install_python_user_packages() {
  log "Installing Python packages not reliably available from apt"
  python3 -m pip install --user --break-system-packages open3d || \
    log "WARN: open3d pip installation failed; Open3D-dependent tools may need manual installation."
}

init_rosdep() {
  log "Initializing rosdep"
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
  rosdep update
}

clone_or_update_workspace() {
  log "Preparing workspace at $WORKSPACE"
  if [ -d "$WORKSPACE/.git" ]; then
    cd "$WORKSPACE"
    git remote set-url origin "$REPO_URL"
    git fetch origin "$BRANCH"
    git checkout "$BRANCH"
    git pull --ff-only origin "$BRANCH"
    return 0
  fi

  if [ -e "$WORKSPACE" ] && [ -n "$(find "$WORKSPACE" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
    die "$WORKSPACE exists and is not a git checkout. Move it away or pass --workspace to an empty path."
  fi

  mkdir -p "$(dirname "$WORKSPACE")"
  git clone --branch "$BRANCH" "$REPO_URL" "$WORKSPACE"
}

install_workspace_dependencies() {
  log "Installing workspace dependencies with rosdep"
  cd "$WORKSPACE"
  # shellcheck disable=SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -r -y \
    --skip-keys "$(printf '%s ' "${ROSDEP_SKIP_KEYS[@]}")"
}

write_fastdds_profile() {
  log "Writing FastDDS profile to $HOME/.config/fastdds_shm.xml"
  [ -f "$FASTDDS_TEMPLATE" ] || die "Missing FastDDS template: $FASTDDS_TEMPLATE"
  mkdir -p "$HOME/.config"
  cp "$FASTDDS_TEMPLATE" "$HOME/.config/fastdds_shm.xml"
}

append_bashrc_block() {
  log "Configuring ~/.bashrc"
  local begin="# >>> humanoid_ws ubuntu24 jazzy bootstrap >>>"
  local end="# <<< humanoid_ws ubuntu24 jazzy bootstrap <<<"

  if grep -Fq "$begin" "$HOME/.bashrc"; then
    log "~/.bashrc already contains humanoid_ws bootstrap block"
    return 0
  fi

  cat >> "$HOME/.bashrc" <<EOF

$begin
export ROS_DISTRO=$ROS_DISTRO
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=\${FASTRTPS_DEFAULT_PROFILES_FILE:-\$HOME/.config/fastdds_shm.xml}
export RMW_FASTRTPS_USE_QOS_FROM_XML=\${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f /opt/intel/oneapi/setvars.sh ]; then
  source /opt/intel/oneapi/setvars.sh > /dev/null 2>&1
fi
if [ -f "$WORKSPACE/install/setup.bash" ]; then
  source "$WORKSPACE/install/setup.bash"
fi
$end
EOF
}

build_workspace() {
  [ "$RUN_BUILD" -eq 1 ] || {
    log "Skipping colcon build"
    return 0
  }

  log "Building workspace"
  cd "$WORKSPACE"
  # shellcheck disable=SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  if [ -f /opt/intel/oneapi/setvars.sh ]; then
    # shellcheck disable=SC1091
    source /opt/intel/oneapi/setvars.sh > /dev/null 2>&1 || true
  fi
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
  colcon build --symlink-install --parallel-workers "$PARALLEL_WORKERS" --cmake-args -DCMAKE_BUILD_TYPE=Release
}

self_check() {
  [ "$RUN_BUILD" -eq 1 ] || return 0
  log "Running deployment self-checks"
  cd "$WORKSPACE"
  [ -f install/setup.bash ] || die "Build did not create install/setup.bash"
  # shellcheck disable=SC1091
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  # shellcheck disable=SC1091
  source install/setup.bash
  ros2 pkg list | grep -q '^humanoid_navigation2$' || die "humanoid_navigation2 is not visible to ROS 2"
  ros2 pkg executables humanoid_websocket | grep -q websocket_server_node || die "humanoid_websocket executable check failed"
  test -f install/humanoid_nav2_bt_nodes/lib/libhumanoid_nav2_bt_nodes.so || die "BT node plugin library missing"
  python3 - <<'PY'
import importlib
for name in ("numpy", "yaml", "websockets"):
    importlib.import_module(name)
print("python deps ok")
PY
}

run_navigation_optional() {
  [ "$RUN_NAVIGATION" -eq 1 ] || return 0
  [ "$RUN_BUILD" -eq 1 ] || die "--run-navigation requires a build; remove --no-build."
  log "Starting navigation"
  cd "$WORKSPACE"
  exec ./start_navigation.sh
}

main() {
  require_ubuntu_2404
  require_sudo
  install_ros_apt_source
  install_apt_dependencies
  install_oneapi_optional
  install_python_user_packages
  init_rosdep
  clone_or_update_workspace
  install_workspace_dependencies
  write_fastdds_profile
  append_bashrc_block
  build_workspace
  self_check
  run_navigation_optional

  log "Deployment finished. Open a new terminal or run: source ~/.bashrc"
  log "Workspace: $WORKSPACE"
  log "Start navigation: cd $WORKSPACE && ./start_navigation.sh"
}

main "$@"
