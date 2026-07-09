#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_URL="https://github.com/Lazy-xiaodeng/humanoid_navigation_ws.git"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
REPO_URL="${REPO_URL:-$DEFAULT_REPO_URL}"
BRANCH="${BRANCH:-c++}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
APT_DEP_FILE="${APT_DEP_FILE:-$SCRIPT_DIR/apt_dependencies_ubuntu22_humble_jetson.txt}"
FASTDDS_TEMPLATE="${FASTDDS_TEMPLATE:-$SCRIPT_DIR/fastdds_shm.xml}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-2}"
RUN_BUILD=1
RUN_OPEN3D=1
RUN_ROSDEP=1
RUN_SELF_CHECK=1
RUN_NAVIGATION=0
CLEAN_BUILD=0
CONFIGURE_TUNA_APT=1
HUMANOID_ENV_FILE="${HUMANOID_ENV_FILE:-}"
DEFAULT_RMW_IMPLEMENTATION="${DEFAULT_RMW_IMPLEMENTATION:-}"
DEFAULT_ENABLE_FASTDDS_SHM="${DEFAULT_ENABLE_FASTDDS_SHM:-}"

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

warn() {
  printf '\n[%s] WARN: %s\n' "$(date '+%F %T')" "$*" >&2
}

die() {
  printf '\nERROR: %s\n' "$*" >&2
  exit 1
}

source_ros_setup() {
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
}

source_workspace_setup() {
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u
}

usage() {
  cat <<USAGE
Usage: $0 [options]

Bootstrap the current humanoid_ws source tree on Ubuntu 22.04 with ROS 2 Humble,
then build the workspace. Supports Jetson/ARM64 and AMD64/x86_64 hosts.

Options:
  --workspace PATH       Workspace path. Default: parent of this tools directory.
  --repo URL             Git repository URL for empty workspaces. Default: $DEFAULT_REPO_URL
  --branch NAME          Git branch for empty/git workspaces. Default: c++
  --parallel N           Open3D build parallel workers. Default: 2.
  --skip-open3d          Skip Open3D source-build fallback.
  --skip-rosdep          Skip rosdep install.
  --no-build             Install dependencies only; skip colcon build.
  --no-self-check        Skip post-build ROS package checks.
  --clean-build          Remove build/install/log before colcon build.
  --no-tuna-apt          Do not rewrite Ubuntu/ROS apt sources to Tsinghua mirrors.
  --run-navigation       Run ./start_navigation.sh after a successful build.
  -h, --help             Show this help.

Examples:
  cd ~/humanoid_ws
  bash tools/bootstrap_ubuntu22_humble.sh
  PARALLEL_WORKERS=1 bash tools/bootstrap_ubuntu22_humble_jetson.sh
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
    --parallel)
      PARALLEL_WORKERS="${2:?--parallel requires a number}"
      shift 2
      ;;
    --skip-open3d)
      RUN_OPEN3D=0
      shift
      ;;
    --skip-rosdep)
      RUN_ROSDEP=0
      shift
      ;;
    --no-build)
      RUN_BUILD=0
      shift
      ;;
    --no-self-check)
      RUN_SELF_CHECK=0
      shift
      ;;
    --clean-build)
      CLEAN_BUILD=1
      shift
      ;;
    --no-tuna-apt)
      CONFIGURE_TUNA_APT=0
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

require_target_system() {
  [ -r /etc/os-release ] || die "/etc/os-release not found."
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${ID:-}" = "ubuntu" ] || die "Unsupported OS ID: ${ID:-unknown}. This script targets Ubuntu 22.04."
  [ "${VERSION_ID:-}" = "22.04" ] || die "Unsupported Ubuntu version: ${VERSION_ID:-unknown}. This script targets Ubuntu 22.04."

  case "$(normalized_arch)" in
    aarch64|x86_64)
      ;;
    *)
      die "Unsupported CPU architecture: $(uname -m). This script supports ARM64/aarch64 and AMD64/x86_64."
      ;;
  esac
}

prepare_workspace() {
  if [ -d "$WORKSPACE/src" ] && [ -f "$WORKSPACE/src/humanoid_bringup/package.xml" ]; then
    log "Existing humanoid_ws source tree detected; skipping git clone."
    return 0
  fi

  if [ -d "$WORKSPACE/.git" ]; then
    log "Existing git workspace detected; updating $BRANCH from $REPO_URL."
    cd "$WORKSPACE"
    git remote set-url origin "$REPO_URL"
    git fetch origin "$BRANCH"
    git checkout "$BRANCH"
    git pull --ff-only origin "$BRANCH"
    return 0
  fi

  if [ -e "$WORKSPACE" ] && [ -n "$(find "$WORKSPACE" -mindepth 1 -maxdepth 1 ! -name tools -print -quit 2>/dev/null)" ]; then
    die "$WORKSPACE exists but does not look like humanoid_ws. Move it away or pass --workspace."
  fi

  mkdir -p "$(dirname "$WORKSPACE")"
  log "Cloning $REPO_URL branch $BRANCH into $WORKSPACE."
  if [ -e "$WORKSPACE" ] && [ -n "$(find "$WORKSPACE" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
    local tmp_workspace
    tmp_workspace="$(mktemp -d "$(dirname "$WORKSPACE")/.humanoid_ws_clone.XXXXXX")"
    git clone --branch "$BRANCH" "$REPO_URL" "$tmp_workspace"
    cp -a "$tmp_workspace"/. "$WORKSPACE"/
    rm -rf "$tmp_workspace"
  else
    git clone --branch "$BRANCH" "$REPO_URL" "$WORKSPACE"
  fi
}

require_workspace() {
  [ -f "$APT_DEP_FILE" ] || die "Missing apt dependency file: $APT_DEP_FILE"
}

require_sudo() {
  if [ "$(id -u)" -eq 0 ]; then
    die "Run as normal user with sudo access, not as root."
  fi
  sudo -v
}

normalized_arch() {
  case "$(uname -m)" in
    aarch64|arm64|ARM64)
      printf '%s\n' aarch64
      ;;
    x86_64|amd64|AMD64)
      printf '%s\n' x86_64
      ;;
    *)
      uname -m
      ;;
  esac
}

configure_host_defaults() {
  local arch
  arch="$(normalized_arch)"

  if [ -z "$HUMANOID_ENV_FILE" ]; then
    case "$arch" in
      aarch64)
        HUMANOID_ENV_FILE="$HOME/.config/humanoid_ws_jetson.env"
        ;;
      x86_64)
        HUMANOID_ENV_FILE="$HOME/.config/humanoid_ws_humble.env"
        ;;
    esac
  fi

  if [ -z "$DEFAULT_RMW_IMPLEMENTATION" ]; then
    case "$arch" in
      aarch64)
        DEFAULT_RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
        ;;
      x86_64)
        DEFAULT_RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
        ;;
    esac
  fi

  if [ -z "$DEFAULT_ENABLE_FASTDDS_SHM" ]; then
    case "$arch" in
      aarch64)
        DEFAULT_ENABLE_FASTDDS_SHM="false"
        ;;
      x86_64)
        DEFAULT_ENABLE_FASTDDS_SHM="true"
        ;;
    esac
  fi
}

configure_tuna_apt_sources() {
  [ "$CONFIGURE_TUNA_APT" -eq 1 ] || {
    log "Skipping Tsinghua apt source configuration"
    return 0
  }

  log "Configuring Tsinghua Ubuntu/ROS apt mirrors"
  local ubuntu_codename
  ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-jammy}")"
  local ubuntu_mirror
  case "$(normalized_arch)" in
    aarch64)
      ubuntu_mirror="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports"
      ;;
    x86_64)
      ubuntu_mirror="https://mirrors.tuna.tsinghua.edu.cn/ubuntu"
      ;;
    *)
      die "Unsupported CPU architecture for apt mirror: $(uname -m)"
      ;;
  esac
  local backup_dir="/etc/apt/backup-humanoid-$(date +%Y%m%d_%H%M%S)"
  sudo mkdir -p "$backup_dir"

  if [ -f /etc/apt/sources.list ]; then
    sudo cp /etc/apt/sources.list "$backup_dir/sources.list"
  fi

  sudo tee /etc/apt/sources.list > /dev/null <<EOF
deb $ubuntu_mirror/ $ubuntu_codename main restricted universe multiverse
deb $ubuntu_mirror/ $ubuntu_codename-updates main restricted universe multiverse
deb $ubuntu_mirror/ $ubuntu_codename-backports main restricted universe multiverse
deb $ubuntu_mirror/ $ubuntu_codename-security main restricted universe multiverse
EOF

  if [ -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo cp /etc/apt/sources.list.d/ros2.list "$backup_dir/ros2.list" || true
    sudo sed -i 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g; s#https://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g' /etc/apt/sources.list.d/ros2.list
  fi

  if [ -f /etc/apt/sources.list.d/ros2.sources ]; then
    sudo cp /etc/apt/sources.list.d/ros2.sources "$backup_dir/ros2.sources" || true
    sudo sed -i \
      -e 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g' \
      -e 's#https://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g' \
      -e 's/^Types:.*/Types: deb/' \
      /etc/apt/sources.list.d/ros2.sources
  fi
}

configure_ros_apt_source() {
  log "Configuring ROS 2 apt repository for $ROS_DISTRO"

  if dpkg -s "ros-${ROS_DISTRO}-ros-base" >/dev/null 2>&1; then
    log "ROS 2 $ROS_DISTRO is already installed; keeping current apt source configuration."
    return 0
  fi

  if apt-cache policy "ros-${ROS_DISTRO}-ros-base" 2>/dev/null | grep -q 'Candidate: [^ (]'; then
    log "ROS 2 apt packages are already visible; keeping current apt source configuration."
    return 0
  fi

  sudo apt update
  sudo apt install -y curl gnupg lsb-release software-properties-common
  sudo add-apt-repository -y universe

  if apt-cache policy "ros-${ROS_DISTRO}-ros-base" 2>/dev/null | grep -q 'Candidate: [^ (]'; then
    log "ROS 2 apt packages are already visible; keeping current apt source configuration."
    return 0
  fi

  if [ -f /etc/apt/sources.list.d/ros2.list ] && grep -q 'packages.ros.org/ros2/ubuntu' /etc/apt/sources.list.d/ros2.list; then
    log "ROS 2 apt repository already exists; keeping current configuration."
    return 0
  fi

  if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
}

read_apt_packages() {
  grep -Ev '^\s*($|#)' "$APT_DEP_FILE"
}

install_apt_dependencies() {
  log "Installing apt dependencies"
  sudo apt update
  mapfile -t apt_packages < <(read_apt_packages)
  sudo apt install -y "${apt_packages[@]}"
}

init_rosdep() {
  log "Initializing rosdep"
  sudo mkdir -p /etc/ros/rosdep/sources.list.d
  sudo tee /etc/ros/rosdep/sources.list.d/20-default.list > /dev/null <<'EOF'
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/osx-homebrew.yaml osx
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/base.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/python.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/ruby.yaml
EOF
  export ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml}"
  rosdep update
}

patch_ros_distro_scripts() {
  log "Patching runtime scripts to use ROS_DISTRO=$ROS_DISTRO"
  cd "$WORKSPACE"

  local files=(
    start_navigation.sh
    start_mapping.sh
    start_websocket.sh
    stop_navigation.sh
    docker/entrypoint.sh
  )

  local file
  for file in "${files[@]}"; do
    [ -f "$file" ] || continue
    cp "$file" "$file.bak_jetson_humble" 2>/dev/null || true
    sed -i \
      -e 's#/opt/ros/jazzy#/opt/ros/${ROS_DISTRO:-humble}#g' \
      -e 's#/opt/ros/humble#/opt/ros/${ROS_DISTRO:-humble}#g' \
      "$file"
  done

  chmod +x start_navigation.sh start_mapping.sh start_websocket.sh stop_navigation.sh 2>/dev/null || true
}

write_fastdds_profile() {
  if [ -f "$FASTDDS_TEMPLATE" ]; then
    log "Writing FastDDS profile to $HOME/.config/fastdds_shm.xml"
    mkdir -p "$HOME/.config"
    cp "$FASTDDS_TEMPLATE" "$HOME/.config/fastdds_shm.xml"
    return 0
  fi

  warn "FastDDS template not found: $FASTDDS_TEMPLATE"
}

detect_open3d_dir() {
  local candidates=(
    "${Open3D_DIR:-}"
    /usr/lib/aarch64-linux-gnu/cmake/Open3D
    /usr/lib/x86_64-linux-gnu/cmake/Open3D
    /usr/lib/cmake/Open3D
    "$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/aarch64/open3d-0.18.0/lib/cmake/Open3D"
    "$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/x86_64/open3d-0.18.0/lib/cmake/Open3D"
  )

  local dir
  for dir in "${candidates[@]}"; do
    [ -n "$dir" ] || continue
    if [ -f "$dir/Open3DConfig.cmake" ]; then
      printf '%s\n' "$dir"
      return 0
    fi
  done

  return 1
}

open3d_library_dir() {
  local open3d_dir="$1"
  if [ "$open3d_dir" = "/usr/lib/aarch64-linux-gnu/cmake/Open3D" ]; then
    printf '%s\n' /usr/lib/aarch64-linux-gnu
    return 0
  fi
  if [ "$open3d_dir" = "/usr/lib/x86_64-linux-gnu/cmake/Open3D" ]; then
    printf '%s\n' /usr/lib/x86_64-linux-gnu
    return 0
  fi
  if [ "$open3d_dir" = "/usr/lib/cmake/Open3D" ]; then
    printf '%s\n' /usr/lib
    return 0
  fi
  printf '%s\n' "$(cd "$open3d_dir/../.." && pwd)"
}

install_open3d_aarch64() {
  [ "$RUN_OPEN3D" -eq 1 ] || {
    log "Skipping Open3D ARM64 build"
    return 0
  }

  if detect_open3d_dir >/dev/null; then
    log "Open3D C++ SDK already exists: $(detect_open3d_dir)"
    return 0
  fi

  if [ "$(normalized_arch)" != "aarch64" ]; then
    die "Open3D C++ SDK not found for $(normalized_arch). Install libopen3d-dev or provide Open3D_DIR."
  fi

  local open3d_dir="$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/aarch64/open3d-0.18.0/lib/cmake/Open3D"
  if [ -f "$open3d_dir/Open3DConfig.cmake" ]; then
    log "Open3D ARM64 C++ SDK already exists: $open3d_dir"
    return 0
  fi

  log "Building Open3D ARM64 C++ SDK. This can take a long time on Jetson."
  cd "$WORKSPACE"
  OPEN3D_JOBS="$PARALLEL_WORKERS" bash src/humanoid_prior_localization_runtime/tools/install_open3d_aarch64.sh
}

install_workspace_dependencies() {
  [ "$RUN_ROSDEP" -eq 1 ] || {
    log "Skipping rosdep install"
    return 0
  }

  log "Installing workspace dependencies with rosdep"
  cd "$WORKSPACE"
  source_ros_setup
  export ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml}"
  rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -r -y \
    --skip-keys "$(printf '%s ' "${ROSDEP_SKIP_KEYS[@]}")"
}

append_bashrc_block() {
  log "Configuring persistent shell environment"
  local begin="# >>> humanoid_ws ubuntu22 humble bootstrap >>>"
  local end="# <<< humanoid_ws ubuntu22 humble bootstrap <<<"
  local open3d_dir
  open3d_dir="$(detect_open3d_dir || true)"
  [ -n "$open3d_dir" ] || {
    if [ "$(normalized_arch)" = "x86_64" ]; then
      open3d_dir="/usr/lib/x86_64-linux-gnu/cmake/Open3D"
    else
      open3d_dir="/usr/lib/aarch64-linux-gnu/cmake/Open3D"
    fi
  }
  local open3d_lib_dir
  open3d_lib_dir="$(open3d_library_dir "$open3d_dir")"

  mkdir -p "$(dirname "$HUMANOID_ENV_FILE")"
  cat > "$HUMANOID_ENV_FILE" <<EOF
# Generated by $0 on $(date '+%F %T').
export HUMANOID_WS="$WORKSPACE"
export ROS_DISTRO="$ROS_DISTRO"
export ROS_DOMAIN_ID="\${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="\${RMW_IMPLEMENTATION:-$DEFAULT_RMW_IMPLEMENTATION}"
export ENABLE_FASTDDS_SHM="\${ENABLE_FASTDDS_SHM:-$DEFAULT_ENABLE_FASTDDS_SHM}"
if [ "\$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ] && [ "\$ENABLE_FASTDDS_SHM" = "true" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="\${FASTRTPS_DEFAULT_PROFILES_FILE:-\$HOME/.config/fastdds_shm.xml}"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="\${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset RMW_FASTRTPS_USE_QOS_FROM_XML
fi
export Open3D_DIR="\${Open3D_DIR:-$open3d_dir}"
export LD_LIBRARY_PATH="$open3d_lib_dir:\${LD_LIBRARY_PATH:-}"

if [ -n "\${BASH_VERSION:-}" ]; then
  if [ -f "/opt/ros/\$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/\$ROS_DISTRO/setup.bash"
  fi
  if [ -f "\$HUMANOID_WS/install/setup.bash" ]; then
    source "\$HUMANOID_WS/install/setup.bash"
  fi
fi
EOF

  local source_line="[ -f \"$HUMANOID_ENV_FILE\" ] && . \"$HUMANOID_ENV_FILE\""
  local shell_file
  for shell_file in "$HOME/.bashrc" "$HOME/.profile"; do
    touch "$shell_file"
    if grep -Fq "$begin" "$shell_file"; then
      sed -i "/^$begin$/,/^$end$/d" "$shell_file"
    fi
    cat >> "$shell_file" <<EOF

$begin
$source_line
$end
EOF
  done
}

check_source_compatibility() {
  log "Checking Humble/Jetson source compatibility patches"
  cd "$WORKSPACE"

  grep -Fq 'behaviortree_cpp_v3' src/humanoid_nav2_bt_nodes/CMakeLists.txt ||
    die "Missing Humble BT.CPP v3 CMake fallback in src/humanoid_nav2_bt_nodes/CMakeLists.txt"
  grep -Fq 'SpinResultTraits' src/humanoid_nav2_bt_nodes/include/humanoid_nav2_bt_nodes/pose_angle_nodes.hpp ||
    die "Missing Humble Spin action result compatibility in pose_angle_nodes.hpp"
  grep -Fq 'std::vector<Point> read_xyz(const sensor_msgs::msg::PointCloud2 & msg)' src/humanoid_obstacle_runtime/src/roi_obstacle_detector_cpp.cpp ||
    die "Missing Humble RCLCPP_WARN_THROTTLE const-clock fix in roi_obstacle_detector_cpp.cpp"
  grep -Fq 'std::optional<json>{' src/humanoid_route_runtime/src/navigation_state_manager.cpp ||
    die "Missing GCC 11 nlohmann::json optional conversion fix in navigation_state_manager.cpp"
}

build_workspace() {
  [ "$RUN_BUILD" -eq 1 ] || {
    log "Skipping colcon build"
    return 0
  }

  log "Building workspace: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
  cd "$WORKSPACE"
  source_ros_setup

  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-$DEFAULT_RMW_IMPLEMENTATION}"
  export ENABLE_FASTDDS_SHM="${ENABLE_FASTDDS_SHM:-$DEFAULT_ENABLE_FASTDDS_SHM}"
  if [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ] && [ "$ENABLE_FASTDDS_SHM" = "true" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
    export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
  else
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    unset RMW_FASTRTPS_USE_QOS_FROM_XML
  fi
  export Open3D_DIR="${Open3D_DIR:-$(detect_open3d_dir || true)}"
  [ -n "$Open3D_DIR" ] || die "Open3DConfig.cmake not found. Install libopen3d-dev/python3-open3d or run without --skip-open3d."
  export LD_LIBRARY_PATH="$(open3d_library_dir "$Open3D_DIR"):${LD_LIBRARY_PATH:-}"

  if [ "$CLEAN_BUILD" -eq 1 ]; then
    log "Removing build/install/log before clean build"
    rm -rf build install log
  fi

  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
}

self_check() {
  [ "$RUN_BUILD" -eq 1 ] || return 0
  [ "$RUN_SELF_CHECK" -eq 1 ] || {
    log "Skipping self-check"
    return 0
  }

  log "Running post-build self-checks"
  cd "$WORKSPACE"
  [ -f install/setup.bash ] || die "Build did not create install/setup.bash"
  source_ros_setup
  source_workspace_setup

  ros2 pkg prefix humanoid_bringup >/dev/null
  ros2 pkg prefix humanoid_navigation2 >/dev/null
  ros2 pkg prefix humanoid_prior_localization_runtime >/dev/null
  test -f install/humanoid_nav2_bt_nodes/lib/libhumanoid_nav2_bt_nodes.so
  python3 - <<'PY'
import importlib
for name in ("numpy", "yaml", "websockets"):
    importlib.import_module(name)
print("python deps ok")
PY
}

run_navigation_optional() {
  [ "$RUN_NAVIGATION" -eq 1 ] || return 0
  [ "$RUN_BUILD" -eq 1 ] || die "--run-navigation requires a build."
  log "Starting navigation"
  cd "$WORKSPACE"
  exec ./start_navigation.sh
}

main() {
  require_target_system
  configure_host_defaults
  require_workspace
  require_sudo
  configure_tuna_apt_sources
  configure_ros_apt_source
  install_apt_dependencies
  init_rosdep
  prepare_workspace
  patch_ros_distro_scripts
  write_fastdds_profile
  install_open3d_aarch64
  install_workspace_dependencies
  check_source_compatibility
  append_bashrc_block
  build_workspace
  self_check
  run_navigation_optional

  log "Ubuntu 22.04 Humble bootstrap finished."
  log "Workspace: $WORKSPACE"
  log "Next shell: source ~/.bashrc"
  log "Start navigation: cd $WORKSPACE && ./start_navigation.sh"
}

main "$@"
