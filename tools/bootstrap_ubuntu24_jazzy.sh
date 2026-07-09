#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_URL="https://github.com/Lazy-xiaodeng/humanoid_navigation_ws.git"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
REPO_URL="${REPO_URL:-$DEFAULT_REPO_URL}"
BRANCH="${BRANCH:-main}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
APT_DEP_FILE="${APT_DEP_FILE:-$SCRIPT_DIR/apt_dependencies_ubuntu24_jazzy.txt}"
FASTDDS_TEMPLATE="${FASTDDS_TEMPLATE:-$SCRIPT_DIR/fastdds_shm.xml}"
HUMANOID_ENV_FILE="${HUMANOID_ENV_FILE:-$HOME/.config/humanoid_ws_jazzy.env}"

INSTALL_ONEAPI=1
RUN_BUILD=1
RUN_ROSDEP=1
RUN_SELF_CHECK=1
RUN_NAVIGATION=0
CLEAN_BUILD=0
CONFIGURE_TUNA_APT=1

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

Bootstrap humanoid_ws on Ubuntu 24.04 x86_64 with ROS 2 Jazzy, Tsinghua
Ubuntu/ROS/rosdep mirrors, persistent shell environment, and a Release build.

Options:
  --workspace PATH       Workspace path. Default: parent of this tools directory.
  --repo URL             Git repository URL for empty workspaces. Default: $DEFAULT_REPO_URL
  --branch NAME          Git branch for empty/git workspaces. Default: main
  --skip-oneapi          Do not install Intel oneAPI packages.
  --skip-rosdep          Skip rosdep install.
  --no-build             Install dependencies only; skip colcon build.
  --no-self-check        Skip post-build ROS package checks.
  --clean-build          Remove build/install/log before colcon build.
  --no-tuna-apt          Do not rewrite Ubuntu/ROS apt sources to Tsinghua mirrors.
  --run-navigation       Run ./start_navigation.sh after a successful build.
  -h, --help             Show this help.

Examples:
  cd ~/humanoid_ws
  bash tools/bootstrap_ubuntu24_jazzy.sh
  bash tools/bootstrap_ubuntu24_jazzy.sh --clean-build --skip-oneapi
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

normalized_arch() {
  case "$(uname -m)" in
    x86_64|amd64|AMD64)
      printf '%s\n' x86_64
      ;;
    *)
      uname -m
      ;;
  esac
}

require_target_system() {
  [ -r /etc/os-release ] || die "/etc/os-release not found."
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${ID:-}" = "ubuntu" ] || die "Unsupported OS ID: ${ID:-unknown}. This script targets Ubuntu 24.04."
  [ "${VERSION_ID:-}" = "24.04" ] || die "Unsupported Ubuntu version: ${VERSION_ID:-unknown}. This script targets Ubuntu 24.04."
  [ "$(normalized_arch)" = "x86_64" ] || die "Unsupported CPU architecture: $(uname -m). Use the Humble/Jetson script for ARM64 boards."
}

require_sudo() {
  if [ "$(id -u)" -eq 0 ]; then
    die "Run as a normal user with sudo access, not as root."
  fi
  sudo -v
}

require_dependency_file() {
  [ -f "$APT_DEP_FILE" ] || die "Missing apt dependency file: $APT_DEP_FILE"
}

configure_tuna_apt_sources() {
  [ "$CONFIGURE_TUNA_APT" -eq 1 ] || {
    log "Skipping Tsinghua apt source configuration"
    return 0
  }

  log "Configuring Tsinghua Ubuntu/ROS apt mirrors"
  local ubuntu_codename
  ubuntu_codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-noble}")"
  local ubuntu_mirror="https://mirrors.tuna.tsinghua.edu.cn/ubuntu"
  local backup_dir="/etc/apt/backup-humanoid-$(date +%Y%m%d_%H%M%S)"
  sudo mkdir -p "$backup_dir"

  if [ -f /etc/apt/sources.list ]; then
    sudo cp /etc/apt/sources.list "$backup_dir/sources.list"
  fi
  if [ -d /etc/apt/sources.list.d ]; then
    sudo find /etc/apt/sources.list.d -maxdepth 1 -type f \
      \( -name '*.list' -o -name '*.sources' \) \
      -exec cp -t "$backup_dir" {} + 2>/dev/null || true
  fi

  sudo tee /etc/apt/sources.list > /dev/null <<EOF
deb $ubuntu_mirror/ $ubuntu_codename main restricted universe multiverse
deb $ubuntu_mirror/ $ubuntu_codename-updates main restricted universe multiverse
deb $ubuntu_mirror/ $ubuntu_codename-backports main restricted universe multiverse
deb $ubuntu_mirror/ $ubuntu_codename-security main restricted universe multiverse
EOF

  if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then
    sudo tee /etc/apt/sources.list.d/ubuntu.sources > /dev/null <<EOF
Types: deb
URIs: $ubuntu_mirror/
Suites: $ubuntu_codename $ubuntu_codename-updates $ubuntu_codename-backports $ubuntu_codename-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
  fi

  if [ -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo sed -i 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g; s#https://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g' /etc/apt/sources.list.d/ros2.list
  fi

  if [ -f /etc/apt/sources.list.d/ros2.sources ]; then
    sudo sed -i \
      -e 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g' \
      -e 's#https://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g' \
      /etc/apt/sources.list.d/ros2.sources
  fi
}

configure_ros_apt_source() {
  log "Configuring ROS 2 apt repository for $ROS_DISTRO"

  if dpkg -s "ros-${ROS_DISTRO}-ros-base" >/dev/null 2>&1; then
    log "ROS 2 $ROS_DISTRO is already installed; keeping current apt source configuration."
    return 0
  fi

  sudo apt update
  sudo apt install -y curl gnupg lsb-release software-properties-common
  sudo add-apt-repository -y universe

  if apt-cache policy "ros-${ROS_DISTRO}-ros-base" 2>/dev/null | grep -q 'Candidate: [^ (]'; then
    log "ROS 2 apt packages are already visible."
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

init_rosdep() {
  [ "$RUN_ROSDEP" -eq 1 ] || {
    log "Skipping rosdep initialization"
    return 0
  }

  log "Initializing rosdep with Tsinghua rosdistro mirror"
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

prepare_workspace() {
  log "Preparing workspace at $WORKSPACE"

  if [ -f "$WORKSPACE/src/humanoid_bringup/package.xml" ]; then
    log "Existing humanoid_ws source tree detected; skipping git clone."
    return 0
  fi

  if [ -d "$WORKSPACE/.git" ]; then
    cd "$WORKSPACE"
    git remote set-url origin "$REPO_URL"
    git fetch origin "$BRANCH"
    git checkout "$BRANCH"
    git pull --ff-only origin "$BRANCH"
    return 0
  fi

  if [ -e "$WORKSPACE" ] && [ -n "$(find "$WORKSPACE" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
    die "$WORKSPACE exists but does not look like humanoid_ws. Move it away or pass --workspace."
  fi

  mkdir -p "$(dirname "$WORKSPACE")"
  git clone --branch "$BRANCH" "$REPO_URL" "$WORKSPACE"
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
    cp "$file" "$file.bak_ubuntu24_jazzy" 2>/dev/null || true
    sed -i \
      -e 's#/opt/ros/jazzy#/opt/ros/${ROS_DISTRO:-jazzy}#g' \
      -e 's#/opt/ros/humble#/opt/ros/${ROS_DISTRO:-jazzy}#g' \
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
    "$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/x86_64/open3d-0.18.0/lib/cmake/Open3D"
    /usr/lib/x86_64-linux-gnu/cmake/Open3D
    /usr/lib/cmake/Open3D
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
  if [ "$open3d_dir" = "/usr/lib/x86_64-linux-gnu/cmake/Open3D" ]; then
    printf '%s\n' /usr/lib/x86_64-linux-gnu
    return 0
  fi
  if [ "$open3d_dir" = "/usr/lib/cmake/Open3D" ]; then
    printf '%s\n' /usr/lib
    return 0
  fi
  printf '%s\n' "$(dirname "$(dirname "$open3d_dir")")"
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
  local begin="# >>> humanoid_ws ubuntu24 jazzy bootstrap >>>"
  local end="# <<< humanoid_ws ubuntu24 jazzy bootstrap <<<"
  local open3d_dir
  open3d_dir="$(detect_open3d_dir || true)"
  [ -n "$open3d_dir" ] || open3d_dir="$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/x86_64/open3d-0.18.0/lib/cmake/Open3D"
  local open3d_lib_dir
  open3d_lib_dir="$(open3d_library_dir "$open3d_dir")"

  mkdir -p "$(dirname "$HUMANOID_ENV_FILE")"
  cat > "$HUMANOID_ENV_FILE" <<EOF
# Generated by $0 on $(date '+%F %T').
export HUMANOID_WS="$WORKSPACE"
export ROS_DISTRO="$ROS_DISTRO"
export ROS_DOMAIN_ID="\${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="\${FASTRTPS_DEFAULT_PROFILES_FILE:-\$HOME/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="\${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
export Open3D_DIR="\${Open3D_DIR:-$open3d_dir}"
export LD_LIBRARY_PATH="$open3d_lib_dir:\${LD_LIBRARY_PATH:-}"

if [ -n "\${BASH_VERSION:-}" ]; then
  if [ -f "/opt/ros/\$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/\$ROS_DISTRO/setup.bash"
  fi
  if [ -f /opt/intel/oneapi/setvars.sh ]; then
    source /opt/intel/oneapi/setvars.sh > /dev/null 2>&1 || true
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
  log "Checking Jazzy/x86 source compatibility"
  cd "$WORKSPACE"

  grep -Fq 'behaviortree_cpp' src/humanoid_nav2_bt_nodes/CMakeLists.txt ||
    die "Missing BehaviorTree.CPP dependency in src/humanoid_nav2_bt_nodes/CMakeLists.txt"
  grep -Fq 'SpinResultTraits' src/humanoid_nav2_bt_nodes/include/humanoid_nav2_bt_nodes/pose_angle_nodes.hpp ||
    die "Missing Spin action result compatibility in pose_angle_nodes.hpp"
  grep -Fq 'std::optional<json>{' src/humanoid_route_runtime/src/navigation_state_manager.cpp ||
    die "Missing nlohmann::json optional conversion fix in navigation_state_manager.cpp"
  [ -f "$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/x86_64/open3d-0.18.0/lib/cmake/Open3D/Open3DConfig.cmake" ] ||
    warn "Packaged x86_64 Open3DConfig.cmake was not found; build will rely on system Open3D_DIR."
}

build_workspace() {
  [ "$RUN_BUILD" -eq 1 ] || {
    log "Skipping colcon build"
    return 0
  }

  log "Building workspace: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
  cd "$WORKSPACE"
  source_ros_setup

  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
  export Open3D_DIR="${Open3D_DIR:-$(detect_open3d_dir || true)}"
  [ -n "$Open3D_DIR" ] || die "Open3DConfig.cmake not found. Keep the packaged x86_64 Open3D directory or provide Open3D_DIR."
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
  require_dependency_file
  require_sudo
  configure_tuna_apt_sources
  configure_ros_apt_source
  install_apt_dependencies
  install_oneapi_optional
  init_rosdep
  prepare_workspace
  patch_ros_distro_scripts
  write_fastdds_profile
  install_workspace_dependencies
  check_source_compatibility
  append_bashrc_block
  build_workspace
  self_check
  run_navigation_optional

  log "Ubuntu 24.04 Jazzy bootstrap finished."
  log "Workspace: $WORKSPACE"
  log "Next shell: source ~/.bashrc"
  log "Start navigation: cd $WORKSPACE && ./start_navigation.sh"
}

main "$@"
