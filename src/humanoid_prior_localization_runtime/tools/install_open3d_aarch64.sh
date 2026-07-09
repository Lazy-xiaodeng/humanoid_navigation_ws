#!/usr/bin/env bash
set -euo pipefail

# install_open3d_aarch64.sh
#
# 用途：
#   在 Jetson Orin NX / ARM64 机器上构建 Open3D C++ SDK，并安装到本包约定目录。
#
# 输出目录：
#   humanoid_prior_localization_runtime/third_party/open3d/aarch64/open3d-0.18.0
#
# 为什么需要这个脚本：
#   Open3D 官方 ARM64 Linux 主要提供 Python wheel；本 OP 定位节点需要 C++ SDK，
#   即 include、lib/libOpen3D.so、lib/cmake/Open3D/Open3DConfig.cmake。
#   因此 ARM64 上最稳妥的方式是在目标机器原生构建一次。
#
# 使用方式：
#   cd /path/to/humanoid_ws
#   bash src/humanoid_prior_localization_runtime/tools/install_open3d_aarch64.sh
#
# 可选环境变量：
#   OPEN3D_VERSION=v0.18.0
#   OPEN3D_BUILD_DIR=/tmp/open3d-aarch64-build
#   OPEN3D_JOBS=6
#   OPEN3D_WITH_CUDA=OFF

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ARCH="$(uname -m)"

case "$ARCH" in
  aarch64|arm64|ARM64)
    ;;
  *)
    echo "ERROR: 当前机器架构是 $ARCH，不是 ARM64/aarch64。"
    echo "这个脚本只应在 Jetson/ARM64 目标机器上运行，避免生成错误架构的 Open3D。"
    exit 2
    ;;
esac

OPEN3D_VERSION="${OPEN3D_VERSION:-v0.18.0}"
OPEN3D_BUILD_DIR="${OPEN3D_BUILD_DIR:-/tmp/open3d-aarch64-build}"
OPEN3D_JOBS="${OPEN3D_JOBS:-$(nproc)}"
OPEN3D_WITH_CUDA="${OPEN3D_WITH_CUDA:-OFF}"
INSTALL_DIR="$PKG_DIR/third_party/open3d/aarch64/open3d-0.18.0"
SRC_DIR="$OPEN3D_BUILD_DIR/Open3D"
BUILD_DIR="$OPEN3D_BUILD_DIR/build"

echo "Open3D ARM64 C++ SDK installer"
echo "  version:     $OPEN3D_VERSION"
echo "  build dir:   $OPEN3D_BUILD_DIR"
echo "  install dir: $INSTALL_DIR"
echo "  jobs:        $OPEN3D_JOBS"
echo "  CUDA:        $OPEN3D_WITH_CUDA"

sudo apt-get update
sudo apt-get install -y \
  git \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  libeigen3-dev \
  libflann-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  libx11-dev \
  libxi-dev \
  libxrandr-dev \
  libxinerama-dev \
  libxcursor-dev \
  libxt-dev \
  libssl-dev \
  python3-dev \
  python3-pip

mkdir -p "$OPEN3D_BUILD_DIR"

if [ ! -d "$SRC_DIR/.git" ]; then
  git clone --recursive --branch "$OPEN3D_VERSION" https://github.com/isl-org/Open3D.git "$SRC_DIR"
else
  git -C "$SRC_DIR" fetch --tags
  git -C "$SRC_DIR" checkout "$OPEN3D_VERSION"
  git -C "$SRC_DIR" submodule update --init --recursive
fi

rm -rf "$BUILD_DIR"
cmake -S "$SRC_DIR" -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_PYTHON_MODULE=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_GUI=OFF \
  -DBUILD_WEBRTC=OFF \
  -DBUILD_CUDA_MODULE="$OPEN3D_WITH_CUDA" \
  -DUSE_SYSTEM_EIGEN3=ON \
  -DUSE_SYSTEM_FLANN=ON

cmake --build "$BUILD_DIR" --parallel "$OPEN3D_JOBS"
cmake --install "$BUILD_DIR"

echo
echo "Open3D ARM64 install finished."
echo "Checking required files..."

test -f "$INSTALL_DIR/lib/cmake/Open3D/Open3DConfig.cmake"
test -f "$INSTALL_DIR/lib/libOpen3D.so"

file "$INSTALL_DIR/lib/libOpen3D.so"
echo "OK: $INSTALL_DIR"
