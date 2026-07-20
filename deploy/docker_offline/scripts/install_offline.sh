#!/bin/bash
set -Eeuo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Please run with sudo: sudo ./scripts/install_offline.sh" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_FILE="$PACKAGE_ROOT/humanoid.env"

set -a
source "$ENV_FILE"
set +a

TARGET_USER="${TARGET_USER:-ubuntu}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
if [ -z "$TARGET_HOME" ]; then
  echo "User not found: $TARGET_USER" >&2
  exit 1
fi

IMAGE_NAME="${IMAGE_NAME:-humanoid_nav:1.0.0}"
IMAGE_TAR="${IMAGE_TAR:-images/humanoid_nav_1.0.0.tar}"
IMAGE_TAR_PATH="$PACKAGE_ROOT/$IMAGE_TAR"
HUMANOID_DATA="${HUMANOID_DATA:-$TARGET_HOME/humanoid_data}"
DEPLOY_TARGET="${DEPLOY_TARGET:-$TARGET_HOME/humanoid_deploy}"
FORCE_LOAD_IMAGE="${FORCE_LOAD_IMAGE:-0}"

install_docker_if_needed() {
  if command -v docker >/dev/null 2>&1; then
    echo "[install] docker already installed"
    return 0
  fi

  if compgen -G "$PACKAGE_ROOT/docker_debs/*.deb" >/dev/null; then
    echo "[install] installing docker debs from $PACKAGE_ROOT/docker_debs"
    if [ -f "$PACKAGE_ROOT/docker_debs/SHA256SUMS" ]; then
      (cd "$PACKAGE_ROOT/docker_debs" && sha256sum -c SHA256SUMS)
    fi
    local empty_sources
    empty_sources="$(mktemp -d)"
    apt-get install -y --no-install-recommends \
      -o Dir::Etc::sourcelist=/dev/null \
      -o Dir::Etc::sourceparts="$empty_sources" \
      -o APT::Get::List-Cleanup=0 \
      "$PACKAGE_ROOT"/docker_debs/*.deb
    rm -rf "$empty_sources"
  else
    echo "Docker is not installed and no offline debs were found in $PACKAGE_ROOT/docker_debs" >&2
    exit 1
  fi
}

load_image_if_needed() {
  if [ "$FORCE_LOAD_IMAGE" != "1" ] && docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "[install] image already exists: $IMAGE_NAME"
    return 0
  fi

  if [ ! -f "$IMAGE_TAR_PATH" ]; then
    echo "Image tar not found: $IMAGE_TAR_PATH" >&2
    exit 1
  fi

  echo "[install] loading image: $IMAGE_TAR_PATH"
  docker load -i "$IMAGE_TAR_PATH"
}

copy_defaults_from_image() {
  local cid
  cid="$(docker create "$IMAGE_NAME")"
  trap 'docker rm -f "$cid" >/dev/null 2>&1 || true' RETURN

  if [ -z "$(find "$HUMANOID_DATA/maps" -maxdepth 1 -type f 2>/dev/null | head -1)" ]; then
    echo "[install] initializing maps from image"
    docker cp "$cid:/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/." "$HUMANOID_DATA/maps/" || true
  fi

  if [ -z "$(find "$HUMANOID_DATA/pcd" -maxdepth 1 -type f 2>/dev/null | head -1)" ]; then
    echo "[install] initializing pcd from image"
    docker cp "$cid:/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/." "$HUMANOID_DATA/pcd/" || true
  fi

  if [ -z "$(find "$HUMANOID_DATA/data" -mindepth 1 -maxdepth 2 2>/dev/null | head -1)" ]; then
    echo "[install] initializing data from image"
    docker cp "$cid:/home/ubuntu/humanoid_ws/data/." "$HUMANOID_DATA/data/" || true
  fi
}

install_docker_if_needed
systemctl enable --now docker
usermod -aG docker "$TARGET_USER" || true

load_image_if_needed

mkdir -p \
  "$HUMANOID_DATA/maps" \
  "$HUMANOID_DATA/pcd" \
  "$HUMANOID_DATA/data" \
  "$HUMANOID_DATA/debug_logs" \
  "$HUMANOID_DATA/fast-lio-bags" \
  "$HUMANOID_DATA/config" \
  "$DEPLOY_TARGET"

install -m 0644 "$PACKAGE_ROOT/config/fastdds_shm.xml" "$HUMANOID_DATA/config/fastdds_shm.xml"
copy_defaults_from_image

rsync -a --delete "$PACKAGE_ROOT/" "$DEPLOY_TARGET/"
chmod +x "$DEPLOY_TARGET"/scripts/*.sh
ln -sf "$DEPLOY_TARGET/scripts/run_navigation.sh" "$DEPLOY_TARGET/run_navigation.sh"
ln -sf "$DEPLOY_TARGET/scripts/stop_navigation.sh" "$DEPLOY_TARGET/stop_navigation.sh"
ln -sf "$DEPLOY_TARGET/scripts/run_mapping.sh" "$DEPLOY_TARGET/run_mapping.sh"
ln -sf "$DEPLOY_TARGET/scripts/finish_mapping.sh" "$DEPLOY_TARGET/finish_mapping.sh"
ln -sf "$DEPLOY_TARGET/scripts/abort_mapping.sh" "$DEPLOY_TARGET/abort_mapping.sh"
ln -sf "$DEPLOY_TARGET/scripts/check_deploy.sh" "$DEPLOY_TARGET/check_deploy.sh"

install -m 0644 "$PACKAGE_ROOT/systemd/humanoid-navigation.service" /etc/systemd/system/humanoid-navigation.service
systemctl daemon-reload
systemctl disable humanoid-navigation.service

chown -R "$TARGET_USER:$TARGET_USER" "$HUMANOID_DATA" "$DEPLOY_TARGET"

echo "[install] running deployment check"
sudo -u "$TARGET_USER" "$DEPLOY_TARGET/scripts/check_deploy.sh"

echo "[install] done"
echo "Navigation service is installed and disabled:"
systemctl is-enabled humanoid-navigation.service || true
