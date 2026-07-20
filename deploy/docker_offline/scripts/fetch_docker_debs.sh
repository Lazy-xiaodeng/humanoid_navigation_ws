#!/bin/bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT_DIR="$PACKAGE_ROOT/docker_debs"

UBUNTU_CODENAME="${UBUNTU_CODENAME:-noble}"
UBUNTU_ARCH="${UBUNTU_ARCH:-amd64}"
FETCH_IMAGE="${FETCH_IMAGE:-ubuntu:24.04}"

mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/*.deb "$OUT_DIR"/SHA256SUMS "$OUT_DIR"/manifest.txt

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is required on the build machine to fetch offline debs in a clean Ubuntu 24.04 container." >&2
  exit 1
fi

docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e UBUNTU_CODENAME="$UBUNTU_CODENAME" \
  -e UBUNTU_ARCH="$UBUNTU_ARCH" \
  -v "$OUT_DIR:/out" \
  "$FETCH_IMAGE" \
  bash -lc '
    set -Eeuo pipefail

    apt-get update
    apt-get install -y --no-install-recommends ca-certificates curl gnupg

    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    chmod a+r /etc/apt/keyrings/docker.asc

    echo "deb [arch=${UBUNTU_ARCH} signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${UBUNTU_CODENAME} stable" \
      > /etc/apt/sources.list.d/docker.list

    apt-get update
    apt-get install -y --download-only --no-install-recommends \
      docker-ce \
      docker-ce-cli \
      containerd.io \
      docker-buildx-plugin \
      docker-compose-plugin

    cp -a /var/cache/apt/archives/*.deb /out/
    apt-cache policy \
      docker-ce \
      docker-ce-cli \
      containerd.io \
      docker-buildx-plugin \
      docker-compose-plugin \
      > /out/manifest.txt

    cd /out
    sha256sum *.deb > SHA256SUMS
  '

echo "Docker offline debs saved to: $OUT_DIR"
ls -lh "$OUT_DIR"
