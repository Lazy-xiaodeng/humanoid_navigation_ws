#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  setup_codex_proxy.sh [--host HOST] [--port PORT] [--socks] [--no-desktop] [--no-shell] [--test]

Examples:
  ./tools/setup_codex_proxy.sh
  ./tools/setup_codex_proxy.sh --host 192.168.50.1 --port 7890
  ./tools/setup_codex_proxy.sh --host 192.168.50.1 --port 7890 --socks

What it configures:
  - ~/.config/environment.d/proxy.conf
  - ~/.config/codex-proxy.env
  - ~/.local/share/applications/code.desktop, if VS Code desktop file exists
  - ~/.bashrc source line, unless --no-shell is used

Notes:
  - For VM use, HOST is usually the VM default gateway or the host-only adapter IP.
  - Make sure the host proxy app enables "Allow LAN" / "Listen LAN".
  - Log out and back in, or fully quit and reopen VS Code, after running.
EOF
}

host=""
port="7890"
scheme="http"
configure_desktop=1
configure_shell=1
run_test=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      host="${2:-}"
      shift 2
      ;;
    --port)
      port="${2:-}"
      shift 2
      ;;
    --socks)
      scheme="socks5h"
      shift
      ;;
    --http)
      scheme="http"
      shift
      ;;
    --no-desktop)
      configure_desktop=0
      shift
      ;;
    --no-shell)
      configure_shell=0
      shift
      ;;
    --test)
      run_test=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$host" ]]; then
  host="$(ip route | awk '/default/ {print $3; exit}')"
fi

if [[ -z "$host" ]]; then
  echo "Could not infer proxy host. Pass --host HOST." >&2
  exit 1
fi

proxy_url="${scheme}://${host}:${port}"
no_proxy="localhost,127.0.0.1,::1"

echo "Configuring proxy:"
echo "  HTTP_PROXY=${proxy_url}"
echo "  HTTPS_PROXY=${proxy_url}"
echo "  ALL_PROXY=${proxy_url}"
echo "  NO_PROXY=${no_proxy}"

mkdir -p "$HOME/.config/environment.d" "$HOME/.config"

cat > "$HOME/.config/environment.d/proxy.conf" <<EOF
HTTP_PROXY=${proxy_url}
HTTPS_PROXY=${proxy_url}
ALL_PROXY=${proxy_url}
NO_PROXY=${no_proxy}
EOF

cat > "$HOME/.config/codex-proxy.env" <<EOF
export HTTP_PROXY=${proxy_url}
export HTTPS_PROXY=${proxy_url}
export ALL_PROXY=${proxy_url}
export NO_PROXY=${no_proxy}
export http_proxy=${proxy_url}
export https_proxy=${proxy_url}
export all_proxy=${proxy_url}
export no_proxy=${no_proxy}
EOF

if [[ "$configure_shell" -eq 1 ]]; then
  touch "$HOME/.bashrc"
  if ! grep -Fq 'source "$HOME/.config/codex-proxy.env"' "$HOME/.bashrc"; then
    {
      echo ''
      echo '# Codex / VS Code proxy environment'
      echo 'if [ -f "$HOME/.config/codex-proxy.env" ]; then'
      echo '  source "$HOME/.config/codex-proxy.env"'
      echo 'fi'
    } >> "$HOME/.bashrc"
  fi
fi

if [[ "$configure_desktop" -eq 1 ]]; then
  desktop_src=""
  for candidate in \
    "/usr/share/applications/code.desktop" \
    "/var/lib/snapd/desktop/applications/code_code.desktop" \
    "/var/lib/snapd/desktop/applications/code.desktop"; do
    if [[ -f "$candidate" ]]; then
      desktop_src="$candidate"
      break
    fi
  done

  if [[ -n "$desktop_src" ]]; then
    mkdir -p "$HOME/.local/share/applications"
    desktop_dst="$HOME/.local/share/applications/$(basename "$desktop_src")"
    cp "$desktop_src" "$desktop_dst"

    tmp_file="${desktop_dst}.tmp"
    awk -v proxy="$proxy_url" -v no_proxy="$no_proxy" '
      BEGIN {
        env_prefix = "env HTTP_PROXY=" proxy " HTTPS_PROXY=" proxy " ALL_PROXY=" proxy " NO_PROXY=" no_proxy " "
      }
      /^Exec=/ {
        cmd = substr($0, 6)
        if (cmd !~ /^env /) {
          print "Exec=" env_prefix cmd
          next
        }
      }
      { print }
    ' "$desktop_dst" > "$tmp_file"
    mv "$tmp_file" "$desktop_dst"

    if command -v update-desktop-database >/dev/null 2>&1; then
      update-desktop-database "$HOME/.local/share/applications" >/dev/null 2>&1 || true
    fi

    echo "Updated desktop launcher: $desktop_dst"
  else
    echo "VS Code desktop file not found; skipped desktop launcher update."
  fi
fi

if command -v systemctl >/dev/null 2>&1; then
  env HTTP_PROXY="$proxy_url" HTTPS_PROXY="$proxy_url" ALL_PROXY="$proxy_url" NO_PROXY="$no_proxy" \
    systemctl --user import-environment HTTP_PROXY HTTPS_PROXY ALL_PROXY NO_PROXY >/dev/null 2>&1 || true
fi

if command -v dbus-update-activation-environment >/dev/null 2>&1; then
  env HTTP_PROXY="$proxy_url" HTTPS_PROXY="$proxy_url" ALL_PROXY="$proxy_url" NO_PROXY="$no_proxy" \
    dbus-update-activation-environment --systemd HTTP_PROXY HTTPS_PROXY ALL_PROXY NO_PROXY >/dev/null 2>&1 || true
fi

if [[ "$run_test" -eq 1 ]]; then
  if command -v curl >/dev/null 2>&1; then
    echo
    echo "Testing https://chatgpt.com/backend-api/ through ${proxy_url} ..."
    if curl -x "$proxy_url" --connect-timeout 10 --max-time 20 -I https://chatgpt.com/backend-api/; then
      echo "Proxy test completed. HTTP 403 from Cloudflare is acceptable for raw curl."
    else
      echo "Proxy test failed. Check host IP, port, firewall, and Allow LAN on the proxy app." >&2
      exit 1
    fi
  else
    echo "curl not found; skipped proxy test."
  fi
fi

echo
echo "Done."
echo "Next:"
echo "  1. Fully quit VS Code."
echo "  2. Log out and log back in, or reopen VS Code from the desktop launcher."
echo "  3. Verify in VS Code terminal: env | grep -i proxy"
