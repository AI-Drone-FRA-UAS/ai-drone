#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-seb-is-pm.local}"
USER="${USER_NAME:-seb}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-300}"
SLEEP_SECONDS="${SLEEP_SECONDS:-5}"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

resolve_host() {
  getent hosts "$HOST" | awk '{print $1; exit}'
}

avahi_resolve_host() {
  if command -v avahi-resolve-host-name >/dev/null 2>&1; then
    avahi-resolve-host-name "$HOST" 2>/dev/null | awk '{print $2; exit}'
  fi
}

tcp_22_open() {
  local ip="$1"
  timeout 3 bash -c ":</dev/tcp/$ip/22" >/dev/null 2>&1
}

ssh_probe() {
  local target="$1"
  ssh \
    -o BatchMode=yes \
    -o ConnectTimeout=5 \
    -o StrictHostKeyChecking=accept-new \
    "$USER@$target" true 2>&1
}

print_network_context() {
  echo
  echo "Local network context:"
  ip -4 addr show scope global 2>/dev/null | sed 's/^/  /' || true
  ip route show default 2>/dev/null | sed 's/^/  /' || true
}

require_command ssh
require_command ping
require_command getent
require_command timeout

echo "Waiting for Raspberry Pi SSH"
echo "Host:    $HOST"
echo "User:    $USER"
echo "Timeout: ${TIMEOUT_SECONDS}s"
echo
echo "Keep the phone hotspot on and keep this laptop connected to the same hotspot."
echo "The first boot can take a few minutes while the Pi expands/configures the system."
echo

deadline=$((SECONDS + TIMEOUT_SECONDS))
last_status=""
resolved_ip=""

while (( SECONDS < deadline )); do
  resolved_ip="$(resolve_host || true)"
  if [[ -z "$resolved_ip" ]]; then
    resolved_ip="$(avahi_resolve_host || true)"
  fi

  if [[ -z "$resolved_ip" ]]; then
    status="not resolved by mDNS/DNS yet"
    if [[ "$status" != "$last_status" ]]; then
      echo "[$(date +%H:%M:%S)] $HOST is not visible by name yet."
      last_status="$status"
    fi
    sleep "$SLEEP_SECONDS"
    continue
  fi

  echo "[$(date +%H:%M:%S)] $HOST resolved to $resolved_ip"

  if ! ping -c 1 -W 2 "$resolved_ip" >/dev/null 2>&1; then
    echo "  The Pi name resolves, but it does not answer ping yet."
    echo "  This can still be normal during boot, or ICMP may be blocked."
    sleep "$SLEEP_SECONDS"
    continue
  fi

  echo "  Ping works."

  if ! tcp_22_open "$resolved_ip"; then
    echo "  The Pi is reachable, but TCP port 22 is not open yet."
    echo "  Likely causes:"
    echo "    - SSH service is still starting on first boot."
    echo "    - cloud-init user-data failed before enabling SSH."
    echo "    - the Pi booted but did not finish setup."
    sleep "$SLEEP_SECONDS"
    continue
  fi

  echo "  SSH port 22 is open."

  probe_output="$(ssh_probe "$resolved_ip" || true)"
  if grep -qi "Permission denied" <<<"$probe_output"; then
    echo "  SSH is ready and password login is available."
    echo
    echo "Connecting now. Use the Pi password you entered while flashing."
    exec ssh "$USER@$resolved_ip"
  fi

  if grep -qi "Host key verification failed" <<<"$probe_output"; then
    echo "  SSH is reachable, but your known_hosts entry conflicts."
    echo "  Fix it with:"
    echo "    ssh-keygen -R $HOST"
    echo "    ssh-keygen -R $resolved_ip"
    exit 2
  fi

  if grep -qi "Connection refused" <<<"$probe_output"; then
    echo "  The Pi is reachable, but SSH refused the connection."
    echo "  Wait a bit longer; if it persists, SSH was not enabled correctly."
    sleep "$SLEEP_SECONDS"
    continue
  fi

  if grep -qi "No route to host\\|Network is unreachable\\|Connection timed out" <<<"$probe_output"; then
    echo "  The Pi resolved, but the network path to SSH is blocked."
    echo "  On phone hotspots this often means client isolation is enabled."
    sleep "$SLEEP_SECONDS"
    continue
  fi

  echo "  SSH probe returned an unexpected response:"
  echo "$probe_output" | sed 's/^/    /'
  echo
  echo "Trying an interactive SSH session anyway."
  exec ssh "$USER@$resolved_ip"
done

echo
echo "Timed out after ${TIMEOUT_SECONDS}s waiting for $HOST."
print_network_context
echo
echo "Most likely causes:"
echo "  - The Pi is still booting; try this script again."
echo "  - The phone hotspot password was typed incorrectly."
echo "  - The phone hotspot changed from 2.4 GHz/WPA2 compatibility mode."
echo "  - The phone hotspot blocks device-to-device traffic."
echo "  - mDNS is not passing; check your phone hotspot client list for the Pi IP."
echo
echo "If your phone shows the Pi IP, run:"
echo "  HOST=<pi-ip-address> ./wait-for-pi-ssh.sh"
exit 1
