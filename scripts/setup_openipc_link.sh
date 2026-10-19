#!/usr/bin/env bash
# scripts/setup_openipc_link.sh — Network configuration for OpenIPC / wireless datalink (§4)
set -euo pipefail

INTERFACE="${1:-eth0}"
DRONE_IP="${2:-192.168.1.20}"
GCS_IP="${3:-192.168.1.10}"
NETMASK="${4:-24}"

echo "Configuring network interface $INTERFACE for OpenIPC / GCS datalink..."

if command -v ip &> /dev/null; then
    sudo ip addr add "${DRONE_IP}/${NETMASK}" dev "$INTERFACE" || true
    sudo ip link set "$INTERFACE" up
    echo "Interface $INTERFACE configured with IP ${DRONE_IP}/${NETMASK}."
    echo "Testing ping to GCS at $GCS_IP..."
    ping -c 2 -W 1 "$GCS_IP" || echo "GCS not yet reachable at $GCS_IP (normal if GCS not powered on)."
else
    echo "Error: 'ip' command not found."
    exit 1
fi
