#!/usr/bin/env bash
# sitl_runner.sh — Helper to launch ArduPilot SITL for testing
set -euo pipefail

VEHICLE="${1:-ArduCopter}"
FRAME="${2:-quad}"
SYSID="${3:-1}"

echo "Starting ArduPilot SITL: ${VEHICLE} (${FRAME})..."
echo "Telemetry out: 127.0.0.1:14550 (GCS), 127.0.0.1:14551 (Air-unit / MAVLink bridge)"

if ! command -v sim_vehicle.py &> /dev/null; then
    echo "Warning: sim_vehicle.py not found in PATH."
    echo "Ensure ArduPilot environment is sourced (e.g. source ~/.profile or export PATH=\$PATH:\$HOME/ardupilot/Tools/autotest)"
    exit 1
fi

sim_vehicle.py -v "${VEHICLE}" -f "${FRAME}" --sysid "${SYSID}" \
    --out=127.0.0.1:14550 \
    --out=127.0.0.1:14551 \
    --console --map
