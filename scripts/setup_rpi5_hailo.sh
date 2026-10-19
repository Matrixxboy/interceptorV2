#!/usr/bin/env bash
# scripts/setup_rpi5_hailo.sh — Raspberry Pi 5 + Hailo-8 setup script (§10, T5.1)
set -euo pipefail

echo "=== InterceptorV2 RPi5 + Hailo-8 Setup ==="

# 1. Ensure running on Linux
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Warning: This setup script is intended to run on Raspberry Pi OS (Linux)."
fi

# 2. Check PCIe Gen 3 enablement for Raspberry Pi 5
CONFIG_TXT="/boot/firmware/config.txt"
if [[ -f "$CONFIG_TXT" ]]; then
    echo "Checking PCIe Gen 3 in $CONFIG_TXT..."
    if ! grep -q "dtparam=pciex1" "$CONFIG_TXT"; then
        echo "Adding PCIe configuration to $CONFIG_TXT..."
        echo -e "\n# Hailo AI HAT PCIe configuration\ndtparam=pciex1\ndtparam=pciex1_gen=3" | sudo tee -a "$CONFIG_TXT"
        echo "PCIe Gen 3 configured. A reboot may be required."
    fi
fi

# 3. Install system dependencies
echo "Updating apt and installing required system packages..."
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    git \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libgstreamer1.0-dev \
    v4l-utils

# 4. Check for HailoRT installation
if command -v hailortcli &> /dev/null; then
    echo "HailoRT CLI found:"
    hailortcli --version || true
    echo "Scanning PCIe devices:"
    hailortcli scan || true
else
    echo "Note: HailoRT package not installed. Download and install HailoRT deb from Hailo Developer Zone."
fi

# 5. Virtual environment setup
VENV_DIR="$HOME/interceptor-venv"
if [[ ! -d "$VENV_DIR" ]]; then
    echo "Creating virtual environment at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
fi

echo "Activating virtual environment and installing python dependencies..."
source "$VENV_DIR/bin/activate"
pip install --upgrade pip setuptools wheel
pip install -e ".[air]"

echo "Setup completed successfully."
