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

# 3. Install system dependencies & official Raspberry Pi AI HAT / Hailo suite
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

# Install official Raspberry Pi OS Hailo package if available
echo "Checking for official Raspberry Pi Hailo suite (hailo-all)..."
if sudo apt-get install -y hailo-all; then
    echo "hailo-all installed successfully (kernel module, firmware, and HailoRT)."
else
    echo "hailo-all apt package not found or failed; continuing with manual check..."
fi

# 4. User permissions for UART & Camera
echo "Ensuring user is in dialout and video groups for serial FC link and camera..."
sudo usermod -aG dialout,video,i2c,gpio "$USER" || true

# 5. Check for HailoRT installation
if command -v hailortcli &> /dev/null; then
    echo "HailoRT CLI found:"
    hailortcli --version || true
    echo "Scanning PCIe devices:"
    hailortcli scan || true
else
    echo "Warning: hailortcli not found. Ensure PCIe is connected and 'sudo apt install hailo-all' was executed."
fi

# 6. Virtual environment setup (CRITICAL: --system-site-packages to use picamera2 & hailo_platform)
VENV_DIR="$HOME/interceptor-venv"
if [[ ! -d "$VENV_DIR" ]]; then
    echo "Creating virtual environment at $VENV_DIR with system-site-packages..."
    python3 -m venv --system-site-packages "$VENV_DIR"
else
    echo "Existing venv found. Ensuring system-site-packages are enabled..."
    python3 -m venv --system-site-packages --upgrade "$VENV_DIR"
fi

echo "Activating virtual environment and installing python dependencies..."
source "$VENV_DIR/bin/activate"
pip install --upgrade pip setuptools wheel
pip install -r requirements-air.txt

echo "=============================================="
echo "Setup completed successfully!"
echo "NOTE: If PCIe was newly enabled, REBOOT your Pi 5:"
echo "      sudo reboot"
echo "=============================================="
