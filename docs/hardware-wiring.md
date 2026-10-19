# Hardware Wiring Guide

> [!NOTE]
> For detailed circuit diagrams, step-by-step pinouts for **Raspberry Pi 5**, **Raspberry Pi 4B**, and **Radxa Zero 3W**, power budgeting, and operating workflows, see the complete [HARDWARE_AND_USER_GUIDE.md](./HARDWARE_AND_USER_GUIDE.md).


### UART (Flight Controller)
- TX: GPIO 14 (Pin 8) → FC RX
- RX: GPIO 15 (Pin 10) → FC TX
- GND: Pin 6 → FC GND
- Baud: 921600

### Hailo-8 HAT
- Connected via M.2 slot on RPi5 HAT
- Requires active cooling

### CSI Camera
- Connected to CAM0 port on RPi5
- Use libcamera-compatible module (e.g., RPi Camera Module 3)

### Power
- RPi5: 5V/5A USB-C PD
- Hailo HAT: powered via M.2 from RPi5
- Ensure stable 5V supply from drone BEC (separate from servo rail)
