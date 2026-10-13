# Findings

- Automatic task start writes work_mode=2; use explicit scheduler activity instead.
- Existing disc_speed_feedback reads a command register, not measured rotation.
- Scheduler/driver have no persistent emergency indicator source; add display-only Bool topics/services without changing stop behavior.
- Python 3.10 available; ROS target is Linux/Noetic/Python 3.8. No pio/arduino-cli found on PATH.
- Ubuntu 20.04 WSL has ROS Noetic/Python 3.8 and g++; used it for complete chassis tests and isolated Catkin validation. Scheduler's external slamware_ros_sdk package is not installed in this test overlay.
- Created an isolated build-tool venv under LOCALAPPDATA/codex-status-strip-build; PlatformIO builds the actual target firmware successfully. No firmware upload was performed.
- Hardware: 3 LEDs, GPIO4, AHCT125, 330 ohm, USB VBUS 5V. Boot yellow; ready green; automatic blue breathing; pause blue slow blink.
