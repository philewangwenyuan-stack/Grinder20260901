# Facts

- Ubuntu 20.04 + ROS1 Noetic; Windows is the editing/ESP32 flashing host.
- Platform mapping profile forces industrial dependencies. Full profile does not include mapping.
- Main script starts Livox/chassis/nav by default; indicator off by default. Service config validates an existing NAV_MAP_YAML even though mapping starts IDLE.
- Managed mapping defaults loop=false; manual mapping launch defaults loop=true, industrial.
- Indicator drives external GPIO4 strip, not GPIO21 onboard RGB. Serial ACK and robot FAULT are independent.
- Startup config is strict KEY=VALUE, not YAML; includes START_INDICATOR with one USB identity when enabled.
- Old README duplicates run/build commands and contains hardware-specific progress notes, stale absolute paths and per-update instructions.
