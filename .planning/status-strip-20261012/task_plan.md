# Control box status strip

Implement authorized ESP32-S3-Zero / USB-powered 10 cm (3 pixel) WS2812B indicator.

- [x] Inspect current state publishers and build integration.
- [x] Add state resolver, bounded USB protocol/transport and ROS node.
- [x] Add persistent display-only emergency sources and control activity publication.
- [x] Add ESP32 firmware, launch/config/docs and navigation index.
- [x] Run offline Python and firmware checks; report Linux/ROS/hardware limits.

Verification: firmware build passed; Python 3.8/Linux 18 indicator tests + 13 chassis tests passed (including C++ wire interoperability). Chassis Catkin build passed in isolated workspace. Scheduler Catkin configure blocked by unavailable slamware_ros_sdk dependency. No hardware or live motion tests.

Do not alter motion arbitration or implement G01/G02. Preserve existing super_lio.yaml edits.
