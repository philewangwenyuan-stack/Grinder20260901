# Super-LIO base pose correction

Implement the approved plan using measured base-to-laser calibration, preserving internal IMU states and map coordinates.

1. Inspect state conventions and launch/loop consumers — complete.
2. Add reusable pose/twist/covariance math and initial-pose conversion — complete.
3. Wire calibration through all mapping/localization launch paths — complete.
4. Add regression tests, document deployment and validate offline — complete.

No hardware actions authorized. Linux ROS build and bag replay require the target environment.

Errors: Catkin validation bootstrap used set -u before sourcing ROS; ROS setup references unset ROS_DISTRO. Fixed by enabling nounset after setup.
Full isolated Catkin configuration is blocked by missing livox_ros_driver2 in WSL. No dependency/source substitution performed. Validate the independent lightweight loop package separately.

Final validation: independent C++ numerical tests passed; 14 Windows scheduler/launch tests passed; lightweight super_lio_loop completed a real isolated Catkin build; Python 3.8 syntax, XML/YAML and scoped git diff checks passed. Full Super-LIO/industrial build and bag replay remain target-environment acceptance steps, not claimed as passed. No hardware run performed.
