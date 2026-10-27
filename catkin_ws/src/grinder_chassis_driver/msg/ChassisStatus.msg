std_msgs/Header header
bool connected
bool enabled
uint16 work_mode
int16 disc_speed_target
int16 disc_speed_feedback
bool disc_enabled
uint16 disc_lift_state
bool light_enabled
uint32 consecutive_failures
string last_error
# Read-only signed int16 values from 0x4019 / 0x401A.
int16 left_motor_fault_code
int16 right_motor_fault_code
