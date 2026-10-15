rostopic echo -n 1 /chassis/status
rostopic echo -n 1 /scheduler/status
rostopic echo -n 1 /chassis/task_enable
rostopic hz /odom_wheel
rostopic info /cmd_vel
