cd "$GRINDER_BASE_DIR"
if [ ! -f catkin_ws/src/grinder_scheduler/config/scheduler.local.yaml ]; then
  cp catkin_ws/src/grinder_scheduler/config/scheduler.local.example.yaml \
    catkin_ws/src/grinder_scheduler/config/scheduler.local.yaml
fi
