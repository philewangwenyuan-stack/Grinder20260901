source /opt/ros/noetic/setup.bash
source "$GRINDER_WS/devel/setup.bash"
rospack find super_lio
rospack find livox_ros_driver2
ls -lh "$GRINDER_WS/devel/lib/super_lio/super_lio_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio/relocation_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/super_lio_loop_node"
ls -lh "$GRINDER_WS/devel/lib/super_lio_loop/super_lio_graph_backend_node"
