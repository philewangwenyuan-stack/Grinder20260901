"""Set Gazebo's target real-time factor without changing the physical step."""
import math


def configure_speed(speed, get_physics, set_physics):
    speed = float(speed)
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("sim_speed must be a finite positive number")
    physics = get_physics()
    if not physics.success:
        raise RuntimeError(physics.status_message)
    step = float(physics.time_step)
    if not math.isfinite(step) or step <= 0:
        raise ValueError("Gazebo returned an invalid physics time step")
    rate = speed / step
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("sim_speed is outside the supported numeric range")
    result = set_physics(time_step=physics.time_step, max_update_rate=rate,
                         gravity=physics.gravity, ode_config=physics.ode_config)
    if not result.success:
        raise RuntimeError(result.status_message)
    return rate


def main():
    import rospy
    from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties

    rospy.init_node("grinder_sim_speed")
    try:
        speed = rospy.get_param("~speed", 1.0)
        for service in ("/gazebo/get_physics_properties", "/gazebo/set_physics_properties"):
            rospy.wait_for_service(service, timeout=30.0)
        rate = configure_speed(speed,
            rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties),
            rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties))
        rospy.loginfo("Gazebo target speed: %sx (max update rate %.3f Hz); actual speed depends on compute capacity", speed, rate)
    except (ValueError, RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr("Cannot set Gazebo simulation speed: %s", exc)
        raise SystemExit(1)
