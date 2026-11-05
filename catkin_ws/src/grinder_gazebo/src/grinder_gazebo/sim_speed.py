"""Set Gazebo's target real-time factor without changing the physical step."""
import math
import time


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


def configure_speed_verified(speed, get_physics, set_physics, sleep=time.sleep):
    """Allow startup reconfigure callbacks to settle; never report an unverified rate."""
    rate = configure_speed(speed, get_physics, set_physics)
    stable = 0
    for _ in range(20):
        # Wall time is essential when Gazebo is paused or /clock has not started.
        sleep(0.25)
        physics = get_physics()
        if not physics.success:
            raise RuntimeError(physics.status_message)
        if math.isclose(physics.max_update_rate, rate, rel_tol=1e-9):
            stable += 1
            if stable >= 12:
                return rate
        else:
            stable = 0
            rate = configure_speed(speed, get_physics, set_physics)
    raise RuntimeError("Gazebo speed was overwritten during startup")


def main():
    import rospy
    from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties
    from dynamic_reconfigure.client import Client

    rospy.init_node("grinder_sim_speed")
    try:
        speed = rospy.get_param("~speed", 1.0)
        for service in ("/gazebo/get_physics_properties", "/gazebo/set_physics_properties"):
            rospy.wait_for_service(service, timeout=30.0)
        # Physics services precede the dynamic-reconfigure server. Its initial
        # callbacks (including gzclient) can otherwise restore the world rate.
        client = Client("/gazebo", timeout=30.0)
        if client.get_configuration(timeout=30.0) is None:
            raise RuntimeError("Gazebo physics configuration is not ready")
        rate = configure_speed_verified(speed,
            rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties),
            rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties))
        rospy.loginfo("Gazebo target speed: %sx (max update rate %.3f Hz); actual speed depends on compute capacity", speed, rate)
    except (ValueError, RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr("Cannot set Gazebo simulation speed: %s", exc)
        raise SystemExit(1)
