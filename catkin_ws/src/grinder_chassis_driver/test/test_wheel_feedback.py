"""Exercise production feedback methods with ROS-shaped messages, no hardware."""
import math
import threading
from types import SimpleNamespace as NS
import unittest
from test_motion_safety import load_class, DRIVER, Publisher, Stamp
import test_motion_safety as safety


def twist_message():
    return NS(header=NS(stamp=None, frame_id=''), twist=NS(
        covariance=[0.0]*36, twist=NS(linear=NS(x=0.0), angular=NS(z=0.0))))


class WheelFeedbackTest(unittest.TestCase):
    def node(self):
        cls, namespace = load_class(DRIVER, {'_publish_wheel_twist'})
        namespace['TwistWithCovarianceStamped'] = twist_message
        n = cls.__new__(cls)
        n.cmd_vel_wheel_radius_m, n.cmd_vel_wheel_track_m, n.cmd_vel_gear_ratio = 0.1475, 0.7, 60.0
        n.actual_speed_left_sign = n.actual_speed_right_sign = 1
        n.actual_speed_swap_lr = False
        n.actual_motor_speed_scale = 1.0
        n.wheel_twist_linear_variance = n.wheel_twist_angular_variance = 0.04
        n.wheel_twist_cross_covariance = 0.003
        n.wheel_twist_pub = Publisher()
        return n

    def test_signed_straight_reverse_and_turns_with_covariance(self):
        for left, right in ((100,100),(-100,-100),(-100,100),(100,-100),(0,0)):
            n = self.node()
            n._publish_wheel_twist(Stamp(10), left, right)
            msg = n.wheel_twist_pub.messages[-1]
            self.assertEqual('base_link',msg.header.frame_id)
            self.assertEqual(Stamp(10),msg.header.stamp)
            self.assertAlmostEqual((left+right)/2*2*math.pi*0.1475/3600,msg.twist.twist.linear.x)
            self.assertAlmostEqual((right-left)*2*math.pi*0.1475/3600/0.7,msg.twist.twist.angular.z)
            self.assertEqual([0.04,0.04,0.003,0.003],[msg.twist.covariance[i] for i in (0,35,5,30)])

    def test_bad_geometry_values_or_timestamp_never_publish(self):
        n = self.node()
        n.cmd_vel_wheel_radius_m = 0
        n._publish_wheel_twist(Stamp(10),100,100)
        self.assertEqual([],n.wheel_twist_pub.messages)
        n.cmd_vel_wheel_radius_m = 0.1475
        n._publish_wheel_twist(Stamp(0),100,100)
        n._publish_wheel_twist(Stamp(10),float('nan'),100)
        self.assertEqual([],n.wheel_twist_pub.messages)

    def test_poll_midpoint_and_failure_validity(self):
        fixture = safety.SafetyTest()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        n = fixture.driver()
        n.wheel_feedback_delay_sec=0.02
        n._poll_once(None)
        self.assertTrue(n.actual_speed_valid)
        self.assertIsNotNone(n._actual_feedback_stamp)
        good_stamp=n._actual_feedback_stamp
        n._memory_serial.fail=True
        n._poll_once(None)
        self.assertFalse(n.actual_speed_valid)
        self.assertEqual(good_stamp,n._actual_feedback_stamp)

    def test_transaction_midpoint_excludes_other_status_reads(self):
        cls, ns=load_class(DRIVER, {'_poll_once'})
        n=cls.__new__(cls)
        clock=[10.0]
        ns['time']=NS(monotonic=lambda:clock[0])
        ns['rospy']=NS(Time=NS(now=lambda:Stamp(100)),Duration=NS(from_sec=lambda x:x))
        n._outputs_ready=True
        n._enforce_command_timeout=lambda:None
        n._next_status_poll_monotonic=n._next_diagnostics_monotonic=1000
        n._lock=threading.RLock()
        n.wheel_feedback_delay_sec=0.02
        def read(*args,**kwargs):
            clock[0]=12.0
            return [-100,-100]
        n.transport=NS(read_register_block=read)
        n._mark_connected=lambda:None
        n._publish_state=lambda **kwargs:None
        n._poll_once(None)
        self.assertAlmostEqual(98.98,n._actual_feedback_stamp)

    def test_invalid_feedback_still_feeds_rpp_but_not_fusion(self):
        cls,ns=load_class(DRIVER, {'_publish_state'})
        ns['rospy']=NS(Time=NS(now=lambda:Stamp(100)))
        ns['WheelSpeedState']=lambda:NS(header=NS())
        n=cls.__new__(cls)
        n._lock=threading.RLock()
        n._motion=NS(latched=False,generation=1,error='')
        n._safe_write_confirmed=n._outputs_ready=True
        n._reset_error=''
        n.safety_pub=n.wheel_speed_pub=Publisher()
        n.command=ns['ChassisCommand']()
        n.snapshot=ns['RegisterSnapshot']()
        n.connected=True
        n.actual_left_motor_speed=n.actual_right_motor_speed=100
        n.actual_speed_valid=False
        n._actual_feedback_stamp=Stamp(99)
        n.enabled=True
        n.last_error=''
        n.consecutive_failures=0
        rpp,fusion=[],[]
        n._publish_wheel_odom=lambda *args:rpp.append(args)
        n._publish_wheel_twist=lambda *args:fusion.append(args)
        n._publish_state(publish_status=False)
        self.assertEqual(1,len(rpp))
        self.assertEqual([],fusion)
        n.actual_speed_valid=True
        n._publish_state(publish_status=False)
        self.assertEqual(1,len(fusion))
        self.assertEqual(Stamp(99),fusion[0][0])


if __name__ == '__main__':
    unittest.main()
