import os
import math
import threading
import time

import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion


class AuroraBridge:
    def __init__(
        self,
        map_topic,
        odom_topic,
        left_image_topic,
        right_image_topic,
        first_frame_save_dir="",
        save_first_frame_on_startup=False,
        use_depth_colorized_image=False,
        depth_image_colorized_topic="",
        localization_quality_position_reference_m=0.1,
        localization_quality_heading_reference_deg=10.0,
        localization_quality_unavailable_variance=1.0e6,
    ):
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._map_msg = None
        self._pose = {
            "x": 0.0,
            "y": 0.0,
            "heading_deg": 0.0,
            "odom_available": False,
            "linear_speed_mps": 0.0,
            "angular_speed_radps": 0.0,
            "odom_received_monotonic": 0.0,
            "odom_timestamp_ns": 0,
            "localization_quality_available": False,
            "localization_quality": 0,
        }
        self._localization_quality_position_reference_m = max(
            1.0e-6, float(localization_quality_position_reference_m)
        )
        self._localization_quality_heading_reference_rad = max(
            1.0e-6, math.radians(float(localization_quality_heading_reference_deg))
        )
        self._localization_quality_unavailable_variance = max(
            1.0e-6, float(localization_quality_unavailable_variance)
        )
        self._initial_pose = None
        self._left_image = None
        self._right_image = None
        self._depth_image = None
        self._left_stamp = rospy.Time(0)
        self._right_stamp = rospy.Time(0)
        self._depth_stamp = rospy.Time(0)
        self._left_recv_stamp = rospy.Time(0)
        self._right_recv_stamp = rospy.Time(0)
        self._depth_recv_stamp = rospy.Time(0)
        default_dir = os.path.abspath(os.path.join(os.getcwd(), "temp"))
        self._first_frame_save_dir = first_frame_save_dir.strip() or default_dir
        self._save_first_frame_on_startup = bool(save_first_frame_on_startup)
        self._saved_left_frame = False
        self._saved_right_frame = False
        self._saved_depth_frame = False
        self._use_depth_colorized_image = bool(use_depth_colorized_image)

        rospy.Subscriber(map_topic, OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self._odom_callback, queue_size=10)
        if self._use_depth_colorized_image:
            rospy.Subscriber(
                depth_image_colorized_topic,
                Image,
                self._depth_image_callback,
                queue_size=1,
                buff_size=2 ** 24,
                tcp_nodelay=True,
            )
        else:
            rospy.Subscriber(
                left_image_topic,
                Image,
                self._left_image_callback,
                queue_size=1,
                buff_size=2 ** 24,
                tcp_nodelay=True,
            )
            rospy.Subscriber(
                right_image_topic,
                Image,
                self._right_image_callback,
                queue_size=1,
                buff_size=2 ** 24,
                tcp_nodelay=True,
            )

    def _map_callback(self, msg):
        with self._lock:
            self._map_msg = msg

    def _odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        covariance = list(getattr(msg.pose, "covariance", []) or [])
        localization_covariance = {"valid": False}
        localization_quality_available = False
        localization_quality = 0
        if len(covariance) >= 36:
            # nav_msgs/Odometry pose covariance is a 6x6 matrix ordered as
            # x, y, z, rotation about X, rotation about Y, rotation about Z.
            x_variance = float(covariance[0])
            y_variance = float(covariance[7])
            yaw_variance = float(covariance[35])
            covariance_valid = all(
                math.isfinite(value) and 0.0 <= value < self._localization_quality_unavailable_variance
                for value in (x_variance, y_variance, yaw_variance)
            )
            if covariance_valid:
                localization_covariance = {
                    "valid": True,
                    "x_variance": x_variance,
                    "y_variance": y_variance,
                    "yaw_variance": yaw_variance,
                    "xy_covariance": float(covariance[1]),
                    "x_yaw_covariance": float(covariance[5]),
                    "y_yaw_covariance": float(covariance[11]),
                }
                position_std_m = math.sqrt((x_variance + y_variance) * 0.5)
                heading_std_rad = math.sqrt(yaw_variance)
                normalized_uncertainty = (
                    position_std_m / self._localization_quality_position_reference_m
                    + heading_std_rad / self._localization_quality_heading_reference_rad
                )
                localization_quality_available = True
                localization_quality = max(
                    0, min(100, int(round(100.0 / (1.0 + normalized_uncertainty))))
                )
        pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "heading_deg": yaw * 180.0 / 3.141592653589793,
            "odom_available": True,
            "linear_speed_mps": float(
                (
                    msg.twist.twist.linear.x ** 2
                    + msg.twist.twist.linear.y ** 2
                    + msg.twist.twist.linear.z ** 2
                )
                ** 0.5
            ),
            "angular_speed_radps": float(
                (
                    msg.twist.twist.angular.x ** 2
                    + msg.twist.twist.angular.y ** 2
                    + msg.twist.twist.angular.z ** 2
                )
                ** 0.5
            ),
            "odom_received_monotonic": float(time.monotonic()),
            "odom_timestamp_ns": int(msg.header.stamp.to_nsec()),
            "orientation": {
                "x": float(orientation.x),
                "y": float(orientation.y),
                "z": float(orientation.z),
                "w": float(orientation.w),
            },
            "localization_covariance": localization_covariance,
            "localization_quality_available": localization_quality_available,
            "localization_quality": localization_quality,
        }
        with self._lock:
            self._pose = pose
            if self._initial_pose is None:
                self._initial_pose = dict(pose)

    def _left_image_callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert left image: %s", exc)
            return
        with self._lock:
            self._left_image = image
            self._left_stamp = msg.header.stamp
            self._left_recv_stamp = rospy.Time.now()
            should_save = self._save_first_frame_on_startup and (not self._saved_left_frame)
            if should_save:
                self._saved_left_frame = True
        if should_save:
            self._save_first_frame("left", image, msg.header.stamp)

    def _right_image_callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert right image: %s", exc)
            return
        with self._lock:
            self._right_image = image
            self._right_stamp = msg.header.stamp
            self._right_recv_stamp = rospy.Time.now()
            should_save = self._save_first_frame_on_startup and (not self._saved_right_frame)
            if should_save:
                self._saved_right_frame = True
        if should_save:
            self._save_first_frame("right", image, msg.header.stamp)

    def _save_first_frame(self, side, image, stamp):
        try:
            os.makedirs(self._first_frame_save_dir, exist_ok=True)
            stamp_ns = stamp.to_nsec() if stamp and stamp != rospy.Time(0) else 0
            filename = os.path.join(self._first_frame_save_dir, "aurora_first_{}_{}.jpg".format(side, stamp_ns))
            if cv2.imwrite(filename, image):
                rospy.loginfo("Saved first %s Aurora image to %s", side, filename)
            else:
                rospy.logwarn("Failed to save first %s Aurora image to %s", side, filename)
        except Exception as exc:
            rospy.logwarn("Failed to save first %s Aurora image: %s", side, exc)

    def _depth_image_callback(self, msg):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert depth_image_colorized: %s", exc)
            return
        with self._lock:
            self._depth_image = image
            self._depth_stamp = msg.header.stamp
            self._depth_recv_stamp = rospy.Time.now()
            should_save = self._save_first_frame_on_startup and (not self._saved_depth_frame)
            if should_save:
                self._saved_depth_frame = True
        if should_save:
            self._save_first_frame("depth_colorized", image, msg.header.stamp)

    def get_map(self):
        with self._lock:
            return self._map_msg

    def get_pose(self):
        with self._lock:
            return dict(self._pose)

    def get_initial_pose(self):
        with self._lock:
            return None if self._initial_pose is None else dict(self._initial_pose)

    def get_latest_frames(self):
        with self._lock:
            if self._use_depth_colorized_image:
                depth = None if self._depth_image is None else self._depth_image.copy()
                stamp = self._depth_recv_stamp if self._depth_recv_stamp != rospy.Time(0) else self._depth_stamp
                return depth, None, stamp, rospy.Time(0)
            left = None if self._left_image is None else self._left_image.copy()
            right = None if self._right_image is None else self._right_image.copy()
            left_stamp = self._left_recv_stamp if self._left_recv_stamp != rospy.Time(0) else self._left_stamp
            right_stamp = self._right_recv_stamp if self._right_recv_stamp != rospy.Time(0) else self._right_stamp
            return left, right, left_stamp, right_stamp
