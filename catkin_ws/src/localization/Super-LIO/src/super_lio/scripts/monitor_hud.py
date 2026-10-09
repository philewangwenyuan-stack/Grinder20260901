#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import math
import psutil
import rospy
from collections import deque

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from jsk_rviz_plugins.msg import OverlayText
from geometry_msgs.msg import PoseStamped


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class MonitorHUD(object):
    def __init__(self):
        # ---------- 参数 ----------
        self.odom_topic        = rospy.get_param("~odom_topic", "/odom")
        self.imu_topic         = rospy.get_param("~imu_topic",  "/imu/data")
        # 与 C++ 发布的处理时间保持一致：/lio/processing_time, geometry_msgs/PoseStamped
        self.proc_time_topic   = rospy.get_param("~proc_time_topic", "/lio/processing_time")

        self.process_name      = rospy.get_param("~process_name", "")            # 要监控的进程名（部分匹配）
        self.cpu_interval      = rospy.get_param("~cpu_interval", 0.5)            # CPU 采样周期(s)
        self.cpu_window        = int(rospy.get_param("~cpu_window", 10))          # 滑窗样本数
        self.cpu_base_percent  = float(rospy.get_param("~cpu_base_percent", psutil.cpu_count() * 100.0)) # e.g. 8核=800

        # 里程计会话的“失活”阈值：超过该时间未收到里程计则结束一次统计
        self.odom_gap_timeout  = float(rospy.get_param("~odom_gap_timeout", 0.5))
        self.use_header_stamp   = bool(rospy.get_param("~use_header_stamp", False))
        self.debug              = bool(rospy.get_param("~debug", False))
        # 会话结束/未开始时是否把仪表清零
        self.reset_gauge_when_idle = bool(rospy.get_param("~reset_gauge_when_idle", True))

        # 输出话题（供 jsk_rviz_plugins 使用）
        self.pub_overlay_lin   = rospy.Publisher("~overlay_linear", OverlayText, queue_size=1, latch=True)
        self.pub_overlay_ang   = rospy.Publisher("~overlay_angular", OverlayText, queue_size=1, latch=True)
        # 处理时延三条曲线：当前/均值/标准差（单位随上游，建议毫秒）
        self.pub_proc_cur      = rospy.Publisher("~plotter_proc_time_current", Float32, queue_size=10)
        self.pub_proc_mean     = rospy.Publisher("~plotter_proc_time_mean",    Float32, queue_size=10)
        # self.pub_proc_std      = rospy.Publisher("~plotter_proc_time_std",     Float32, queue_size=10)
        # CPU 线性仪表（0~1）
        self.pub_cpu_ratio     = rospy.Publisher("~linear_gauge_cpu", Float32, queue_size=10)
        self.pub_cpu_avg_percent   = rospy.Publisher("~cpu_avg_percent", Float32, queue_size=10)

        self.pub_badge_cur  = rospy.Publisher("~badge_current", OverlayText, queue_size=1, latch=True)
        self.pub_badge_mean = rospy.Publisher("~badge_mean",    OverlayText, queue_size=1, latch=True)
        self.pub_badge_cpu  = rospy.Publisher("~badge_cpu",     OverlayText, queue_size=1, latch=True)
        self.pub_mean_cpu   = rospy.Publisher("~badge_cpu_mean",     OverlayText, queue_size=1, latch=True)

        self.pub_overlay_current_time   = rospy.Publisher("~overlay_current_time", OverlayText, queue_size=1, latch=True)
        # self.pub_overlay_mean_time   = rospy.Publisher("~overlay_mean_time", OverlayText, queue_size=1, latch=True)
        self.pub_overlay_current_cpu   = rospy.Publisher("~overlay_current_cpu", OverlayText, queue_size=1, latch=True)
        # self.pub_overlay_mean_cpu   = rospy.Publisher("~overlay_mean_cpu", OverlayText, queue_size=1, latch=True)


        self.cpu_sum_valid   = 0.0
        self.cpu_count_valid = 0


        # 订阅
        rospy.Subscriber(self.odom_topic, Odometry, self.cb_odom, queue_size=50)
        rospy.Subscriber(self.imu_topic,  Imu,     self.cb_imu,  queue_size=50)
        rospy.Subscriber(self.proc_time_topic, PoseStamped, self.cb_proc_time_pose, queue_size=50)

        # 状态
        self.lin_speed = 0.0
        self.ang_speed = 0.0

        # 里程计活动会话状态
        self.last_odom_ts = None   # float (ROS time, seconds)
        self.odom_active  = False  # 是否处于“统计会话”中

        # CPU 监控
        self.cpu_samples   = deque(maxlen=self.cpu_window)
        self.proc_handle   = None   # psutil.Process

        # 先尝试锁定进程（也可在第一次 tick 时再找）
        if self.process_name:
            self.proc_handle = self._find_process(self.process_name)
            if self.proc_handle:
                # 首次调用丢弃，建立基线
                try:
                    self.proc_handle.cpu_percent(interval=None)
                except Exception:
                    pass

        # 定时器
        self.timer = rospy.Timer(rospy.Duration(self.cpu_interval), self.tick, oneshot=False)

        rospy.loginfo("sys_monitor_hud ready. base=%.1f%%, interval=%.3fs, window=%d, odom_gap=%.3fs",
                      self.cpu_base_percent, self.cpu_interval, self.cpu_window, self.odom_gap_timeout)

    # ----------- Callbacks -----------
    def cb_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.lin_speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        self.publish_linear_text(self.lin_speed)

        self.pub_overlay_current_time.publish(self._make_overlay("Process Time Current & Mean"))
        # self.pub_overlay_mean_time.publish(self._make_overlay("Process Time Mean"))
        self.pub_overlay_current_cpu.publish(self._make_overlay("CPU Usage Current & Mean"))
        # self.pub_overlay_mean_cpu.publish(self._make_overlay("CPU Usage Mean"))

        # 记录最新“收到消息”的时间（默认用收包时间，避免 header.stamp 与当前时钟系不一致导致抖动）
        if self.use_header_stamp and msg.header and msg.header.stamp and msg.header.stamp.to_sec() > 0.0:
            self.last_odom_ts = msg.header.stamp.to_sec()
        else:
            self.last_odom_ts = rospy.get_time()

        # 若之前不活跃，收到里程计 -> 开启新会话
        if not self.odom_active:
            self.start_cpu_session()

    def cb_imu(self, msg: Imu):
        w = msg.angular_velocity
        self.ang_speed = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z) * (180.0 / math.pi)
        self.publish_angular_text(self.ang_speed)

    def cb_proc_time_pose(self, msg: PoseStamped):
        # 约定：x=当前处理时间, y=均值, z=标准差（单位由上游决定，建议毫秒）
        cur = float(msg.pose.position.x)
        mean = float(msg.pose.position.y)
        std = float(msg.pose.position.z)

        f = Float32()
        f.data = cur
        self.pub_proc_cur.publish(f)
        self.pub_badge_cur.publish(self._make_overlay("%.2f ms" % cur))

        f = Float32()
        f.data = mean
        self.pub_proc_mean.publish(f)
        self.pub_badge_mean.publish(self._make_overlay("%.2f ms" % mean))

        # f = Float32()
        # f.data = std
        # self.pub_proc_std.publish(f)

    # ----------- Overlay publishers -----------
    def _make_overlay(self, text):
        o = OverlayText()
        o.text = text
        o.width, o.height = 360, 70
        o.left, o.top = 10, 10
        o.text_size = 18.0
        o.line_width = 2
        o.bg_color.a = 0.35
        o.bg_color.r, o.bg_color.g, o.bg_color.b = 0.0, 0.0, 0.0
        o.fg_color.a = 1.0
        o.fg_color.r, o.fg_color.g, o.fg_color.b = 1.0, 1.0, 1.0
        return o

    def publish_linear_text(self, v):
        txt = "Linear_ Speed  | |v| = %.2f m/s" % v
        o = self._make_overlay(txt)
        self.pub_overlay_lin.publish(o)

    def publish_angular_text(self, w):
        txt = "Angular Speed | |ω| = %.2f deg/s" % w
        o = self._make_overlay(txt)
        o.left, o.top = 10, 90  # 右上角显示一个（示意）
        self.pub_overlay_ang.publish(o)

    # ----------- CPU 相关 -----------
    def _find_process(self, name_part):
        name_part = name_part.lower()
        for p in psutil.process_iter(attrs=["pid", "name"]):
            try:
                if name_part in (p.info["name"] or "").lower():
                    rospy.loginfo("CPU monitor attached to %s (pid=%d)", p.info["name"], p.info["pid"])
                    return psutil.Process(p.info["pid"])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        rospy.logwarn("No process matched: %s", name_part)
        return None

    def _ensure_proc_handle(self):
        if self.proc_handle:
            return
        if self.process_name:
            self.proc_handle = self._find_process(self.process_name)
            if self.proc_handle:
                try:
                    self.proc_handle.cpu_percent(interval=None)  # 建立基线
                except Exception:
                    pass
        else:
            # 默认当前进程
            self.proc_handle = psutil.Process(os.getpid())
            try:
                self.proc_handle.cpu_percent(interval=None)
            except Exception:
                pass

    def _sample_cpu_percent(self):
        self._ensure_proc_handle()
        try:
            # 非阻塞采样：相对于上次调用的区间百分比（全核加总，可能>100）
            val = self.proc_handle.cpu_percent(interval=None)
            return max(0.0, float(val))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            self.proc_handle = None
            return 0.0

    def start_cpu_session(self):
        self.cpu_samples.clear()
        # 重置采样基线
        self._ensure_proc_handle()
        try:
            if self.proc_handle:
                self.proc_handle.cpu_percent(interval=None)
        except Exception:
            pass
        self.odom_active = True
        rospy.loginfo("CPU session START (odom active)")

    def end_cpu_session(self):
        avg_percent = sum(self.cpu_samples) / float(len(self.cpu_samples)) if self.cpu_samples else 0.0
        base = max(1e-6, self.cpu_base_percent)
        ratio = clamp(avg_percent / base, 0.0, 1.0)
        rospy.loginfo("CPU session END: avg=%.2f%%, ratio=%.3f (n=%d)", avg_percent, ratio, len(self.cpu_samples))
        self.cpu_samples.clear()
        self.odom_active = False
        if self.reset_gauge_when_idle:
            out = Float32(); out.data = 0.0
            self.pub_cpu_ratio.publish(out)

    def tick(self, _event):
        now = rospy.get_time()

        # 如果会话中，但超过阈值未收到里程计 -> 结束会话
        if self.odom_active and self.last_odom_ts is not None:
            gap = now - self.last_odom_ts
            if self.debug:
                rospy.logdebug("odom gap = %.3fs (timeout=%.3fs)", gap, self.odom_gap_timeout)
            if gap > self.odom_gap_timeout:
                self.end_cpu_session()
                return

        # 未处于会话，不采样
        if not self.odom_active:
            return

        # 采样
        raw_percent = self._sample_cpu_percent()

        self.cpu_sum_valid   += raw_percent
        self.cpu_count_valid += 1
        
        self.cpu_samples.append(raw_percent)
        avg_percent = sum(self.cpu_samples) / float(len(self.cpu_samples))

        # 归一化到 0~1（按配置的“核总百分比”）
        base = max(1e-6, self.cpu_base_percent)
        ratio = clamp(avg_percent / base, 0.0, 1.0)

        # 发布给 LinearGauge（Float32，0~1）
        out = Float32(); out.data = ratio
        self.pub_cpu_ratio.publish(out)
        self.pub_badge_cpu.publish(self._make_overlay("%.1f%%" % (ratio * 100.0)))

        avg_pct = 0.0
        if self.cpu_count_valid > 0:
            avg_pct = clamp((self.cpu_sum_valid / self.cpu_count_valid) / base * 100.0, 0.0, 100.0)
        
        o = Float32(); o.data = avg_pct
        self.pub_cpu_avg_percent.publish(o)
        self.pub_mean_cpu.publish(self._make_overlay("%.1f%%" % avg_pct))



if __name__ == "__main__":
    rospy.init_node("sys_monitor_hud")
    node = MonitorHUD()
    rospy.spin()
