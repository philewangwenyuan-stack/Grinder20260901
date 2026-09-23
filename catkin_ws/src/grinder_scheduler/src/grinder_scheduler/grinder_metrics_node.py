"""Optional 1 Hz Linux/ROS health sampler for RK3588 deployments."""

import os
import time

import rosnode
import rospy

from grinder_scheduler.metrics_exporter import MetricsExporter
from grinder_scheduler.metrics_registry import MetricsRegistry


def _cpu_times():
    with open("/proc/stat", "r", encoding="ascii") as handle:
        fields = [int(value) for value in handle.readline().split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    # guest/guest_nice 已包含在 user/nice 中，不重复加入总 CPU 时间。
    return sum(fields[:8]), idle


def _memory():
    values = {}
    with open("/proc/meminfo", "r", encoding="ascii") as handle:
        for line in handle:
            key, _, tail = line.partition(":")
            if key in ("MemTotal", "MemAvailable"):
                values[key] = int(tail.strip().split()[0]) * 1024
    return values


def main():
    rospy.init_node("grinder_metrics")
    metrics = MetricsRegistry()
    export_dir = rospy.get_param("~metrics_export_dir", "/tmp/grinder_metrics")
    exporter = MetricsExporter(
        metrics, export_dir, "grinder_metrics",
        interval_sec=rospy.get_param("~metrics_export_interval_sec", 10.0),
        logwarn=rospy.logwarn,
    )
    exporter.start()
    rospy.on_shutdown(exporter.stop)
    previous_total, previous_idle = _cpu_times()
    next_ros_check = 0.0
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            total, idle = _cpu_times()
            delta = total - previous_total
            if delta > 0:
                # 1 秒采一次系统 CPU；采样值仅存内存，后台线程定时导出 JSON。
                metrics.gauge("system_cpu_percent").set(100.0 * (1.0 - (idle - previous_idle) / delta))
            previous_total, previous_idle = total, idle
            memory = _memory()
            metrics.gauge("system_memory_total_bytes").set(memory.get("MemTotal", 0))
            metrics.gauge("system_memory_available_bytes").set(memory.get("MemAvailable", 0))
            metrics.gauge("system_process_count").set(sum(name.isdigit() for name in os.listdir("/proc")))
            if time.monotonic() >= next_ros_check:
                # 查询 ROS master 的频率较低，避免每秒网络调用拖慢健康采样。
                metrics.gauge("ros_node_count").set(len(rosnode.get_node_names()))
                next_ros_check = time.monotonic() + 10.0
        except Exception as exc:
            metrics.counter("system_sample_errors").inc()
            rospy.logwarn_throttle(30.0, "System metrics sample failed: %s", exc)
        rate.sleep()
