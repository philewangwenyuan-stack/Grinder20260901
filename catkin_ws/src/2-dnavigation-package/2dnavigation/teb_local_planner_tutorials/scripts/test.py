#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
import math

rospy.init_node("z_path_generator")

# 发布路径（Global Planner 订阅）
path_pub = rospy.Publisher("/move_base/GlobalPlanner/plan", Path, queue_size=1)

# ✅ 新增：发布路径给 TEB（同步）
# global_plan_pub = rospy.Publisher("/global_plan", Path, queue_size=1)

# 发布目标点（Move Base 接收）
goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

# 存储当前位姿
current_x = 0.0
current_y = 0.0
current_yaw = 0.0
odom_received = False

# ✅ 修复：目标点只发布一次（初始化为 False）
goal_published = False

def odom_callback(msg):
    """更新当前位姿"""
    global current_x, current_y, current_yaw, odom_received
    
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    
    q = msg.pose.pose.orientation
    current_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    
    odom_received = True

def safe_interpolate(p1, p2, step=0.1):
    """安全的插值，避免除零和NaN"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dist = math.sqrt(dx*dx + dy*dy)
    
    # ✅ 避免距离为0
    if dist < 1e-6:
        return [(p1[0], p1[1])]
    
    # ✅ 确保至少有2个点
    num_steps = max(2, int(dist / step))
    points = []
    
    for i in range(num_steps + 1):
        t = i / num_steps
        x = p1[0] + t * dx
        y = p1[1] + t * dy
        points.append((x, y))
    
    return points

def generate_safe_z_path(x_start, y_start, yaw_start):
    """生成安全的 Z 字路径，避免 TEB 崩溃"""
    # 先生成稀疏的关键点
    key_points = []
    step = 0.5           # 每段长度
    offset = 1.0         # Z 字横向偏移
    num_segments = 8     # Z 字段数
    
    x = x_start
    y = y_start
    yaw = yaw_start
    
    # 收集所有关键点
    for i in range(num_segments + 1):
        key_points.append((x, y))
        
        # Z 字形逻辑
        if i % 2 == 0:
            x += step * math.cos(yaw)
            y += step * math.sin(yaw)
        else:
            yaw += math.pi / 6  # 左偏 30°
            x += offset * math.cos(yaw)
            y += offset * math.sin(yaw)
    
    # 在关键点之间插值，密度为 0.1m
    dense_points = []
    for i in range(len(key_points) - 1):
        p1 = key_points[i]
        p2 = key_points[i + 1]
        interpolated = safe_interpolate(p1, p2, 0.1)
        dense_points.extend(interpolated)
    
    # 添加最后一个点
    dense_points.append(key_points[-1])
    
    # 确保路径点足够分散
    filtered_points = []
    min_dist = 0.05  # ✅ 最小间距
    for pt in dense_points:
        if not filtered_points:
            filtered_points.append(pt)
        else:
            last_pt = filtered_points[-1]
            dist = math.sqrt((pt[0]-last_pt[0])**2 + (pt[1]-last_pt[1])**2)
            if dist >= min_dist:
                filtered_points.append(pt)
    
    # 生成 Path 消息
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    
    poses = []
    for i, (px, py) in enumerate(filtered_points):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = 0.0
        
        # ✅ 关键修复：计算平滑的朝向
        if i < len(filtered_points) - 1:
            next_x, next_y = filtered_points[i + 1]
            point_yaw = math.atan2(next_y - py, next_x - px)
        else:
            # 最后一个点使用前一个点的朝向
            if i > 0:
                prev_x, prev_y = filtered_points[i - 1]
                point_yaw = math.atan2(py - prev_y, px - prev_x)
            else:
                point_yaw = yaw_start
        
        # ✅ 确保朝向是有限值
        if not math.isfinite(point_yaw):
            point_yaw = yaw_start
        
        qz = math.sin(point_yaw / 2.0)
        qw = math.cos(point_yaw / 2.0)
        pose.pose.orientation = Quaternion(0, 0, qz, qw)
        
        poses.append(pose)
    
    path.poses = poses
    return path, poses[-1]

# 订阅 odom
rospy.Subscriber("/odom", Odometry, odom_callback)

rospy.loginfo("Z-path generator started. Waiting for odometry...")

# ✅ 降低发布频率，给 TEB 缓冲时间
rate = rospy.Rate(2)  # 0.5Hz

while not rospy.is_shutdown():
    if odom_received:
        # 生成安全的路径
        path, last_pose = generate_safe_z_path(current_x, current_y, current_yaw)
        
        # ✅ 同时发布到两个话题
        path_pub.publish(path)           # 给 Global Planner
        # global_plan_pub.publish(path)     # ✅ 给 TEB
        
        # ✅ 目标点只发布一次，但连续发布两次确保接收
        if not goal_published:
            last_pose.header.stamp = rospy.Time.now()
            
            # ✅ 连续发布两次，确保 move_base 接收成功
            goal_pub.publish(last_pose)
            rospy.sleep(0.01)  # 微小延迟
            goal_pub.publish(last_pose)
            
            goal_published = True
            rospy.loginfo(f"✅ Goal published twice: ({last_pose.pose.position.x:.2f}, {last_pose.pose.position.y:.2f})")
        
        rospy.loginfo_throttle(5, f"Safe path published | Points: {len(path.poses)} | Goal sent: {goal_published}")
    else:
        rospy.logwarn_throttle(5, "Waiting for odometry data...")
    
    rate.sleep()