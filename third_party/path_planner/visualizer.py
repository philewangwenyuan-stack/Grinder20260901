import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json

# 导入我们之前写的自适应规划器和障碍物类
from hybrid_planner_adaptive import AdaptiveHybridPlanner, RectObstacle, CircleObstacle, PolygonObstacle

def quaternion_to_yaw(**kwargs) -> float:
    """兼容版：无论传入的是 x 还是 qx，都能正确解析偏航角"""
    x = kwargs.get('x', kwargs.get('qx', 0.0))
    y = kwargs.get('y', kwargs.get('qy', 0.0))
    z = kwargs.get('z', kwargs.get('qz', 0.0))
    w = kwargs.get('w', kwargs.get('qw', 1.0))
    
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def draw_arrow(ax, x, y, yaw, length=1.5, color='black'):
    """在图上画一个表示方向的箭头（按比例动态调整头部大小）"""
    dx = length * math.cos(yaw)
    dy = length * math.sin(yaw)
    
    # 让箭头头部的宽度和长度，变成整体长度的 30% 左右
    hw = length * 0.3
    hl = length * 0.3
    
    ax.arrow(x, y, dx, dy, head_width=hw, head_length=hl, fc=color, ec=color, zorder=4)

def visualize():
    # ==========================================
    # 1. 手动设置输入参数 
    # ==========================================
    input_data = {
        "start": {
            "point": {"x": 0.0, "y": 0.0},
            "orientation": {"qx": 0.0, "qy": 0.1, "qz": 1.0, "qw": 1.0} # Yaw = 0 (朝东)
        },
        "goal": {
            "point": {"x": 17.0, "y": 17.0},
            "orientation": {"qx": 1.0, "qy": 0.50, "qz": 0.7071, "qw": 0.0071} # Yaw = 90度 (朝北)
        },
        "parameters": {
            "safety_margin": 1.6,
            "turning_radius": 1.4,
            "point_spacing": 0.5
        }
    }

    # 设置障碍物实体
    obstacles = [
        CircleObstacle(x=8.0, y=2.0, r=2.0),
        RectObstacle(x1=12.0, y1=6.0, x2=15.0, y2=18.0),
        PolygonObstacle(points=[(18.0, 0.0), (22.0, 0.0), (22.0, 8.0), (18.0, 5.0)])
    ]

    # ==========================================
    # 2. 调用规划器生成路径
    # ==========================================
    planner = AdaptiveHybridPlanner()
    print("正在计算路径，请稍候...")
    result = planner.plan(
        start=input_data["start"],
        goal=input_data["goal"],
        obstacles=obstacles,
        safety_margin=input_data["parameters"]["safety_margin"],
        turning_radius=input_data["parameters"]["turning_radius"],
        point_spacing=input_data["parameters"]["point_spacing"]
    )

    if not result["ok"]:
        print(f"规划失败: {result['message']}")
        return

    print("规划成功！正在生成可视化图像...")
    path_data = result["path"]

    import json
    output_filename = "path_output.json"
    with open(output_filename, "w", encoding="utf-8") as f:
        # 直接存 path_data，它就是一个纯粹的数组 [...]
        json.dump(path_data, f, indent=4, ensure_ascii=False)
    print(f"✅ 路径数据已成功导出至文件: {output_filename}")

    # ==========================================
    # 3. 使用 Matplotlib 绘图
    # ==========================================
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal') # 保证 X 轴和 Y 轴比例 1:1，这样圆才不会变成椭圆

    # 3.1 画障碍物 (画出实际大小和膨胀后的安全边界)
    safety_margin = input_data["parameters"]["safety_margin"]
    for obs in obstacles:
        if isinstance(obs, CircleObstacle):
            # 膨胀区 (浅红色)
            ax.add_patch(patches.Circle((obs.x, obs.y), obs.r + safety_margin, color='lightcoral', alpha=0.3))
            # 实体区 (深红色)
            ax.add_patch(patches.Circle((obs.x, obs.y), obs.r, color='darkred', zorder=2))
        
        elif isinstance(obs, RectObstacle):
            width = obs.x2 - obs.x1
            height = obs.y2 - obs.y1
            # 膨胀区 (利用 Rectangle 近似表示边界)
            ax.add_patch(patches.Rectangle((obs.x1 - safety_margin, obs.y1 - safety_margin), 
                                           width + 2*safety_margin, height + 2*safety_margin, color='lightcoral', alpha=0.3))
            # 实体区
            ax.add_patch(patches.Rectangle((obs.x1, obs.y1), width, height, color='darkred', zorder=2))
            
        elif isinstance(obs, PolygonObstacle):
            # 1. 尝试使用 shapely 画出精确的膨胀区
            try:
                from shapely.geometry import Polygon as ShapelyPolygon
                poly = ShapelyPolygon(obs.points)
                inflated_poly = poly.buffer(safety_margin) # 向外膨胀
                x, y = inflated_poly.exterior.xy
                ax.fill(x, y, color='lightcoral', alpha=0.3, zorder=1) # 浅红色膨胀区
            except ImportError:
                print("提示: 未安装 shapely 库，将不显示多边形的膨胀区。")
                
            # 2. 画实体区
            ax.add_patch(patches.Polygon(obs.points, closed=True, color='darkred', zorder=2))

    # 3.2 画路径线
    xs = [p["point"]["x"] for p in path_data]
    ys = [p["point"]["y"] for p in path_data]
    ax.plot(xs, ys, color='blue', linewidth=2, label='Hybrid A* Path', zorder=3)

    # 3.3 画起点和终点的朝向箭头
    sx, sy = input_data["start"]["point"]["x"], input_data["start"]["point"]["y"]
    syaw = quaternion_to_yaw(**input_data["start"]["orientation"])
    draw_arrow(ax, sx, sy, syaw, length=2.0, color='green')
    ax.plot(sx, sy, 'go', markersize=8, label='Start') # 起点绿圆

    gx, gy = input_data["goal"]["point"]["x"], input_data["goal"]["point"]["y"]
    gyaw = quaternion_to_yaw(**input_data["goal"]["orientation"])
    draw_arrow(ax, gx, gy, gyaw, length=2.0, color='purple')
    ax.plot(gx, gy, 'mo', markersize=8, label='Goal') # 终点紫圆

    # 3.4 每隔几个点画一个小箭头，展示车辆行驶过程中的姿态
    step_for_arrow = max(1, len(path_data) // 15) # 大概画15个箭头
    for i in range(0, len(path_data), step_for_arrow):
        px, py = path_data[i]["point"]["x"], path_data[i]["point"]["y"]
        pyaw = quaternion_to_yaw(**path_data[i]["orientation"])
        draw_arrow(ax, px, py, pyaw, length=0.8, color='deepskyblue')

    # 3.5 设置图表样式
    ax.set_title("Hybrid A* Path Planning Visualization", fontsize=16)
    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left')

    # 自动调整视野范围
    all_x = xs + [obs.x for obs in obstacles if isinstance(obs, CircleObstacle)] + \
            [obs.x1 for obs in obstacles if isinstance(obs, RectObstacle)] + \
            [obs.x2 for obs in obstacles if isinstance(obs, RectObstacle)]
    all_y = ys + [obs.y for obs in obstacles if isinstance(obs, CircleObstacle)] + \
            [obs.y1 for obs in obstacles if isinstance(obs, RectObstacle)] + \
            [obs.y2 for obs in obstacles if isinstance(obs, RectObstacle)]
            
    margin = 5.0
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    # 显示窗口
    plt.show()

if __name__ == "__main__":
    visualize()