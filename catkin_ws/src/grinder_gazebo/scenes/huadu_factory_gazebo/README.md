# 花都构件厂 Gazebo 静态场景

来源：花都构件厂_精简无轴网版.ifc。适配 Gazebo Classic 11（本仓库 ROS1 Noetic 使用的版本），SDF 1.6。
包含厂房建筑、钢结构、机电构件的可视及碰撞几何；这是环境模型，未附带机器人或导航程序。

## 运行

解压后在 Ubuntu / WSL 中进入 `huadu_factory_gazebo`：

```bash
bash run.sh
```

纯服务器运行：

```bash
bash run.sh --headless
```

`run.sh` 自动设置模型搜索路径，所有模型资源随包提供，不需要联网下载。
已安装 ROS Noetic 和 gazebo_ros 时也可运行：

```bash
source /opt/ros/noetic/setup.bash
roslaunch ./launch/huadu_factory.launch
# 无 GUI：roslaunch ./launch/huadu_factory.launch gui:=false
```

模型可独立复制到 `~/.gazebo/models/huadu_factory`，或把本包 `models` 目录加入 `GAZEBO_MODEL_PATH`。

## 坐标与机器人放置

- 单位：米；Z 轴向上。IFC 毫米单位已转换为米。
- IFC 项目坐标原点平移：`[89.488228773, 145.768769448, 0.000000000] m`。
- 换算：`Gazebo坐标(m) = IFC项目坐标(mm) / 1000 - 上述平移`。不使用 IFC 的地理坐标 MapConversion。
- 首层原始楼板顶面保持在 `z=0`。地下部分保持在负 Z；没有额外添加会封住地下区域的无限地面。
- 仿真原点位于原始首层楼板覆盖且周边空旷的位置，已用碰撞承托和竖直激光射线验证。
- 仓库 X920 机器人可放在 `x=0, y=0, z=0.1675, yaw=0`。
- 场景范围约 192.03 × 294.84 × 29.02 m；完整坐标、构件来源及网格统计见 `source_report.json`。

现有 Grinder 的 `spawn_x920.launch` 固定加载原 B1 场景。本包不会替换该文件；直接运行本包即可查看新厂房。
若后续接入整套 Grinder，需让其 world 指向本包世界文件，并配置足够覆盖当前作业区域的地图范围。
当前仓库建图默认每轴 `[-60,+60]m`，不能直接覆盖整座厂房。

## 几何与碰撞

- 本次提取 51,931 个有几何的实体，合并为 131 组材质/类型网格。
- 首层、楼上楼板、地下构件、梁柱、墙、屋面、门窗和机电均保留在场景中。
- IFC 房间空间体积 `IfcSpace` 不作为实体导出；没有面片的空几何也跳过。没有未处理的几何转换失败。
- 上一版已删除的紧固件、短连接板、轴网保持删除。
- 视觉与碰撞共享三角网格，不用会填死门洞的整座建筑凸包。
- 清理重复顶点/面、使用保留开放边界的无损面简化，再剔除 STL 单精度退化面。
- 模型固定在世界中。原有静态门扇按 IFC 当前姿态保留，不含开门关节或电机。
- 仍有 5,413,705 个三角面，约 270.7 MB 网格。完整厂房加载和激光仿真有一定资源开销；实际机器人全链路实时性需按目标电脑验证。

## 文件

```text
models/huadu_factory/model.config
models/huadu_factory/model.sdf
models/huadu_factory/meshes/*.stl
worlds/huadu_factory.world
launch/huadu_factory.launch
run.sh
source_report.json
mesh_validation.json
gazebo_validation.json
```

已检查网格索引/有限坐标、STL长度、资源引用和地面朝向，并通过 Gazebo 11 的 SDF 检查及独立无 GUI 碰撞/射线测试。
测试结果见 `gazebo_validation.json`；未运行 X920、Super-LIO、导航或真实硬件。

格式说明参考：[Gazebo 模型结构](https://get.gazebosim.org/tutorials/?tut=build_model)、[SDFormat 1.6](https://sdformat.org/spec/1.6/model/)。
