# grinder_scheduler 包说明

本文档说明 `catkin_ws/src/grinder_scheduler` 目录下各文件/子目录的作用，便于开发、联调和排障。

## 1. 目录结构与作用

```text
grinder_scheduler/
├── CMakeLists.txt
├── package.xml
├── launch/
├── config/
├── msg/
├── scripts/
├── src/grinder_scheduler/
├── test/
└── README.md
```

## 2. 顶层文件

- `CMakeLists.txt`  
  Catkin 构建配置。定义消息生成、安装脚本、依赖链接等。

- `package.xml`  
  ROS 包元信息与依赖声明（编译依赖、运行依赖、版本、维护者等）。

## 3. 子目录说明

## 3.1 launch/

- 存放调度系统启动文件（`.launch`）。
- 常见用途：
  - 启动调度节点；
  - 组合启动底盘驱动、导航、雷达相关节点；
  - 暴露可配置参数（如 IP、地图路径、是否启用导航等）。

## 3.2 config/

- 调度程序 YAML 配置目录。
- 典型内容：
  - 机器人参数（宽度、长度、路径间距、转弯半径等）；
  - 运行时策略参数（话题名、频率、功能开关、持久化路径等）。

## 3.3 msg/

- 自定义 ROS 消息定义目录（`.msg`）。
- 例如：
  - `SchedulerStatus.msg`：调度状态发布结构；
  - `MapPreviewMetadata.msg`：地图预览元信息结构。

## 3.4 scripts/

- 可执行脚本入口（通常包含 ROS 节点启动脚本）。
- 常见是将 `src/grinder_scheduler/` 中核心 Python 模块通过脚本方式运行。

## 3.5 src/grinder_scheduler/

核心业务代码目录（Python 模块）。

- `scheduler_node.py`  
  调度主节点。  
  负责：
  - 任务配置/任务控制（TaskConfig/TaskCommand）；
  - 路径规划请求与状态推进；
  - 地图管理（保存、查询、删除、切换、预览）；
  - 与导航、底盘、雷达、SL-LinkA 协议的总调度逻辑；
  - 状态上报与本地持久化（map/task registry 等）。

- `models.py`  
  数据模型定义（任务配置、路径模型、状态模型等），用于统一内部结构体。

- `planner_adapter.py`  
  规划器适配层。把调度输入转换为规划器可用输入，并标准化输出路径结果。

- `map_service.py`  
  地图数据处理服务。包含地图叠加区域管理、地图预览相关处理等。

- `aurora_bridge.py`  
  雷达/Aurora 侧桥接层。负责获取地图、位姿、相机帧等传感输入。

- `sl_linka_adapter.py`  
  SL-LinkA 协议适配入口。解析协议消息并分发到调度处理函数。

- `map_catalog_response.py`  
  地图查询返回拼装工具（如 MapCatalogResponse 填充、缩略图预算控制）。

- `local_rtsp_server.py`  
  本地 RTSP 服务封装（配合 mediamtx/ffmpeg），用于视频流转发。

- `media_streamer.py`  
  媒体流推送状态管理，维护流在线状态与流地址信息。

## 3.6 test/

- 测试或联调用脚本目录（若存在）。  
- 用于本地开发验证，不属于线上运行主链路。

## 4. 运行主链路（简要）

1. 上位机/安卓通过 SL-LinkA 下发请求；  
2. `sl_linka_adapter.py` 解析后调用 `scheduler_node.py` 对应处理函数；  
3. `scheduler_node.py` 调用地图/规划/底盘/导航接口执行；  
4. 通过 TaskStatus/DeviceStatus/MapCatalog 等消息回传状态与结果。

## 5. 常见排查入口

- 任务不启动：优先看 `scheduler_node.py` 中 `handle_task_config` / `handle_task_command`。  
- 地图查询异常：看 `handle_map_catalog_request` 和 `map_registry` 持久化文件。  
- 路径预览异常：看 `handle_path_plan_request`、`planner_adapter.py`、预览图生成逻辑。  
- 底盘无动作：看 `cmd_vel` 转发、底盘驱动话题连接和 `chassis_driver` 日志。

