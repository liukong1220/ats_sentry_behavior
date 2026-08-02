# ats_sentry_behavior

ATS 哨兵行为树决策仓库，负责把比赛状态、视觉目标、资源状态和巡逻策略转换为
可取消、可抢占的导航目标与云台/底盘模式请求。

> 当前正式参数已提供 ATS 自研 action `/ats_navigate_to_pose`；Nav2 action
> 插件仍为明确的对照 profile 保留，尚未完成构建依赖清理。

## 目录

- [功能模块](#功能模块)
- [依赖](#依赖)
- [Quick Start](#quick-start)
- [启动节点](#启动节点)
- [输入输出](#输入输出)
- [配置文件](#配置文件)
- [数据流](#数据流)
- [目录结构](#目录结构)
- [测试与边界](#测试与边界)
- [参考与致谢](#参考与致谢)

## 功能模块

| 模块 | 说明 |
| :--- | :--- |
| 行为树服务端/客户端 | 加载 XML、维护黑板并执行行为树 |
| `SendAtsNavGoal` | 调用 `ats_navigation_interfaces/action/NavigateToPose` |
| Nav2 对照插件 | `SendNav2Goal`、`SendNavThroughPoses`，仅用于基线 |
| 目标选择 | 固定点、巡逻点、补给点、撤退点和视觉环绕目标 |
| 视觉接管 | 基于目标有效性、可通行栅格和滞回生成局部目标 |
| 资源门控 | 弹量、血量、受击、补给状态和比赛阶段决策 |
| 姿态输出 | 云台角速度/绝对角、机器人模式与自旋命令 |

行为层选择“去哪里”和“何时取消/抢占”，不拥有地图融合、路径安全、MPC 或
底盘速度最终授权。

## 依赖

- ROS 2 Humble
- BehaviorTree.CPP/`behaviortree_ros2`
- `ats_navigation_interfaces`、`ats_rm_interfaces`、`sp_msgs`
- `geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf2_ros`
- `nav2_msgs` 当前仅用于对照 action 插件

## Quick Start

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-up-to ats_sentry_behavior \
  --parallel-workers 1 --symlink-install
source install/setup.bash
```

## 启动节点

```bash
ros2 launch ats_sentry_behavior ats_sentry_behavior_launch.py \
  params_file:=src/ats_sentry_bringup/params/node_params.yaml
```

正常部署由 `ats_sentry_bringup` 外层入口启动。launch 同时创建：

- `ats_sentry_behavior_server`
- `ats_sentry_behavior_client`

两者共享同一参数文件和 namespace。

## 输入输出

### 导航接口

| Profile | Action | Server | 约束 |
| :--- | :--- | :--- | :--- |
| 正式 Nav2-free | `/ats_navigate_to_pose` | `ats_goal_manager` | 使用 `SendAtsNavGoal`，支持 feedback/result/cancel/preempt/timeout |
| Nav2 对照 | `/navigate_to_pose`、`/navigate_through_poses` | Nav2 navigator | 只用于显式对照入口 |

两组插件可以共存于源码，但运行 profile 必须互斥。Nav2-free 运行图中没有
`bt_navigator`，不得让正式树回退调用 Nav2 action。

### 主要输入

- 视觉目标和敌方状态：来自 `sp_vision25`/`sp_msgs`。
- 裁判系统、血量、弹量和比赛状态：来自 `ats_rm_interfaces`。
- 当前位姿与定位状态：正式 profile 使用 `/localization` 与 `/localization/status`。
- 地图状态：正式 profile 使用 `/rog_map_adapter/status`/`ready`。
- 行为树启动/控制 action：`ats_sentry_behavior`。

### 主要输出

- ATS/Nav2 导航 action goal，取决于 profile。
- 云台角速度、云台绝对角和 yaw authority 请求。
- `decision/robot_mode`、`cmd_spin` 等模式输出。
- 视觉跟随路径、候选点和 Marker 调试输出。

## 配置文件

| 文件 | 用途 |
| :--- | :--- |
| `../ats_sentry_bringup/params/node_params.yaml` | 正式 profile 的 behavior 参数唯一权威来源 |
| `params/sentry_behavior.example.yaml` | standalone 调试时显式传入的示例，不由任何正式入口默认加载 |
| `params/sentry_behavior_loopback.yaml` | loopback/Nav2 对照 |
| `params/sentry_behavior_decision_vision_test.yaml` | 视觉接管专项测试 |
| `params/rmuc_waypoints.csv` | 场地目标点 |
| `params/rmuc_patrol_waypoints.csv` | 巡逻序列 |
| `behavior_trees/*.xml` | 行为树拓扑 |

未来“统一总 YAML”迁移只应统一 ROS 参数、topic、frame、QoS 和 timeout；行为树
XML 与 CSV 仍保留为各自结构化资产，action 字段仍由 `.action` 定义。

## 数据流

```text
比赛状态 + 资源状态 + 视觉目标 + 定位/地图健康
  -> behavior blackboard
  -> 目标/模式选择与滞回
  -> SendAtsNavGoal
  -> /ats_navigate_to_pose
  -> ats_goal_manager
  -> feedback/result/cancel/preempt
  -> behavior transition
```

主树 halt、模式切换和目标替换都必须传播 cancel/preempt。仅停止 tick 而不取消
远端 action 会让旧目标继续执行，是 Nav2 移除阶段的重点回归项。

## 目录结构

```text
ats_sentry_behavior/
├── include/ats_sentry_behavior/
├── plugins/
│   ├── action/
│   └── condition/
├── src/
├── behavior_trees/
├── params/
├── launch/
└── test/
```

## 测试与边界

```bash
colcon test --base-paths src --packages-select ats_sentry_behavior
colcon test-result --test-result-base build/ats_sentry_behavior --verbose
```

- **已验证**：README 中 action 名、参数入口和插件边界已静态核对。
- **已实现未运行**：`SendAtsNavGoal` 对 ATS action 的正式接线。
- **未验证**：本轮未运行 action cancel/preempt/timeout、扩大矩形或红框闭环。
- **未完成**：从 CMake/package 清除 Nav2 对照依赖，以及所有行为树 XML 的 Nav2-free 运行验收。

工作区级说明见
[行为树决策链路](../../docs/行为树决策链路.md)。

## 参考与致谢

行为执行框架基于 BehaviorTree.CPP 与 BehaviorTree.ROS2；Nav2 action 仅作为当前
baseline 依赖保留。许可证与版权以依赖仓库和本仓声明为准。
