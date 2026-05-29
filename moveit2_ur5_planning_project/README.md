# 基于 ROS 2 和 MoveIt 2 的 UR5 机械臂运动规划项目

本项目是一个独立的 **ROS 2 Humble + MoveIt 2 + UR5** 机械臂运动规划仿真工作空间。它的目标不是只做一个最小 demo，而是做成一个可以展示、可以解释、可以对比、可以分析的完整机械臂规划项目。

项目已经覆盖：

- UR5 模型加载。
- MoveIt 2 规划环境启动。
- RViz 规划和执行显示。
- C++ MoveGroupInterface 自动规划节点。
- fake trajectory controller 虚拟执行。
- 目标点、路径、障碍物 Marker 可视化。
- 单目标、多目标、避障、笛卡尔路径四类规划。
- 成功案例和失败案例展示。
- CSV 结果记录和统一汇总分析。

## 1. 项目定位

这个项目可以用一句话概括：

> 基于 ROS 2 Humble 和 MoveIt 2 搭建 UR5 六自由度机械臂运动规划仿真系统，并用 RViz 与 CSV 指标展示不同规划方法的工程差异。

它适合展示以下能力：

1. 能规划：调用 MoveIt 2 生成机械臂轨迹。
2. 能执行：fake controller 接收轨迹并发布关节状态。
3. 能显示：RViz 展示模型、轨迹、场景和 marker。
4. 能对比：用统一 CSV 比较不同规划方法。
5. 能解释：通过障碍物、目标点、路径提示线和失败案例说明规划原因。
6. 能分析：用轨迹点数、规划时间、关节路径长度等指标分析结果。

## 2. 项目目录

```text
moveit2_ur5_planning_project/
├── README.md
├── src/
│   ├── ur5_motion_planner/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/
│   │   │   └── ur5_motion_planner/
│   │   │       └── planner_utils.hpp
│   │   ├── launch/
│   │   │   ├── demo.launch.py
│   │   │   ├── planning_scene.launch.py
│   │   │   ├── multi_target.launch.py
│   │   │   ├── cartesian_path.launch.py
│   │   │   └── ur5_launch_common.py
│   │   ├── scripts/
│   │   │   └── fake_trajectory_controller.py
│   │   └── src/
│   │       ├── move_group_demo_node.cpp
│   │       ├── planning_scene_node.cpp
│   │       ├── multi_target_planner_node.cpp
│   │       └── cartesian_path_node.cpp
│   └── ur5_analysis/
│       ├── package.xml
│       ├── setup.py
│       └── ur5_analysis/
│           ├── fk_verify.py
│           └── plot_trajectory.py
├── build/      # 构建后生成，Git 不上传
├── install/    # 构建后生成，Git 不上传
├── log/        # 构建和运行日志，Git 不上传
└── results/    # 运行结果 CSV 和图表，Git 不上传
```

## 3. 两个 ROS 包说明

### 3.1 ur5_motion_planner

这是核心 C++ 规划包，负责启动和执行四类规划任务。

主要文件：

- `move_group_demo_node.cpp`：单目标位姿规划。
- `planning_scene_node.cpp`：障碍物场景、避障规划、预期失败案例。
- `multi_target_planner_node.cpp`：多目标连续规划。
- `cartesian_path_node.cpp`：笛卡尔路径规划。
- `planner_utils.hpp`：共用工具，包括目标位姿、轨迹指标、CSV 写入、Marker 生成。
- `fake_trajectory_controller.py`：虚拟 FollowJointTrajectory controller。
- `launch/*.launch.py`：四个演示入口。

### 3.2 ur5_analysis

这是 Python 分析包，负责读取 CSV 并输出统计摘要或图表。

主要文件：

- `plot_trajectory.py`：读取 `trajectory_summary.csv` 或单项 CSV，打印统计并可生成 `planning_metrics.png`。
- `fk_verify.py`：手写 UR5 正运动学验证脚本。

## 4. 环境要求

推荐系统：

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Python 3
- colcon

不建议直接在 Windows PowerShell 里运行 ROS 2 launch。Windows 可以用来查看和编辑代码，但真实构建、运行和 RViz 展示建议放在 Ubuntu 22.04 或 WSLg 图形环境中。

## 5. 安装依赖

先安装 ROS 2 Humble 后，再安装本项目需要的依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  ros-humble-ur-description \
  ros-humble-ur-moveit-config \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher
```

可选安装绘图依赖：

```bash
sudo apt install -y python3-matplotlib
```

检查 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
ros2 pkg list | grep moveit
ros2 pkg list | grep ur_moveit_config
```

如果 `ur_moveit_config` 或 `ur_description` 查不到，说明依赖还没有安装完整。

## 6. 克隆和构建

从 GitHub 克隆：

```bash
git clone git@github.com:wwuguangwei123-jpg/UR5.git
cd UR5/moveit2_ur5_planning_project
```

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

构建成功后检查两个包是否可见：

```bash
ros2 pkg list | grep ur5_motion_planner
ros2 pkg list | grep ur5_analysis
```

如果查不到，通常是忘了执行：

```bash
source install/setup.bash
```

## 7. 运行前建议清理结果

如果你想做一次干净展示，可以先清空旧结果：

```bash
rm -rf results
mkdir -p results
```

运行后结果会自动写到：

```bash
~/moveit2_ur5_planning_project/results
```

如果你是从 GitHub 克隆到 `~/UR5/moveit2_ur5_planning_project`，代码里的默认结果路径仍使用 `~/moveit2_ur5_planning_project/results`。为了路径完全一致，建议把项目放在：

```bash
~/moveit2_ur5_planning_project
```

或者把克隆后的项目移动到该路径：

```bash
mv ~/UR5/moveit2_ur5_planning_project ~/moveit2_ur5_planning_project
cd ~/moveit2_ur5_planning_project
```

## 8. 常用 launch 参数

四个 launch 基本都支持下面这些参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `launch_rviz` | `true` | 是否启动 RViz |
| `execute` | `false` | 是否执行规划轨迹 |
| `fake_execution` | `true` | 是否启动项目内置 fake controller |
| `use_joint_state_publisher` | `false` | 不使用 fake controller 时可打开 |
| `planning_time` | `10.0` 或 `12.0` | MoveIt 单次规划允许时间 |
| `planning_attempts` | `10` 或 `15` | 规划尝试次数 |
| `velocity_scaling` | `0.2` | MoveIt 轨迹速度缩放 |
| `acceleration_scaling` | `0.2` | MoveIt 加速度缩放 |
| `fake_execution_speed_scale` | `0.5` | fake controller 执行速度缩放 |
| `marker_publish_seconds` | `10.0` | marker 保持重复发布的时间 |
| `auto_shutdown` | `false` | 节点结束后是否自动关闭 launch |

`fake_execution_speed_scale` 的理解：

- `1.0`：按轨迹原始时间执行。
- `0.5`：半速执行，动作时间约变成 2 倍。
- `0.25`：四分之一速度，适合展示。
- `0.1`：非常慢，适合逐帧观察，但要等较久。

## 9. RViz 通用操作

启动任意 launch 后，RViz 会打开。建议每次都按下面步骤检查：

1. 看左侧 `Displays` 面板。
2. 确认有 `RobotModel`。
3. 确认有 `MotionPlanning`。
4. 确认有 `PlanningScene`。
5. 如果没有 marker，点击左下角 `Add`。
6. 在弹窗里选择 `By topic`。
7. 找到 `/ur5_planning_markers`。
8. 选择 `MarkerArray`。
9. 点击 `OK`。
10. 看 3D 视图里是否出现彩色球、文字、线条或透明方块。

如果 marker 没看到：

```bash
ros2 launch ur5_motion_planner planning_scene.launch.py \
  launch_rviz:=true marker_publish_seconds:=30.0
```

如果 RViz Fixed Frame 报错：

1. 在左侧 `Global Options` 找到 `Fixed Frame`。
2. 尝试选择 `base_link`。
3. 如果没有 `base_link`，看终端 TF 或 robot_description 是否报错。

## 10. 四个 launch 详细流程

### 10.1 单目标位姿规划 demo.launch.py

功能：

从 Home 状态出发，规划到一个固定末端目标位姿，展示最基础的 MoveIt 目标位姿规划。

启动：

```bash
cd ~/moveit2_ur5_planning_project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_motion_planner demo.launch.py launch_rviz:=true execute:=false
```

你应该做什么：

1. 等 RViz 打开。
2. 在 RViz 左侧确认 UR5 模型显示正常。
3. 等约 20 秒，`move_group_demo_node` 会自动开始规划。
4. 看终端输出是否有：

   ```text
   demo plan success=true
   ```

5. 看 RViz 里是否出现规划轨迹。
6. 打开结果文件：

   ```bash
   cat results/move_group_demo.csv
   cat results/trajectory_summary.csv
   ```

7. 确认 `trajectory_summary.csv` 中有：

   ```text
   single_target_pose
   move_group_pose_goal
   ```

想看执行过程：

```bash
ros2 launch ur5_motion_planner demo.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- 终端出现 `success=true`。
- RViz 能看到轨迹或机械臂执行。
- `results/move_group_demo.csv` 有记录。
- `results/trajectory_summary.csv` 有 `single_target_pose`。

工程解释：

这个 launch 说明项目已经打通“目标位姿输入 -> MoveIt 规划 -> 轨迹输出 -> 可选执行”的基础链路。

### 10.2 障碍物避障 planning_scene.launch.py

功能：

在规划场景中加入桌面、偏置箱体和后墙，规划一条绕障路径，并额外加入一个故意不可达的目标，形成成功和失败案例对比。

启动：

```bash
ros2 launch ur5_motion_planner planning_scene.launch.py launch_rviz:=true execute:=false
```

你应该做什么：

1. RViz 打开后，添加 `/ur5_planning_markers` 的 `MarkerArray`。
2. 在 3D 视图中寻找透明灰色桌面。
3. 寻找红色 `offset_box`。
4. 寻找蓝色 `back_wall`。
5. 寻找绿色 `success_target`。
6. 寻找远处红色 `expected_failure_target`。
7. 看终端是否有：

   ```text
   added 3 collision objects
   obstacle plan success=true
   expected failure demo success=false
   ```

8. 打开结果：

   ```bash
   cat results/planning_scene.csv
   cat results/trajectory_summary.csv
   ```

9. 确认有两类场景：

   ```text
   obstacle_avoidance
   expected_unreachable_failure
   ```

想看执行：

```bash
ros2 launch ur5_motion_planner planning_scene.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- RViz 中能看到障碍物。
- `obstacle_avoidance` 成功。
- `expected_unreachable_failure` 失败。
- CSV 同时记录成功和失败。

工程解释：

真实机械臂不是在空中随便动，而是在有桌面、物体、墙面、夹具等约束的环境中运动。这个 launch 说明规划器需要避开障碍物；如果目标超出工作空间，失败是合理结果，不是程序崩溃。

### 10.3 多目标连续规划 multi_target.launch.py

功能：

连续规划多个目标点，展示机械臂完成一组任务点的能力。

目标顺序：

```text
target_a -> target_b -> target_c -> return_home_region
```

启动：

```bash
ros2 launch ur5_motion_planner multi_target.launch.py launch_rviz:=true execute:=false
```

你应该做什么：

1. RViz 打开后添加 `/ur5_planning_markers` 的 `MarkerArray`。
2. 看 3D 视图中的 `target_a`、`target_b`、`target_c`、`return_home_region`。
3. 看黄色线条，它表示目标访问顺序。
4. 看终端逐个输出：

   ```text
   target_a success=...
   target_b success=...
   target_c success=...
   return_home_region success=...
   multi-target success ratio
   ```

5. 打开结果：

   ```bash
   cat results/multi_target.csv
   cat results/trajectory_summary.csv
   ```

6. 确认有四条目标记录。

想看连续执行：

```bash
ros2 launch ur5_motion_planner multi_target.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- RViz 能看到目标点和访问顺序线。
- 终端能看到每个目标的成功/失败。
- 最终成功率理想情况为 `4/4 = 1.00`。
- CSV 能对比每段轨迹的耗时、点数和路径长度。

工程解释：

工业场景里机械臂通常不是只到一个点，而是要连续完成取放、检测、避让、回位等任务。这个 launch 把单目标规划扩展成任务序列。

### 10.4 笛卡尔路径 cartesian_path.launch.py

功能：

使用 `computeCartesianPath` 生成末端沿直线方向移动的笛卡尔路径。

启动：

```bash
ros2 launch ur5_motion_planner cartesian_path.launch.py launch_rviz:=true execute:=false
```

你应该做什么：

1. RViz 打开后添加 `/ur5_planning_markers` 的 `MarkerArray`。
2. 看蓝色 `cartesian_start`。
3. 看红色 `cartesian_end`。
4. 看黄色短线，它表示末端期望移动方向。
5. 看终端输出：

   ```text
   cartesian path fraction=...
   ```

6. 如果 `fraction >= 0.85`，项目认为这次笛卡尔路径规划成功。
7. 打开结果：

   ```bash
   cat results/cartesian_path.csv
   cat results/trajectory_summary.csv
   ```

8. 确认有：

   ```text
   cartesian_rectangle_segment
   cartesian_compute_path
   cartesian_fraction
   ```

想看执行：

```bash
ros2 launch ur5_motion_planner cartesian_path.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- RViz 能看到起点、终点和路径提示线。
- `fraction` 达到或超过 `0.85`。
- CSV 中有笛卡尔路径记录。

工程解释：

普通 MoveGroup 位姿规划更关注“能到终点”，笛卡尔路径更关注“末端按指定空间路径走”。这对焊接、喷涂、擦拭、打磨、直线插入等任务更有工程意义。

## 11. RViz 手动拖动目标

这个步骤用于展示 MoveIt 的交互式规划能力。

启动：

```bash
ros2 launch ur5_motion_planner demo.launch.py launch_rviz:=true execute:=false
```

操作：

1. 在 RViz 左侧找到 `MotionPlanning` 面板。
2. 找到 `Planning` 标签页。
3. 确认 Planning Group 是 `ur_manipulator`。
4. 在 3D 视图中找到末端执行器附近的交互式标记。
5. 拖动彩色箭头改变末端位置。
6. 拖动彩色圆环改变末端姿态。
7. 点击 `Plan`。
8. 如果看到轨迹，说明交互式规划成功。
9. 如果启动了 fake controller，可以点击 `Plan & Execute`。

注意：

手动拖动产生的规划是 RViz MotionPlanning 插件里的交互式规划，不一定写入本项目 CSV。本项目 CSV 主要记录四个 C++ 自动规划节点的结果。

## 12. 结果文件说明

运行后 `results/` 中可能出现：

```text
move_group_demo.csv
planning_scene.csv
multi_target.csv
cartesian_path.csv
trajectory_summary.csv
planning_metrics.png
warehouse_ros.sqlite
```

各文件含义：

| 文件 | 含义 |
| --- | --- |
| `move_group_demo.csv` | 单目标位姿规划结果 |
| `planning_scene.csv` | 避障规划和失败案例结果 |
| `multi_target.csv` | 多目标连续规划结果 |
| `cartesian_path.csv` | 笛卡尔路径规划结果 |
| `trajectory_summary.csv` | 所有规划方法统一汇总表 |
| `planning_metrics.png` | 分析脚本生成的对比图 |
| `warehouse_ros.sqlite` | MoveIt warehouse 数据库 |

## 13. trajectory_summary.csv 字段解释

统一汇总表字段：

| 字段 | 含义 |
| --- | --- |
| `timestamp` | 记录时间 |
| `source_file` | 来源 CSV |
| `planner_method` | 规划方法 |
| `scenario` | 具体案例 |
| `success` | `1` 成功，`0` 失败 |
| `planning_time_ms` | 规划耗时，单位毫秒 |
| `trajectory_duration_sec` | 轨迹执行时间 |
| `trajectory_points` | 轨迹点数量 |
| `joint_space_length` | 关节空间路径长度 |
| `max_joint_speed` | 最大关节速度 |
| `metric_name` | 附加指标名称 |
| `metric_value` | 附加指标数值 |

常见 `planner_method`：

| 方法 | 含义 |
| --- | --- |
| `move_group_pose_goal` | 普通目标位姿规划 |
| `move_group_pose_goal_sequence` | 多目标连续位姿规划 |
| `move_group_pose_goal_with_collision_scene` | 带障碍物的位姿规划 |
| `move_group_pose_goal_unreachable_demo` | 不可达目标失败演示 |
| `cartesian_compute_path` | 笛卡尔路径规划 |

## 14. 如何分析结果

打印摘要：

```bash
source ~/moveit2_ur5_planning_project/install/setup.bash
ros2 run ur5_analysis plot_trajectory.py --results ~/moveit2_ur5_planning_project/results
```

生成图表：

```bash
ros2 run ur5_analysis plot_trajectory.py \
  --results ~/moveit2_ur5_planning_project/results \
  --plot
```

看表时可以这样解释：

- `success=1`：规划器找到了满足约束的轨迹。
- `success=0`：目标不可达、碰撞约束无法满足、姿态不合适或规划时间内未找到解。
- `planning_time_ms` 越小：规划越快，但不一定轨迹越好。
- `trajectory_points` 越多：轨迹离散点更多，执行更细，但不一定更优。
- `joint_space_length` 越大：关节总运动量越大，可能意味着绕路或目标距离更远。
- `cartesian_fraction` 越接近 `1.0`：笛卡尔路径完成比例越高。

## 15. 推荐完整展示脚本

如果你要给老师、同学或项目评审展示，建议按这个顺序：

1. 先打开项目结构，说明这是 ROS 2 工作空间。
2. 展示 `src/ur5_motion_planner` 和 `src/ur5_analysis` 两个包。
3. 运行 `demo.launch.py`，说明基础规划链路。
4. 打开 RViz，看 UR5 模型和规划轨迹。
5. 用 `execute:=true fake_execution_speed_scale:=0.25` 再跑一次，说明 fake controller 执行。
6. 运行 `multi_target.launch.py`，看多个目标点和顺序线。
7. 运行 `cartesian_path.launch.py`，解释笛卡尔路径和普通位姿规划的区别。
8. 运行 `planning_scene.launch.py`，看障碍物、成功目标、失败目标。
9. 打开 `trajectory_summary.csv`，用数据横向比较。
10. 运行 `plot_trajectory.py --plot`，展示图表。

## 16. 一键跑四个 launch 的建议

如果只想生成 CSV，可以不打开 RViz，并自动结束：

```bash
ros2 launch ur5_motion_planner demo.launch.py \
  launch_rviz:=false auto_shutdown:=true execute:=false

ros2 launch ur5_motion_planner multi_target.launch.py \
  launch_rviz:=false auto_shutdown:=true execute:=false

ros2 launch ur5_motion_planner cartesian_path.launch.py \
  launch_rviz:=false auto_shutdown:=true execute:=false

ros2 launch ur5_motion_planner planning_scene.launch.py \
  launch_rviz:=false auto_shutdown:=true execute:=false
```

然后查看汇总：

```bash
ros2 run ur5_analysis plot_trajectory.py --results ~/moveit2_ur5_planning_project/results
```

## 17. 常见问题排查

### 17.1 colcon 找不到

现象：

```text
colcon: command not found
```

处理：

```bash
sudo apt install -y python3-colcon-common-extensions
```

### 17.2 ros2 找不到包

现象：

```text
Package 'ur5_motion_planner' not found
```

处理：

```bash
cd ~/moveit2_ur5_planning_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 17.3 RViz 没有机器人

检查：

1. `ur_description` 是否安装。
2. `ur_moveit_config` 是否安装。
3. 终端是否有 xacro 报错。
4. RViz Fixed Frame 是否正确。

### 17.4 Marker 看不到

处理：

1. RViz 左侧点击 `Add`。
2. 选择 `By topic`。
3. 添加 `/ur5_planning_markers` 的 `MarkerArray`。
4. 重新运行 launch，加长发布时间：

   ```bash
   marker_publish_seconds:=30.0
   ```

### 17.5 机械臂执行太快

处理：

```bash
fake_execution_speed_scale:=0.25
```

### 17.6 机械臂不执行

检查：

1. launch 命令是否有 `execute:=true`。
2. 是否启动了 `fake_execution:=true`。
3. 终端是否出现 fake controller ready 日志。
4. RViz 是否接收到 `/joint_states`。

### 17.7 CSV 没有生成

检查：

1. 规划节点是否真的启动。
2. 终端是否有 `success=`、`points=`、`joint_length=`。
3. 当前用户是否有项目目录写权限。
4. `results/` 是否被误删后没有重新创建。

## 18. 可以如何继续扩展

后续可以继续升级：

- 增加更多规划器对比，例如 RRTConnect、PRM、EST。
- 增加路径平滑前后对比。
- 增加碰撞距离或最小安全距离统计。
- 增加真实 UR ros2_control 控制器接入。
- 增加 MoveIt Servo 实时控制。
- 增加更多典型工业场景，例如抓取、放置、绕障搬运。
- 把 CSV 自动生成 Markdown 或 HTML 报告。

## 19. 简历描述

基于 ROS 2 Humble 和 MoveIt 2 搭建 UR5 六自由度机械臂运动规划仿真系统，完成 UR5 模型加载、MoveIt 2 规划环境启动、C++ MoveGroupInterface 控制节点开发，实现目标位姿规划、障碍物避障、多目标连续任务和笛卡尔路径规划；进一步加入 RViz MarkerArray 可视化、fake controller 慢速执行参数、成功/失败案例展示以及 `trajectory_summary.csv` 统一指标汇总，用于对比不同规划方法的规划耗时、成功率、轨迹点数、关节空间路径长度和最大关节速度。
