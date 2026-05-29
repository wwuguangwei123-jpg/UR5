# 基于 ROS 2 和 MoveIt 2 的 UR5 机械臂运动规划项目

这是一个独立的 ROS 2 Humble + MoveIt 2 项目，用 UR5 机械臂展示运动规划、避障规划、多目标任务、笛卡尔路径规划、可视化解释和规划结果统计分析。

推荐在 Ubuntu 22.04 / ROS 2 Humble 环境中使用，目录建议放在：

```bash
/home/guangwei/moveit2_ur5_planning_project
```

项目不要求修改已有 `~/ros2_ws`、`.bashrc` 或系统配置。构建产物和实验数据会保存在本项目下的 `build/`、`install/`、`log/`、`results/` 中。

## 本次升级内容

1. fake trajectory controller 增加执行速度参数 `fake_execution_speed_scale`，可以把 RViz 里的执行动作放慢，方便看清机械臂运动过程。
2. `multi_target`、`cartesian_path`、`planning_scene` 增加 RViz MarkerArray 可视化，能看到目标点、路径提示线、障碍物标签、成功目标和预期失败目标。
3. 所有规划节点除了写各自 CSV，还会追加写入 `results/trajectory_summary.csv`，用于横向对比不同规划方法的成功率、规划耗时、轨迹点数、关节空间路径长度、最大关节速度等指标。
4. `planning_scene` 增加一个预期失败案例 `expected_unreachable_failure`，用于演示“为什么某些目标无法规划”，但 launch 的最终退出仍以主要避障规划是否成功为准。

## 环境依赖

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

## 构建

每次修改代码后都建议重新构建：

```bash
cd ~/moveit2_ur5_planning_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果想重新做一次干净实验，可以先清空旧结果：

```bash
rm -rf results
mkdir -p results
```

## RViz 通用检查方法

启动任意 launch 后，RViz 会打开 MoveIt 的 MotionPlanning 面板。建议先按下面顺序检查界面：

1. 看左侧 `Displays` 是否有 `RobotModel`、`MotionPlanning`、`PlanningScene`。
2. 如果没有 Marker 显示，点击 `Displays` 面板左下角 `Add`。
3. 选择 `By topic`。
4. 找到 `/ur5_planning_markers`。
5. 选择 `MarkerArray` 并点击 `OK`。
6. 在 3D 视图里看彩色球、文字标签、线条或透明方块是否出现。
7. 如果没有看到 marker，重新运行对应 launch，或者把 `marker_publish_seconds:=30.0` 加到命令后面，让 marker 多发布一会儿。
8. 如果要看机械臂执行过程，把 launch 命令里的 `execute:=true` 打开，并把 `fake_execution_speed_scale` 调小，例如 `0.25`，动作会更慢、更适合展示。

`fake_execution_speed_scale` 的含义：

- `1.0`：按轨迹原始时间执行。
- `0.5`：半速执行，动作时间约变为 2 倍。
- `0.25`：四分之一速度执行，适合课堂展示。
- 不建议小于 `0.1`，否则一次执行会等很久。

## 四个 launch 功能和完整操作流程

### 1. 单目标位姿规划 demo.launch.py

功能：展示 MoveGroupInterface 如何从 Home 状态规划到一个固定末端位姿，并把结果写入 `move_group_demo.csv` 和 `trajectory_summary.csv`。

启动：

```bash
cd ~/moveit2_ur5_planning_project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_motion_planner demo.launch.py launch_rviz:=true execute:=false
```

自己操作一遍：

1. 等 RViz 打开，左侧确认 `RobotModel` 能显示 UR5。
2. 等约 20 秒，`move_group_demo_node` 会自动开始规划。
3. 看终端日志是否出现 `demo plan success=true`。
4. 看 RViz 中是否出现规划轨迹，通常是一条从当前机械臂姿态到目标位姿的轨迹预览。
5. 打开 `results/move_group_demo.csv`，确认有 `single_target_pose` 一行。
6. 打开 `results/trajectory_summary.csv`，确认有 `planner_method=move_group_pose_goal` 的记录。

如果想看机械臂慢速执行：

```bash
ros2 launch ur5_motion_planner demo.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- 终端出现 `success=true`。
- RViz 能看到机械臂规划或执行。
- `trajectory_summary.csv` 出现 `single_target_pose`。

### 2. 障碍物避障 planning_scene.launch.py

功能：向规划场景中加入桌面、偏置箱体、后墙障碍物，规划一条绕障轨迹，并额外记录一个不可达目标的预期失败案例。

启动：

```bash
ros2 launch ur5_motion_planner planning_scene.launch.py launch_rviz:=true execute:=false
```

自己操作一遍：

1. RViz 打开后，确认左侧有 `PlanningScene` 显示。
2. 如果还没有 marker，按“RViz 通用检查方法”添加 `/ur5_planning_markers` 的 `MarkerArray`。
3. 看 3D 视图里是否出现透明桌面、红色 `offset_box`、蓝色 `back_wall`。
4. 看绿色 `success_target`，这是避障规划要到达的目标。
5. 看远处红色 `expected_failure_target`，这是故意放在 UR5 工作空间外的失败目标。
6. 看终端是否出现 `added 3 collision objects`。
7. 看终端是否出现 `obstacle plan success=true`。
8. 看终端是否出现 `expected failure demo success=false`。如果这里是 `false`，说明失败案例演示成功：目标不可达，所以规划器无法给出有效轨迹。
9. 打开 `results/planning_scene.csv`，会看到 `obstacle_avoidance` 和 `expected_unreachable_failure` 两行。
10. 打开 `results/trajectory_summary.csv`，对比两行的 `success`、`trajectory_points`、`joint_space_length`。失败案例通常点数为 0 或明显不同。

如果想看避障执行：

```bash
ros2 launch ur5_motion_planner planning_scene.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- 障碍物在 RViz 中可见。
- `obstacle_avoidance` 成功。
- `expected_unreachable_failure` 记录为失败。
- 总表能同时展示成功和失败案例。

### 3. 多目标连续任务 multi_target.launch.py

功能：连续规划 `target_a -> target_b -> target_c -> return_home_region`，展示多目标任务的连续性和中间成功率。

启动：

```bash
ros2 launch ur5_motion_planner multi_target.launch.py launch_rviz:=true execute:=false
```

自己操作一遍：

1. RViz 打开后，添加 `/ur5_planning_markers` 的 `MarkerArray`。
2. 看 3D 视图中的 `target_a`、`target_b`、`target_c`、`return_home_region` 彩色球。
3. 看黄色线条，它表示多目标任务的目标访问顺序。
4. 看终端依次输出每个目标的规划结果，例如 `target_a success=true`。
5. 看最后一行 `multi-target success ratio`，理想情况是 `4/4 = 1.00`。
6. 打开 `results/multi_target.csv`，确认有四个目标的记录。
7. 打开 `results/trajectory_summary.csv`，用 `planner_method=move_group_pose_goal_sequence` 找到这些记录。

如果想看连续执行：

```bash
ros2 launch ur5_motion_planner multi_target.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- RViz 里能看到目标点和访问顺序线。
- 终端能看到每个目标的成功/失败和最终成功率。
- CSV 中每个目标都有一行，能对比每段轨迹耗时、长度和点数。

### 4. 笛卡尔路径 cartesian_path.launch.py

功能：调用 `computeCartesianPath`，从 Home 状态的末端位姿出发，沿末端直线方向生成笛卡尔路径。它和普通 MoveGroup 位姿规划不同，更适合演示末端轨迹约束。

启动：

```bash
ros2 launch ur5_motion_planner cartesian_path.launch.py launch_rviz:=true execute:=false
```

自己操作一遍：

1. RViz 打开后，添加 `/ur5_planning_markers` 的 `MarkerArray`。
2. 看蓝色 `cartesian_start` 和红色 `cartesian_end`。
3. 看黄色短线，它表示期望的末端笛卡尔路径方向。
4. 看终端里的 `cartesian path fraction=...`。
5. 如果 `fraction >= 0.85`，本项目判定为成功。
6. 打开 `results/cartesian_path.csv`，确认 `cartesian_fraction` 列。
7. 打开 `results/trajectory_summary.csv`，确认 `planner_method=cartesian_compute_path`。

如果想看执行：

```bash
ros2 launch ur5_motion_planner cartesian_path.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

完成标准：

- `fraction` 达到或超过 `0.85`。
- RViz 中能看到起点、终点和路径提示线。
- CSV 中有 `cartesian_rectangle_segment` 的记录。

## 在 RViz 里手动拖动目标并执行

这个步骤适合展示“MoveIt 本身可以交互式规划”，和代码节点的自动规划形成对比。

1. 启动任意带 RViz 的 launch，例如：

   ```bash
   ros2 launch ur5_motion_planner demo.launch.py launch_rviz:=true execute:=false
   ```

2. 在 RViz 左侧 `MotionPlanning` 面板中找到 `Planning` 标签页。
3. 选择 planning group，一般应为 `ur_manipulator`。
4. 在 3D 视图里找到末端执行器附近的交互式标记。
5. 用鼠标拖动彩色箭头改变末端位置。
6. 用鼠标拖动彩色圆环改变末端姿态。
7. 点击 MotionPlanning 面板中的 `Plan`，观察是否出现轨迹。
8. 如果启动时用了 `execute:=true` 或 fake controller 正常运行，可以点击 `Plan & Execute`。
9. 若机械臂运动太快，重新启动 launch 并加入：

   ```bash
   fake_execution_speed_scale:=0.25
   ```

注意：手动拖动产生的交互式规划主要用于 RViz 展示，不一定会写入本项目 CSV。CSV 统计主要来自四个 C++ 规划节点。

## 规划结果汇总表

每次运行规划节点后，结果会写入：

```bash
~/moveit2_ur5_planning_project/results/trajectory_summary.csv
```

重要字段含义：

- `source_file`：来自哪个单项 CSV。
- `planner_method`：规划方法，例如普通位姿规划、多目标序列、带障碍物位姿规划、笛卡尔路径。
- `scenario`：具体案例名。
- `success`：`1` 表示成功，`0` 表示失败。
- `planning_time_ms`：规划耗时，单位毫秒。
- `trajectory_duration_sec`：轨迹执行时间，来自轨迹时间戳。
- `trajectory_points`：轨迹点数量。
- `joint_space_length`：关节空间路径长度，越大代表关节总运动量越大。
- `max_joint_speed`：轨迹点中记录到的最大关节速度。
- `metric_name` / `metric_value`：额外指标，例如笛卡尔完成比例、碰撞物数量、运行中成功率。

用脚本打印汇总：

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

图表会保存为：

```bash
~/moveit2_ur5_planning_project/results/planning_metrics.png
```

## 建议的完整演示顺序

建议按下面顺序完整跑一遍，这样观众能逐步看出工程意义：

1. 跑 `demo.launch.py`，说明“机械臂能从起点规划到目标位姿”。
2. 跑 `multi_target.launch.py`，说明“单次规划可以扩展为连续任务”。
3. 跑 `cartesian_path.launch.py`，说明“不是所有规划都只是找一个终点，也可以约束末端沿直线路径运动”。
4. 跑 `planning_scene.launch.py`，说明“真实工程环境有桌面、箱体、墙等障碍物，规划必须考虑碰撞”。
5. 打开 `trajectory_summary.csv`，比较不同方法的成功率、耗时、轨迹点数和路径长度。
6. 指出 `expected_unreachable_failure`，说明失败不是 bug，而是工作空间、碰撞约束、目标姿态共同作用下的合理结果。
7. 用 `execute:=true fake_execution_speed_scale:=0.25` 再跑一次最直观的案例，让观众看到机械臂真的按轨迹运动。

## 常见问题

如果 RViz 里没有机械臂：

- 确认 `source /opt/ros/humble/setup.bash` 和 `source install/setup.bash` 都执行过。
- 确认 `ur_description` 和 `ur_moveit_config` 已安装。
- 看终端是否有 xacro 或 robot_description 报错。

如果 Marker 看不到：

- 确认添加的是 `/ur5_planning_markers` 下的 `MarkerArray`。
- 重新运行 launch，并加入 `marker_publish_seconds:=30.0`。
- 确认 RViz 的 Fixed Frame 和 MoveIt planning frame 一致，通常是 `base_link` 或 MoveIt 自动配置的基坐标系。

如果执行时机械臂不动：

- 确认命令里有 `execute:=true`。
- 确认 `fake_execution:=true`。
- 看终端是否显示 fake controller 已启动。
- 把 `fake_execution_speed_scale` 设为 `0.25`，运动会更慢但更容易观察。

如果 CSV 没有生成：

- 确认对应规划节点已经真正运行。
- 看终端是否有 `success=`、`points=`、`joint_length=` 日志。
- 确认当前用户对项目目录有写权限。

## 简历描述

基于 ROS 2 Humble 和 MoveIt 2 搭建 UR5 六自由度机械臂运动规划仿真系统，完成 UR5 模型加载、MoveIt 2 规划环境启动、C++ MoveGroupInterface 控制节点开发，实现目标位姿规划、障碍物避障、多目标连续任务和笛卡尔路径规划；进一步加入 RViz MarkerArray 可视化、fake controller 慢速执行参数、成功/失败案例展示以及 `trajectory_summary.csv` 统一指标汇总，用于对比不同规划方法的规划耗时、成功率、轨迹点数、关节空间路径长度和最大关节速度。
