# UR5 MoveIt 2 机械臂运动规划项目

本仓库是一个基于 **ROS 2 Humble + MoveIt 2 + UR5** 的机械臂运动规划仿真项目。项目已经完成从模型加载、MoveIt 规划、RViz 可视化、fake controller 执行、结果 CSV 统计到不同规划方法对比的完整闭环，适合课程设计、毕业设计、项目展示和工程能力说明。

项目主体工作空间位于：

```bash
moveit2_ur5_planning_project/
```

最详细的安装、运行、RViz 操作、四个 launch 验收流程和结果分析说明在：

```bash
moveit2_ur5_planning_project/README.md
```

## 项目能展示什么

这个项目不只是“能让机械臂动起来”，而是围绕一个完整的运动规划系统展示以下能力：

1. 能规划：通过 MoveIt 2 的 `MoveGroupInterface` 对 UR5 进行目标位姿规划。
2. 能执行：通过项目内置 fake trajectory controller 接收 FollowJointTrajectory action，并发布 `/joint_states` 驱动 RViz 中的机械臂运动。
3. 能显示：在 RViz 中显示 UR5 模型、规划轨迹、规划场景和自定义 Marker。
4. 能对比：用 `trajectory_summary.csv` 统一记录不同规划方法的结果。
5. 能解释：用目标点、障碍物、路径提示线、成功目标和失败目标解释规划结果。
6. 能分析：统计规划耗时、轨迹点数、关节空间路径长度、最大关节速度、笛卡尔路径完成比例等指标。
7. 能展示工程意义：包含避障、多目标任务、笛卡尔路径、不可达目标失败案例等工程场景。

## 仓库结构

```text
UR5/
├── README.md
├── LICENSE
├── .gitignore
├── MoveIt2_RViz_joint_states_issue_note.docx
├── make_moveit_note.py
└── moveit2_ur5_planning_project/
    ├── README.md
    └── src/
        ├── ur5_motion_planner/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/ur5_motion_planner/planner_utils.hpp
        │   ├── launch/
        │   │   ├── demo.launch.py
        │   │   ├── planning_scene.launch.py
        │   │   ├── multi_target.launch.py
        │   │   ├── cartesian_path.launch.py
        │   │   └── ur5_launch_common.py
        │   ├── scripts/fake_trajectory_controller.py
        │   └── src/
        │       ├── move_group_demo_node.cpp
        │       ├── planning_scene_node.cpp
        │       ├── multi_target_planner_node.cpp
        │       └── cartesian_path_node.cpp
        └── ur5_analysis/
            ├── package.xml
            ├── setup.py
            └── ur5_analysis/
                ├── fk_verify.py
                └── plot_trajectory.py
```

## 四个核心 launch

| Launch 文件 | 作用 | 核心看点 | 输出结果 |
| --- | --- | --- | --- |
| `demo.launch.py` | 单目标位姿规划 | 从 Home 到固定目标位姿 | `move_group_demo.csv` |
| `planning_scene.launch.py` | 障碍物避障规划 | 桌面、箱体、墙体、成功和失败案例 | `planning_scene.csv` |
| `multi_target.launch.py` | 多目标连续规划 | `target_a -> target_b -> target_c -> return_home_region` | `multi_target.csv` |
| `cartesian_path.launch.py` | 笛卡尔路径规划 | `computeCartesianPath` 末端直线路径 | `cartesian_path.csv` |

所有 launch 还会统一追加写入：

```bash
moveit2_ur5_planning_project/results/trajectory_summary.csv
```

## 主要技术点

- ROS 2 Humble package 管理和 launch 系统。
- MoveIt 2 `MoveGroupInterface` 位姿目标规划。
- MoveIt 2 Planning Scene 障碍物建模和碰撞约束。
- MoveIt 2 `computeCartesianPath` 笛卡尔路径规划。
- FollowJointTrajectory action fake controller。
- `/joint_states` 状态发布和 RViz 机械臂运动显示。
- `visualization_msgs/MarkerArray` 目标点、路径、障碍物可视化。
- CSV 数据记录和 Python 分析脚本。
- 成功案例和失败案例对比展示。

## 环境要求

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Python 3
- colcon

依赖安装：

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

## 快速开始

```bash
git clone git@github.com:wwuguangwei123-jpg/UR5.git
cd UR5/moveit2_ur5_planning_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ur5_motion_planner demo.launch.py launch_rviz:=true execute:=false
```

如果想看机械臂在 RViz 中慢速执行：

```bash
ros2 launch ur5_motion_planner demo.launch.py \
  launch_rviz:=true execute:=true fake_execution_speed_scale:=0.25
```

## 结果分析

运行任意规划 launch 后，查看结果目录：

```bash
ls results
```

打印统一统计摘要：

```bash
ros2 run ur5_analysis plot_trajectory.py --results ~/moveit2_ur5_planning_project/results
```

生成图表：

```bash
ros2 run ur5_analysis plot_trajectory.py \
  --results ~/moveit2_ur5_planning_project/results \
  --plot
```

## 推荐展示顺序

1. `demo.launch.py`：证明机械臂能完成基本目标位姿规划。
2. `multi_target.launch.py`：证明单次规划能力可以扩展成连续任务。
3. `cartesian_path.launch.py`：说明末端路径可以按笛卡尔约束生成。
4. `planning_scene.launch.py`：展示工程环境中的障碍物避障和失败案例。
5. 打开 `trajectory_summary.csv`：用数据对比不同规划方法。
6. 用 `execute:=true fake_execution_speed_scale:=0.25` 再演示一次慢速执行。

## GitHub 说明

本仓库上传的是源码、launch、README、分析脚本和必要说明文件。以下文件不会上传：

- `build/`
- `install/`
- `log/`
- `results/`
- Python 缓存
- RViz/ROS 运行中生成的临时文件

## License

Apache-2.0
