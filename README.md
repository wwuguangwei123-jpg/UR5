# UR5 MoveIt 2 Motion Planning Project

This repository contains a ROS 2 Humble + MoveIt 2 UR5 motion planning project. It demonstrates target pose planning, obstacle-aware planning, multi-target planning, Cartesian path planning, RViz visualization markers, fake trajectory execution, and CSV-based trajectory analysis.

The main ROS 2 workspace is in:

```bash
moveit2_ur5_planning_project/
```

For the full Chinese user guide, build steps, launch commands, RViz workflow, and completion checklist, see:

```bash
moveit2_ur5_planning_project/README.md
```

## Highlights

- ROS 2 Humble and MoveIt 2 based UR5 planning demo.
- Four launch workflows:
  - `demo.launch.py`
  - `planning_scene.launch.py`
  - `multi_target.launch.py`
  - `cartesian_path.launch.py`
- RViz `MarkerArray` visualization for targets, paths, obstacles, and success/failure cases.
- Fake trajectory controller with adjustable execution speed.
- `trajectory_summary.csv` for comparing planning methods and results.

## Quick Start

```bash
git clone git@github.com:wwuguangwei123-jpg/UR5.git
cd UR5/moveit2_ur5_planning_project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ur5_motion_planner demo.launch.py launch_rviz:=true execute:=false
```

## Repository Notes

Generated folders such as `build/`, `install/`, `log/`, and runtime `results/` are intentionally ignored. Source code, launch files, package manifests, analysis scripts, README files, and supporting notes are tracked.

## License

Apache-2.0
