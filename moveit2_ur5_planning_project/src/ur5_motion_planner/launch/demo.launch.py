import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

sys.path.append(os.path.dirname(__file__))

from ur5_launch_common import moveit_environment_actions, planner_node


def generate_launch_description():
    demo_node = planner_node(
        "move_group_demo_node",
        "move_group_demo_node",
        {
            "target_x": 0.36,
            "target_y": -0.28,
            "target_z": 0.40,
            "target_qx": 0.0,
            "target_qy": 0.7071,
            "target_qz": 0.0,
            "target_qw": 0.7071,
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("auto_shutdown", default_value="false"),
            DeclareLaunchArgument("fake_execution", default_value="true"),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="false"),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("planning_time", default_value="10.0"),
            DeclareLaunchArgument("planning_attempts", default_value="10"),
            DeclareLaunchArgument("demo_start_delay", default_value="20.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.2"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.2"),
            DeclareLaunchArgument("fake_execution_speed_scale", default_value="0.5"),
            DeclareLaunchArgument("fake_state_publish_period", default_value="0.03"),
            DeclareLaunchArgument("marker_publish_seconds", default_value="10.0"),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
        ]
        + moveit_environment_actions()
        + [
            TimerAction(
                period=LaunchConfiguration("demo_start_delay"),
                actions=[demo_node],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=demo_node,
                    on_exit=[Shutdown(reason="demo complete")],
                ),
                condition=IfCondition(LaunchConfiguration("auto_shutdown")),
            ),
        ]
    )
