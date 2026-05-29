import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

sys.path.append(os.path.dirname(__file__))

from ur5_launch_common import moveit_environment_actions, planner_node


def generate_launch_description():
    node = planner_node("planning_scene_node", "planning_scene_node")
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("auto_shutdown", default_value="false"),
            DeclareLaunchArgument("fake_execution", default_value="true"),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="false"),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("planning_time", default_value="12.0"),
            DeclareLaunchArgument("planning_attempts", default_value="15"),
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
            node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=node,
                    on_exit=[Shutdown(reason="planning scene complete")],
                ),
                condition=IfCondition(LaunchConfiguration("auto_shutdown")),
            ),
        ]
    )
