import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml


def project_root_path(*parts):
    home = os.environ.get("HOME", "/home/guangwei")
    return os.path.join(home, "moveit2_ur5_planning_project", *parts)


def ros_log_env_action():
    return SetEnvironmentVariable(
        name="ROS_LOG_DIR",
        value=project_root_path("log", "ros_logs"),
    )


def python_cache_env_action():
    return SetEnvironmentVariable(
        name="PYTHONDONTWRITEBYTECODE",
        value="1",
    )


def results_dir():
    return project_root_path("results")


def warehouse_path():
    return project_root_path("results", "warehouse_ros.sqlite")


def moveit_parameters():
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    use_sim_time = LaunchConfiguration("use_sim_time")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            'prefix:=""',
            " ",
        ]
    )

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=ur",
            " ",
            'prefix:=""',
            " ",
        ]
    )

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(
        load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    )

    return [
        {"robot_description": ParameterValue(robot_description_content, value_type=str)},
        {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)},
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]),
        {"robot_description_planning": load_yaml("ur_moveit_config", "config/joint_limits.yaml")},
        ompl_planning_pipeline_config,
        {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
    ]


def moveit_environment_actions():
    ur_moveit_launch = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    )

    include_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "warehouse_sqlite_path": warehouse_path(),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=moveit_parameters(),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_joint_state_publisher"),
                    "' == 'true' and '",
                    LaunchConfiguration("fake_execution"),
                    "' == 'false'",
                ]
            )
        ),
        parameters=moveit_parameters()
        + [
            {
                "rate": 30,
                "zeros": {
                    "shoulder_pan_joint": 0.0,
                    "shoulder_lift_joint": -1.5707,
                    "elbow_joint": 0.0,
                    "wrist_1_joint": -1.5707,
                    "wrist_2_joint": 0.0,
                    "wrist_3_joint": 0.0,
                },
            }
        ],
    )

    fake_trajectory_controller = Node(
        package="ur5_motion_planner",
        executable="fake_trajectory_controller.py",
        name="fake_trajectory_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("fake_execution")),
        parameters=[
            {
                "execution_speed_scale": ParameterValue(
                    LaunchConfiguration("fake_execution_speed_scale"), value_type=float
                ),
                "state_publish_period": ParameterValue(
                    LaunchConfiguration("fake_state_publish_period"), value_type=float
                ),
            }
        ],
    )

    return [
        ros_log_env_action(),
        python_cache_env_action(),
        include_moveit,
        robot_state_publisher,
        joint_state_publisher,
        fake_trajectory_controller,
    ]


def planner_node(executable, name, extra_parameters=None):
    parameters = moveit_parameters() + [
        {
                "results_dir": results_dir(),
                "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
                "planning_time": ParameterValue(LaunchConfiguration("planning_time"), value_type=float),
                "planning_attempts": ParameterValue(LaunchConfiguration("planning_attempts"), value_type=int),
                "velocity_scaling": ParameterValue(LaunchConfiguration("velocity_scaling"), value_type=float),
                "acceleration_scaling": ParameterValue(
                    LaunchConfiguration("acceleration_scaling"), value_type=float
                ),
                "marker_publish_seconds": ParameterValue(
                    LaunchConfiguration("marker_publish_seconds"), value_type=float
                ),
            }
    ]
    if extra_parameters:
        parameters.append(extra_parameters)

    return Node(
        package="ur5_motion_planner",
        executable=executable,
        name=name,
        output="screen",
        parameters=parameters,
    )
