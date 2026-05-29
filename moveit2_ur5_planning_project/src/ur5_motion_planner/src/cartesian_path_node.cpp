#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ur5_motion_planner/planner_utils.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "cartesian_path_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto display_publisher =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
  auto marker_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ur5_planning_markers", rclcpp::QoS(1).transient_local().reliable());

  moveit::planning_interface::MoveGroupInterface move_group(
    node, ur5_motion_planner::kPlanningGroup);
  move_group.setEndEffectorLink(ur5_motion_planner::kEndEffectorLink);
  move_group.setMaxVelocityScalingFactor(
    node->get_parameter("velocity_scaling").as_double());
  move_group.setMaxAccelerationScalingFactor(
    node->get_parameter("acceleration_scaling").as_double());

  std::vector<geometry_msgs::msg::Pose> waypoints;
  const auto start_state = ur5_motion_planner::makeNamedRobotState(move_group, "home");
  move_group.setStartState(start_state);
  auto start_pose = ur5_motion_planner::poseFromState(
    start_state, ur5_motion_planner::kEndEffectorLink);
  waypoints.push_back(start_pose);

  auto p1 = start_pose;
  p1.position.z += 0.03;
  waypoints.push_back(p1);

  const std::string frame_id = move_group.getPlanningFrame();
  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;
  markers.markers.push_back(ur5_motion_planner::makeSphereMarker(
    frame_id,
    "cartesian_waypoints",
    marker_id++,
    waypoints.front(),
    0.045,
    ur5_motion_planner::makeColor(0.1F, 0.55F, 0.95F, 0.9F)));
  markers.markers.push_back(ur5_motion_planner::makeTextMarker(
    frame_id,
    "cartesian_labels",
    marker_id++,
    waypoints.front(),
    "cartesian_start",
    0.035,
    ur5_motion_planner::makeColor(0.1F, 0.55F, 0.95F, 0.9F)));
  markers.markers.push_back(ur5_motion_planner::makeSphereMarker(
    frame_id,
    "cartesian_waypoints",
    marker_id++,
    waypoints.back(),
    0.045,
    ur5_motion_planner::makeColor(0.95F, 0.25F, 0.15F, 0.9F)));
  markers.markers.push_back(ur5_motion_planner::makeTextMarker(
    frame_id,
    "cartesian_labels",
    marker_id++,
    waypoints.back(),
    "cartesian_end",
    0.035,
    ur5_motion_planner::makeColor(0.95F, 0.25F, 0.15F, 0.9F)));
  markers.markers.push_back(ur5_motion_planner::makeLineStripMarker(
    frame_id,
    "cartesian_path_hint",
    marker_id++,
    waypoints,
    0.012,
    ur5_motion_planner::makeColor(0.95F, 0.95F, 0.05F, 0.9F)));
  marker_publisher->publish(markers);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = node->get_parameter("eef_step").as_double();
  const double jump_threshold = node->get_parameter("jump_threshold").as_double();

  const auto started = std::chrono::steady_clock::now();
  const double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);
  const auto finished = std::chrono::steady_clock::now();
  const double planning_time_ms =
    std::chrono::duration<double, std::milli>(finished - started).count();

  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  moveit::core::robotStateToRobotStateMsg(start_state, display_trajectory.trajectory_start);
  display_trajectory.trajectory.push_back(trajectory);
  display_publisher->publish(display_trajectory);

  const bool success = fraction >= node->get_parameter("success_fraction").as_double();
  RCLCPP_INFO(
    node->get_logger(),
    "cartesian path fraction=%.3f success=%s time=%.2f ms points=%zu joint_length=%.4f",
    fraction,
    success ? "true" : "false",
    planning_time_ms,
    trajectory.joint_trajectory.points.size(),
    ur5_motion_planner::jointSpaceLength(trajectory.joint_trajectory));

  ur5_motion_planner::appendPlanCsv(
    node->get_parameter("results_dir").as_string(),
    "cartesian_path.csv",
    "cartesian_rectangle_segment",
    success,
    planning_time_ms,
    trajectory.joint_trajectory,
    fraction,
    "cartesian_fraction",
    "cartesian_compute_path");

  if (success && node->get_parameter("execute").as_bool()) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group.execute(plan);
  }

  ur5_motion_planner::publishMarkerArrayForDuration(
    node, marker_publisher, markers, node->get_parameter("marker_publish_seconds").as_double());

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 2;
}
