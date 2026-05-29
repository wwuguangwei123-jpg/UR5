#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "ur5_motion_planner/planner_utils.hpp"

struct Target
{
  std::string name;
  geometry_msgs::msg::Pose pose;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "multi_target_planner_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto marker_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ur5_planning_markers", rclcpp::QoS(1).transient_local().reliable());

  moveit::planning_interface::MoveGroupInterface move_group(
    node, ur5_motion_planner::kPlanningGroup);
  move_group.setEndEffectorLink(ur5_motion_planner::kEndEffectorLink);
  move_group.setPlanningTime(node->get_parameter("planning_time").as_double());
  move_group.setNumPlanningAttempts(
    static_cast<unsigned int>(node->get_parameter("planning_attempts").as_int()));
  move_group.setMaxVelocityScalingFactor(
    node->get_parameter("velocity_scaling").as_double());
  move_group.setMaxAccelerationScalingFactor(
    node->get_parameter("acceleration_scaling").as_double());

  const std::vector<Target> targets = {
    {"target_a", ur5_motion_planner::makePose(0.36, -0.28, 0.40, 0.0, 0.7071, 0.0, 0.7071)},
    {"target_b", ur5_motion_planner::makePose(0.44, 0.02, 0.48, 0.0, 0.7071, 0.0, 0.7071)},
    {"target_c", ur5_motion_planner::makePose(0.30, 0.30, 0.36, 0.0, 0.7071, 0.0, 0.7071)},
    {"return_home_region", ur5_motion_planner::makePose(0.32, 0.0, 0.46, 0.0, 0.7071, 0.0, 0.7071)},
  };

  const std::string frame_id = move_group.getPlanningFrame();
  visualization_msgs::msg::MarkerArray markers;
  std::vector<geometry_msgs::msg::Pose> target_poses;
  int marker_id = 0;
  const std::vector<std_msgs::msg::ColorRGBA> colors = {
    ur5_motion_planner::makeColor(0.1F, 0.7F, 0.2F, 0.9F),
    ur5_motion_planner::makeColor(0.1F, 0.35F, 0.9F, 0.9F),
    ur5_motion_planner::makeColor(0.95F, 0.55F, 0.1F, 0.9F),
    ur5_motion_planner::makeColor(0.5F, 0.5F, 0.5F, 0.9F),
  };
  for (std::size_t i = 0; i < targets.size(); ++i) {
    target_poses.push_back(targets[i].pose);
    markers.markers.push_back(ur5_motion_planner::makeSphereMarker(
      frame_id, "multi_target_points", marker_id++, targets[i].pose, 0.055, colors[i]));
    markers.markers.push_back(ur5_motion_planner::makeTextMarker(
      frame_id, "multi_target_labels", marker_id++, targets[i].pose, targets[i].name, 0.04, colors[i]));
  }
  markers.markers.push_back(ur5_motion_planner::makeLineStripMarker(
    frame_id,
    "multi_target_sequence",
    marker_id++,
    target_poses,
    0.012,
    ur5_motion_planner::makeColor(0.95F, 0.95F, 0.05F, 0.85F)));
  marker_publisher->publish(markers);

  int success_count = 0;
  for (const auto & target : targets) {
    bool success = false;
    double planning_time_ms = 0.0;
    auto plan = ur5_motion_planner::planToPose(move_group, target.pose, success, planning_time_ms);
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    success_count += success ? 1 : 0;

    RCLCPP_INFO(
      node->get_logger(),
      "%s success=%s time=%.2f ms points=%zu joint_length=%.4f",
      target.name.c_str(),
      success ? "true" : "false",
      planning_time_ms,
      trajectory.points.size(),
      ur5_motion_planner::jointSpaceLength(trajectory));

    ur5_motion_planner::appendPlanCsv(
      node->get_parameter("results_dir").as_string(),
      "multi_target.csv",
      target.name,
      success,
      planning_time_ms,
      trajectory,
      static_cast<double>(success_count) / static_cast<double>(targets.size()),
      "running_success_ratio",
      "move_group_pose_goal_sequence");

    if (success && node->get_parameter("execute").as_bool()) {
      move_group.execute(plan);
    }
  }

  RCLCPP_INFO(
    node->get_logger(),
    "multi-target success ratio: %d/%zu = %.2f",
    success_count,
    targets.size(),
      static_cast<double>(success_count) / static_cast<double>(targets.size()));

  ur5_motion_planner::publishMarkerArrayForDuration(
    node, marker_publisher, markers, node->get_parameter("marker_publish_seconds").as_double());

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return success_count == static_cast<int>(targets.size()) ? 0 : 2;
}
