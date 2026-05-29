#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "ur5_motion_planner/planner_utils.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "move_group_demo_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

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

  const auto target = ur5_motion_planner::makePose(
    node->get_parameter("target_x").as_double(),
    node->get_parameter("target_y").as_double(),
    node->get_parameter("target_z").as_double(),
    node->get_parameter("target_qx").as_double(),
    node->get_parameter("target_qy").as_double(),
    node->get_parameter("target_qz").as_double(),
    node->get_parameter("target_qw").as_double());

  bool success = false;
  double planning_time_ms = 0.0;
  auto plan = ur5_motion_planner::planToPose(
    move_group, target, success, planning_time_ms);

  const auto & trajectory = plan.trajectory_.joint_trajectory;
  RCLCPP_INFO(
    node->get_logger(),
    "demo plan success=%s time=%.2f ms points=%zu joint_length=%.4f",
    success ? "true" : "false",
    planning_time_ms,
    trajectory.points.size(),
    ur5_motion_planner::jointSpaceLength(trajectory));

  ur5_motion_planner::appendPlanCsv(
    node->get_parameter("results_dir").as_string(),
    "move_group_demo.csv",
    "single_target_pose",
    success,
    planning_time_ms,
    trajectory,
    0.0,
    "extra",
    "move_group_pose_goal");

  if (success && node->get_parameter("execute").as_bool()) {
    const auto execution_result = move_group.execute(plan);
    RCLCPP_INFO(
      node->get_logger(),
      "execution result: %s",
      execution_result == moveit::core::MoveItErrorCode::SUCCESS ? "SUCCESS" : "FAILED");
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
