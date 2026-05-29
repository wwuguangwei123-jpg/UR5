#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "ur5_motion_planner/planner_utils.hpp"

namespace
{

moveit_msgs::msg::CollisionObject makeBox(
  const std::string & id,
  const std::string & frame_id,
  double x,
  double y,
  double z,
  double size_x,
  double size_y,
  double size_z)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "planning_scene_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto marker_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ur5_planning_markers", rclcpp::QoS(1).transient_local().reliable());

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
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

  const std::string frame_id = move_group.getPlanningFrame();
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(makeBox("table", frame_id, 0.55, 0.0, -0.25, 1.1, 0.9, 0.08));
  objects.push_back(makeBox("offset_box", frame_id, 0.52, 0.18, 0.18, 0.16, 0.22, 0.28));
  objects.push_back(makeBox("back_wall", frame_id, 0.86, 0.0, 0.36, 0.05, 1.2, 0.72));

  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;
  markers.markers.push_back(ur5_motion_planner::makeBoxMarker(
    frame_id,
    "planning_scene_objects",
    marker_id++,
    ur5_motion_planner::makePose(0.55, 0.0, -0.25, 0.0, 0.0, 0.0, 1.0),
    1.1,
    0.9,
    0.08,
    ur5_motion_planner::makeColor(0.45F, 0.45F, 0.45F, 0.35F)));
  markers.markers.push_back(ur5_motion_planner::makeTextMarker(
    frame_id,
    "planning_scene_labels",
    marker_id++,
    ur5_motion_planner::makePose(0.55, 0.0, -0.25, 0.0, 0.0, 0.0, 1.0),
    "table",
    0.04,
    ur5_motion_planner::makeColor(0.95F, 0.95F, 0.95F, 0.95F)));
  markers.markers.push_back(ur5_motion_planner::makeBoxMarker(
    frame_id,
    "planning_scene_objects",
    marker_id++,
    ur5_motion_planner::makePose(0.52, 0.18, 0.18, 0.0, 0.0, 0.0, 1.0),
    0.16,
    0.22,
    0.28,
    ur5_motion_planner::makeColor(0.95F, 0.25F, 0.15F, 0.55F)));
  markers.markers.push_back(ur5_motion_planner::makeTextMarker(
    frame_id,
    "planning_scene_labels",
    marker_id++,
    ur5_motion_planner::makePose(0.52, 0.18, 0.18, 0.0, 0.0, 0.0, 1.0),
    "offset_box",
    0.04,
    ur5_motion_planner::makeColor(0.95F, 0.25F, 0.15F, 0.95F)));
  markers.markers.push_back(ur5_motion_planner::makeBoxMarker(
    frame_id,
    "planning_scene_objects",
    marker_id++,
    ur5_motion_planner::makePose(0.86, 0.0, 0.36, 0.0, 0.0, 0.0, 1.0),
    0.05,
    1.2,
    0.72,
    ur5_motion_planner::makeColor(0.15F, 0.35F, 0.95F, 0.35F)));
  markers.markers.push_back(ur5_motion_planner::makeTextMarker(
    frame_id,
    "planning_scene_labels",
    marker_id++,
    ur5_motion_planner::makePose(0.86, 0.0, 0.36, 0.0, 0.0, 0.0, 1.0),
    "back_wall",
    0.04,
    ur5_motion_planner::makeColor(0.15F, 0.35F, 0.95F, 0.95F)));

  planning_scene_interface.applyCollisionObjects(objects);
  RCLCPP_INFO(node->get_logger(), "added %zu collision objects in frame %s", objects.size(), frame_id.c_str());
  rclcpp::sleep_for(std::chrono::milliseconds(800));

  const auto target = ur5_motion_planner::makePose(0.36, -0.24, 0.42, 0.0, 0.7071, 0.0, 0.7071);
  const auto unreachable_target =
    ur5_motion_planner::makePose(2.20, 0.00, 1.20, 0.0, 0.7071, 0.0, 0.7071);
  markers.markers.push_back(ur5_motion_planner::makeSphereMarker(
    frame_id,
    "planning_scene_targets",
    marker_id++,
    target,
    0.055,
    ur5_motion_planner::makeColor(0.1F, 0.8F, 0.25F, 0.95F)));
  markers.markers.push_back(ur5_motion_planner::makeTextMarker(
    frame_id,
    "planning_scene_labels",
    marker_id++,
    target,
    "success_target",
    0.04,
    ur5_motion_planner::makeColor(0.1F, 0.8F, 0.25F, 0.95F)));
  markers.markers.push_back(ur5_motion_planner::makeSphereMarker(
    frame_id,
    "planning_scene_targets",
    marker_id++,
    unreachable_target,
    0.065,
    ur5_motion_planner::makeColor(0.95F, 0.05F, 0.05F, 0.95F)));
  markers.markers.push_back(ur5_motion_planner::makeTextMarker(
    frame_id,
    "planning_scene_labels",
    marker_id++,
    unreachable_target,
    "expected_failure_target",
    0.05,
    ur5_motion_planner::makeColor(0.95F, 0.05F, 0.05F, 0.95F)));
  marker_publisher->publish(markers);

  bool success = false;
  double planning_time_ms = 0.0;
  auto plan = ur5_motion_planner::planToPose(move_group, target, success, planning_time_ms);

  const auto & trajectory = plan.trajectory_.joint_trajectory;
  RCLCPP_INFO(
    node->get_logger(),
    "obstacle plan success=%s time=%.2f ms points=%zu joint_length=%.4f",
    success ? "true" : "false",
    planning_time_ms,
    trajectory.points.size(),
    ur5_motion_planner::jointSpaceLength(trajectory));

  ur5_motion_planner::appendPlanCsv(
    node->get_parameter("results_dir").as_string(),
    "planning_scene.csv",
    "obstacle_avoidance",
    success,
    planning_time_ms,
    trajectory,
    static_cast<double>(objects.size()),
    "collision_objects",
    "move_group_pose_goal_with_collision_scene");

  const double configured_planning_time = node->get_parameter("planning_time").as_double();
  move_group.setPlanningTime(2.0);
  bool failure_demo_success = false;
  double failure_demo_time_ms = 0.0;
  auto failure_demo_plan = ur5_motion_planner::planToPose(
    move_group, unreachable_target, failure_demo_success, failure_demo_time_ms);
  move_group.setPlanningTime(configured_planning_time);

  const auto & failure_trajectory = failure_demo_plan.trajectory_.joint_trajectory;
  RCLCPP_INFO(
    node->get_logger(),
    "expected failure demo success=%s time=%.2f ms points=%zu joint_length=%.4f",
    failure_demo_success ? "true" : "false",
    failure_demo_time_ms,
    failure_trajectory.points.size(),
    ur5_motion_planner::jointSpaceLength(failure_trajectory));

  ur5_motion_planner::appendPlanCsv(
    node->get_parameter("results_dir").as_string(),
    "planning_scene.csv",
    "expected_unreachable_failure",
    failure_demo_success,
    failure_demo_time_ms,
    failure_trajectory,
    2.20,
    "target_x_m",
    "move_group_pose_goal_unreachable_demo");

  if (success && node->get_parameter("execute").as_bool()) {
    const auto execution_result = move_group.execute(plan);
    RCLCPP_INFO(
      node->get_logger(),
      "execution result: %s",
      execution_result == moveit::core::MoveItErrorCode::SUCCESS ? "SUCCESS" : "FAILED");
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
