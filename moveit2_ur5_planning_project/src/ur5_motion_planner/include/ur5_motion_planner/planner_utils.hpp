#pragma once

#include <chrono>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ur5_motion_planner
{

constexpr const char * kPlanningGroup = "ur_manipulator";
constexpr const char * kEndEffectorLink = "tool0";

inline geometry_msgs::msg::Pose makePose(
  double x, double y, double z, double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

inline std::string nowStamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return out.str();
}

inline void ensureDirectory(const std::string & path)
{
  if (!path.empty()) {
    std::filesystem::create_directories(path);
  }
}

inline moveit::core::RobotState makeNamedRobotState(
  const moveit::planning_interface::MoveGroupInterface & move_group,
  const std::string & state_name)
{
  moveit::core::RobotState state(move_group.getRobotModel());
  const auto * joint_model_group = state.getJointModelGroup(kPlanningGroup);
  if (joint_model_group != nullptr) {
    state.setToDefaultValues(joint_model_group, state_name);
  } else {
    state.setToDefaultValues();
  }
  state.update();
  return state;
}

inline void setHomeStartState(moveit::planning_interface::MoveGroupInterface & move_group)
{
  const auto state = makeNamedRobotState(move_group, "home");
  move_group.setStartState(state);
}

inline geometry_msgs::msg::Pose poseFromState(
  const moveit::core::RobotState & state,
  const std::string & link_name)
{
  const auto & transform = state.getGlobalLinkTransform(link_name);
  const Eigen::Vector3d translation = transform.translation();
  const Eigen::Quaterniond rotation(transform.rotation());

  geometry_msgs::msg::Pose pose;
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

inline std::size_t pointCount(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  return trajectory.points.size();
}

inline double jointSpaceLength(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & a = trajectory.points[i - 1].positions;
    const auto & b = trajectory.points[i].positions;
    const auto n = std::min(a.size(), b.size());
    double step = 0.0;
    for (std::size_t j = 0; j < n; ++j) {
      const double d = b[j] - a[j];
      step += d * d;
    }
    length += std::sqrt(step);
  }
  return length;
}

inline double maxJointSpeed(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  double max_speed = 0.0;
  for (const auto & point : trajectory.points) {
    for (const auto velocity : point.velocities) {
      max_speed = std::max(max_speed, std::abs(velocity));
    }
  }
  return max_speed;
}

inline double trajectoryDurationSec(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  const auto & duration = trajectory.points.back().time_from_start;
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

inline void appendPlanCsv(
  const std::string & results_dir,
  const std::string & file_name,
  const std::string & scenario,
  bool success,
  double planning_time_ms,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  double extra_value = 0.0,
  const std::string & extra_name = "extra",
  const std::string & planner_method = "move_group_pose_goal")
{
  ensureDirectory(results_dir);
  const std::string path = results_dir + "/" + file_name;
  const bool new_file = !std::filesystem::exists(path);
  std::ofstream csv(path, std::ios::app);
  if (new_file) {
    csv << "timestamp,scenario,success,planning_time_ms,trajectory_points,"
        << "joint_space_length,max_joint_speed," << extra_name << "\n";
  }
  csv << nowStamp() << ","
      << scenario << ","
      << (success ? 1 : 0) << ","
      << planning_time_ms << ","
      << pointCount(trajectory) << ","
      << jointSpaceLength(trajectory) << ","
      << maxJointSpeed(trajectory) << ","
      << extra_value << "\n";

  const std::string summary_path = results_dir + "/trajectory_summary.csv";
  const bool new_summary = !std::filesystem::exists(summary_path);
  std::ofstream summary(summary_path, std::ios::app);
  if (new_summary) {
    summary << "timestamp,source_file,planner_method,scenario,success,planning_time_ms,"
            << "trajectory_duration_sec,trajectory_points,joint_space_length,max_joint_speed,"
            << "metric_name,metric_value\n";
  }
  summary << nowStamp() << ","
          << file_name << ","
          << planner_method << ","
          << scenario << ","
          << (success ? 1 : 0) << ","
          << planning_time_ms << ","
          << trajectoryDurationSec(trajectory) << ","
          << pointCount(trajectory) << ","
          << jointSpaceLength(trajectory) << ","
          << maxJointSpeed(trajectory) << ","
          << extra_name << ","
          << extra_value << "\n";
}

inline std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

inline visualization_msgs::msg::Marker baseMarker(
  const std::string & frame_id,
  const std::string & ns,
  int id,
  int type,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color = color;
  marker.pose.orientation.w = 1.0;
  return marker;
}

inline visualization_msgs::msg::Marker makeSphereMarker(
  const std::string & frame_id,
  const std::string & ns,
  int id,
  const geometry_msgs::msg::Pose & pose,
  double diameter,
  const std_msgs::msg::ColorRGBA & color)
{
  auto marker = baseMarker(
    frame_id, ns, id, visualization_msgs::msg::Marker::SPHERE, color);
  marker.pose = pose;
  marker.scale.x = diameter;
  marker.scale.y = diameter;
  marker.scale.z = diameter;
  return marker;
}

inline visualization_msgs::msg::Marker makeTextMarker(
  const std::string & frame_id,
  const std::string & ns,
  int id,
  const geometry_msgs::msg::Pose & pose,
  const std::string & text,
  double text_height,
  const std_msgs::msg::ColorRGBA & color)
{
  auto marker = baseMarker(
    frame_id, ns, id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, color);
  marker.pose = pose;
  marker.pose.position.z += text_height * 1.8;
  marker.scale.z = text_height;
  marker.text = text;
  return marker;
}

inline visualization_msgs::msg::Marker makeLineStripMarker(
  const std::string & frame_id,
  const std::string & ns,
  int id,
  const std::vector<geometry_msgs::msg::Pose> & poses,
  double width,
  const std_msgs::msg::ColorRGBA & color)
{
  auto marker = baseMarker(
    frame_id, ns, id, visualization_msgs::msg::Marker::LINE_STRIP, color);
  marker.scale.x = width;
  for (const auto & pose : poses) {
    marker.points.push_back(pose.position);
  }
  return marker;
}

inline visualization_msgs::msg::Marker makeBoxMarker(
  const std::string & frame_id,
  const std::string & ns,
  int id,
  const geometry_msgs::msg::Pose & pose,
  double size_x,
  double size_y,
  double size_z,
  const std_msgs::msg::ColorRGBA & color)
{
  auto marker = baseMarker(
    frame_id, ns, id, visualization_msgs::msg::Marker::CUBE, color);
  marker.pose = pose;
  marker.scale.x = size_x;
  marker.scale.y = size_y;
  marker.scale.z = size_z;
  return marker;
}

inline void publishMarkerArrayForDuration(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const visualization_msgs::msg::MarkerArray & markers,
  double seconds)
{
  const auto start = std::chrono::steady_clock::now();
  const auto duration = std::chrono::duration<double>(std::max(seconds, 0.0));
  do {
    auto stamped_markers = markers;
    const auto stamp = node->get_clock()->now().to_msg();
    for (auto & marker : stamped_markers.markers) {
      marker.header.stamp = stamp;
    }
    publisher->publish(stamped_markers);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  } while (
    rclcpp::ok() &&
    std::chrono::steady_clock::now() - start < duration);
}

inline moveit::planning_interface::MoveGroupInterface::Plan planToPose(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target,
  bool & success,
  double & planning_time_ms)
{
  move_group.setPoseTarget(target, kEndEffectorLink);
  move_group.setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto started = std::chrono::steady_clock::now();
  setHomeStartState(move_group);
  success = static_cast<bool>(move_group.plan(plan));
  const auto finished = std::chrono::steady_clock::now();
  planning_time_ms =
    std::chrono::duration<double, std::milli>(finished - started).count();
  move_group.clearPoseTargets();
  return plan;
}

}  // namespace ur5_motion_planner
