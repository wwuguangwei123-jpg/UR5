#!/usr/bin/env python3
import copy
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

HOME_POSITIONS = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]


def duration_to_sec(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class FakeTrajectoryController(Node):
    def __init__(self):
        super().__init__("fake_trajectory_controller")
        self.declare_parameter("execution_speed_scale", 0.5)
        self.declare_parameter("state_publish_period", 0.03)
        self.execution_speed_scale = max(
            0.05, float(self.get_parameter("execution_speed_scale").value)
        )
        state_publish_period = max(
            0.005, float(self.get_parameter("state_publish_period").value)
        )
        self.joint_names = list(DEFAULT_JOINTS)
        self.positions = list(HOME_POSITIONS)
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(state_publish_period, self.publish_state)
        self.scaled_server = ActionServer(
            self,
            FollowJointTrajectory,
            "scaled_joint_trajectory_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.standard_server = ActionServer(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            "fake FollowJointTrajectory server ready at "
            "/scaled_joint_trajectory_controller/follow_joint_trajectory and "
            "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.get_logger().info(
            "fake execution_speed_scale=%.2f (1.0 = planned speed, 0.5 = half speed)"
            % self.execution_speed_scale
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            "received trajectory goal with %d joints and %d points"
            % (len(goal_request.trajectory.joint_names), len(goal_request.trajectory.points))
        )
        if not goal_request.trajectory.points:
            self.get_logger().warn("Rejected empty trajectory goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.positions)
        msg.velocity = [0.0] * len(self.positions)
        msg.effort = [0.0] * len(self.positions)
        self.publisher.publish(msg)

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self.get_logger().info(
            "executing fake trajectory with %d points" % len(trajectory.points)
        )
        self.joint_names = list(trajectory.joint_names)
        if len(self.positions) != len(self.joint_names):
            self.positions = [0.0] * len(self.joint_names)

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(self.joint_names)

        start_time = time.monotonic()
        previous_time = 0.0
        previous_positions = list(self.positions)

        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Fake execution canceled"
                return result

            target_time = max(
                duration_to_sec(point.time_from_start) / self.execution_speed_scale,
                previous_time,
            )
            target_positions = list(point.positions)
            if len(target_positions) != len(self.joint_names):
                target_positions = previous_positions

            while True:
                elapsed = time.monotonic() - start_time
                if elapsed >= target_time:
                    break
                span = max(target_time - previous_time, 1e-6)
                ratio = min(max((elapsed - previous_time) / span, 0.0), 1.0)
                self.positions = [
                    a + (b - a) * ratio
                    for a, b in zip(previous_positions, target_positions)
                ]
                feedback.actual.positions = copy.copy(self.positions)
                feedback.desired.positions = copy.copy(target_positions)
                goal_handle.publish_feedback(feedback)
                self.publish_state()
                time.sleep(0.02)

            self.positions = target_positions
            previous_positions = target_positions
            previous_time = target_time
            self.publish_state()

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Fake trajectory execution completed"
        self.get_logger().info("fake trajectory execution completed")
        return result


def main():
    rclpy.init()
    node = FakeTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
