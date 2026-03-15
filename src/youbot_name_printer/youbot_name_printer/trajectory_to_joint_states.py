#!/usr/bin/env python3
"""
trajectory_to_joint_states.py  -  Bridges JointTrajectory → JointState
Republishes each waypoint from the trajectory as a JointState message
so RViz can visualize the motion.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

import threading
import time


class TrajectoryToJointStates(Node):

    def __init__(self):
        super().__init__("trajectory_to_joint_states")

        self.subscription = self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )
        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # Publishes the current joint state at a fixed rate so RViz
        # always has a fresh message (required by robot_state_publisher)
        self.current_positions = None
        self.current_velocities = None  # ← add this
        self.joint_names = []
        self.timer = self.create_timer(0.05, self.publish_current_state)  # 20 Hz

        self.get_logger().info("Listening on /joint_trajectory_controller/joint_trajectory ...")

    def trajectory_callback(self, msg: JointTrajectory):
        """
        When a trajectory arrives, replay each waypoint in real time
        by sleeping between points according to time_from_start.
        Runs in a separate thread so it doesn't block the ROS executor.
        """
        self.joint_names = msg.joint_names
        self.get_logger().info(f"Received trajectory with {len(msg.points)} waypoints.")
        thread = threading.Thread(target=self.replay_trajectory, args=(msg,), daemon=True)
        thread.start()

    def replay_trajectory(self, msg: JointTrajectory):
        """Smoothly interpolate between waypoints at 20Hz."""
        start_time = time.time()
        points = msg.points
        hz = 20
        dt = 1.0 / hz

        while True:
            elapsed = time.time() - start_time

            # Find which segment we're in
            if elapsed >= points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9:
                # Past the end — hold final pose
                self.current_positions  = list(points[-1].positions)
                self.current_velocities = list(points[-1].velocities)
                break

            # Find the two waypoints we're currently between
            seg_start = None
            seg_end   = None
            for i in range(len(points) - 1):
                t0 = points[i].time_from_start.sec     + points[i].time_from_start.nanosec     * 1e-9
                t1 = points[i+1].time_from_start.sec   + points[i+1].time_from_start.nanosec   * 1e-9
                if t0 <= elapsed <= t1:
                    seg_start = points[i]
                    seg_end   = points[i+1]
                    alpha = (elapsed - t0) / (t1 - t0)  # 0.0 → 1.0
                    break

            if seg_start is not None:
                # Linear interpolation between the two waypoints
                self.current_positions = [
                    seg_start.positions[j] + alpha * (seg_end.positions[j] - seg_start.positions[j])
                    for j in range(len(seg_start.positions))
                ]
                self.current_velocities = [
                    seg_start.velocities[j] + alpha * (seg_end.velocities[j] - seg_start.velocities[j])
                    for j in range(len(seg_start.velocities))
                ]

            time.sleep(dt)

    def publish_current_state(self):
        if self.current_positions is None or not self.joint_names:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = self.joint_names
        msg.position     = self.current_positions
        msg.velocity     = self.current_velocities  # ← and this
        msg.effort       = [0.0] * len(self.current_positions)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()