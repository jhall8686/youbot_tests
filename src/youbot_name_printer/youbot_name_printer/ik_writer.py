#!/usr/bin/env python3
"""
ros2_ik_writer.py  –  ROS2 node for end-effector position control via IK
Usage: ros2 run <your_pkg> ros2_ik_writer
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
from scipy.optimize import minimize

from youbot_name_printer.name_to_waypoints import name_to_waypoints

# ─── 1. DH PARAMETERS ────────────────────────────────────────────────────────
# Each row: [a, alpha, d, theta_offset]
#   a           = link length (m)
#   alpha       = link twist (rad)
#   d           = link offset (m)
#   theta_offset= constant added to the joint variable θᵢ
#
# Edit this table to match YOUR robot (example: a simple 3-DOF planar arm).

DH_PARAMS = [
    #  a      alpha        d     theta_offset
    [33.0,    np.pi/2,   147.0,   0.0],   # joint 1
    [155.0,   0.0,        0.0,   0.0],   # joint 2
    [135.0,   0.0,        0.0,   0.0],   # joint 3
    [0.0,    -np.pi/2,    0.0,   0.0],   # joint 4
    [0.0,     0.0,       217.6,   0.0],   # joint 5
]

JOINT_NAMES = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

# Joint angle limits [min, max] in radians
JOINT_LIMITS = [
    (-np.pi,  np.pi),
    (-np.pi,  np.pi),
    (-np.pi,  np.pi),
    (-np.pi,  np.pi),
    (-np.pi,  np.pi),
]


# ─── 2. FORWARD KINEMATICS ───────────────────────────────────────────────────

def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Standard DH 4×4 homogeneous transform for a single joint/link.
    Returns T = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct,  -st*ca,   st*sa,  a*ct],
        [st,   ct*ca,  -ct*sa,  a*st],
        [0.0,     sa,      ca,     d],
        [0.0,    0.0,     0.0,   1.0],
    ])


def forward_kinematics(joint_angles: list[float]) -> np.ndarray:
    """
    Chain DH transforms for all joints.
    Returns the 4x4 end-effector transform T_0n.
    Position is T_0n[:3, 3]; rotation matrix is T_0n[:3, :3].
    """
    T = np.eye(4)
    for (a, alpha, d, theta_off), theta in zip(DH_PARAMS, joint_angles):
        T = T @ dh_transform(a, alpha, d, theta + theta_off)
    return T


def get_position(joint_angles: list[float]) -> np.ndarray:
    """Return (x, y, z) of the end-effector."""
    return forward_kinematics(joint_angles)[:3, 3]


# ─── 3. INVERSE KINEMATICS ───────────────────────────────────────────────────

def inverse_kinematics(
    target_xyz: np.ndarray,
    initial_guess: list[float] | None = None,
    tol: float = 1e-4,
    n_restarts: int = 20,
) -> tuple[list[float], bool]:
    """
    IK with random restarts. Tries multiple starting points and returns
    the best solution found. Much more robust for redundant arms (5-DOF+).
    """
    n_joints = len(DH_PARAMS)

    def cost(angles):
        pos = get_position(angles)
        return float(np.sum((pos - target_xyz) ** 2))

    best_result = None
    best_cost = np.inf

    # Build list of starting points: user guess (if any) + random samples
    starts = []
    if initial_guess is not None:
        starts.append(np.array(initial_guess))
    
    rng = np.random.default_rng(seed=42)
    for _ in range(n_restarts):
        random_start = rng.uniform(
            low  = [lo for lo, _ in JOINT_LIMITS],
            high = [hi for _, hi in JOINT_LIMITS],
        )
        starts.append(random_start)

    for start in starts:
        result = minimize(
            cost,
            x0=start,
            method="L-BFGS-B",
            bounds=JOINT_LIMITS,
            options={"ftol": 1e-12, "gtol": 1e-8, "maxiter": 2000},
        )
        if result.fun < best_cost:
            best_cost = result.fun
            best_result = result

    success = best_cost < tol ** 2
    return best_result.x.tolist(), success


# ─── 4. ROS2 NODE ────────────────────────────────────────────────────────────

class IKWriterNode(Node):

    def __init__(self):
        super().__init__("ik_writer_node")
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        # Example: move to a few waypoints (replace with your name-tracing points)
        waypoints_xyz = name_to_waypoints(
            name="Jack",
            z_write=150.0,     # mm — pen touching surface
            z_travel=180.0,    # mm — pen lifted between strokes
            x_offset=200.0,    # mm — adjust to center in your workspace
            y_offset=0.0,
            scale=30.0,        # mm per font unit — tune this
            n_points=50,
        )
        self.execute_waypoints(waypoints_xyz)

    def execute_waypoints(self, waypoints_xyz: list[list[float]]):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        # Solve IK for all waypoints first
        solutions = []
        current_angles = [0.0] * len(DH_PARAMS)
        time_per_point = 1.5  # seconds between waypoints — decrease to speed up

        for target in waypoints_xyz:
            angles, ok = inverse_kinematics(
                np.array(target),
                initial_guess=current_angles,
            )
            if not ok:
                self.get_logger().warn(f"IK failed for target {target}, skipping.")
                continue
            solutions.append(angles)
            current_angles = angles

        if not solutions:
            self.get_logger().error("No valid IK solutions found.")
            return

        # Build trajectory points with finite-difference velocities
        for i, angles in enumerate(solutions):
            time_s = (i + 1) * time_per_point

            # Velocity: finite difference to next point, zero at last point
            if i < len(solutions) - 1:
                next_angles = solutions[i + 1]
                velocities = [
                    (next_angles[j] - angles[j]) / time_per_point
                    for j in range(len(angles))
                ]
            else:
                velocities = [0.0] * len(angles)

            point = JointTrajectoryPoint()
            point.positions  = angles
            point.velocities = velocities
            point.time_from_start = Duration(
                sec=int(time_s),
                nanosec=int((time_s % 1) * 1e9),
            )
            traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"Trajectory published with {len(solutions)} points.")


def main(args=None):
    rclpy.init(args=args)
    node = IKWriterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()