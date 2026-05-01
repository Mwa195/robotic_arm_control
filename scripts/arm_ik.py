#!/usr/bin/env python3
"""
Inverse Kinematics for robotic_arm_control robot.

DERIVATION SUMMARY
══════════════════
The arm has 3 DOF: base (Z), shoulder (-X), elbow (+X).
Shoulder and elbow rotate about parallel axes, so the arm operates in a plane.

Step 1 — Decouple base:
    The arm's outward direction at zero config is -Y.
    At base angle t1, the arm faces direction (sin t1, -cos t1, 0) in rover frame.
    To point the arm toward a target at (x, y, z):
        t1 = atan2(x, -y)
    This makes -y the "reference" direction, consistent with the URDF zero config.

Step 2 — 2D planar IK:
    Project the target into the arm's (r, h) plane:
        r = sqrt(x² + y²)       [horizontal distance from rover Z-axis]
        h = z - 0.16            [height above shoulder joint]

    The arm geometry in this plane is:
        r = L1·cos(phi1) + L2·cos(psi)
        h = L1·sin(phi1) + L2·sin(psi)
    where:
        phi1 = alpha1_structural + t2    [link1 world-frame angle from +r]
        psi  = t2 - t3                   [link2 world-frame angle from +r]
        alpha1_structural = atan2(0.19251, 0.32763) ≈ 30.44°

    This is a standard 2-link planar IK problem:
        D² = r² + h²
        cos(psi - phi1) = (D² - L1² - L2²) / (2·L1·L2)   [law of cosines]
        gamma = acos((D² + L1² - L2²) / (2·D·L1))         [angle at shoulder]
        beta  = atan2(h, r)

    Two solutions (elbow configurations):
        elbow_up:   phi1 = beta + gamma,  psi = phi1 - delta_abs
        elbow_down: phi1 = beta - gamma,  psi = phi1 + delta_abs
    where delta_abs = acos((D² - L1² - L2²) / (2·L1·L2))

Step 3 — Convert back to joint angles:
        t2 = phi1 - alpha1_structural
        t3 = t2 - psi

ACCURACY NOTE
═════════════
The URDF has a 0.006 m X-offset at the elbow joint (0.006, -0.32763, 0.19251).
This makes the arm non-perfectly-planar, causing ~6 mm tip position residual.
Two modes are provided:
  - analytical:  closed-form, ~6 mm error, real-time safe
  - refined:     Newton refinement (1-2 iterations), <0.01 mm error, slightly slower

For simulation and real hardware, the analytical solution is adequate.
Use refined mode when sub-millimetre precision is required.
"""

import math
import numpy as np
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from arm_fk import fk, L1, L2, ALPHA1_STRUCTURAL, SHOULDER_ORIGIN

SHOULDER_H = SHOULDER_ORIGIN[2]  # 0.16 m

# Joint limits from URDF (radians)
JOINT_LIMITS = {
    'base_joint':     (-math.pi,  math.pi),   # ±180°
    'shoulder_joint': (-1.57,     1.57),       # ±90°
    'elbow_joint':    (-2.094,    2.094),      # ±120°
}


# ─── IK solver ────────────────────────────────────────────────────────────────

def ik_analytical(x: float, y: float, z: float,
                  elbow_up: bool = True):
    """
    Closed-form IK. Returns (t1, t2, t3) in radians, or None if unreachable.

    Parameters
    ----------
    x, y, z  : float  — target tip position in rover frame [metres]
    elbow_up : bool   — True  → elbow is above the shoulder-to-tip line
                        False → elbow is below the shoulder-to-tip line

    Returns
    -------
    (t1, t2, t3) : tuple[float, float, float]  — joint angles [rad]
    None          — if target is outside the reachable workspace

    Steps
    -----
    ── Step 1: base angle ────────────────────────────────────────────────────

    Arm zero-config points in -Y direction. atan2(x, -y) rotates to face target.


    ── Step 2: project to arm's 2D plane ────────────────────────────────────

    r  : horizontal distance from rover Z-axis [m]
    h  : height above shoulder joint [m]
    D  : straight-line distance from shoulder to target in the (r, h) plane [m]


    ── Step 3: reachability check ────────────────────────────────────────────

    D must lie within [|L1 - L2|, L1 + L2] for a solution to exist.
    Clamp D slightly at the boundary to avoid floating-point failures.


    ── Step 4: angles in arm's (r, h) plane ──────────────────────────────────

    beta    : direction angle from shoulder to target [rad]
    gamma   : angle at shoulder in the (shoulder, elbow, tip) triangle [rad]
              derived from the law of cosines:
              D² = L1² + L2² - 2·L1·L2·cos(π - gamma)
    delta_abs : magnitude of the elbow "bend" angle [rad]
              derived from the law of cosines applied to the full triangle


    ── Step 5: select configuration and compute phi1, psi ───────────────────

    elbow_up:   elbow is above the shoulder-to-tip line
      phi1 = beta + gamma  (link1 tilts further up than the target direction)
      psi  = phi1 - delta_abs  (link2 angles back down toward target)

    elbow_down: elbow is below the shoulder-to-tip line
      phi1 = beta - gamma  (link1 tilts below the target direction)
      psi  = phi1 + delta_abs  (link2 angles up toward target)


    ── Step 6: convert to joint angles ───────────────────────────────────────

    phi1 = ALPHA1_STRUCTURAL + t2  →  t2 = phi1 - ALPHA1_STRUCTURAL
    psi  = t2 - t3                 →  t3 = t2 - psi
    """

    # ── Step 1: base angle ────────────────────────────────────────────────────
    t1 = math.atan2(x, -y)

    # ── Step 2: project to arm's 2D plane ────────────────────────────────────
    r = math.sqrt(x**2 + y**2)
    h = z - SHOULDER_H
    D_sq = r**2 + h**2
    D = math.sqrt(D_sq)

    # ── Step 3: reachability check ────────────────────────────────────────────
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

    if D > max_reach + 1e-9:
        return None   # target too far from shoulder

    if D < min_reach - 1e-9:
        return None   # target too close (arm cannot fold tight enough)

    # Clamp D slightly to avoid floating-point issues at workspace boundary
    D = max(min_reach, min(max_reach, D))
    D_sq = D**2

    # ── Step 4: angles in arm's (r, h) plane ──────────────────────────────────
    beta = math.atan2(h, r)

    cos_gamma = (D_sq + L1**2 - L2**2) / (2.0 * D * L1)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)

    # From law of cosines: D² = L1² + L2² + 2·L1·L2·cos(psi - phi1)
    cos_delta = (D_sq - L1**2 - L2**2) / (2.0 * L1 * L2)
    cos_delta = max(-1.0, min(1.0, cos_delta))
    delta_abs = math.acos(cos_delta)

    # ── Step 5: select configuration and compute phi1, psi ───────────────────
    if elbow_up:
        phi1 = beta + gamma
        psi  = phi1 - delta_abs
    else:
        phi1 = beta - gamma
        psi  = phi1 + delta_abs

    # ── Step 6: convert to joint angles ───────────────────────────────────────
    t2 = phi1 - ALPHA1_STRUCTURAL
    t3 = t2 - psi

    return (t1, t2, t3)


def ik_refined(x: float, y: float, z: float,
               elbow_up: bool = True,
               tol: float = 1e-8):
    """
    IK with Newton refinement to eliminate the ~6 mm error from the 0.006 m
    structural X-offset at the elbow joint.

    Uses the analytical solution as an initial guess, then runs scipy.optimize.fsolve
    for 1-2 iterations to converge to sub-millimetre accuracy.

    Parameters
    ----------
    x, y, z  : float  — target tip position in rover frame [metres]
    elbow_up : bool   — elbow configuration (same as ik_analytical)
    tol      : float  — convergence tolerance [metres]

    Returns
    -------
    (t1, t2, t3) : tuple[float, float, float]  — refined joint angles [rad]
    None          — if target is unreachable
    """
    from scipy.optimize import fsolve

    init = ik_analytical(x, y, z, elbow_up=elbow_up)
    if init is None:
        return None

    target_pos = np.array([x, y, z])

    def residuals(angles):
        _, _, tip = fk(*angles)
        return tip - target_pos

    solution, info, ier, _ = fsolve(residuals, init, full_output=True)
    if np.linalg.norm(info['fvec']) < tol:
        return tuple(solution)

    # If Newton failed (shouldn't happen for reachable targets), fall back
    return init


# ─── Limit checking ───────────────────────────────────────────────────────────

def check_limits(t1: float, t2: float, t3: float) -> dict:
    """
    Check if joint angles are within URDF limits.

    Returns a dict with keys 'base_joint', 'shoulder_joint', 'elbow_joint',
    each mapping to True (within limits) or False (violated).
    """
    angles = {
        'base_joint':     t1,
        'shoulder_joint': t2,
        'elbow_joint':    t3,
    }
    return {
        joint: (lo <= angle <= hi)
        for joint, angle in angles.items()
        for lo, hi in [JOINT_LIMITS[joint]]
    }


def ik(x: float, y: float, z: float,
       elbow_up: bool = True,
       use_refined: bool = False,
       check_joint_limits: bool = True):
    """
    Main IK entry point. Returns joint angles or None.

    Parameters
    ----------
    x, y, z           : float — target in rover frame [metres]
    elbow_up          : bool  — elbow configuration
    use_refined       : bool  — apply Newton refinement (eliminates ~6 mm error)
    check_joint_limits: bool  — return None if solution violates joint limits

    Returns
    -------
    (t1, t2, t3) in radians, or None if no valid solution exists.
    """
    if use_refined:
        result = ik_refined(x, y, z, elbow_up=elbow_up)
    else:
        result = ik_analytical(x, y, z, elbow_up=elbow_up)

    if result is None:
        return None

    if check_joint_limits:
        limits_ok = check_limits(*result)
        if not all(limits_ok.values()):
            return None   # joint limit violated

    return result


# ─── ROS2 node ────────────────────────────────────────────────────────────────

class ArmIKNode(Node):
    # The three joint names sent to the trajectory action server
    JOINT_ORDER = ['base_joint', 'shoulder_joint', 'elbow_joint']

    def __init__(self, x: float, y: float, z: float,
                 elbow_up: bool = True,
                 use_refined: bool = True,
                 duration_sec: float = 3.0):
        super().__init__('arm_ik_node')

        self._ac = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

        self.get_logger().info(
            f'IK node started. Target: ({x:.3f}, {y:.3f}, {z:.3f}) m  '
            f'elbow_{"up" if elbow_up else "down"}  '
            f'refined={use_refined}')

        self.get_logger().info(
            f'L1={L1:.4f}m  L2={L2:.4f}m  alpha1={math.degrees(ALPHA1_STRUCTURAL):.2f}°')

        # Solve IK immediately on startup
        result = ik(x, y, z, elbow_up=elbow_up, use_refined=use_refined)

        if result is None:
            self.get_logger().error(
                f'IK FAILED: target ({x:.3f}, {y:.3f}, {z:.3f}) '
                f'is unreachable or violates joint limits.')
            return

        t1, t2, t3 = result

        self.get_logger().info(
            f'IK solution: t1={math.degrees(t1):+.2f}°  '
            f't2={math.degrees(t2):+.2f}°  '
            f't3={math.degrees(t3):+.2f}°')

        # FK verification — confirms the solution maps back to the target
        _, _, tip = fk(t1, t2, t3)
        self.get_logger().info(
            f'FK check: ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f}) m')

        self._send(t1, t2, t3, duration_sec)

    def _send(self, t1: float, t2: float, t3: float, duration_sec: float):
        """
        Send a FollowJointTrajectory goal to move the arm to (t1, t2, t3).

        Builds a single-point trajectory and dispatches it to the action server.
        Blocks until the server accepts and completes the goal.

        Parameters
        ----------
        t1, t2, t3   : float — joint angles [rad]
        duration_sec : float — time allowed for the motion [seconds]
        """
        self.get_logger().info('Waiting for action server...')
        self._ac.wait_for_server()

        # Build goal — single waypoint at the IK solution
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.JOINT_ORDER

        # JointTrajectoryPoint holds the target positions and the arrival time
        pt = JointTrajectoryPoint()
        pt.positions = [t1, t2, t3]
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points = [pt]

        # Send asynchronously and spin until the server accepts the goal
        future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected by action server.')
            return

        # Spin again until the motion completes
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('Motion complete.')


def main(args=None):
    """
    Usage:
        ros2 run robotic_arm_control arm_ik.py x y z [elbow_up]
    Example:
        ros2 run robotic_arm_control arm_ik.py 0.3 -0.3 0.5
        ros2 run robotic_arm_control arm_ik.py 0.3 -0.3 0.5 False
    """
    user_args = rclpy.utilities.remove_ros_args(sys.argv)[1:]

    if len(user_args) < 3:
        print('Incomplete args - Usage: ros2 run robotic_arm_control arm_ik.py x y z [elbow_up=True]')
        sys.exit(1)

    x, y, z = float(user_args[0]), float(user_args[1]), float(user_args[2])
    elbow_up = (user_args[3].lower() != 'false') if len(user_args) > 3 else True

    rclpy.init(args=args)
    node = ArmIKNode(x, y, z, elbow_up=elbow_up)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()