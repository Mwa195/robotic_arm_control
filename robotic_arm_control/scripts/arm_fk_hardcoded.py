#!/usr/bin/env python3
"""
Forward Kinematics for robotic_arm_control robot.

Kinematic chain (from URDF):
  rover_link
    -> base_joint  (Rz(t1), origin: 0,0,0)      -> base_Link
    -> shoulder_joint (Rx(-t2), origin: 0,0,0.16) -> link1_Link
    -> elbow_joint  (Rx(+t3), origin: 0.006,-0.32763,0.19251) -> link2_Link
  Tip = link2_Link origin + Rx(t3) @ (0, -L2, 0)

Coordinate conventions:
  t1 (base_joint):     rotation about +Z   [±180°]
  t2 (shoulder_joint): rotation about -X   [±90°]
  t3 (elbow_joint):    rotation about +X   [±120°]
  At t1=t2=t3=0 the arm hangs toward -Y and angled ~30° up.

This script can run in two modes:
  1. Standalone: computes FK for a given set of angles and prints the result.
  2. ROS2 node:  subscribes to /joint_states and prints live FK as the arm moves.
"""

import math
import numpy as np
import sys


# ─── Kinematic parameters (from URDF) ─────────────────────────────────────────

# Shoulder joint origin in rover frame (constant)
SHOULDER_ORIGIN = np.array([0.0, 0.0, 0.16])   # metres

# Elbow joint origin in link1_Link frame (constant structural offset)
ELBOW_IN_LINK1 = np.array([0.006, -0.32763, 0.19251])  # metres

# Link lengths derived from the structural offset vectors:
#   L1 = |ELBOW_IN_LINK1|_yz = sqrt(0.32763² + 0.19251²) = 0.3800 m
#   L2 = tip-to-elbow length (estimated from link2 CoM; verify in Gazebo via /tf)
L1 = math.sqrt(ELBOW_IN_LINK1[1]**2 + ELBOW_IN_LINK1[2]**2)  # 0.3800 m
L2 = 0.3087  # metres — TO VERIFY against /tf output in simulation

# Structural angle of link1 at zero config (angle of elbow offset from horizontal in arm plane)
# alpha1_s = atan2(z_elbow, -y_elbow) = atan2(0.19251, 0.32763) ≈ 30.44°
ALPHA1_STRUCTURAL = math.atan2(ELBOW_IN_LINK1[2], -ELBOW_IN_LINK1[1])


# ─── Rotation matrices ─────────────────────────────────────────────────────────

def Rx(theta: float) -> np.ndarray:
    """3x3 rotation matrix about X axis."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])


def Rz(theta: float) -> np.ndarray:
    """3x3 rotation matrix about Z axis."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


# ─── Forward kinematics ────────────────────────────────────────────────────────

def fk(t1: float, t2: float, t3: float, l2: float = L2):
    """
    Compute forward kinematics for the arm.

    Parameters
    ----------
    t1 : float   base_joint angle [rad], rotation about +Z
    t2 : float   shoulder_joint angle [rad], rotation about -X
    t3 : float   elbow_joint angle [rad], rotation about +X
    l2 : float   tip-to-elbow length [m] (override for calibration)

    Returns
    -------
    shoulder : np.ndarray shape (3,)  — shoulder joint position in rover frame
    elbow    : np.ndarray shape (3,)  — elbow joint position in rover frame
    tip      : np.ndarray shape (3,)  — end-effector position in rover frame
    """
    # Rotation from rover frame to link1_Link frame:
    #   base rotates by t1 about Z, then shoulder rotates by t2 about -X (= Rx(-t2))
    R_rover_to_link1 = Rz(t1) @ Rx(-t2)

    # Elbow position in rover frame
    elbow = SHOULDER_ORIGIN + R_rover_to_link1 @ ELBOW_IN_LINK1

    # Rotation from rover frame to link2_Link frame (add elbow rotation about +X)
    R_rover_to_link2 = R_rover_to_link1 @ Rx(t3)

    # Tip offset in link2_Link frame (arm extends in -Y direction of link2)
    tip_offset_in_link2 = np.array([0.0, -l2, 0.0])

    # Tip position in rover frame
    tip = elbow + R_rover_to_link2 @ tip_offset_in_link2

    return SHOULDER_ORIGIN.copy(), elbow, tip


def print_fk_result(t1: float, t2: float, t3: float):
    """Print a formatted FK result for given joint angles."""
    shoulder, elbow, tip = fk(t1, t2, t3)
    print(f"\n{'─'*55}")
    print(f"  Joint angles:  t1={math.degrees(t1):+7.2f}°  "
          f"t2={math.degrees(t2):+7.2f}°  t3={math.degrees(t3):+7.2f}°")
    print(f"  Shoulder:  ({shoulder[0]:+.4f}, {shoulder[1]:+.4f}, {shoulder[2]:+.4f}) m")
    print(f"  Elbow:     ({elbow[0]:+.4f}, {elbow[1]:+.4f}, {elbow[2]:+.4f}) m")
    print(f"  Tip:       ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f}) m")
    r = math.sqrt(tip[0]**2 + tip[1]**2)
    print(f"  Reach:     r={r:.4f} m  z={tip[2]:.4f} m  "
          f"D={math.sqrt(r**2 + (tip[2]-0.16)**2):.4f} m from shoulder")
    print(f"{'─'*55}")


# ─── ROS2 node mode ────────────────────────────────────────────────────────────

def run_ros2_node():
    """
    Subscribe to /joint_states and continuously print FK.
    Use this to verify L2 against Gazebo's /tf output.

    Run:
        ros2 run robotic_arm_control arm_fk.py --ros
    Then in another terminal:
        ros2 topic echo /joint_states
        ros2 run tf2_tools view_frames   (to inspect /tf tree)
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    JOINT_ORDER = ['base_joint', 'shoulder_joint', 'elbow_joint']

    class FKNode(Node):
        def __init__(self):
            super().__init__('arm_fk_node')
            self.sub = self.create_subscription(
                JointState, '/joint_states', self._cb, 10)
            self.get_logger().info(
                'FK node running. Listening on /joint_states...')
            self.get_logger().info(
                f'L1={L1:.4f}m  L2={L2:.4f}m  alpha1={math.degrees(ALPHA1_STRUCTURAL):.2f}°')

        def _cb(self, msg: JointState):
            # Build angle dict (joint_states order is not guaranteed)
            angles = dict(zip(msg.name, msg.position))
            try:
                t1 = angles['base_joint']
                t2 = angles['shoulder_joint']
                t3 = angles['elbow_joint']
            except KeyError:
                self.get_logger().warn(
                    f'Missing joints. Got: {list(angles.keys())}')
                return
            print_fk_result(t1, t2, t3)

    rclpy.init()
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ─── Standalone test ────────────────────────────────────────────────────────────

def run_standalone():
    """
    Print FK for a set of known configurations.
    These match the positions in sim_control_testing.py so you can cross-check
    against Gazebo's /tf output.
    """
    print("Forward Kinematics — robotic_arm_control")
    print(f"L1={L1:.4f}m  L2={L2:.4f}m (ESTIMATE — verify with /tf)")
    print(f"alpha1_structural={math.degrees(ALPHA1_STRUCTURAL):.2f}°")

    configs = [
        (0.0,   0.0,   0.0,   "home (all zeros)"),
        (1.57,  0.0,   0.0,   "base 90° only"),
        (1.57,  0.785, 0.0,   "base + shoulder"),
        (1.57,  0.785, -1.0,  "base + shoulder + elbow"),
        (0.0,   0.785, -1.0,  "shoulder + elbow only"),
    ]

    for t1, t2, t3, label in configs:
        print(f"\n[{label}]")
        print_fk_result(t1, t2, t3)

    print("\n\nHOW TO VERIFY L2 IN SIMULATION:")
    print("  1. Launch Gazebo:  ros2 launch robotic_arm_control gazebo.launch.py")
    print("  2. Run test script:  ros2 run robotic_arm_control sim_control_testing.py")
    print("  3. In another terminal:")
    print("     ros2 run tf2_ros tf2_echo rover_link link2_Link")
    print("     Compare the translation with the FK tip output above.")
    print("     If they differ, update L2 at the top of this file.")


# ─── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    if '--ros' in sys.argv:
        run_ros2_node()
    else:
        run_standalone()