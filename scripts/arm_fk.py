#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ─── Kinematic parameters (from URDF) ─────────────────────────────────────────

SHOULDER_ORIGIN = np.array([0.0, 0.0, 0.16])
ELBOW_IN_LINK1 = np.array([0.006, -0.32763, 0.19251])
L1 = math.sqrt(ELBOW_IN_LINK1[1]**2 + ELBOW_IN_LINK1[2]**2)
L2 = 0.3087
ALPHA1_STRUCTURAL = math.atan2(ELBOW_IN_LINK1[2], -ELBOW_IN_LINK1[1])


# ─── Rotation matrices ─────────────────────────────────────────────────────────

def Rx(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])

def Rz(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


# ─── Forward kinematics ────────────────────────────────────────────────────────

def fk(t1: float, t2: float, t3: float, l2: float = L2):
    R_rover_to_link1 = Rz(t1) @ Rx(-t2)
    elbow = SHOULDER_ORIGIN + R_rover_to_link1 @ ELBOW_IN_LINK1
    R_rover_to_link2 = R_rover_to_link1 @ Rx(t3)
    tip_offset_in_link2 = np.array([0.0, -l2, 0.0])
    tip = elbow + R_rover_to_link2 @ tip_offset_in_link2
    return SHOULDER_ORIGIN.copy(), elbow, tip


# ─── ROS2 Node ────────────────────────────────────────────────────────────────

class ArmFKNode(Node):
    JOINT_ORDER = ['base_joint', 'shoulder_joint', 'elbow_joint']

    def __init__(self):
        super().__init__("arm_fk_node")
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._cb, 10)
        self.get_logger().info('FK node running. Listening on /joint_states...')
        self.get_logger().info(
            f'L1={L1:.4f}m  L2={L2:.4f}m  alpha1={math.degrees(ALPHA1_STRUCTURAL):.2f}°')

    def _cb(self, msg: JointState):
        angles = dict(zip(msg.name, msg.position))
        try:
            t1 = angles['base_joint']
            t2 = angles['shoulder_joint']
            t3 = angles['elbow_joint']
        except KeyError:
            self.get_logger().warn(f'Missing joints. Got: {list(angles.keys())}')
            return
        self._print_fk_result(t1, t2, t3)

    def _print_fk_result(self, t1: float, t2: float, t3: float):
        shoulder, elbow, tip = fk(t1, t2, t3)
        r = math.sqrt(tip[0]**2 + tip[1]**2)
        print(f"\n{'─'*55}")
        print(f"  Joint angles:  t1={math.degrees(t1):+7.2f}°  "
            f"t2={math.degrees(t2):+7.2f}°  t3={math.degrees(t3):+7.2f}°")
        print(f"  Shoulder:  ({shoulder[0]:+.4f}, {shoulder[1]:+.4f}, {shoulder[2]:+.4f}) m")
        print(f"  Elbow:     ({elbow[0]:+.4f}, {elbow[1]:+.4f}, {elbow[2]:+.4f}) m")
        print(f"  Tip:       ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f}) m")
        print(f"  Reach:     r={r:.4f} m  z={tip[2]:.4f} m  "
            f"D={math.sqrt(r**2 + (tip[2]-0.16)**2):.4f} m from shoulder")
        print(f"{'─'*55}")


# ─── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    myNode = ArmFKNode()
    rclpy.spin(myNode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()