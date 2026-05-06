#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ─── Kinematic parameters (from URDF) ─────────────────────────────────────────

# Position of the shoulder joint in the rover/world frame (x=0, y=0, z=0.16m)
# This is a FIXED point — it never moves regardless of joint angles
SHOULDER_ORIGIN = np.array([0.0, 0.0, 0.16])

# Position of the elbow joint relative to the shoulder, expressed in link1's LOCAL frame
# Taken directly from URDF: elbow_joint origin xyz="0.006 -0.32763 0.19251"
ELBOW_IN_LINK1 = np.array([0.006, -0.32763, 0.19251])

# Effective length of link1 (shoulder → elbow straight-line distance in the Y-Z plane)
# We ignore the 0.006m X offset because link1 swings in the Y-Z plane
# Pythagorean theorem: L1 = sqrt((-0.32763)² + (0.19251)²) ≈ 0.3809m
L1 = math.sqrt(ELBOW_IN_LINK1[1]**2 + ELBOW_IN_LINK1[2]**2)

# Length of link2 (elbow → tip straight-line distance)
# Taken directly from URDF: tip_joint origin xyz="0 -0.3087 0"
L2 = 0.3087

# Structural tilt angle of link1 at zero joint angles, measured from the horizontal
# At zero config, the elbow is at (+0.19251m in Z, -0.32763m in Y) relative to shoulder
# atan2(0.19251, 0.32763) ≈ 0.5313 rad ≈ 30.44°
# This means link1 is already tilted 30.44° upward even when shoulder_joint = 0
# Must be subtracted when converting geometric angles → actual joint angles
ALPHA1_STRUCTURAL = math.atan2(ELBOW_IN_LINK1[2], -ELBOW_IN_LINK1[1])


# ─── Rotation matrices ─────────────────────────────────────────────────────────

def Rx(theta: float) -> np.ndarray:
    """
    3x3 rotation matrix for a rotation of `theta` radians around the X axis.
    Rotating a vector by this matrix spins it around the X axis.

    Formula:
        [1,    0,     0  ]
        [0,  cos θ, -sin θ]
        [0,  sin θ,  cos θ]

    X row is unchanged (stays [1,0,0]) because X is the rotation axis.
    Y and Z rows mix together via cos/sin.
    """
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])

def Rz(theta: float) -> np.ndarray:
    """
    3x3 rotation matrix for a rotation of `theta` radians around the Z axis.
    Rotating a vector by this matrix spins it around the Z axis.

    Formula:
        [cos θ, -sin θ, 0]
        [sin θ,  cos θ, 0]
        [0,      0,     1]

    Z row is unchanged (stays [0,0,1]) because Z is the rotation axis.
    X and Y rows mix together via cos/sin.
    """
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def fk(t1: float, t2: float, t3: float, l2: float = L2):
    """
    Forward kinematics: given joint angles → compute positions of shoulder, elbow, tip.

    Parameters
    ----------
    t1 : base_joint angle [rad]      — rotates around Z axis
    t2 : shoulder_joint angle [rad]  — rotates around -X axis
    t3 : elbow_joint angle [rad]     — rotates around +X axis
    l2 : link2 length [m]            — defaults to L2 from URDF

    Returns
    -------
    (shoulder, elbow, tip) — three (3,) numpy arrays, all in the rover/world frame [metres]
    
    Steps
    -------
    ── Step 1: Compute link1's orientation in the world frame ────────────────
    
    base_joint  rotates around Z  → apply Rz(t1)
    shoulder_joint rotates around -X → apply Rx(-t2)   [note the negative sign]
    
    @ is matrix multiplication. Order matters — rightmost is applied first:
      First rotate by shoulder (Rx(-t2)), then rotate by base (Rz(t1))
      This gives the combined orientation of link1 in the world frame.
    
    R_rover_to_link1 : 3x3 matrix
      Transforms any vector from link1's LOCAL frame → world frame
    

    ── Step 2: Compute elbow position in world frame ─────────────────────────
    
    ELBOW_IN_LINK1 is the elbow offset expressed in link1's LOCAL frame.
    Multiply by R_rover_to_link1 to rotate it into the WORLD frame.
    Then add SHOULDER_ORIGIN (the fixed shoulder position) to get the absolute position.
    
    elbow [metres] : (3,) array — elbow joint position in world frame
    

    ── Step 3: Compute link2's orientation in the world frame ────────────────
    
    elbow_joint rotates around +X → apply Rx(t3)   [positive this time]
    Compose with link1's orientation to get link2's full world-frame orientation.
    
    R_rover_to_link2 : 3x3 matrix
      Transforms any vector from link2's LOCAL frame → world frame
    

    ── Step 4: Compute tip position in world frame ───────────────────────────
    
    In link2's LOCAL frame, the tip is L2 metres along the -Y axis.
    This matches the URDF tip_joint origin: xyz="0 -0.3087 0"
    
    tip_offset_in_link2 [metres] : (3,) array — tip offset in link2's local frame
    

    Rotate tip offset into world frame, then add elbow position
    tip [metres] : (3,) array — tip position in world frame
    """
    
    R_rover_to_link1 = Rz(t1) @ Rx(-t2) #1
    elbow = SHOULDER_ORIGIN + R_rover_to_link1 @ ELBOW_IN_LINK1 #2
    R_rover_to_link2 = R_rover_to_link1 @ Rx(t3) #3
    tip_offset_in_link2 = np.array([0.0, -l2, 0.0]) #4
    tip = elbow + R_rover_to_link2 @ tip_offset_in_link2 #5

    return SHOULDER_ORIGIN.copy(), elbow, tip


class ArmFKNode(Node):
    # The three joint names we expect in the /joint_states message
    # Used as a reference — actual lookup is by name via dict in _cb
    JOINT_ORDER = ['base_joint', 'shoulder_joint', 'elbow_joint']

    def __init__(self):
        super().__init__("arm_fk_node")

        self.sub = self.create_subscription(JointState, '/joint_states', self._cb, 10)

        self.get_logger().info('FK node running. Listening on /joint_states...')

        self.get_logger().info(f'L1={L1:.4f}m  L2={L2:.4f}m  alpha1={math.degrees(ALPHA1_STRUCTURAL):.2f}°')

    def _cb(self, msg: JointState):
        """
        Callback — fires every time a /joint_states message is received.

        msg.name     : list of joint name strings  e.g. ['base_joint', 'shoulder_joint', ...]
        msg.position : list of floats in same order e.g. [0.0, 0.785, -1.0]
        """

        # zip() pairs each name with its angle: [('base_joint', 0.0), ('shoulder_joint', 0.785), ...]
        # dict() converts those pairs into a name→angle lookup table
        angles = dict(zip(msg.name, msg.position))

        # Extract the 3 joint angles by name
        try:
            t1 = angles['base_joint']       # base rotation [rad]
            t2 = angles['shoulder_joint']   # shoulder tilt [rad]
            t3 = angles['elbow_joint']      # elbow bend [rad]
        except KeyError:
            self.get_logger().warn(f'Missing joints. Got: {list(angles.keys())}')
            return

        # All 3 angles found — compute and print FK result
        self._print_fk_result(t1, t2, t3)

    def _print_fk_result(self, t1: float, t2: float, t3: float):
        """Runs FK and prints a formatted summary to the terminal."""

        # Run FK — get shoulder, elbow, tip positions in world frame
        shoulder, elbow, tip = fk(t1, t2, t3)

        # Horizontal reach: distance from the arm's vertical (Z) axis to the tip
        # Ignores height — purely in the ground plane
        # r = sqrt(tip_x² + tip_y²)
        r = math.sqrt(tip[0]**2 + tip[1]**2)

        # ── Formatted print block ──────────────────────────────────────────────
        print(f"\n{'─'*55}")

        # Joint angles converted to degrees for readability
        print(f"  Joint angles:  t1={math.degrees(t1):+7.2f}°  "
            f"t2={math.degrees(t2):+7.2f}°  t3={math.degrees(t3):+7.2f}°")

        # Shoulder is always fixed at (0, 0, 0.16) — printed for reference
        print(f"  Shoulder:  ({shoulder[0]:+.4f}, {shoulder[1]:+.4f}, {shoulder[2]:+.4f}) m")

        # Elbow position in world frame — changes with t1 and t2
        print(f"  Elbow:     ({elbow[0]:+.4f}, {elbow[1]:+.4f}, {elbow[2]:+.4f}) m")

        # Tip (end effector) position in world frame — changes with all 3 joints
        print(f"  Tip:       ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f}) m")

        # r : horizontal reach from the arm's Z axis [m]
        # tip[2] : tip height in world frame [m]
        # D : straight-line 3D distance from shoulder joint to tip [m]
        #     D = sqrt(r² + (tip_z - shoulder_z)²)
        #     Useful for verifying the arm is within its reach envelope (should be ≤ L1+L2)
        print(f"  Reach:     Horizontal r={r:.4f} m  Vertical z={tip[2]:.4f} m  "
            f"Distance from shoulder to tip D={math.sqrt(r**2 + (tip[2]-0.16)**2):.4f} m")

        print(f"{'─'*55}")


def main(args=None):
    rclpy.init(args=args)
    myNode = ArmFKNode()
    rclpy.spin(myNode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()