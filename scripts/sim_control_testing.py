#! /usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from builtin_interfaces.msg import Duration
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# class armController(Node):
#     def __init__(self):
#         super().__init__("armController")
        
#         self.declare_parameter("pos_j1", 0.0)
#         self.declare_parameter("pos_j2", 0.5)
#         self.declare_parameter("pos_j3", -0.5)
#         self.declare_parameter("joint1", 'base_joint')
#         self.declare_parameter("joint2", 'shoulder_joint')
#         self.declare_parameter("joint3", 'elbow_joint')
        
#         self.pub = self.create_publisher(
#             JointTrajectory,
#             "/arm_controller/joint_trajectory",
#             10
#         )

#         self.get_logger().info("Arm Controller Node Started")
#         self.pub_target()

#     def pub_target(self):
#         pos_j1 = self.get_parameter("pos_j1").value
#         pos_j2 = self.get_parameter("pos_j2").value
#         pos_j3 = self.get_parameter("pos_j3").value
#         joint1 = self.get_parameter("joint1").value
#         joint2 = self.get_parameter("joint2").value
#         joint3 = self.get_parameter("joint3").value
#         msg = JointTrajectory()
#         point = JointTrajectoryPoint()
#         time = Duration()
#         time.sec = 2
#         point.positions = [pos_j1, pos_j2, pos_j3]
#         point.time_from_start = time
#         msg.points = [point]
#         msg.joint_names = [joint1, joint2, joint3]

#         self.pub.publish(msg)
#         self.get_logger().warn("Published Joint Trajectory")


# def main(args=None):
#     rclpy.init(args=args)
#     arm_node = armController()
#     # rclpy.spin(arm_node)
#     arm_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

"""
Simple script to test arm control
Moves joints through predefined positions
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class ArmController(Node):
    
    def __init__(self):
        super().__init__('arm_controller')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')
        
        # Joint names
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint']
    
    def send_goal(self, positions, duration_sec=3.0):
        """
        Send a goal to move joints to specified positions
        
        Args:
            positions: List of joint positions [base, shoulder, elbow] in radians
            duration_sec: Time to reach the goal
        """
        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        # Send goal
        self.get_logger().info(f'Sending goal: {positions}')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Result: {result.error_code}')
        return True
    
    def run_test_sequence(self):
        """Run a test sequence of movements"""
        
        # Home position
        self.get_logger().info('Moving to HOME position')
        self.send_goal([0.0, 0.0, 0.0], duration_sec=3.0)
        time.sleep(3.5)
        
        # Position 1: Base rotation
        self.get_logger().info('Rotating base')
        self.send_goal([1.57, 0.0, 0.0], duration_sec=3.0)
        time.sleep(3.5)
        
        # Position 2: Shoulder tilt
        self.get_logger().info('Tilting shoulder')
        self.send_goal([1.57, 0.785, 0.0], duration_sec=3.0)
        time.sleep(3.5)
        
        # Position 3: Elbow bend
        self.get_logger().info('Bending elbow')
        self.send_goal([1.57, 0.785, -1.0], duration_sec=3.0)
        time.sleep(3.5)
        
        # Return to home
        self.get_logger().info('Returning to HOME')
        self.send_goal([0.0, 0.0, 0.0], duration_sec=4.0)
        time.sleep(4.5)
        
        self.get_logger().info('Test sequence complete!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = ArmController()
    
    try:
        controller.run_test_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()