#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class IIWA7JointController(Node):
    def __init__(self):
        super().__init__('iiwa7_joint_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        time.sleep(1)  # Allow some time for initialization

    def move_joints(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = [
            'A1', 'A2', 'A3',
            'A4', 'A5', 'A6', 'A7'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.5, 0.5, -1.0, 1.0, -0.5, 0.0]  # Example joint positions
        point.time_from_start.sec = 2  # Move over 2 seconds

        trajectory_msg.points.append(point)

        self.publisher_.publish(trajectory_msg)
        self.get_logger().info('Published joint trajectory command!')


def main(args=None):
    rclpy.init(args=args)
    controller = IIWA7JointController()
    controller.move_joints()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
