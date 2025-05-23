#!/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose2D, 'py_publish_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose2D()
        msg.x = 20.0
        msg.y = 30.0
        msg.theta = 3.14
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ({msg.x}, {msg.y}, {msg.theta},)')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()