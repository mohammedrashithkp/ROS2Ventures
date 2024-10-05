#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircle(Node):
    def __init__(self):
        super().__init__('DrawCircleNode')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.send_velocity)
        self.get_logger().info('Draw Circle Node Started')

    def send_velocity(self):
        msg = Twist()
        msg.linear.x = 0.6
        msg.angular.z = 0.5
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()