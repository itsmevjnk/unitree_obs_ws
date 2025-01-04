import rclpy
from rclpy.node import Node

from .fixer_node import FixerNode
from nav_msgs.msg import Odometry

def main(args=None):
    rclpy.init(args=args)

    fixer = FixerNode('odom', Odometry)
    rclpy.spin(fixer)

    fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
