import rclpy
from rclpy.node import Node

from .fixer_node import FixerNode
from sensor_msgs.msg import LaserScan

def main(args=None):
    rclpy.init(args=args)

    fixer = FixerNode('scan', LaserScan)
    rclpy.spin(fixer)

    fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
