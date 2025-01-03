import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class LaserScanFixer(Node):
    def __init__(self):
        super().__init__('laserscan_fixer')
        
        self.sub = self.create_subscription(LaserScan, 'scan_raw', self.msg_cb, 1000)
        self.pub = self.create_publisher(LaserScan, 'scan', 1000)

    def msg_cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg() # set correct timestamp
        self.pub.publish(msg)
        self.get_logger().info('new LaserScan message')
    
def main(args=None):
    rclpy.init(args=args)

    fixer = LaserScanFixer()
    rclpy.spin(fixer)

    fixer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
