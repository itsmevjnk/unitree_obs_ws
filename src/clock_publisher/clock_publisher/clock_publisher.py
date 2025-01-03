import rclpy
from rclpy.node import Node
from rclpy import qos

from datetime import datetime

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

from geometry_msgs.msg import TransformStamped

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')

        self.clock_pub = self.create_publisher(Clock, 'clock', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.msg_cb, qos.qos_profile_sensor_data)

    def msg_cb(self, data: LaserScan):
        time_msg = Clock()
        dt = datetime.fromtimestamp(data.header.stamp.sec + data.header.stamp.nanosec / 1e9)
        self.get_logger().info(f'message timestamp: {dt.timestamp()} ({dt})')
        time_msg.clock.sec = data.header.stamp.sec
        time_msg.clock.nanosec = data.header.stamp.nanosec
        self.clock_pub.publish(time_msg)

def main():
    rclpy.init()
    node = ClockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
