import rclpy
from rclpy import qos
from rclpy.node import Node

from builtin_interfaces.msg import Time

from datetime import datetime

class FixerNode(Node):
    def __init__(self, topic_name, msg_type):
        super().__init__(topic_name + '_fixer')
        
        self.sub = self.create_subscription(msg_type, topic_name + '_in', self.msg_cb, qos.qos_profile_sensor_data)
        self.pub = self.create_publisher(msg_type, topic_name + '_out', qos.qos_profile_system_default)

        self.get_logger().info(f'fixing timestamp of messages of type {msg_type}')

    def stamp_to_dt(self, stamp: Time):
        return datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1e9)

    def msg_cb(self, msg):
        self.get_logger().debug(f'new message at {self.stamp_to_dt(msg.header.stamp)}')
        msg.header.stamp = self.get_clock().now().to_msg() # set correct timestamp
        self.pub.publish(msg)
