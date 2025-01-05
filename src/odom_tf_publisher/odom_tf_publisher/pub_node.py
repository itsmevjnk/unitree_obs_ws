import rclpy
from rclpy.node import Node
from rclpy import qos

from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').get_parameter_value().string_value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').get_parameter_value().string_value
        self.fix_timestamp = self.declare_parameter('fix_timestamp', False).get_parameter_value().bool_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.msg_cb, qos.qos_profile_sensor_data)

    def msg_cb(self, data: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() if self.fix_timestamp else data.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.robot_frame

        t.transform.translation.x = data.pose.pose.position.x
        t.transform.translation.y = data.pose.pose.position.y
        t.transform.translation.z = data.pose.pose.position.z # normally this would be 0
        t.transform.rotation = data.pose.pose.orientation

        self.get_logger().debug(f'sending {self.odom_frame} -> {self.robot_frame} transform')
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
