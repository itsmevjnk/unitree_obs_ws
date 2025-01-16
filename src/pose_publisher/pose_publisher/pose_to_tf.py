import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

import numpy as np

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        self.pose_sub = self.create_subscription(TransformStamped, 'pose', self.pose_cb, qos.qos_profile_sensor_data) # pose subscriber
        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_cb(self, data: TransformStamped):
        data.header.stamp = self.get_clock().now().to_msg() # fix timestamp in case weird things happen
        self.get_logger().info(f'sending out {data.header.frame_id} -> {data.child_frame_id} transform to tf2')
        self.tf_broadcaster.sendTransform(data)

def main(args=None):
    rclpy.init(args=args)

    node = PoseToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
