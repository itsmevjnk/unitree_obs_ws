import rclpy
from rclpy import qos
from rclpy.time import Time

from .project_node import ProjectNode

from nav_msgs.msg import Odometry

import numpy as np
from scipy.spatial.transform import Rotation

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class OdomProjectNode(ProjectNode):
    def __init__(self):
        super().__init__('odom')

        self.create_subscription(Odometry, 'odom', self.odom_cb, qos.qos_profile_sensor_data)
        
        self.intrinsic_vel = self.declare_parameter('intrinsic_vel', True).get_parameter_value().bool_value # set to True to force intrinsic velocity (no TF reads, will only use orientation data from odom topic)
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_cb(self, data: Odometry):
        pose = data.pose.pose
        _, _, yaw = Rotation.from_quat([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ]).as_euler('xyz')
        twist = data.twist.twist
        if self.intrinsic_vel or data.header.frame_id != data.child_frame_id: # apply rotation transform
            orientation = None # quaternion
            if not self.intrinsic_vel: # try tf first
                try:
                    t = self.tf_buffer.lookup_transform(data.header.frame_id, data.child_frame_id, Time.from_msg(data.header.stamp))
                    orientation = t.transform.rotation
                except:
                    self.get_logger().warn(f'cannot retrieve transform, relying on odom-published orientation data')
            if orientation is None: # try odom
                orientation = data.pose.pose.orientation
            
            vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            vel = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix().dot(vel)
            vx, vy, _ = vel.tolist()
        else:
            vx = twist.linear.x
            vy = twist.linear.y
        self.project(
            pose.position.x, pose.position.y, yaw,
            vx, vy, twist.angular.z,
            data.header.frame_id,
            Time.from_msg(data.header.stamp)
        )

def main(args=None):
    rclpy.init(args=args)

    node = OdomProjectNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
