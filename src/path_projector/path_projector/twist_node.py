import rclpy
from rclpy import qos
from rclpy.time import Time

from .project_node import ProjectNode

from geometry_msgs.msg import Twist, TwistStamped

import numpy as np
from scipy.spatial.transform import Rotation

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TwistProjectNode(ProjectNode):
    def __init__(self):
        super().__init__('odom')

        self.stamped = self.declare_parameter('stamped', False).get_parameter_value().bool_value
        self.latest_tf = self.declare_parameter('latest_tf', True).get_parameter_value().bool_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').get_parameter_value().string_value

        self.create_subscription(TwistStamped if self.stamped else Twist, 'twist', self.twist_cb, qos.qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def twist_cb(self, data: Twist | TwistStamped):
        stamp = Time()
        if self.stamped and not self.latest_tf:
            stamp = Time.from_msg(data.header.stamp)
        if self.stamped:
            data = data.twist # force to Twist
    
        # get robot pose
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, stamp)
        except Exception as e:
            self.get_logger().warn(f'cannot retrieve {self.map_frame} -> {self.robot_frame} transform, skipping: {e}')
            return
        
        orientation = Rotation.from_quat([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        _, _, yaw = orientation.as_euler('xyz')

        vel = np.array([data.linear.x, data.linear.y, data.linear.z])
        vel = orientation.as_matrix().dot(vel)
        vx, vy, _ = vel.tolist()

        self.project(
            t.transform.translation.x, t.transform.translation.y, yaw,
            vx, vy, data.angular.z,
            self.map_frame,
            stamp
        )

def main(args=None):
    rclpy.init(args=args)

    node = TwistProjectNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
