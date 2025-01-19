import rclpy
from rclpy import qos
from rclpy.time import Time

from .project_node import ProjectNode

from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation

class OdomProjectNode(ProjectNode):
    def __init__(self):
        super().__init__('odom')

        self.create_subscription(Odometry, 'odom', self.odom_cb, qos.qos_profile_sensor_data)
    
    def odom_cb(self, data: Odometry):
        pose = data.pose.pose
        _, _, yaw = Rotation.from_quat([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ]).as_euler('sxyz')
        twist = data.twist.twist
        self.project(
            pose.position.x, pose.position.y, yaw,
            twist.linear.x, twist.linear.y, twist.angular.z,
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
