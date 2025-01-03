import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np

class PoseToPCL(Node):
    def __init__(self):
        super().__init__('pose_to_pcl')

        self.num_points = self.declare_parameter('num_points', 100).get_parameter_value().integer_value
        self.x_spread = self.declare_parameter('x_spread', 0.5).get_parameter_value().double_value
        self.y_spread = self.declare_parameter('y_spread', 0.5).get_parameter_value().double_value
        self.ignored_ids = self.declare_parameter('ignored_ids', ['robot']).get_parameter_value().string_array_value
        self.pub_rate = self.declare_parameter('pub_rate', 0.1).get_parameter_value().double_value
        self.msg_expiry = self.declare_parameter('msg_expiry', 1.0).get_parameter_value().double_value
        self.outer_radius = self.declare_parameter('outer_radius', 5.0).get_parameter_value().double_value
        self.outer_points = self.declare_parameter('outer_points', 1000).get_parameter_value().integer_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value

        self.sync_scan = self.declare_parameter('sync_scan', False).get_parameter_value().bool_value

        self.last_poses: dict[str, TransformStamped] = dict() # list of last poses of each robot

        self.pcl_pub = self.create_publisher(PointCloud2, 'pose_cloud', 10) # point cloud of robot positions
        self.pose_sub = self.create_subscription(TransformStamped, 'pose', self.pose_cb, qos.qos_profile_sensor_data) # pose subscriber

        if self.sync_scan:
            self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, qos.qos_profile_sensor_data)
        else:
            self.pub_timer = self.create_timer(self.pub_rate, self.timer_cb)

    def pose_cb(self, data: TransformStamped):
        frame_id = data.child_frame_id
        if frame_id in self.ignored_ids:
            self.get_logger().info(f'ignoring pose of {frame_id}')
            return
        if data.header.frame_id != self.map_frame:
            self.get_logger().error(f'{frame_id} pose is given in the wrong frame {data.header.frame_id} - ignoring')
            return

        self.get_logger().info(f'received pose of {frame_id}')
        self.last_poses[frame_id] = data
    
    def timer_cb(self):
        self.pub_cb(self.get_clock().now().to_msg())
    
    def scan_cb(self, data: LaserScan):
        self.pub_cb(data.header.stamp)

    def pub_cb(self, stamp):
        # create surrounding ring for raytracing
        theta = np.linspace(0, 2 * np.pi, self.outer_points)
        points = np.tile([self.outer_radius], (self.outer_points, 2)) * np.vstack((np.cos(theta), np.sin(theta))).T
        
        delete_ids = [] # frame ids to be deleted

        for frame_id in self.last_poses:
            pose = self.last_poses[frame_id]
            if (self.get_clock().now() - Time.from_msg(pose.header.stamp)).nanoseconds / 1e9 > self.msg_expiry:
                self.get_logger().warn(f'{frame_id} pose has expired, ignoring and removing from dict')
                delete_ids.append(frame_id)
                continue
            
            # create points
            half_x = self.x_spread / 2; half_y = self.y_spread / 2
            points: np.ndarray = np.append(points, (np.tile([pose.transform.translation.x, pose.transform.translation.y], (self.num_points, 1)) + np.random.uniform((-half_x, -half_y), (half_x, half_y), (self.num_points, 2))), 0)

        num_points = points.shape[0]
        points = np.hstack((points, np.tile((0, 100), (num_points, 1)))).tolist()
        self.get_logger().info(f'{num_points} points will be sent out')

        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1)  # Intensity field
        ]

        pcl = pc2.create_cloud(header, fields, points)
        self.pcl_pub.publish(pcl)

        for id in delete_ids:
            del self.last_poses[id]

def main(args=None):
    rclpy.init(args=args)

    node = PoseToPCL()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
