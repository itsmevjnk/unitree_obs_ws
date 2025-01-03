import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import TransformStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.out_map_frame = self.declare_parameter('out_map_frame', 'map').get_parameter_value().string_value
        self.out_robot_frame = self.declare_parameter('out_robot_frame', 'robot').get_parameter_value().string_value
        self.map_frame = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(TransformStamped, 'pose', 1)

        self.timer = self.create_timer(0.05, self.timer_cb) # publish pose with 20Hz frequency
        self.get_logger().info(f'publishing {self.map_frame} -> {self.robot_frame} transform as {self.out_map_frame} -> {self.out_robot_frame}')

    def timer_cb(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'cannot get {self.map_frame} -> {self.robot_frame} tf: {ex}')
            return
        
        # self.get_logger().info(f'got {t.header.frame_id} -> {t.child_frame_id} tf')
        
        t.header.frame_id = self.out_map_frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = self.out_robot_frame

        self.pub.publish(t)

def main():
    rclpy.init()
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
