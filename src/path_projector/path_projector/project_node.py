import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
from scipy.spatial.transform import Rotation


class PoseState:
    def __init__(self, x: float = 0, y: float = 0, yaw: float = 0, vx: float = 0, vy: float = 0, omega: float = 0, max_radius: float = 10.0):
        self.pos = np.array([x, y], dtype=np.float64)
        self.yaw = np.float64(yaw)

        self.v = np.array([vx, vy], dtype=np.float64)
        self.omega = np.float64(omega)
        
        self.max_radius = max_radius # for creating new objects

        v_mag: np.float64 = np.sqrt(self.v.dot(self.v)) # velocity magnitude
        radius = v_mag / self.omega
        r_mag: np.float64 = np.abs(radius) # radius magnitude
        self.linear = r_mag > max_radius # set if this is a linear (straight) motion

        if self.linear:
            self.centre = None
        else:
            rvect: np.ndarray = radius * np.array([[0, -1], [1, 0]]).dot(self.v / v_mag) # radius vector (90deg CCW, multiplied by signed radius)
            self.centre = self.pos + rvect
    
    def next(self, dt: float, inplace: bool = True, nomod: bool = False) -> tuple[float, float, float, object]: # x, y, yaw, object containing next state (or this object if inplace=True). if nomod=True, inplace will be forced to True, and this method will not change the object.
        if nomod: inplace = True
        # output = self if inplace else PoseState(self.x, self.y, self.yaw, self.vx, self.vy, self.omega, self.max_radius)

        if self.linear: # straight motion - advance x and y only
            new_pos = self.pos + self.v * dt
            new_v = self.v
            new_yaw = self.yaw # no heading change
        else: # follow circular motion
            d_yaw = self.omega * dt; new_yaw = self.yaw + d_yaw
            
            s = np.sin(d_yaw); c = np.cos(d_yaw) # for rotation matrix construction
            rotmat = np.array([[c, -s], [s, c]]) # rotation matrix by d_yaw
            new_pos = self.centre + rotmat.dot(self.pos - self.centre) # apply rotation by d_yaw
            new_v = rotmat.dot(self.v)
        
        new_x, new_y = new_pos.tolist(); new_yaw = new_yaw.item()
        if inplace:
            if not nomod:
                self.pos = new_pos
                self.v = new_v
            return (new_x, new_y, new_yaw, self)
        else:
            new_vx, new_vy = new_v.tolist()
            return (
                new_x, new_y, new_yaw,
                PoseState(
                    new_x, new_y, new_yaw,
                    new_vx, new_vy, self.omega,
                    self.max_radius
                )
            )

class ProjectNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name + '_path_projector')

        self.dt = self.declare_parameter('dt', 0.1).get_parameter_value().double_value # time step for path projection (in seconds)
        self.steps = self.declare_parameter('steps', 50).get_parameter_value().integer_value # number of projection steps
        self.min_distance = self.declare_parameter('min_distance', 0.1).get_parameter_value().double_value # minimum distance between points (in metres)
        self.max_radius = self.declare_parameter('max_radius', 10.0).get_parameter_value().double_value # maximum radius for circular motion (in metres)

        self.pub = self.create_publisher(Path, 'path', qos.qos_profile_system_default)
    
    def compute_motion(self, x: float, y: float, yaw: float, vx: float, vy: float, omega: float) -> list[tuple[float, float, float]]: # outputs filtered list of (x, y, yaw) poses (including current pose)
        poses = [(0, x, y, yaw)]

        pose = PoseState(x, y, yaw, vx, vy, omega, self.max_radius)
        
        for i in range(1, self.steps + 1):
            _, last_x, last_y, _ = poses[-1]
            new_x, new_y, new_yaw, _ = pose.next(i * self.dt, nomod=True) # compute next step
            pos_diff = ((new_x - last_x)**2 + (new_y - last_y)**2)**0.5 # position difference
            if pos_diff < self.min_distance: # less than difference
                if i < self.steps: continue # try next timestep
                else: poses.pop() # remove last pose (so we can store this in instead)
            poses.append((i, new_x, new_y, new_yaw))
        
        # self.get_logger().info(f'created path with {len(poses)} point(s): {poses[0]} -> {poses[-1]}')
        return poses
    
    def project(self, x: float, y: float, yaw: float, vx: float, vy: float, omega: float, frame_id: str = '', msg_time: Time | None = None):
        poses = self.compute_motion(x, y, yaw, vx, vy, omega)

        msg = Path()
        if msg_time is None: msg_time = self.get_clock().now()
        msg.header.stamp = msg_time.to_msg()
        msg.header.frame_id = frame_id
        for (i, x, y, yaw) in poses:
            p = PoseStamped()
            p.header.stamp = (msg_time + Duration(seconds=i*self.dt)).to_msg()
            p.header.frame_id = frame_id
            p.pose.position.x = x; p.pose.position.y = y
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = Rotation.from_euler('xyz', [0, 0, yaw]).as_quat().tolist()
            msg.poses.append(p)
        
        self.pub.publish(msg)
