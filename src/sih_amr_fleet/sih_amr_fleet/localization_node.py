import rclpy
import math
import pathlib
import yaml
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sih_amr_interfaces.msg import DockProtocol, RobotState

from .algorithms import body_velocity_to_map, map_transform_for_anchor
from .common import FLEET_STATE_QOS, POSE_QOS, PROTOCOL_QOS, header, new_session_id, wrap_angle, yaw_from_quaternion


class LocalizationNode(Node):
    """Normalizes simulator odometry into the fleet's validated RobotState contract."""
    def __init__(self):
        super().__init__('localization_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.odom_origin_x = self.declare_parameter('odom_origin_x', 0.0).value
        self.odom_origin_y = self.declare_parameter('odom_origin_y', 0.0).value
        self.odom_origin_yaw = self.declare_parameter('odom_origin_yaw', 0.0).value
        self.map_file = self.declare_parameter('map_file', '').value
        self.session_id, self.sequence = new_session_id(), 0
        self.dock_anchors = {}
        self.last_raw_odom = None
        self.last_odom_time = None
        self.last_map_pos = None
        if not self.map_file or not pathlib.Path(self.map_file).exists():
            try:
                from ament_index_python.packages import get_package_share_directory
                self.map_file = str(pathlib.Path(get_package_share_directory('sih_amr_fleet')) / 'maps' / 'demo_warehouse.yaml')
            except Exception:
                pass
        if self.map_file and pathlib.Path(self.map_file).exists():
            try:
                for dock_id, spec in (yaml.safe_load(pathlib.Path(self.map_file).read_text()) or {}).get('anchors', {}).items():
                    pose = spec.get('map_pose', [])
                    if len(pose) >= 3:
                        self.dock_anchors[dock_id] = tuple(float(v) for v in pose[:3])
            except Exception as error:
                self.get_logger().error(f'Could not load dock anchors: {error}')
        self.publisher = self.create_publisher(RobotState, '/fleet/robot_state', FLEET_STATE_QOS)
        # Local pose is a continuously refreshed sensor stream.  Its consumers
        # use POSE_QOS, so use that exact contract here rather than relying on
        # DDS' offered/requsted QoS relaxation.  A planner that starts late
        # receives the next odometry sample immediately; it must not act on a
        # stale, retained pose.
        self.local_publisher = self.create_publisher(RobotState, 'state', POSE_QOS)
        self.amcl_publisher = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', POSE_QOS)
        self.create_subscription(Odometry, 'odom', self.on_odom, POSE_QOS)
        self.create_subscription(DockProtocol, '/fleet/dock_protocol', self.on_dock_protocol, PROTOCOL_QOS)

    def on_dock_protocol(self, msg):
        """Only a charging-pad confirmation may move the map origin."""
        if msg.event != DockProtocol.CONFIRMED or msg.fleet_header.robot_id != self.robot_id:
            return
        anchor = self.dock_anchors.get(msg.dock_id)
        if anchor is None:
            self.get_logger().warning(f'Ignoring confirmation for unknown dock {msg.dock_id}')
            return
        if self.last_raw_odom is None:
            self.get_logger().warning('Ignoring dock confirmation before a raw odometry sample')
            return
        local_x, local_y, local_yaw = self.last_raw_odom
        # Solve the odom->map transform so *this raw sample* lands exactly on
        # the dock anchor. No Gazebo world-pose API participates in this reset.
        self.odom_origin_x, self.odom_origin_y, self.odom_origin_yaw = map_transform_for_anchor(
            anchor, (local_x, local_y, local_yaw))
        self.get_logger().info(f'Applied confirmed dock-anchor correction from {msg.dock_id}')

    def on_odom(self, odom):
        self.sequence += 1
        local_x, local_y = odom.pose.pose.position.x, odom.pose.pose.position.y
        self.last_raw_odom = (local_x, local_y, yaw_from_quaternion(odom.pose.pose.orientation))
        cosine, sine = math.cos(self.odom_origin_yaw), math.sin(self.odom_origin_yaw)
        map_x = self.odom_origin_x + cosine * local_x - sine * local_y
        map_y = self.odom_origin_y + sine * local_x + cosine * local_y
        msg = RobotState()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 0.5)
        msg.pose = Pose2D(
            x=map_x,
            y=map_y,
            theta=wrap_angle(self.odom_origin_yaw + yaw_from_quaternion(odom.pose.pose.orientation)))
        # nav_msgs/Odometry expresses twist in child_frame_id (base_link for
        # these AMRs).  Fleet consumers predict peers in the map frame, so a
        # body-forward velocity cannot be copied and mislabeled as map +x.
        body_twist = odom.twist.twist
        raw_speed = math.hypot(body_twist.linear.x, body_twist.linear.y)
        if raw_speed > 0.01:
            map_vx, map_vy = body_velocity_to_map(
                body_twist.linear.x, body_twist.linear.y, msg.pose.theta)
        else:
            now_sec = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
            if now_sec == 0.0:
                now_sec = now_seconds(self)
            if self.last_odom_time is not None and self.last_map_pos is not None and now_sec > self.last_odom_time:
                dt = now_sec - self.last_odom_time
                if dt >= 0.02:
                    map_vx = (map_x - self.last_map_pos[0]) / dt
                    map_vy = (map_y - self.last_map_pos[1]) / dt
                else:
                    map_vx, map_vy = 0.0, 0.0
            else:
                map_vx, map_vy = 0.0, 0.0
            self.last_odom_time = now_sec
            self.last_map_pos = (map_x, map_y)
        msg.twist = Twist()
        msg.twist.linear.x = map_vx
        msg.twist.linear.y = map_vy
        msg.twist.linear.z = body_twist.linear.z
        msg.twist.angular = body_twist.angular
        msg.position_covariance_xy = [odom.pose.covariance[0], odom.pose.covariance[1],
                                      odom.pose.covariance[6], odom.pose.covariance[7]]
        msg.localization_valid = True
        self.publisher.publish(msg)
        self.local_publisher.publish(msg)
        # Gazebo odometry transformed into the warehouse map frame. This keeps
        # AMCL-compatible consumers usable before a physical AMCL integration.
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = odom.header.stamp
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x, pose.pose.pose.position.y = msg.pose.x, msg.pose.y
        pose.pose.pose.orientation.z = math.sin(msg.pose.theta / 2.0)
        pose.pose.pose.orientation.w = math.cos(msg.pose.theta / 2.0)
        pose.pose.covariance[0], pose.pose.covariance[7], pose.pose.covariance[35] = 0.02, 0.02, 0.05
        self.amcl_publisher.publish(pose)


def main():
    rclpy.init(); node = LocalizationNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
