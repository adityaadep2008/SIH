import rclpy
from rclpy.node import Node
from sih_amr_interfaces.msg import RoutePlan, TrajectoryIntent

from .common import (
    FLEET_STATE_QOS, PROTOCOL_QOS, header, new_session_id, now_seconds,
    stamp_seconds,
)


class ReservationManagerNode(Node):
    """Converts a local WHCA* route into the fleet's expiring trajectory intent."""
    def __init__(self):
        super().__init__('reservation_manager_node')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        configured_dt = self.declare_parameter('time_slot_seconds', 0.0).value
        grid_resolution = self.declare_parameter('grid_resolution_m', 0.5).value
        tracking_speed = self.declare_parameter('path_tracking_speed_mps', 1.0).value
        derived_dt = grid_resolution / max(tracking_speed, 0.01)
        self.dt = derived_dt if configured_dt <= 0.0 else configured_dt
        if configured_dt > 0.0 and abs(configured_dt - derived_dt) > 1e-3:
            raise ValueError(
                f'time_slot_seconds={configured_dt} does not match grid/speed={derived_dt}; '
                'use 0.0 for automatic derivation')
        self.priority = self.declare_parameter('priority', 100).value
        self.session_id, self.sequence, self.current = new_session_id(), 0, None
        self.pub = self.create_publisher(TrajectoryIntent, '/fleet/trajectory_intent', PROTOCOL_QOS)
        self.create_subscription(RoutePlan, 'planned_route', self.on_route, FLEET_STATE_QOS)
        self.create_timer(0.75, self.publish_intent)

    def on_route(self, route):
        # An infeasible replacement invalidates the previous plan; continuing
        # to refresh that old route would create a permanent ghost reservation.
        self.current = route if route.route_feasible else None
    def publish_intent(self):
        if self.current is None:
            return
        if stamp_seconds(self.current.fleet_header.valid_until) < now_seconds(self):
            # The planner stops publishing after completion.  Do not turn its
            # last finite-lived route into an indefinitely renewed intent.
            self.current = None
            return
        self.sequence += 1; msg = TrajectoryIntent()
        msg.fleet_header = header(self, self.robot_id, self.session_id, self.sequence, 1.5)
        route_priority = getattr(self.current, 'priority', self.priority)
        msg.plan_id, msg.t0, msg.dt_seconds, msg.reservations, msg.priority = self.current.plan_id, msg.fleet_header.sent_at, self.dt, self.current.cells, (route_priority if route_priority != 0 else self.priority)
        self.pub.publish(msg)


def main():
    rclpy.init(); node = ReservationManagerNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
