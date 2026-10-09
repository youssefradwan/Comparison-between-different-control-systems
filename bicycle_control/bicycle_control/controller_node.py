"""
Integrated Two-Tier Autonomous Vehicle Controller Node.
Coordinates target speed profiling, longitudinal ESC regulation, and lateral path tracking.
"""

import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32

from bicycle_control.longitudinal_pid import PIDLongitudinalController
from bicycle_control.velocity_profiler import VelocityProfiler
from bicycle_control.lateral_pid import LateralPIDController
from bicycle_control.pure_pursuit import PurePursuitController
from bicycle_control.mpc import KinematicBicycleMPC

MIN_WAYPOINT_SPACING = 0.3   # m, waypoints closer than this are dropped
MIN_CURVATURE_DS = 0.2        # m, floor on the chord used for curvature
CURVATURE_HALF_WINDOW = 2    # curvature measured between idx-2 and idx+2


def wrap_angle(a):
    """Wraps an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class ControllerNode(Node):
    """ROS 2 Node coordinating Two-Tier Autonomous Vehicle Control."""

    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Initializing Two-Tier Autonomous Vehicle Controller...')

        # Parameters
        self.declare_parameter('control_mode', 'pure_pursuit')
        self.declare_parameter('target_speed', 4.0)          # m/s base speed
        self.declare_parameter('velocity_mode', 'curvature')  # 'curvature', 'constant'
        self.declare_parameter('wheelbase', 1.25)
        self.declare_parameter('min_speed', 1.5)             # m/s floor for profiler
        self.declare_parameter('debug_log', False)           # Disabled by default to prevent I/O lag

        self.control_mode = str(self.get_parameter('control_mode').value).lower()
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.velocity_mode = str(self.get_parameter('velocity_mode').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.min_speed = min(float(self.get_parameter('min_speed').value), self.target_speed)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        # Initialize Controllers
        self.pid_longitudinal = PIDLongitudinalController(kp=1.0, ki=0.2, kd=0.05, dt=0.1)
        self.profiler = VelocityProfiler(default_speed=self.target_speed, max_speed=7.5)
        self.lateral_pid = LateralPIDController(kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1)
        self.pure_pursuit = PurePursuitController(
            wheelbase=self.wheelbase, kv=0.25, l_min=0.8, l_max=2.5
        )
        
        # MPC Controller initialized with horizon N=10
        self.mpc = KinematicBicycleMPC(wheelbase=self.wheelbase, dt=0.1, horizon=10)

        # Publishers (10 Hz rate)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.state_sub = self.create_subscription(
            Odometry, '/state', self.state_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10)

        # State storage
        self.current_state = None   # (x, y, yaw, v)
        self.path_points = []       # [(x, y, yaw)], filtered
        self.raw_path_len = 0
        self.current_steer = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Vehicle Controller Active in mode: {self.control_mode.upper()}')

    # ------------------------------------------------------------------ Callbacks
    def state_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x
        self.current_state = (x, y, yaw, v)

    def path_callback(self, msg: Path):
        """Stores the path, dropping clustered waypoints and the closing duplicate."""
        if len(msg.poses) == self.raw_path_len:
            return  # Path unchanged, skip rebuild
        self.raw_path_len = len(msg.poses)

        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            if pts and math.hypot(x - pts[-1][0], y - pts[-1][1]) < MIN_WAYPOINT_SPACING:
                continue
            yaw = 2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)
            pts.append((x, y, yaw))

        while len(pts) > 2 and math.hypot(pts[-1][0] - pts[0][0],
                                         pts[-1][1] - pts[0][1]) < MIN_WAYPOINT_SPACING:
            pts.pop()

        self.path_points = pts
        self.get_logger().info(
            f'Path loaded: {self.raw_path_len} raw -> {len(pts)} filtered waypoints')

    # ------------------------------------------------------------------ Path Helpers
    def nearest_index(self, x, y):
        """Index of the waypoint closest to (x, y)."""
        best_d = float('inf')
        best_i = 0
        for i, p in enumerate(self.path_points):
            d = (p[0] - x) ** 2 + (p[1] - y) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def curvature_at(self, idx):
        """Signed curvature from heading change across +-window of waypoints."""
        pts = self.path_points
        n = len(pts)
        a = pts[(idx - CURVATURE_HALF_WINDOW) % n]
        b = pts[(idx + CURVATURE_HALF_WINDOW) % n]
        dyaw = wrap_angle(b[2] - a[2])
        ds = max(math.hypot(b[0] - a[0], b[1] - a[1]), MIN_CURVATURE_DS)
        return dyaw / ds

    def compute_track_errors(self, x, y, yaw, nearest_idx):
        """Orthogonal CTE (positive = left of path), heading error, and curvature."""
        pts = self.path_points
        n = len(pts)

        best_dist = float('inf')
        cte = 0.0
        path_yaw = pts[nearest_idx][2]
        for i1, i2 in (((nearest_idx - 1) % n, nearest_idx),
                       (nearest_idx, (nearest_idx + 1) % n)):
            x1, y1, _ = pts[i1]
            dx = pts[i2][0] - x1
            dy = pts[i2][1] - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                continue
            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq))
            dist = math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))
            if dist < best_dist:
                best_dist = dist
                cross = dx * (y - y1) - dy * (x - x1)
                cte = math.copysign(dist, cross)
                path_yaw = math.atan2(dy, dx)

        heading_err = wrap_angle(yaw - path_yaw)
        kappa = self.curvature_at(nearest_idx)
        return cte, heading_err, kappa

    def speed_target(self, kappa):
        """Curvature-limited target speed with a minimum-speed floor."""
        if self.velocity_mode != 'curvature':
            return self.target_speed
        v_t = self.profiler.compute_target_speed(
            kappa, fallback_speed=self.target_speed)
        return max(v_t, self.min_speed)

    # ------------------------------------------------------------------ Control Loop
    def control_loop(self):
        """Executes selected controller at 10 Hz."""
        if self.current_state is None or len(self.path_points) < 2:
            return

        t_start = time.perf_counter()
        x, y, yaw, v = self.current_state
        nearest_idx = self.nearest_index(x, y)
        target_v = self.target_speed

        if self.control_mode == 'lateral_pid':
            # Mode A: Lateral PID Benchmark
            cte, heading_err, kappa = self.compute_track_errors(x, y, yaw, nearest_idx)
            steer_rad = self.lateral_pid.compute_steering(cte, heading_err)
            target_v = self.speed_target(kappa)
            throttle_cmd = self.pid_longitudinal.compute(target_v, v)

        elif self.control_mode == 'mpc':
            # Mode C: Kinematic Bicycle MPC Benchmark
            ref_traj = self.build_mpc_reference(nearest_idx, v, horizon=10)
            steer_rad, throttle_cmd = self.mpc.solve(
                [x, y, yaw, v], ref_traj, current_steer=self.current_steer)

        else:
            # Mode B: Geometric Pure Pursuit Benchmark (Default)
            lookahead = self.pure_pursuit.compute_lookahead(v)
            tgt_idx, tgt_pt = self.pure_pursuit.find_target_waypoint(
                x, y, self.path_points, lookahead)
            steer_rad = self.pure_pursuit.compute_steering(
                x, y, yaw, tgt_pt, lookahead)
            target_v = self.speed_target(self.curvature_at(tgt_idx))
            throttle_cmd = self.pid_longitudinal.compute(target_v, v)

        self.current_steer = float(steer_rad)

        # Publish commands
        s_msg = Float32()
        s_msg.data = float(steer_rad)
        self.steer_pub.publish(s_msg)

        t_msg = Float32()
        t_msg.data = float(throttle_cmd)
        self.throttle_pub.publish(t_msg)

        loop_ms = (time.perf_counter() - t_start) * 1e3
        if loop_ms > 80.0:
            self.get_logger().warn(
                f'Control step took {loop_ms:.0f} ms (budget 100 ms)',
                throttle_duration_sec=1.0)

    def get_waypoint_at_distance(self, start_idx, distance_ahead):
        """Walks forward along path by distance_ahead and interpolates reference pose."""
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return pts[0] if pts else (0.0, 0.0, 0.0)

        cur_idx = start_idx
        d_acc = 0.0
        while d_acc < distance_ahead:
            next_idx = (cur_idx + 1) % n
            seg = math.hypot(
                pts[next_idx][0] - pts[cur_idx][0],
                pts[next_idx][1] - pts[cur_idx][1])
            if d_acc + seg >= distance_ahead:
                frac = (distance_ahead - d_acc) / max(seg, 1e-4)
                x = pts[cur_idx][0] + frac * (pts[next_idx][0] - pts[cur_idx][0])
                y = pts[cur_idx][1] + frac * (pts[next_idx][1] - pts[cur_idx][1])
                return (x, y, pts[cur_idx][2])
            d_acc += seg
            cur_idx = next_idx
            if cur_idx == start_idx:
                break
        return pts[cur_idx]

    def build_mpc_reference(self, nearest_idx, v=4.0, horizon=10):
        """Generates distance-based future reference poses along path for MPC."""
        if len(self.path_points) < 2:
            return []
        speed = max(float(v), 1.5)  # Enforce minimum trajectory projection speed
        dt = 0.1
        ref = []
        for k in range(horizon):
            px, py, pyaw = self.get_waypoint_at_distance(
                nearest_idx, (k + 1) * speed * dt)
            ref.append([px, py, pyaw, self.target_speed])
        return ref


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()