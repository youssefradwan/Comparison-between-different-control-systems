"""
Lap Analyzer Node:
Performance evaluation, real-time telemetry, lap timing, CSV logging, and RViz HUD visualization.
"""

import csv
import json
import math
import os

import numpy as np
import rclpy  # noqa: F401
from rclpy.node import Node  # noqa: F401
from nav_msgs.msg import Path, Odometry  # noqa: F401
from std_msgs.msg import String, Float32  # noqa: F401
from geometry_msgs.msg import Point  # noqa: F401
from visualization_msgs.msg import Marker, MarkerArray  # noqa: F401


MIN_WAYPOINT_SPACING = 0.3
CSV_COLUMNS = (
    'controller', 'lap', 'laps_completed', 'lap_time_s',
    'best_lap_time_s', 'top_speed_mps', 'mean_cte_m', 'max_cte_m',
    'rms_cte_m', 'mean_speed_mps', 'max_speed_mps',
    'mean_heading_error_deg', 'max_heading_error_deg',
    'rms_heading_error_deg',
)


class LapAnalyzer(Node):
    def __init__(self):
        super().__init__('lap_analyzer')
        self.get_logger().info('Initializing Lap Analyzer Node...')

        # Parameters
        self.declare_parameter('log_file', '')
        self.declare_parameter('controller_name', 'unspecified')
        self.declare_parameter('max_laps', 0)

        self.log_file = str(self.get_parameter('log_file').value)
        self.controller_name = str(self.get_parameter('controller_name').value)
        self.max_laps = int(self.get_parameter('max_laps').value)

        self.done = False
        self.raw_path_len = 0

        # Subscriptions & Publishers
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.state_sub = self.create_subscription(Odometry, '/state', self.state_callback, 10)

        self.metrics_pub = self.create_publisher(String, '/lap/metrics', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/lap/visualization', 10)

        self.cte_pub = self.create_publisher(Float32, '/telemetry/cte', 10)
        self.speed_pub = self.create_publisher(Float32, '/telemetry/speed', 10)
        self.heading_err_pub = self.create_publisher(Float32, '/telemetry/heading_err_deg', 10)
        self.lap_time_pub = self.create_publisher(Float32, '/telemetry/lap_time', 10)

        self.path_points = []
        self.path_cum_dist = []
        self.track_length = 0.0
        self.path_received = False

        self.last_state_time = None
        self.current_lap_time = 0.0

        self.lap_count = 0
        self.last_s = 0.0
        self.total_distance = 0.0
        self.lap_distance = 0.0
        self.last_xy = None

        self.last_lap_time = None
        self.best_lap_time = None
        self.lap_times = []

        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

        self.global_ctes = []
        self.global_max_speed = 0.0

        self.current_cte = 0.0
        self.current_heading_err = 0.0
        self.current_speed = 0.0
        self.proj_xy = (0.0, 0.0)

        self.timer = self.create_timer(0.1, self.publish_telemetry)

    def path_callback(self, msg: Path):
        """Processes received path and precomputes cumulative distance."""
        if self.path_received and len(msg.poses) == self.raw_path_len:
            return

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

        if len(pts) < 2:
            return

        self.raw_path_len = len(msg.poses)
        self.path_points = pts
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                            pts[i][1] - pts[i - 1][1]))
        self.path_cum_dist = cum
        closing_distance = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
        self.track_length = cum[-1] + closing_distance
        self.path_received = True
        self.get_logger().info(
            f"Lap Analyzer: Loaded path with {len(pts)} waypoints "
            f"({self.raw_path_len} raw), perimeter: {self.track_length:.2f} m"
        )

    def state_callback(self, msg: Odometry):
        """Processes vehicle odometry using delta clock ticks to ensure robust lap timing."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        v = msg.twist.twist.linear.x

        if self.last_state_time is None:
            self.last_state_time = now_sec
            return

        dt = max(now_sec - self.last_state_time, 0.0)
        self.last_state_time = now_sec

        # Accumulate lap time only when car is actually driving
        if v > 0.05:
            self.current_lap_time += dt

        self.current_speed = v
        self.global_max_speed = max(self.global_max_speed, v)

        if self.last_xy is not None:
            step_d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.total_distance += step_d
            self.lap_distance += step_d
        self.last_xy = (x, y)

        if not self.path_received or len(self.path_points) < 2:
            return

        proj_x, proj_y, s, cte, heading_err = self.project_to_path(x, y, yaw)
        self.proj_xy = (proj_x, proj_y)
        self.current_cte = cte
        self.current_heading_err = heading_err

        if v > 0.05:
            abs_cte = abs(cte)
            self.lap_ctes.append(abs_cte)
            self.lap_heading_errors.append(abs(heading_err))
            self.lap_speeds.append(v)
            self.global_ctes.append(abs_cte)

        # Lap completion check: require driving >= 90% track length before crossing finish line
        if self.track_length > 5.0 and v > 0.1 and self.lap_distance > 0.9 * self.track_length:
            if self.last_s > 0.75 * self.track_length and s < 0.25 * self.track_length:
                lap_duration = self.current_lap_time
                self.record_lap_completion(lap_duration)
                self.current_lap_time = 0.0

        self.last_s = s

    def project_to_path(self, x, y, yaw):
        """Finds closest segment and projects (x, y) to compute exact orthogonal CTE."""
        pts = self.path_points
        n = len(pts)

        min_dist_sq = float('inf')
        nearest_idx = 0
        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                nearest_idx = i

        best_dist = float('inf')
        best_proj = (pts[nearest_idx][0], pts[nearest_idx][1])
        best_s = self.path_cum_dist[nearest_idx]
        best_seg_yaw = pts[nearest_idx][2]
        best_signed_cte = 0.0

        candidate_segments = [
            ((nearest_idx - 1) % n, nearest_idx),
            (nearest_idx, (nearest_idx + 1) % n)
        ]
        for prev_i, next_i in candidate_segments:
            x1, y1, yaw1 = pts[prev_i]
            x2, y2, _ = pts[next_i]
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                continue

            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq))
            px = x1 + t * dx
            py = y1 + t * dy
            dist = math.hypot(x - px, y - py)

            if dist < best_dist:
                best_dist = dist
                best_proj = (px, py)
                best_s = self.path_cum_dist[prev_i] + t * math.sqrt(seg_len_sq)
                best_seg_yaw = math.atan2(dy, dx)

                cross = dx * (y - y1) - dy * (x - x1)
                best_signed_cte = math.copysign(dist, cross)

        heading_err = math.atan2(
            math.sin(yaw - best_seg_yaw), math.cos(yaw - best_seg_yaw)
        )

        return best_proj[0], best_proj[1], best_s, best_signed_cte, heading_err

    def record_lap_completion(self, lap_duration):
        """Records finished lap, prints summary, exports CSV, and handles max_laps termination."""
        self.lap_count += 1
        self.last_lap_time = lap_duration
        self.lap_times.append(lap_duration)

        if self.best_lap_time is None or lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration

        ctes_arr = np.array(self.lap_ctes) if self.lap_ctes else np.array([0.0])
        speeds_arr = np.array(self.lap_speeds) if self.lap_speeds else np.array([0.0])

        mean_cte = float(np.mean(ctes_arr))
        max_cte = float(np.max(ctes_arr))
        rms_cte = float(np.sqrt(np.mean(ctes_arr ** 2)))

        mean_speed = float(np.mean(speeds_arr))
        max_speed = float(np.max(speeds_arr))
        heading_errors_deg = np.degrees(
            np.array(self.lap_heading_errors)
            if self.lap_heading_errors else np.array([0.0])
        )
        mean_heading_error = float(np.mean(heading_errors_deg))
        max_heading_error = float(np.max(heading_errors_deg))
        rms_heading_error = float(np.sqrt(np.mean(heading_errors_deg ** 2)))

        summary = (
            f"\n{'='*50}\n"
            f"LAP {self.lap_count} COMPLETED\n"
            f"{'='*50}\n"
            f"Controller:  {self.controller_name}\n"
            f"Lap Time:    {lap_duration:.2f} s\n"
            f"Best Lap:    {self.best_lap_time:.2f} s\n"
            f"Mean CTE:    {mean_cte:.4f} m\n"
            f"Max CTE:     {max_cte:.4f} m\n"
            f"RMS CTE:     {rms_cte:.4f} m\n"
            f"Mean Speed:  {mean_speed:.2f} m/s\n"
            f"Max Speed:   {max_speed:.2f} m/s\n"
            f"Mean Heading Error: {mean_heading_error:.2f} deg\n"
            f"Total Dist:  {self.total_distance:.1f} m\n"
            f"{'='*50}"
        )
        self.get_logger().info(summary)

        # Reset lap distance buffer
        self.lap_distance = 0.0

        # Export metrics to CSV file if parameter is specified
        if self.log_file:
            new_file = (
                not os.path.isfile(self.log_file)
                or os.path.getsize(self.log_file) == 0
            )

            with open(self.log_file, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                if new_file:
                    writer.writerow(CSV_COLUMNS)
                writer.writerow((
                    self.controller_name,
                    self.lap_count,
                    self.lap_count,
                    f'{lap_duration:.3f}',
                    f'{self.best_lap_time:.3f}',
                    f'{self.global_max_speed:.3f}',
                    f'{mean_cte:.4f}',
                    f'{max_cte:.4f}',
                    f'{rms_cte:.4f}',
                    f'{mean_speed:.3f}',
                    f'{max_speed:.3f}',
                    f'{mean_heading_error:.3f}',
                    f'{max_heading_error:.3f}',
                    f'{rms_heading_error:.3f}',
                ))

        self.lap_ctes.clear()
        self.lap_heading_errors.clear()
        self.lap_speeds.clear()

        # Shutdown node if target lap count is reached
        if self.max_laps > 0 and self.lap_count >= self.max_laps:
            self.get_logger().info(
                f"Target max_laps ({self.max_laps}) completed for {self.controller_name}! Shutting down..."
            )
            self.done = True
            rclpy.shutdown()

    def publish_telemetry(self):
        """Periodically publishes numerical telemetry and RViz visual markers at 10 Hz."""
        if self.done:
            return

        self.cte_pub.publish(Float32(data=float(self.current_cte)))
        self.speed_pub.publish(Float32(data=float(self.current_speed)))
        self.heading_err_pub.publish(
            Float32(data=float(math.degrees(self.current_heading_err)))
        )
        self.lap_time_pub.publish(Float32(data=float(self.current_lap_time)))

        ctes_arr = np.array(self.lap_ctes) if self.lap_ctes else np.array([0.0])
        live_rms_cte = float(np.sqrt(np.mean(ctes_arr ** 2)))

        g = np.array(self.global_ctes) if self.global_ctes else np.array([0.0])

        telemetry = {
            "lap": self.lap_count + 1,
            "laps_completed": self.lap_count,
            "current_lap_time": round(self.current_lap_time, 2),
            "last_lap_time": round(self.last_lap_time, 2) if self.last_lap_time else None,
            "best_lap_time": round(self.best_lap_time, 2) if self.best_lap_time else None,
            "speed": round(self.current_speed, 2),
            "top_speed": round(self.global_max_speed, 2),
            "current_cte": round(self.current_cte, 4),
            "rms_cte": round(live_rms_cte, 4),
            "mean_cte_all": round(float(g.mean()), 4),
            "max_cte_all": round(float(g.max()), 4),
            "rms_cte_all": round(float(np.sqrt((g ** 2).mean())), 4),
            "heading_err_deg": round(math.degrees(self.current_heading_err), 2)
        }
        self.metrics_pub.publish(String(data=json.dumps(telemetry)))
        self.publish_rviz_markers(telemetry)

    def publish_rviz_markers(self, telemetry=None):
        """Renders start gate, error whisker, and on-screen HUD text in RViz."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        if self.path_points:
            p0 = self.path_points[0]
            gate = Marker()
            gate.header.frame_id = 'map'
            gate.header.stamp = now
            gate.ns = 'start_gate'
            gate.id = 0
            gate.type = Marker.CYLINDER
            gate.action = Marker.ADD
            gate.pose.position.x = p0[0]
            gate.pose.position.y = p0[1]
            gate.pose.position.z = 0.5

            gate.pose.orientation.z = math.sin(p0[2] / 2.0)
            gate.pose.orientation.w = math.cos(p0[2] / 2.0)

            gate.scale.x = 0.1
            gate.scale.y = 1.2
            gate.scale.z = 1.0
            gate.color.r = 0.1
            gate.color.g = 0.9
            gate.color.b = 0.2
            gate.color.a = 0.7
            ma.markers.append(gate)

        if self.last_xy is not None and self.proj_xy is not None:
            whisker = Marker()
            whisker.header.frame_id = 'map'
            whisker.header.stamp = now
            whisker.ns = 'cte_whisker'
            whisker.id = 1
            whisker.type = Marker.LINE_STRIP
            whisker.action = Marker.ADD
            whisker.scale.x = 0.05

            p_vehicle = Point(x=float(self.last_xy[0]), y=float(self.last_xy[1]), z=0.0)
            p_proj = Point(x=float(self.proj_xy[0]), y=float(self.proj_xy[1]), z=0.0)
            whisker.points = [p_vehicle, p_proj]

            abs_cte = abs(self.current_cte)
            if abs_cte < 0.2:
                whisker.color.r, whisker.color.g, whisker.color.b = 0.0, 1.0, 0.0
            elif abs_cte > 0.5:
                whisker.color.r, whisker.color.g, whisker.color.b = 1.0, 0.0, 0.0
            else:
                whisker.color.r, whisker.color.g, whisker.color.b = 1.0, 1.0, 0.0
            whisker.color.a = 1.0

            ma.markers.append(whisker)

        if telemetry is not None and self.path_points:
            hud = Marker()
            hud.header.frame_id = 'map'
            hud.header.stamp = now
            hud.ns = 'telemetry_hud'
            hud.id = 2
            hud.type = Marker.TEXT_VIEW_FACING
            hud.action = Marker.ADD

            p0 = self.path_points[0]
            hud.pose.position.x = p0[0]
            hud.pose.position.y = p0[1]
            hud.pose.position.z = 5.0

            hud.scale.z = 0.8
            hud.color.r, hud.color.g, hud.color.b, hud.color.a = 1.0, 1.0, 1.0, 1.0

            best = f"{telemetry['best_lap_time']}s" if telemetry['best_lap_time'] else "N/A"
            hud.text = (
                f"LAP {telemetry['lap']}\n"
                f"Time: {telemetry['current_lap_time']:.1f}s | Best: {best}\n"
                f"Speed: {telemetry['speed']:.1f} m/s\n"
                f"CTE: {telemetry['current_cte']:.3f} m\n"
                f"RMS CTE: {telemetry['rms_cte']:.3f} m"
            )

            ma.markers.append(hud)

        self.viz_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    analyzer = LapAnalyzer()
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()