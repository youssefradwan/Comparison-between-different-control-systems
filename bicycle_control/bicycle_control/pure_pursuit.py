"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math # noqa: F401
import numpy as np # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        ld = self.kv * v + self.l_min
        return float(np.clip(ld, self.l_min, self.l_max))

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Returns (idx, point) at the look-ahead distance, wrapping the closed loop."""
        if not path_points:
            return 0, None

        n = len(path_points)
        nearest = min(
            range(n),
            key=lambda i: math.hypot(
                path_points[i][0] - x, path_points[i][1] - y)
        )

        for k in range(n):
            idx = (nearest + k) % n
            pt = path_points[idx]
            if math.hypot(pt[0] - x, pt[1] - y) >= lookahead:
                return idx, pt

        return nearest, path_points[nearest]

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        if target_pt is None:
            return 0.0

        dx = target_pt[0] - x
        dy = target_pt[1] - y

        target_yaw = math.atan2(dy, dx)
        alpha = target_yaw - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        raw_steer = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        return float(np.clip(raw_steer, -self.max_steer_rad, self.max_steer_rad))
