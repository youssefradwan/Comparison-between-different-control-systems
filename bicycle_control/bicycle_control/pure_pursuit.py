"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


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
        # Calculate dynamic lookahead based on speed
        ld = self.kv * v + self.l_min
        
        # Clamp to ensure we don't look too close or too far
        return float(np.clip(ld, self.l_min, self.l_max))

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        if not path_points:
            return None

        # 1. Find the nearest waypoint index to the vehicle
        min_dist = float('inf')
        nearest_idx = 0
        for i, pt in enumerate(path_points):
            dist = math.hypot(pt[0] - x, pt[1] - y)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        # 2. Walk forward from the nearest waypoint to find the lookahead target
        target_pt = path_points[-1]  # Fallback to the final point
        for i in range(nearest_idx, len(path_points)):
            pt = path_points[i]
            dist = math.hypot(pt[0] - x, pt[1] - y)
            if dist >= lookahead:
                target_pt = pt
                break
                
        return target_pt

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        if target_pt is None:
            return 0.0
            
        # 1. Global coordinate translation to the target
        dx = target_pt[0] - x
        dy = target_pt[1] - y
        
        # 2. Compute the absolute angle to the target waypoint
        target_yaw = math.atan2(dy, dx)
        
        # 3. Transform to vehicle local frame to find lookahead heading error (alpha)
        alpha = target_yaw - yaw
        
        # Wrap alpha to [-pi, pi] to handle branch cuts
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        
        # 4. Pure Pursuit Arc Law: delta = atan(2 * L * sin(alpha) / Ld)
        raw_steer = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        
        # 5. Actuator clamping
        return float(np.clip(raw_steer, -self.max_steer_rad, self.max_steer_rad))
