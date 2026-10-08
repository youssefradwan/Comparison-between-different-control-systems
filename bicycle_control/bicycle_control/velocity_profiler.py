"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math  # noqa: F401


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculates curvature-limited velocity: v_max = sqrt(a_lat_max / |kappa|)."""
        
        # Determine our baseline target speed if no curvature constraint existed
        base_speed = fallback_speed if fallback_speed is not None else self.default_speed
        
        # Protect against division by zero on straight road segments
        if abs(kappa) < 1e-5:
            return float(min(base_speed, self.max_speed))
            
        # Calculate the physics-based cornering limit
        safe_cornering_speed = math.sqrt(self.max_lat_accel / abs(kappa))
        
        # Output the most restrictive bound: the cornering limit, the requested base speed, or the global max
        return float(min(safe_cornering_speed, base_speed, self.max_speed))
