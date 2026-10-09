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
        # Baseline speed used ONLY when curvature data is unavailable
        base_speed = fallback_speed if fallback_speed is not None else self.default_speed

        # No usable curvature data -> conservative fallback (cruise speed)
        if kappa is None or not math.isfinite(kappa):
            return float(min(base_speed, self.max_speed))

        # Straight road (near-zero curvature): no lateral constraint -> full speed
        if abs(kappa) < 1e-5:
            return float(self.max_speed)

        # Corner: physics limit from the lateral acceleration budget,
        # capped by the requested base speed and the global maximum
        safe_cornering_speed = math.sqrt(self.max_lat_accel / abs(kappa))
        return float(min(safe_cornering_speed, base_speed, self.max_speed))
