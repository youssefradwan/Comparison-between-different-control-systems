"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math  # noqa: F401
import numpy as np  # noqa: F401
from scipy.optimize import minimize  # noqa: F401


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None, c_drag=0.005, c_roll=0.05):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)
        self.c_drag = c_drag
        self.c_roll = c_roll

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 30.0
        self.w_long = 1.0
        self.w_yaw = 10.0
        self.w_v = 1.0
        self.w_steer = 0.2
        self.w_dsteer = 6.0
        self.w_accel = 0.1

        self.last_u = np.zeros(2 * self.N)

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N."""
        N_eff = min(self.N, len(ref_trajectory))
        if N_eff < 2:
            return 0.0, 0.0

        bounds = [(-self.max_steer_rad, self.max_steer_rad),
                  (-self.k_a, self.k_a)] * N_eff

        def objective(u):
            cost = 0.0
            x, y, yaw, v = x0
            prev_delta = current_steer

            for k in range(N_eff):
                delta_k = u[2 * k]
                a_k = u[2 * k + 1]

                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
                yaw += (v / self.L) * math.tan(delta_k) * self.dt

                # Match simulation resistance dynamics exactly
                a_resistance = self.c_drag * \
                    (v ** 2) + self.c_roll * v if v > 0 else 0.0
                v += (a_k - a_resistance) * self.dt

                x_ref, y_ref, yaw_ref, v_ref = ref_trajectory[k]
                dx = x - x_ref
                dy = y - y_ref

                cte = -dx * math.sin(yaw_ref) + dy * math.cos(yaw_ref)
                e_long = dx * math.cos(yaw_ref) + dy * math.sin(yaw_ref)

                e_yaw = yaw - yaw_ref
                e_yaw = math.atan2(math.sin(e_yaw), math.cos(e_yaw))

                e_v = v - v_ref
                d_steer = (delta_k - prev_delta) / self.dt

                cost += self.w_lat * (cte ** 2)
                cost += self.w_long * (e_long ** 2)
                cost += self.w_yaw * (e_yaw ** 2)
                cost += self.w_v * (e_v ** 2)
                cost += self.w_steer * (delta_k ** 2)
                cost += self.w_dsteer * (d_steer ** 2)
                cost += self.w_accel * (a_k ** 2)

                prev_delta = delta_k

            return cost

        u_init = np.zeros(2 * N_eff)
        if len(self.last_u) >= 2 * N_eff:
            u_init[:-2] = self.last_u[2:2 * N_eff]
            u_init[-2:] = self.last_u[2 * N_eff - 2: 2 * N_eff]

        res = minimize(
            objective,
            u_init,
            bounds=bounds,
            method='SLSQP',
            options={'maxiter': 25, 'ftol': 1e-3}
        )

        self.last_u = np.zeros(2 * self.N)
        self.last_u[:2 * N_eff] = res.x

        delta_cmd = float(res.x[0])
        accel_cmd = float(res.x[1])
        throttle_cmd = float(np.clip(accel_cmd / self.k_a, -1.0, 1.0))

        return delta_cmd, throttle_cmd
