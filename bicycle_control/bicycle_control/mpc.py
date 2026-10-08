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
                 max_accel=None, max_brake=None):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 30.0
        self.w_long = 1.0
        self.w_yaw = 10.0
        self.w_v = 1.0
        self.w_steer = 0.2
        self.w_dsteer = 6.0
        self.w_accel = 0.1

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        # 1. Horizon & Bounds Setup
        N_eff = min(self.N, len(ref_trajectory))
        if N_eff < 2:
            return 0.0, 0.0
            
        # Variables: [delta_0, a_0, delta_1, a_1, ... delta_N-1, a_N-1]
        bounds = [(-self.max_steer_rad, self.max_steer_rad), (-self.k_a, self.k_a)] * N_eff
        
        # 2. Objective Function
        def objective(u):
            cost = 0.0
            x, y, yaw, v = x0
            prev_delta = current_steer
            
            for k in range(N_eff):
                delta_k = u[2 * k]
                a_k = u[2 * k + 1]
                
                # a. Forward simulate Extended Kinematic Bicycle
                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
                yaw += (v / self.L) * math.tan(delta_k) * self.dt
                v += a_k * self.dt
                
                # b. Path-aligned Frenet frame projection
                x_ref, y_ref, yaw_ref, v_ref = ref_trajectory[k]
                dx = x - x_ref
                dy = y - y_ref
                
                cte = -dx * math.sin(yaw_ref) + dy * math.cos(yaw_ref)
                e_long = dx * math.cos(yaw_ref) + dy * math.sin(yaw_ref)
                
                e_yaw = yaw - yaw_ref
                e_yaw = math.atan2(math.sin(e_yaw), math.cos(e_yaw))
                
                e_v = v - v_ref
                d_steer = (delta_k - prev_delta) / self.dt
                
                # c. Accumulate weighted quadratic costs
                cost += self.w_lat * (cte ** 2)
                cost += self.w_long * (e_long ** 2)
                cost += self.w_yaw * (e_yaw ** 2)
                cost += self.w_v * (e_v ** 2)
                cost += self.w_steer * (delta_k ** 2)
                cost += self.w_dsteer * (d_steer ** 2)
                cost += self.w_accel * (a_k ** 2)
                
                # d. Update memory for slew rate penalty
                prev_delta = delta_k
                
            return cost
            
        # 3. Warm-Start Initialization
        u_init = np.zeros(2 * N_eff)
        if len(self.last_u) >= 2 * N_eff:
            # Shift the previous solution left by one timestep (2 parameters: steer & accel)
            u_init[:-2] = self.last_u[2:2 * N_eff]
            u_init[-2:] = self.last_u[2 * N_eff - 2: 2 * N_eff]
            
        # 4. Numerical Optimization & Control Extraction
        res = minimize(
            objective, 
            u_init, 
            bounds=bounds, 
            method='SLSQP', 
            options={'maxiter': 25, 'ftol': 1e-3}
        )
        
        # Save optimal solution for the next iteration's warm start
        self.last_u = np.zeros(2 * self.N)
        self.last_u[:2 * N_eff] = res.x
        
        # Extract the first optimal control actions
        delta_cmd = float(res.x[0])
        accel_cmd = float(res.x[1])
        
        # Map acceleration to normalized throttle [-1.0, 1.0]
        throttle_cmd = float(np.clip(accel_cmd / self.k_a, -1.0, 1.0))
        
        return delta_cmd, throttle_cmd
