"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        self.max_throttle = float(max_throttle)
        self.max_brake = float(max_brake)
        self.integral_limit = float(integral_limit)

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-max_brake, max_throttle]."""
        # If target velocity is zero or car is stopping, cut power immediately
        if abs(target_vel) < 1e-3:
            self.reset()
            # If still moving forward, apply gentle brake to stop
            if current_vel > 0.1:
                return -0.5
            return 0.0

        # 1. Compute velocity error
        error = float(target_vel - current_vel)

        # 2. Proportional term
        p_term = self.kp * error

        # 3. Integral accumulation with anti-windup clamping
        self.integral += error * self.dt
        self.integral = float(
            np.clip(self.integral, -self.integral_limit, self.integral_limit))
        i_term = self.ki * self.integral

        # 4. Derivative term (prevent kick on first step)
        if self.first_run:
            d_term = 0.0
            self.first_run = False
        elif self.dt > 0.0:
            derivative = (error - self.prev_error) / self.dt
            d_term = self.kd * derivative
        else:
            d_term = 0.0

        # 5. Total output
        raw_output = p_term + i_term + d_term

        # 6. Actuator clamping [-max_brake, max_throttle]
        output = float(np.clip(raw_output, -self.max_brake, self.max_throttle))

        # 7. Update memory
        self.prev_error = error

        return output

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True
