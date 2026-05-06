import numpy as np


class PID:
    def __init__(
        self,
        kp,
        ki,
        kd,
        integrator_limit=1.0,
        output_limit=1.0,
        integral_fade=0.98,
        d_filter_alpha=0.7,
        use_gyro_derivative=False
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Integral state
        self.integral = 0.0
        self.integrator_limit = integrator_limit
        self.integral_fade = integral_fade  # <-- NEW

        # Derivative state
        self.last_error = 0.0
        self.d_filtered = 0.0
        self.d_filter_alpha = d_filter_alpha

        # Output limit
        self.output_limit = output_limit

        # Mode
        self.use_gyro_derivative = use_gyro_derivative

    def GetCoefs(self):
        return self.kp, self.ki, self.kd

    def SetCoefs(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd

    def Update(self, error, dt, gyro_rate=None):
        if dt <= 0:
            return 0.0

        # ---------------------------
        # PROPORTIONAL
        # ---------------------------
        p = self.kp * error

        # ---------------------------
        # INTEGRAL (WITH FADING)
        # ---------------------------
        # Leaky integrator (decays over time)
        self.integral *= self.integral_fade

        self.integral += error * dt

        # Clamp integral
        self.integral = max(
            min(self.integral, self.integrator_limit),
            -self.integrator_limit
        )

        i = self.ki * self.integral

        # ---------------------------
        # DERIVATIVE
        # ---------------------------
        if self.use_gyro_derivative and gyro_rate is not None:
            # Preferred for drones
            d_raw = -gyro_rate
        else:
            d_raw = (error - self.last_error) / dt
            self.last_error = error

        # Low-pass filter derivative
        self.d_filtered = (
            self.d_filter_alpha * d_raw +
            (1 - self.d_filter_alpha) * self.d_filtered
        )

        d = self.kd * self.d_filtered

        # ---------------------------
        # OUTPUT
        # ---------------------------
        output = p + i + d

        # Clamp output
        output = max(min(output, self.output_limit), -self.output_limit)

        return output
