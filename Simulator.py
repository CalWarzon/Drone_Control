import math
import random
import numpy as np


class DroneSimulator:

    def __init__(
        self,
        mass=0.5,
        arm_length=0.1,
        max_thrust_per_motor=5.0,
        inertia=(0.002, 0.002, 0.004),
        dt=0.002,
        wind_force = 1.5,
        wind_torque = .03
    ):

        # Physics
        self.mass = mass
        self.g = 9.81

        self.arm_length = arm_length
        self.max_thrust = max_thrust_per_motor

        self.Ix, self.Iy, self.Iz = inertia

        self.dt = dt

        # Position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Velocity
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Attitude
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Angular velocity
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        # IMU outputs
        self.ax = 0.0
        self.ay = 0.0
        self.az = 1.0

        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        # Motor lag state
        self.motor_FR = 0.0
        self.motor_FL = 0.0
        self.motor_BR = 0.0
        self.motor_BL = 0.0

        # Motor response speed
        # lower = slower motors
        self.motor_response = 0.08

        # Wind state
        self.wind_force = np.zeros(3)
        self.wind_torque = np.zeros(3)

        # Wind configuration
        self.wind_force_strength = wind_force
        self.wind_torque_strength = wind_torque

        # Wind persistence
        # higher = smoother wind
        self.wind_smoothing = 0.995

        # Drag coefficients
        self.linear_drag = 0.25
        self.angular_drag = 0.03

    def rotation_matrix(self):

        cr = math.cos(self.roll)
        sr = math.sin(self.roll)

        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)

        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)

        return np.array([
            [
                cy * cp,
                cy * sp * sr - sy * cr,
                cy * sp * cr + sy * sr
            ],
            [
                sy * cp,
                sy * sp * sr + cy * cr,
                sy * sp * cr - cy * sr
            ],
            [
                -sp,
                cp * sr,
                cp * cr
            ]
        ])

    def update_motor(self, current, target):

        return current + (
            target - current
        ) * self.motor_response

    def update_wind(self):

        rand_force = np.array([
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-0.4, 0.4)
        ])

        rand_torque = np.array([
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        ])

        self.wind_force = (
            self.wind_force * self.wind_smoothing
            +
            rand_force *
            (1 - self.wind_smoothing) *
            self.wind_force_strength
        )

        self.wind_torque = (
            self.wind_torque * self.wind_smoothing
            +
            rand_torque *
            (1 - self.wind_smoothing) *
            self.wind_torque_strength
        )

    def step(self, FR, FL, BR, BL):

        # Clamp inputs
        FR = max(0.0, min(1.0, FR))
        FL = max(0.0, min(1.0, FL))
        BR = max(0.0, min(1.0, BR))
        BL = max(0.0, min(1.0, BL))

        # Apply motor lag
        self.motor_FR = self.update_motor(
            self.motor_FR,
            FR
        )

        self.motor_FL = self.update_motor(
            self.motor_FL,
            FL
        )

        self.motor_BR = self.update_motor(
            self.motor_BR,
            BR
        )

        self.motor_BL = self.update_motor(
            self.motor_BL,
            BL
        )

        # Convert to thrust
        FR_thrust = (
            self.motor_FR *
            self.max_thrust
        )

        FL_thrust = (
            self.motor_FL *
            self.max_thrust
        )

        BR_thrust = (
            self.motor_BR *
            self.max_thrust
        )

        BL_thrust = (
            self.motor_BL *
            self.max_thrust
        )

        # Total thrust
        total_thrust = (
            FR_thrust +
            FL_thrust +
            BR_thrust +
            BL_thrust
        )

        # Torques
        roll_torque = (
            (FL_thrust + BL_thrust)
            -
            (FR_thrust + BR_thrust)
        ) * self.arm_length

        pitch_torque = (
            (BR_thrust + BL_thrust)
            -
            (FR_thrust + FL_thrust)
        ) * self.arm_length

        yaw_torque = (
            (FR_thrust + BL_thrust)
            -
            (FL_thrust + BR_thrust)
        ) * 0.01

        # Update wind
        self.update_wind()

        # Add wind torque
        roll_torque += self.wind_torque[0]
        pitch_torque += self.wind_torque[1]
        yaw_torque += self.wind_torque[2]

        # Angular drag
        roll_torque -= self.p * self.angular_drag
        pitch_torque -= self.q * self.angular_drag
        yaw_torque -= self.r * self.angular_drag

        # Angular acceleration
        p_dot = roll_torque / self.Ix
        q_dot = pitch_torque / self.Iy
        r_dot = yaw_torque / self.Iz

        # Integrate angular velocity
        self.p += p_dot * self.dt
        self.q += q_dot * self.dt
        self.r += r_dot * self.dt

        # Integrate attitude
        self.roll += self.p * self.dt
        self.pitch += self.q * self.dt
        self.yaw += self.r * self.dt

        # Rotation matrix
        R = self.rotation_matrix()

        # Thrust in body frame
        thrust_body = np.array([
            0.0,
            0.0,
            total_thrust
        ])

        # Convert thrust to world frame
        thrust_world = R @ thrust_body

        # Gravity
        gravity = np.array([
            0.0,
            0.0,
            -self.mass * self.g
        ])

        # Aerodynamic drag
        drag = np.array([
            -self.vx * self.linear_drag,
            -self.vy * self.linear_drag,
            -self.vz * self.linear_drag
        ])

        # Net force
        net_force = (
            thrust_world
            +
            gravity
            +
            drag
            +
            self.wind_force
        )

        # Linear acceleration
        accel_world = net_force / self.mass

        # Integrate velocity
        self.vx += accel_world[0] * self.dt
        self.vy += accel_world[1] * self.dt
        self.vz += accel_world[2] * self.dt

        # Integrate position
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt

        # Simulated accelerometer
        accel_body = R.T @ accel_world

        gravity_body = R.T @ np.array([
            0.0,
            0.0,
            self.g
        ])

        sensed_accel = (
            accel_body + gravity_body
        )

        # Convert to g
        self.ax = sensed_accel[0] / self.g
        self.ay = sensed_accel[1] / self.g
        self.az = sensed_accel[2] / self.g

        # Gyro deg/s
        self.gx = math.degrees(self.p)
        self.gy = math.degrees(self.q)
        self.gz = math.degrees(self.r)

        return {

            "ax": self.ax,
            "ay": self.ay,
            "az": self.az,

            "gx": self.gx,
            "gy": self.gy,
            "gz": self.gz,

            "x": self.x,
            "y": self.y,
            "z": self.z,

            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,

            "roll": math.degrees(self.roll),
            "pitch": math.degrees(self.pitch),
            "yaw": math.degrees(self.yaw),

            "motor_FR": self.motor_FR,
            "motor_FL": self.motor_FL,
            "motor_BR": self.motor_BR,
            "motor_BL": self.motor_BL,

            "wind_force": self.wind_force.copy(),
            "wind_torque": self.wind_torque.copy()
        }