import time
import json
import numpy as np


class IMU:
    def __init__(self):
        self.calibration = {
            "accel_bias": np.zeros(3),
            "gyro_bias": np.zeros(3),
            "accel_scale": np.ones(3),
            "gyro_scale": np.ones(3),
            "temp_points": [],  # multi-point temp calibration
            "temp_poly_coeffs": None,
            "rotation_matrix": np.eye(3)
        }

        # Filtering / state
        self.filtered_accel = np.zeros(3)
        self.filtered_gyro = np.zeros(3)
        self.alpha_lp = 0.2  # low-pass filter strength

        # Complementary filter state
        self.angle = np.zeros(3)
        self.last_time = time.time()

    # ---------------------------
    # BASIC UTILITIES
    # ---------------------------
    def ConvertTemp(self, raw_temp):
        return (raw_temp / 340.0) + 36.53

    def LowPass(self, new, old):
        return self.alpha_lp * new + (1 - self.alpha_lp) * old

    # ---------------------------
    # STANDARD + SCALE CALIBRATION
    # ---------------------------
    def CalibrateStandard(self, samples=500):
        print("Keep IMU flat and still...")
        time.sleep(2)

        accel = []
        gyro = []

        for _ in range(samples):
            accel.append(self.read_accel_raw())
            gyro.append(self.read_gyro_raw())
            time.sleep(0.005)

        accel = np.array(accel)
        gyro = np.array(gyro)

        accel_mean = np.mean(accel, axis=0)
        gyro_mean = np.mean(gyro, axis=0)

        # Bias
        self.calibration["accel_bias"] = accel_mean - np.array([0, 0, 16384])
        self.calibration["gyro_bias"] = gyro_mean

        print("Standard calibration complete.")

    def CalibrateScale(self):
        print("Scale calibration (place each axis ±1g).")

        scale = []

        for axis in range(3):
            input(f"Place +{axis} axis up. Press ENTER")
            pos = np.mean([self.read_accel_raw() for _ in range(200)], axis=0)

            input(f"Place -{axis} axis up. Press ENTER")
            neg = np.mean([self.read_accel_raw() for _ in range(200)], axis=0)

            scale_factor = (2 * 16384) / (pos[axis] - neg[axis])
            scale.append(scale_factor)

        self.calibration["accel_scale"] = np.array(scale)
        print("Scale calibration complete.")

    # ---------------------------
    # MULTI-POINT TEMP CALIBRATION
    # ---------------------------
    def CalibrateTemperatureMulti(self, points=4):
        print("Temperature calibration (multi-point)")

        for i in range(points):
            input(f"Set temperature point {i+1}, then press ENTER")

            accel = []
            gyro = []
            temps = []

            for _ in range(200):
                accel.append(self.read_accel_raw())
                gyro.append(self.read_gyro_raw())
                temps.append(self.ConvertTemp(self.read_temp_raw()))
                time.sleep(0.01)

            self.calibration["temp_points"].append({
                "temp": float(np.mean(temps)),
                "accel_bias": np.mean(accel, axis=0).tolist(),
                "gyro_bias": np.mean(gyro, axis=0).tolist()
            })

        self.FitTempPolynomial()

    def FitTempPolynomial(self):
        temps = [p["temp"] for p in self.calibration["temp_points"]]
        gyro_biases = [p["gyro_bias"] for p in self.calibration["temp_points"]]

        coeffs = []
        for axis in range(3):
            axis_bias = [b[axis] for b in gyro_biases]
            coeffs.append(np.polyfit(temps, axis_bias, 2))

        self.calibration["temp_poly_coeffs"] = coeffs
        print("Temperature polynomial fitted.")

    # ---------------------------
    # AXIS MISALIGNMENT
    # ---------------------------
    def SetRotationMatrix(self, R):
        self.calibration["rotation_matrix"] = np.array(R)

    # ---------------------------
    # APPLY FULL CALIBRATION
    # ---------------------------
    def ApplyCalibration(self, ax, ay, az, gx, gy, gz, raw_temp):
        temp = self.ConvertTemp(raw_temp)

        accel = np.array([ax, ay, az], dtype=float)
        gyro = np.array([gx, gy, gz], dtype=float)

        # Bias
        accel -= self.calibration["accel_bias"]
        gyro -= self.calibration["gyro_bias"]

        # Scale
        accel *= self.calibration["accel_scale"]
        gyro *= self.calibration["gyro_scale"]

        # Temperature polynomial compensation (gyro)
        if self.calibration["temp_poly_coeffs"] is not None:
            for i in range(3):
                poly = self.calibration["temp_poly_coeffs"][i]
                gyro[i] -= np.polyval(poly, temp)

        # Axis alignment
        R = self.calibration["rotation_matrix"]
        accel = R @ accel
        gyro = R @ gyro

        # Low-pass filtering
        self.filtered_accel = self.LowPass(accel, self.filtered_accel)
        self.filtered_gyro = self.LowPass(gyro, self.filtered_gyro)

        # Online drift correction (if stable)
        if np.linalg.norm(self.filtered_accel - np.array([0, 0, 16384])) < 500:
            self.calibration["gyro_bias"] = (
                0.999 * self.calibration["gyro_bias"] +
                0.001 * gyro
            )

        return self.filtered_accel, self.filtered_gyro, temp

    # ---------------------------
    # COMPLEMENTARY FILTER
    # ---------------------------
    def UpdateOrientation(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        ax, ay, az = self.filtered_accel
        gx, gy, gz = self.filtered_gyro

        # Convert accel to angles
        accel_angle_x = np.arctan2(ay, az)
        accel_angle_y = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # Integrate gyro
        self.angle[0] += gx * dt
        self.angle[1] += gy * dt

        # Complementary filter
        alpha = 0.98
        self.angle[0] = alpha * self.angle[0] + (1 - alpha) * accel_angle_x
        self.angle[1] = alpha * self.angle[1] + (1 - alpha) * accel_angle_y

        return self.angle

    # ---------------------------
    # SAVE / LOAD
    # ---------------------------
    def SaveCalibration(self, filename="imu_cal.json"):
        data = {
            k: (v.tolist() if isinstance(v, np.ndarray) else v)
            for k, v in self.calibration.items()
        }
        with open(filename, "w") as f:
            json.dump(data, f, indent=4)

    def LoadCalibration(self, filename="imu_cal.json"):
        with open(filename, "r") as f:
            data = json.load(f)

        for k, v in data.items():
            if isinstance(v, list):
                try:
                    self.calibration[k] = np.array(v)
                except:
                    self.calibration[k] = v
            else:
                self.calibration[k] = v
