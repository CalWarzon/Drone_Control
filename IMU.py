import time
import json
import numpy as np
from mpu6050 import mpu6050
from ahrs.filters import Mahony


class IMU:
    #accel: g
    #gyro: deg/sec
    #Orientation: deg
    def __init__(self, connection = 0x68):
        self.calibration = {
            "accel_bias": np.zeros(3),
            "gyro_bias": np.zeros(3),
            "accel_scale": np.ones(3),
            "gyro_scale": np.ones(3),
            "temp_points": [],  # multi-point temp calibration
            "temp_poly_coeffs_gyro": None,
            "temp_poly_coeffs_accel": None,
            "rotation_matrix": np.eye(3)
        }

        # Filtering / state
        self.filtered_accel = np.zeros(3)
        self.filtered_gyro = np.zeros(3)
        self.alpha_lp = 0.35  # low-pass filter strength

        # Online Drift Correction
        self.alpha_ODC = .001
        self.accel_cutoff = .05
        self.gyro_cutoff = 5

        # Mahony Fliter
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.ahrs = Mahony(
            k_P=1.0,
            k_I=0.3
        )

        # Complementary filter state
        self.angle = np.zeros(3)
        self.last_time = time.time()

        #Scaleing
        self.accel_range = 2      # g
        self.gyro_range = 250     # deg/sec
        self.accel_lsb = 16384
        self.gyro_lsb = 131.0
        self.accel_regs = 0x00
        self.gyro_regs = 0x00

        #Create MPU-6050 Connection
        self.IMUInput = mpu6050.mpu6050(connection)

    # ---------------------------
    # BASIC UTILITIES
    # ---------------------------
    def ConvertTemp(self, raw_temp):
        return (raw_temp / 340.0) + 36.53

    def LowPass(self, new, old):
        return self.alpha_lp * new + (1 - self.alpha_lp) * old

    def QuaternionToEuler(self, q):
        w, x, y, z = q

        roll = np.arctan2(
            2*(w*x + y*z),
            1 - 2*(x*x + y*y)
        )

        val = 2*(w*y - z*x)
        val = np.clip(val, -1.0, 1.0)
        pitch = np.arcsin(val)

        yaw = np.arctan2(
            2*(w*z + x*y),
            1 - 2*(y*y + z*z)
        )

        return np.degrees([roll, pitch, yaw])

    def SetSensorRanges(self, accel_range=2, gyro_range=250):
        accel_scales = {
        2: 16384,
        4: 8192,
        8: 4096,
        16: 2048
    }

        gyro_scales = {
        250: 131.0,
        500: 65.5,
        1000: 32.8,
        2000: 16.4
    }

        if accel_range not in accel_scales:
            raise ValueError("Invalid accel range")

        if gyro_range not in gyro_scales:
            raise ValueError("Invalid gyro range")

        self.accel_range = accel_range
        self.gyro_range = gyro_range

        self.accel_lsb = accel_scales[accel_range]
        self.gyro_lsb = gyro_scales[gyro_range]

        accel_registers = {
        2: 0x00,
        4: 0x08,
        8: 0x10,
        16: 0x18 
        }
        gyro_registers = {
        250: 0x00,
        500: 0x08,
        1000: 0x10,
        2000: 0x18
        }
        
        self.accel_regs = accel_registers[accel_range]
        self.gyro_regs = gyro_registers[gyro_range]

        self.SetAccelRange(self.accel_regs)
        self.SetGyroRange(self.gyro_regs)

        print(
            f"Accel: ±{accel_range}g ({self.accel_lsb} LSB/g), "
            f"Gyro: ±{gyro_range}°/s ({self.gyro_lsb} LSB/deg/s)"
        )

    #----------------------------
    # RAW MPU-6050 CONNECTION
    #----------------------------
    def GetAllData(self):
        return self.IMUInput.get_all_data()

    def GetAccelData(self):
        return self.IMUInput.get_accel_data()

    def GetGyroData(self):
        return self.IMUInput.get_gyro_data()

    def GetTempData(self):
        return self.IMUInput.get_temp()

    def GetRawI2C(self):
        return self.IMUInput.read_i2c_word()
    
    def SetAccelRange(self, range):
        self.IMUInput.set_accel_range(range)

    def GetAccelRange(self):
        return self.IMUInput.read_accel_range()

    def SetGyroRange(self, range):
        self.IMUInput.set_gyro_range(range)

    def GetGyroRange(self):
        return self.IMUInput.read_gyro_range()

    # ---------------------------
    # STANDARD + SCALE CALIBRATION
    # ---------------------------
    def CalibrateStandard(self, samples=500):
        print("Keep IMU flat and still...")
        time.sleep(2)

        accel = []
        gyro = []

        for _ in range(samples):
            accel.append(self.GetAccelData())
            gyro.append(self.GetGyroData())
            time.sleep(0.005)

        accel = np.array(accel)
        gyro = np.array(gyro)

        accel_mean = np.mean(accel, axis=0)
        gyro_mean = np.mean(gyro, axis=0)

        # Bias
        self.calibration["accel_bias"] = accel_mean - np.array([0, 0, self.accel_lsb])
        self.calibration["gyro_bias"] = gyro_mean

        print("Standard calibration complete.")

    def CalibrateScale(self):
        print("Scale calibration (place each axis ±1g).")

        scale = []

        for axis in range(3):
            input(f"Place +{axis} axis up. Press ENTER")
            pos = np.mean([self.GetAccelData() for _ in range(200)], axis=0)

            input(f"Place -{axis} axis up. Press ENTER")
            neg = np.mean([self.GetAccelData() for _ in range(200)], axis=0)

            scale_factor = (2 * self.accel_lsb) / (pos[axis] - neg[axis])
            scale.append(scale_factor)

        self.calibration["accel_scale"] = np.array(scale)
        print("Scale calibration complete.")

    # ---------------------------
    # MULTI-POINT TEMP CALIBRATION
    # ---------------------------
    def CalibrateTemperatureMulti(self, points=4):
        print("Temperature calibration (multi-point)")
        print("Use freezer → room temp sweep for best results")

        self.calibration["temp_points"] = []

        for i in range(points):
            input(f"Stabilize at temp point {i+1}, press ENTER")

            accel = []
            gyro = []
            temps = []

            for _ in range(200):
                ax, ay, az, gx, gy, gz, t = self.GetAllData()
                t = self.ConvertTemp(t)

                accel.append([ax, ay, az])
                gyro.append([gx, gy, gz])
                temps.append(t)

                time.sleep(0.01)
            
            accel_mean = np.mean(accel, axis=0)
            accel_bias = accel_mean - np.array([0, 0, self.accel_lsb])

            self.calibration["temp_points"].append({
                "temp": float(np.mean(temps)),
                "accel_bias": accel_bias,
                "gyro_bias": np.mean(gyro, axis=0)
        })

        self.FitTempPolynomial()

    def FitTempPolynomial(self):
        temps = np.array([p["temp"] for p in self.calibration["temp_points"]])

        accel_biases = np.array([p["accel_bias"] for p in self.calibration["temp_points"]])
        gyro_biases  = np.array([p["gyro_bias"]  for p in self.calibration["temp_points"]])

        accel_coeffs = []
        gyro_coeffs = []

        for axis in range(3):
            accel_coeffs.append(np.polyfit(temps, accel_biases[:, axis], 2))
            gyro_coeffs.append(np.polyfit(temps, gyro_biases[:, axis], 2))

        self.calibration["temp_poly_coeffs_accel"] = accel_coeffs
        self.calibration["temp_poly_coeffs_gyro"] = gyro_coeffs

        print("Temperature calibration (accel + gyro) complete.")

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
        rawGyro = gyro.copy()

        # Temperature polynomial compensation (gyro) or standard Bias if not supplyed
        if self.calibration["temp_poly_coeffs_gyro"] is not None:
            for i in range(3):
                gyro[i] -= np.polyval(self.calibration["temp_poly_coeffs_gyro"][i], temp)
        else:
            gyro  = gyro  - self.calibration["gyro_bias"]

        if self.calibration["temp_poly_coeffs_accel"] is not None:
            for i in range(3):
                accel[i] -= np.polyval(self.calibration["temp_poly_coeffs_accel"][i], temp)
        else: 
            accel = accel - self.calibration["accel_bias"]

        #Calibrated Scale
        accel *= self.calibration["accel_scale"]

        #Scale Values to gs and deg/s
        accel = accel / self.accel_lsb
        gyro = gyro / self.gyro_lsb

        # Axis alignment
        R = self.calibration["rotation_matrix"]
        accel = R @ accel
        gyro = R @ gyro

        # Low-pass filtering
        self.filtered_accel = self.LowPass(accel, self.filtered_accel)
        self.filtered_gyro = self.LowPass(gyro, self.filtered_gyro)

        # Online drift correction (if stable)
        if abs(np.linalg.norm(self.filtered_accel - np.array([0,0,1.0]))) < self.accel_cutoff and np.linalg.norm(self.filtered_gyro) < self.gyro_cutoff:
            self.calibration["gyro_bias"] = (
                (1-self.alpha_ODC) * self.calibration["gyro_bias"] +
                self.alpha_ODC * rawGyro
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
        accel_angle_x = np.degrees(np.arctan2(ay, az))
        accel_angle_y = np.degrees(np.arctan2(-ax, np.sqrt(ay**2 + az**2)))

        # Integrate gyro
        
        self.angle[0] += gx * dt
        self.angle[1] += gy * dt

        # Complementary filter
        alpha = 0.98
        self.angle[0] = alpha * self.angle[0] + (1 - alpha) * accel_angle_x
        self.angle[1] = alpha * self.angle[1] + (1 - alpha) * accel_angle_y

        return self.angle

    def UpdateOrientationMahony(self):
        accel, gyro, temp = self.GetCalibratedData()

        gx, gy, gz = np.radians(gyro)
        ax, ay, az = accel

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        dt = min(dt, 0.02)

        # Update Mahony timestep
        self.ahrs.Dt = dt

        self.q = self.ahrs.updateIMU(
            self.q,
            gyr=np.array([gx, gy, gz]),
            acc=np.array([ax, ay, az])
        )

        return self.QuaternionToEuler(self.q)

    # ---------------------------
    # SAVE / LOAD
    # ---------------------------
    def _to_serializable(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: self._to_serializable(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self._to_serializable(v) for v in obj]
        else:
            return obj


    def _from_serializable(self, obj):
        if isinstance(obj, list):
            try:
                arr = np.array(obj)
                if np.issubdtype(arr.dtype, np.number):
                    return arr
            except:
                pass
            return [self._from_serializable(v) for v in obj]

        elif isinstance(obj, dict):
            return {k: self._from_serializable(v) for k, v in obj.items()}
        return obj

    def SaveCalibration(self, filename="imu_cal.json"):
        data = self._to_serializable(self.calibration)

        data["accel_range"] = self.accel_range
        data["gyro_range"] = self.gyro_range

        with open(filename, "w") as f:
            json.dump(data, f, indent=4)

        print(f"Calibration saved to {filename}")

    def LoadCalibration(self, filename="imu_cal.json"):
       
        with open(filename, "r") as f:
            data = json.load(f)

        self.calibration = self._from_serializable(data)

        # Restore ranges safely
        accel_range = self.calibration["accel_range"]
        gyro_range = self.calibration["gyro_range"]
        self.SetSensorRanges(accel_range, gyro_range)

        print(f"Calibration loaded from {filename}")

        