import time as t
import json
import numpy as np
import sys
from smbus2 import SMBus
from ahrs.filters import Mahony


class IMU:
    #accel: g
    #gyro: deg/sec
    #Orientation: deg
    #Tempature: C

    MPU_ADDR = 0x68

    # Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C

    ACCEL_XOUT_H = 0x3B

    def __init__(self, bus_num = 1):
        self.calibration = {
            "accel_bias": np.zeros(3),
            "gyro_bias": np.zeros(3),
            "accel_scale": np.ones(3),
            "gyro_scale": np.ones(3),
            "temp_points": [],  # multi-point temp calibration
            "temp_poly_coeffs_gyro": None,
            "temp_poly_coeffs_accel": None,
            "rotation_matrix": np.eye(3),
            "use_temp_gyro": True,
            "use_temp_accel":False
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
        self.last_time = t.time()

        #Scaleing
        self.accel_range = 2      # g
        self.gyro_range = 250     # deg/sec
        self.accel_lsb = 16384
        self.gyro_lsb = 131.
        self.accel_regs = 0x00
        self.gyro_regs = 0x00

        self.bus = SMBus(bus_num)
        
        # Wake up MPU6050
        self.bus.write_byte_data(
            self.MPU_ADDR,
            self.PWR_MGMT_1,
            0x00
        )   

        t.sleep(0.1)

        # Sample rate divider
        # 1000Hz / (1 + 0) = 1000Hz
        self.bus.write_byte_data(
            self.MPU_ADDR,
            self.SMPLRT_DIV,
            0
        )

        # DLPF config
        # 0x03 ≈ 44Hz accel / 42Hz gyro bandwidth
        self.bus.write_byte_data(
            self.MPU_ADDR,
            self.CONFIG,
            0x03
        )

        # Gyro 250 deg/s
        self.bus.write_byte_data(
            self.MPU_ADDR,
            self.GYRO_CONFIG,
            self.gyro_regs
        )

        # Accel 2g
        self.bus.write_byte_data(
            self.MPU_ADDR,
            self.ACCEL_CONFIG,
            self.accel_regs
        )
    
    # ---------------------------
    # BASIC UTILITIES
    # ---------------------------
    def ReadWord(self, high, low):

        value = (high << 8) | low

        if value >= 0x8000:
            value -= 65536

        return value

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

        #Send command to IMU
        self.bus.write_byte_data(
            self.MPU_ADDR,
            self.ACCEL_CONFIG,
            self.accel_regs
        )   

        self.bus.write_byte_data(
            self.MPU_ADDR,
            self.GYRO_CONFIG,
            self.gyro_regs
        )

        print(
            f"Accel: ±{accel_range}g ({self.accel_lsb} LSB/g), "
            f"Gyro: ±{gyro_range}°/s ({self.gyro_lsb} LSB/deg/s)"
            )

    #----------------------------
    # RAW MPU-6050 CONNECTION
    #----------------------------
    def GetAllData(self):

        # SINGLE burst read
        data = self.bus.read_i2c_block_data(
            self.MPU_ADDR,
            self.ACCEL_XOUT_H,
            14
        )

        ax = self.ReadWord(data[0], data[1])
        ay = self.ReadWord(data[2], data[3])
        az = self.ReadWord(data[4], data[5])

        temp = self.ReadWord(data[6], data[7])

        gx = self.ReadWord(data[8], data[9])
        gy = self.ReadWord(data[10], data[11])
        gz = self.ReadWord(data[12], data[13])

        return ax, ay, az, gx, gy, gz, temp

    # ---------------------------
    # STANDARD + SCALE CALIBRATION
    # ---------------------------
    def CalibrateStandard(self, samples=500):
        print("Keep IMU flat and still...")
        t.sleep(2)

        accel = []
        gyro = []

        for _ in range(samples):
            ax, ay, az, gx, gy, gz, _ = self.GetAllData()
            accel.append([ax, ay, az])
            gyro.append([gx, gy, gz])
            t.sleep(0.005)

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
        axisName = ('x','y','z')
        for axis in range(3):
            input(f"Place +{axisName[axis]} axis up. Press ENTER")
            pos = np.mean([self.GetAllData()[:3] for _ in range(200)], axis=0)

            input(f"Place -{axisName[axis]} axis up. Press ENTER")
            neg = np.mean([self.GetAllData()[:3] for _ in range(200)], axis=0)

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
            try:
                while True:
                    start = t.time()
                
                    _,_,_,_,_,_,temp = self.GetAllData()
                    temp = self.ConvertTemp(temp)

                    # Move cursor to top-left and clear screen
                    sys.stdout.write("\033[H\033[J")
                    print(f"Temp: {temp:.2f} C, press ctrl + C to calibrate.")

                    sys.stdout.flush()

                    # Maintain update rate
                    elapsed = t.time() - start
                    sleep_time = .1 - elapsed

                    if sleep_time > 0:
                        t.sleep(sleep_time)

            except KeyboardInterrupt:
                print(f"Stabilized at temp point {i+1}, Collecting Data")
            

            accel = []
            gyro = []
            temps = []

            for _ in range(200):
                ax, ay, az, gx, gy, gz, temp = self.GetAllData()
                temp = self.ConvertTemp(temp)

                accel.append([ax, ay, az])
                gyro.append([gx, gy, gz])
                temps.append(temp)

                t.sleep(0.01)
            
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
        if self.calibration["temp_poly_coeffs_gyro"] is not None and self.calibration["use_temp_gyro"]:
            for i in range(3):
                gyro[i] -= np.polyval(self.calibration["temp_poly_coeffs_gyro"][i], temp)
        else:
            gyro  = gyro  - self.calibration["gyro_bias"]

        if self.calibration["temp_poly_coeffs_accel"] is not None and self.calibration["use_temp_accel"]:
            for i in range(3):
                accel[i] -= np.polyval(self.calibration["temp_poly_coeffs_accel"][i], temp)
        else: 
            accel = accel - self.calibration["accel_bias"]

        #Calibrated Scale
        accel *= self.calibration["accel_scale"]

        #Scale Values to gs
        accel = accel / self.accel_lsb
        gyro = gyro/ self.gyro_lsb

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

        return self.filtered_accel[0], self.filtered_accel[1], self.filtered_accel[2], self.filtered_gyro[0], self.filtered_gyro[1], self.filtered_gyro[2], temp

    # ---------------------------
    # COMPLEMENTARY FILTER
    # ---------------------------
    def UpdateOrientation(self):
        now = t.time()
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
        self.angle[2] += gz * dt

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
        #self.SetSensorRanges(accel_range, gyro_range)

        print(f"Calibration loaded from {filename}")

        