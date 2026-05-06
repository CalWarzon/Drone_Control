class Flight_Control():
    def __init__(self, 
        baseThrust,
        pitchPID, 
        rollPID, 
        yawPID, 
        IMU,
        motorFR,
        motorFL,
        motorBR,
        motorBL,
        motorSpeedCoef = 1):

        self.motorSpeedCoef = motorSpeedCoef
        self.baseThrust = baseThrust
        self.thrust = baseThrust
        self.PIDs = {"pitch":pitchPID, "roll":rollPID, "yawRate":yawPID} 
        self.IMU = IMU
        self.motors = {"FR":motorFR, "FL":motorFL, "BR":motorBR, "BL":motorBL}
    
    def LoopStep(self, throttle, target_pitch, target_roll, target_yaw_rate, dt):

        # --- Read IMU ---
        ax, ay, az = imu.read_accel_raw()
        gx, gy, gz = imu.read_gyro_raw()
        temp = imu.read_temp_raw()

        # --- Save Data ---
        self.throttle
        imu.apply_calibration(ax, ay, az, gx, gy, gz, temp)
        roll, pitch, yaw = imu.update_orientation()

        # --- PID Errors ---
        errors = {"pitch":target_pitch - pitch, "roll":target_roll - roll, "yawRate":target_yaw_rate - gz}
        
        # --- PID Outputs ---
        pitch_output = self.PIDs["pitch"].Update(errors["pitch"], dt, gy)
        roll_output = self.PIDs["roll"].Update(errors["roll"], dt, gx)
        yaw_output = self.PIDs["yawRate"].Update(errors["yawRate"], dt, gz)

        # --- Motor Mixing (X quad) ---
        self.motorCommand = {
        "FR":throttle + pitch_output + roll_output - yaw_output,
        "FL":throttle + pitch_output - roll_output + yaw_output,
        "BR":throttle - pitch_output - roll_output - yaw_output,
        "BL":throttle - pitch_output + roll_output + yaw_output
        }

        # --- Send Commands to Motors ---
        self.SetMotors(self.motorCommand)

    def SetMotors(self, command):

        for pos, m in self.motors:
            speed = max(0, min(1, command[pos]*self.motorSpeedCoef))
            m.SetSpeed(speed)