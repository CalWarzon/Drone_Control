class Flight_Controller():
    def __init__(self,
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
        self.PIDs = {"pitch":pitchPID, "roll":rollPID, "yawRate":yawPID} 
        self.IMU = IMU
        self.motors = {"FR":motorFR, "FL":motorFL, "BR":motorBR, "BL":motorBL}
    
    def LoopStep(self, throttle, target_pitch, target_roll, target_yaw_rate, dt):

        # --- Read IMU ---
        data = self.IMU.GetAllData()

        ax = data[0]['x']
        ay = data[0]['y']
        az = data[0]['z']
        gx = data[1]['x'] 
        gy = data[1]['y']
        gx = data[1]['z']
        temp = data[2]

        # --- Save Data ---
        self.throttle
        self.IMU.ApplyCalibration(ax, ay, az, gx, gy, gz, temp)
        roll, pitch, yaw = self.IMU.UpdateOrientation()

        # --- PID Errors ---
        errors = {"pitch":target_pitch - pitch, "roll":target_roll - roll, "yawRate":target_yaw_rate - gz}
        
        # --- PID Outputs ---
        safedt = min(dt, 0.1)  # Prevent large dt values
        pitch_output = self.PIDs["pitch"].Update(errors["pitch"], safedt, gy)
        roll_output = self.PIDs["roll"].Update(errors["roll"], safedt, gx)
        yaw_output = self.PIDs["yawRate"].Update(errors["yawRate"], safedt, gz)

        # --- Motor Mixing (X quad) ---
        self.motorCommand = {
        "FR":(throttle + pitch_output + roll_output - yaw_output)*self.motorSpeedCoef,
        "FL":(throttle + pitch_output - roll_output + yaw_output)*self.motorSpeedCoef,
        "BR":(throttle - pitch_output - roll_output - yaw_output)*self.motorSpeedCoef,
        "BL":(throttle - pitch_output + roll_output + yaw_output)*self.motorSpeedCoef
        }

        # --- Return Motor Commands ---
        return self.motorCommand, roll, pitch, yaw

    def SetMotors(self, command):

        for pos, m in self.motors.items():
            speed = command[pos]
            m.SetSpeed(speed)

    def KillMotors(self):
        for m in self.motors:
            m.Kill()

        