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
        motorSpeedCoef = 1,
        motorMaxSpeed = 1,
        motorMaxDelta = .03
        ):

        self.motorMaxSpeed = motorMaxSpeed
        self.motorMaxDelta = motorMaxDelta
        self.motorSpeedCoef = motorSpeedCoef
        self.PIDs = {"pitch":pitchPID, "roll":rollPID, "yawRate":yawPID} 
        self.IMU = IMU
        self.motors = (motorFR, motorFL, motorBR, motorBL)
        self.motorCommand = [0,0,0,0]
        self.lastMotorCommand = [0,0,0,0]

    def IMUStep(self):
        # --- Read IMU ---
        ax, ay, az, gx, gy, gz, temp = self.IMU.GetAllData()

        # --- Save Data ---
        ax, ay, az, gx, gy, gz, temp = self.IMU.ApplyCalibration(ax, ay, az, gx, gy, gz, temp)
        roll, pitch, yaw = self.IMU.UpdateOrientation()
        return ax, ay, az, gx, gy, gz, temp, roll, pitch, yaw
    
    def LoopStep(self, throttle, target_pitch, target_roll, target_yaw_rate, dt):
        # --- Update IMU ---
        ax, ay, az, gx, gy, gz, temp, roll, pitch, yaw = self.IMUStep()

        # --- PID Errors ---
        errors = {"pitch":target_pitch - pitch, "roll":target_roll - roll, "yawRate":target_yaw_rate - gz}
        
        # --- PID Outputs ---
        safedt = min(dt, 0.1)  # Prevent large dt values
        pitch_output = self.PIDs["pitch"].Update(errors["pitch"], safedt, gy)
        roll_output = self.PIDs["roll"].Update(errors["roll"], safedt, gx)
        yaw_output = self.PIDs["yawRate"].Update(errors["yawRate"], safedt, gz)

        # --- Motor Mixing (X quad) ---
        self.motorCommand = [
        (throttle - pitch_output - roll_output - yaw_output)*self.motorSpeedCoef,
        (throttle - pitch_output + roll_output + yaw_output)*self.motorSpeedCoef,
        (throttle + pitch_output + roll_output - yaw_output)*self.motorSpeedCoef,
        (throttle + pitch_output - roll_output + yaw_output)*self.motorSpeedCoef
        ]

        # --- Motor Clamping ---
        max_motor = max(motors)
        min_motor = min(motors)

        if max_motor > 1.0:
            excess = max_motor - 1.0
            motors = [m - excess for m in motors]

        if min_motor < 0.0:
            deficit = -min_motor
            motors = [m + deficit for m in motors]

        # --- Slew Limiter ---
        self.motorCommand = [max(prev - self.motorMaxDelta, min(cmd, prev + self.motorMaxDelta)) 
                                for cmd, prev in zip(self.motorCommand, self.lastMotorCommand)]
        self.lastMotorCommand = self.motorCommand
        
        # --- Return Motor Commands ---
        return self.motorCommand, roll, pitch, yaw

    def SetMotors(self, command):

        for c, m in zip(command, self.motors):
            m.SetSpeed(c)

    def KillMotors(self):
        for m in self.motors:
            m.Kill()

        