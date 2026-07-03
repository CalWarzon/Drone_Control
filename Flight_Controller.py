import time as t

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
        motorMaxDelta = .03,
        useIMU = True
        ):

        self.motorMaxSpeed = motorMaxSpeed
        self.motorMaxDelta = motorMaxDelta
        self.motorSpeedCoef = motorSpeedCoef
        self.PIDs = {"pitch":pitchPID, "roll":rollPID, "yawRate":yawPID} 
        self.IMU = IMU
        self.useIMU = useIMU
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
    
    def LoopStep(self, throttle, target_pitch, target_roll, target_yaw_rate, dt, 
                 pitch = None, roll = None, yaw = None, gx = None, gy = None, gz = None):
        # --- Update IMU ---
        if self.useIMU:
            ax, ay, az, gx, gy, gz, temp, roll, pitch, yaw = self.IMUStep()

        # --- PID Errors ---
        errors = {"pitch":target_pitch - pitch, "roll":target_roll - roll, "yawRate":target_yaw_rate - gz}
        
        # --- PID Outputs ---
        safedt = min(dt, 0.1)  # Prevent large dt values
        pitch_output = self.PIDs["pitch"].Update(errors["pitch"], safedt, gy)
        roll_output = self.PIDs["roll"].Update(errors["roll"], safedt, gx)
        yaw_output = self.PIDs["yawRate"].Update(errors["yawRate"], safedt, gz)

        # --- Throttle Clamp ---
        throttle = max(.15, min(.75, throttle))

        # --- Motor Mixing (X quad) ---
        self.motorCommand = [
        (throttle - pitch_output - roll_output - yaw_output)*self.motorSpeedCoef,
        (throttle - pitch_output + roll_output + yaw_output)*self.motorSpeedCoef,
        (throttle + pitch_output + roll_output - yaw_output)*self.motorSpeedCoef,
        (throttle + pitch_output - roll_output + yaw_output)*self.motorSpeedCoef
        ]

        # --- Motor Clamping ---
        max_motor = max(self.motorCommand)
        min_motor = min(self.motorCommand)

        if max_motor > self.motorMaxSpeed:
            excess = max_motor - self.motorMaxSpeed
            self.motorCommand = [m - excess for m in self.motorCommand]

        if min_motor < 0.0:
            deficit = -min_motor
            self.motorCommand = [m + deficit for m in self.motorCommand]

        self.motorCommand = [max(0.0, min(self.motorMaxSpeed, m)) for m in self.motorCommand]

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

    def ArmMotors(self):
        for m in self.motors:
            m.Arm()

    def CalibrateMotors(self):
        input("Ensure props are removed and battery is discontected then press Enter to start motor calibration...")
        print("Starting ESC calibration")

        # Send max throttle
        for m in self.motors:
            m.SetPWM(m.maxp)

        input(
            "Connect battery now.\n"
            "After calibration beeps press ENTER."
        )

        # Send minimum throttle
        for m in self.motors:
            m.SetPWM(m.minp)

        print("Waiting for ESC to confirm calibration...")

        t.sleep(3)

        # Hold minimum throttle to arm
        print("Arming ESC...")

        self.pi.set_servo_pulsewidth(self.pin, self.minp)

        t.sleep(2)

        print("Calibration complete")
        