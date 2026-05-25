from IMU import IMU
from ESC_Control import ESC_Brushless as ESC
from PID import PID
from Xbox_Controller import XboxController as Xbox
from Flight_Controller import Flight_Controller as FC
from SafetyManager import SafetyManager as Safety
import random as r
import time as t
import sys
import numpy as np


def OneTimeIMUCalibration(imu, saveFile, tempPoints = 4, rotMatrix = None):
    # Multi-point Tempature Calibration
    input("Press Enter to Start Multi-Point Temp Calibration")
    imu.CalibrateTemperatureMulti(tempPoints)

    # Standard IMU Calibration
    input("Press Enter to Start Bias Calibration")
    imu.CalibrateStandard()

    # IMU Scale Calibration
    input("Press Enter to Start Scale Calibration")
    imu.CalibrateScale()

    # Add Rotation Matrix if Supplyed
    if rotMatrix != None:
        imu.SetRotationMatrix(rotMatrix)

    # Save Calibration Data
    imu.SaveCalibration(saveFile)


def FlightControlLoop(fc, xbox, safety, baseThrust = .3, throttleRange = .25, rate = 100, inputSkips = 1):
    loopTime = 1/rate
    sleepLastTime = t.perf_counter()
    dtLastTime = t.perf_counter()
    motorWait = 0
    inputWait = 0
    # Main Control Loop
    while True:
        #Runs input code every n timesteps
        if inputWait == 0:
            inputWait = inputSkips

            #Read Xbox Controller Input
            controlerInput = xbox.Read()

            #Safety Shutoff
            if controlerInput is None:
                fc.KillMotors()
                continue
            safety.UpdateController()

            #Check Arming
            safety.UpdateArming(controlerInput)

            #Convert Xbox Input to Flight Controller Commands
            throttle = controlerInput['ly']  # Left stick vertical for throttle
            throttle = baseThrust + throttleRange * throttle  # Scale to a range centered around base thrust
            target_pitch = controlerInput['ry']  # Right stick vertical for pitch
            target_roll = controlerInput['rx']  # Right stick horizontal for roll
            target_yaw_rate = controlerInput['lx']  # Left stick horizontal for yaw rate

        # Update Flight Controller with new input
        now = t.perf_counter()
        dt = now - dtLastTime
        dtLastTime = now
        motorCommands, roll, pitch, yaw = fc.LoopStep(throttle, target_pitch, target_roll, target_yaw_rate, dt)
        
        #Safety Checks
        safety.UpdateLoop()

        if safety.CheckFailsafe((roll, pitch, yaw)):
            fc.KillMotors()
            continue

        if not safety.MotorsEnabled():
            fc.KillMotors()
            continue

        #Send Motor Commands
        fc.SetMotors(motorCommands)

        #Step Skips
        inputWait -= 1
    
        # Sleep to maintain loop rate
        t.sleep(max(0, loopTime - (t.perf_counter()-sleepLastTime)))
        sleepLastTime = t.perf_counter()


def LiveDisplay(data_function, hz=10):
    """
    Continuously updates terminal output.

    Parameters:
        data_function : function
            Function returning either:
                - string
                - dict
                - list/tuple
                - any printable object

        hz : float
            Update rate in Hertz
    """

    delay = 1.0 / hz

    try:
        while True:
            start = t.time()

            data = data_function()

            # Move cursor to top-left and clear screen
            sys.stdout.write("\033[H\033[J")

            # Pretty printing
            if isinstance(data, dict):
                for k, v in data.items():
                    print(f"{k}: {v}")

            elif isinstance(data, (list, tuple)):
                for item in data:
                    print(item)

            else:
                print(data)

            sys.stdout.flush()

            # Maintain update rate
            elapsed = t.time() - start
            sleep_time = delay - elapsed

            if sleep_time > 0:
                t.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped.")

def LiveDisplayStep(data):
    """
    Parameters:
        data_function : function
            Function returning either:
                - string
                - dict
                - list/tuple
                - any printable object

    """
    # Move cursor to top-left and clear screen
    sys.stdout.write("\033[H\033[J")

    # Pretty printing
    if isinstance(data, dict):
        for k, v in data.items():
            print(f"{k}: {v}")

    elif isinstance(data, (list, tuple)):
        for item in data:
            print(item)

    else:
        print(data)

    sys.stdout.flush()

#IMU Read Speed: .00147s
    
    
imu = IMU()
imu.LoadCalibration("IMU_cal_v1")
#OneTimeIMUCalibration(imu, "IMU_cal_v1", 5)
#imu.CalibrateStandard()
#imu.CalibrateScale()
#imu.SaveCalibration("IMU_cal_v1")

i = 50
rate = 400
rest = 1/rate 
while True:
    lastT = t.perf_counter()
    ax, ay, az, gx, gy, gz, temp = imu.GetAllData()
    accel,gyro,temp = imu.ApplyCalibration(
        ax = ax,
        ay = ay,
        az = az,
        gx = gx,
        gy = gy,
        gz = gz,
        raw_temp = temp
               )
    pitch, roll, yaw = imu.UpdateOrientation()
    if i == 0:
        LiveDisplayStep((pitch, roll, yaw, dt<rest))
        i = 20
    i-=1
    dt = (t.perf_counter()-lastT)
    t.sleep(max(0,rest-dt))
#print(sum(speed)/len(speed))
