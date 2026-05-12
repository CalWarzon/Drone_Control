from IMU import IMU
from ESC_Control import ESC_Brushless as ESC
from PID import PID
from Xbox_Controller import XboxController as Xbox
from Flight_Controller import Flight_Controller as FC
import random as r
import time as t
import sys


def OneTimeIMUCalibration(imu, saveFile, tempPoints = 4, rotMatrix = None):
    # Multi-point Tempature Calibration
    imu.CalibrateTemperatureMulti(tempPoints)

    # Standard IMU Calibration
    imu.CalibrateStandard()

    # IMU Scale Calibration
    imu.CalibrateScale()

    # Add Rotation Matrix if Supplyed
    if rotMatrix != None:
        imu.SetRotationMatrix(rotMatrix)

    # Save Calibration Data
    imu.SaveCalibration(saveFile)
    print("IMU Calibration Done and Saved to", saveFile)


def FlightControlLoop(fc, xbox, baseThrust = .3, throttleRange = .25, rate = 100, inputSkips = 1):
    loopTime = 1/rate
    lastTime = t.time()
    motorWait = 0
    inputWait = 0
    # Main Control Loop
    while True:
        #Runs input code every n timesteps
        if inputWait == 0:
            inputWait = inputSkips

            #Read Xbox Controller Input
            input = xbox.Read()

            #Safety Shutoff
            if input is None:
                fc.SetMotors({
                    "FR":0,
                    "FL":0,
                    "BR":0,
                    "BL":0
                })
                continue

            #Convert Xbox Input to Flight Controller Commands
            throttle = input['ly']  # Left stick vertical for throttle
            throttle = baseThrust + throttleRange*throttle  # Scale to a range centered around base thrust
            target_pitch = input['ry']  # Right stick vertical for pitch
            target_roll = input['rx']  # Right stick horizontal for roll
            target_yaw_rate = input['lx']  # Left stick horizontal for yaw rate

        # Update Flight Controller with new input
        now = t.perf_counter()
        dt = now - dtLastTime
        dtLastTime = now
        motorCommands = fc.LoopStep(throttle, target_pitch, target_roll, target_yaw_rate, dt)
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

pid = PID(kp = .5, ki = .08, kd = .2)
num = 2
current = t.time()

while True:
    num = pid.Update(num, t.time()-current)
    current = t.time()
    LiveDisplayStep(num)
    num += .2 + r.uniform(-1,1)