from IMU import IMU
from ESC_Control import ESC_Brushless as ESC
from PID import PID
from Xbox_Controller import XboxController as Xbox
from Flight_Controller import Flight_Controller as FC
import time as t
import sys


def OneTimeIMUCalibration(imu, saveFile, tempPoints = 4, rotMatrix = none):
    # Multi-point Tempature Calibration
    imu.CalibrateTemperatureMulti(tempPoints)

    # Standard IMU Calibration
    imu.CalibrateStandard()

    # IMU Scale Calibration
    imu.CalibrateScale()

    # Add Rotation Matrix if Supplyed
    if rotMatrix != none:
        imu.SetRotationMatrix(rotMatrix)

    # Save Calibration Data
    imu.SaveCalibration(saveFile)
    print("IMU Calibration Done and Saved to", saveFile)


def FlightControlLoop(fc, xbox, baseThrust = .3, rate = 100):
    loopTime = 1/rate
    lastTime = t.time()

    # Main Control Loop
    while True:

        #Read Xbox Controller Input
        input = xbox.Read()

        #Convert Xbox Input to Flight Controller Commands
        throttle = input['ly']  # Left stick vertical for throttle
        throttle = baseThrust + (1-baseThrust)*throttle  # Scale to [baseThrust, 1]
        target_pitch = input['ry']  # Right stick vertical for pitch
        target_roll = input['rx']  # Right stick horizontal for roll
        target_yaw_rate = input['lx']  # Left stick horizontal for yaw rate

        # Update Flight Controller with new input
        dt = t.time() - lastTime
        lastTime = t.time()
        fc.LoopStep(throttle, target_pitch, target_roll, target_yaw_rate, dt)

        # Sleep to maintain loop rate
        t.sleep(max(0, loopTime - (t.time() - lastTime)))


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

