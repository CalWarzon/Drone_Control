import IMU
import ESC_Control as ESC
import PID
import Xbox_Controller as Xbox
import Flight_Controller as FC
import time as t


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

