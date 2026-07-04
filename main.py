from IMU import IMU
from ESC_Control import ESC_Brushless as ESC
from PID import PID
from Xbox_Controller import XboxController as Xbox
from Flight_Controller import Flight_Controller as FC
from SafetyManager import SafetyManager as Safety
from Simulator import DroneSimulator as Sim
import random as r
import time as t
import sys
import numpy as np


def OneTimeIMUCalibration(imu, saveFile, tempPoints = 4, rotMatrix = None):
    # Multi-point Tempature Calibration
    input("Press Enter to Start Multi-Point Temp Calibration")
    #imu.CalibrateTemperatureMulti(tempPoints)

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

def FlightControlLoop(fc, xbox, safety, 
                      baseThrust = .3, throttleRange = .25,
                      yawSpeed = 50, pitchLean = 20, rollLean = 20,
                      rate = 100, inputEvery = 4,
                      doSim = False, sim = None
                      ):
    loopTime = 1/rate
    #dt = 1/rate
    sleepLastTime = t.perf_counter()
    dtLastTime = t.perf_counter()
    inputCount = 0
    speedTest = []
    displayTime = t.perf_counter()
    throttle = .25
    target_pitch = 0
    target_roll = 0
    target_yaw_rate = 0
    state = dict()
    motorCommands = [0,0,0,0]

    # Main Control Loop
    #try:
    while True:
        
        if doSim:
            state = sim.Step(*motorCommands)
            pitch = state["pitch"]
            roll = state["roll"]
            yaw = state["yaw"]
            gx = state["gx"]
            gy = state["gy"]
            gz = state["gz"]
            x = state["x"]
            y = state["y"]
            z = state["z"]

        #Runs input code every n timesteps
        if inputCount == 0:
            inputCount = inputEvery

            #Read Xbox Controller Input
            controllerInput = xbox.Read()

            #Safety Shutoff
            if controllerInput is None:
                fc.KillMotors()
                continue
            safety.UpdateController()

            #Check Arming
            safety.UpdateArming(controllerInput)

            #Convert Xbox Input to Flight Controller Commands
            throttle = controllerInput['ly']  # Left stick vertical for throttle
            throttle = baseThrust + throttleRange * throttle  # Scale to a range centered around base thrust
            target_pitch = pitchLean * controllerInput['ry']  # Right stick vertical for pitch
            target_roll = rollLean * controllerInput['rx']  # Right stick horizontal for roll
            target_yaw_rate =  yawSpeed * controllerInput['lx']  # Left stick horizontal for yaw rate
        
        #Advance Countdown
        inputCount -= 1
        
        # Update Flight Controller with new input
        now = t.perf_counter()
        dt = now - dtLastTime
        dtLastTime = now
        motorCommands, roll, pitch, yaw = fc.LoopStep(throttle, target_pitch, target_roll, target_yaw_rate, dt)
        #motorCommands, roll, pitch, yaw = fc.LoopStep(throttle, target_pitch, target_roll, target_yaw_rate, dt, pitch, roll, yaw, gx, gy, gz)
         
        LiveDisplayStep((motorCommands, roll, pitch, yaw, dt))
        
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
        '''
        if displayTime + .2 < t.perf_counter():
            displayTime = t.perf_counter()
            LiveDisplayStep((np.round(np.array([[motorCommands[0], motorCommands[1]],[motorCommands[2], motorCommands[3]]]),2),pitch, roll, gz, throttle, "\n", target_pitch, target_roll, target_yaw_rate, "\n",x ,y ,z, "\n",dt))
        '''
        # Sleep to maintain loop rate
        #speedTest.append(t.perf_counter()-sleepLastTime)
        t.sleep(max(0, loopTime - (t.perf_counter()-sleepLastTime)))
        sleepLastTime = t.perf_counter()
    #except:
        #fc.KillMotors()
        #print(speedTest[:30], sum(speedTest)/len(speedTest))


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

mFR = ESC(
    pin = 16,
    maxPulseWidth = 2000,
    minPulseWidth = 1000,
    maxPulseWidthSafe = 1700,
    minPulseWidthSafe = 1090)
mFL = ESC(
    pin = 19, 
    maxPulseWidth = 2000,
    minPulseWidth = 1000,
    maxPulseWidthSafe = 1700,
    minPulseWidthSafe = 1090)
mBR = ESC(
    pin = 20, 
    maxPulseWidth = 2000,
    minPulseWidth = 1000,
    maxPulseWidthSafe = 1700,
    minPulseWidthSafe = 1090)
mBL = ESC(
    pin = 26,    
    maxPulseWidth = 2000,
    minPulseWidth = 1000,
    maxPulseWidthSafe = 1700,
    minPulseWidthSafe = 1090)

imu = IMU()
imu.LoadCalibration("IMU_cal_v1")

controller = Xbox()

safe = Safety()

fc = FC(pitchPID = PID(
            kp = .008,
            ki = .007,
            kd = .0018,
            integrator_limit=.3,
            output_limit=.2,
            integral_fade=0.97,
            d_filter_alpha=0.15,
            use_gyro_derivative=True
        ), 
        rollPID = PID(
            kp = .008,
            ki = .007,
            kd = .0018,
            integrator_limit=.3,
            output_limit=.2,
            integral_fade=0.97,
            d_filter_alpha=0.15,
            use_gyro_derivative=True
        ), 
        yawPID = PID(
            kp = .001,
            ki = .0012,
            kd = .0001,
            integrator_limit=.3,
            output_limit=.2,
            integral_fade=0.97,
            d_filter_alpha=0.15,
            use_gyro_derivative=False
        ), 
        IMU = imu,
        motorFR = mFR,
        motorFL = mFL,
        motorBR = mBR,
        motorBL = mBL,
        motorSpeedCoef = 1,
        motorMaxSpeed = 1,
        motorMaxDelta = .005,
        useIMU=True
        )
'''
sim = Sim(mass = .25,
          arm_length = .1,
          max_thrust_per_motor = 2.5,
          dt = .005,
          wind_force = 0,
          wind_torque = .03)
'''

#OneTimeIMUCalibration(imu = imu, saveFile = "IMU_cal_v1")

fc.CalibrateMotors()

#fc.ArmMotors()
#fc.SetMotors([.5,.5,.5,.5])
#t.sleep(3)
#fc.KillMotors()

'''
FlightControlLoop(
    fc = fc, 
    xbox = controller, 
    safety = safe, 
    baseThrust = .25, 
    throttleRange = .3,
    pitchLean = 45,
    rollLean = 45,
    rate = 200, 
    inputEvery = 1, 
    doSim=False
)

'''

