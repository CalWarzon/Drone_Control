import pigpio
import time as t
from mpu6050 import mpu6050

class ESC_Brushless():
    def __init__(self, pin, maxPulseWidth = 2000, minPulseWidth = 1000):
        self.pin = pin
        self.maxp = maxPulseWidth
        self.minp = minPulseWidth
        self.pi = pigpio.pi()
        self.pi.set_servo_pulsewidth(self.pin, 0)

    def Calibrate(self):
        
        self.pi.set_servo_pulsewidth(self.pin, self.maxp)
        input("Connect Battery Wait for falling tone then 2 beeps then hit enter")
        self.pi.set_servo_pulsewidth(self.pin, self.minp)
        t.sleep(12)
        self.pi.set_servo_pulsewidth(self.pin, 0)
        t.sleep(2)
        print("Arming ESC")
        self.pi.set_servo_pulsewidth(self.pin, self.minp)
    
    def Arm(self):
        
        self.pi.set_servo_pulsewidth(self.pin, 0)
        t.sleep(1)
        self.pi.set_servo_pulsewidth(self.pin, self.maxp)
        t.sleep(1)
        self.pi.set_servo_pulsewidth(self.pin, self.minp)
        print("ESC armed")
    
    def SetPWM(self, pwm):
        
        self.pi.set_servo_pulsewidth(self.pin, pwm)
        
'''
accelGyro = mpu6050(0x68)
print(accelGyro.read_accel_range())
while True:
    print(accelGyro.get_accel_data())
    t.sleep(.3)
'''
motor = ESC_Brushless(26)
motor.calibrate()
t.sleep(3)
for pwm in range(1000,1200,20):
    print(pwm)
    motor.speed(pwm)
    t.sleep(8)
motor.speed(1000)

