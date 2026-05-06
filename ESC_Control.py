import pigpio
import time as t


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
        
        self.pi.set_servo_pulsewidth(pwm)

    def SetSpeed(speed):
        speed = max(0, min(1, speed))
        pwm = self.minp + speed * (self.maxp-self.minp)
        


