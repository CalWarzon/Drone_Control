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

        print("Starting ESC calibration")

        # Send max throttle
        self.pi.set_servo_pulsewidth(self.pin, self.maxp)

        input(
            "Connect battery now.\n"
            "After calibration beeps press ENTER."
        )

        # Send minimum throttle
        self.pi.set_servo_pulsewidth(self.pin, self.minp)

        print("Waiting for ESC to confirm calibration...")

        t.sleep(3)

        # Hold minimum throttle to arm
        print("Arming ESC...")

        self.pi.set_servo_pulsewidth(self.pin, self.minp)

        t.sleep(2)

        print("Calibration complete")
    
    def Arm(self):

        print("Arming ESC...")

        # Ensure stopped signal first
        self.pi.set_servo_pulsewidth(self.pin, 0)

        t.sleep(1)

        # Send minimum throttle
        self.pi.set_servo_pulsewidth(self.pin, self.minp)

        # Wait for ESC to arm
        t.sleep(2)

        print("ESC armed")
    
    def SetPWM(self, pwm):
        
        self.pi.set_servo_pulsewidth(self.pin, pwm)

    def SetSpeed(self, speed):
        speed = max(0, min(1, speed))
        pwm = self.minp + speed * (self.maxp - self.minp)
        self.pi.set_servo_pulsewidth(self.pin, pwm)
        
    def SweepPWM(self, start_pwm, end_pwm, step, time_per_speed):
    
        # Ensure correct step direction
        if start_pwm < end_pwm and step < 0:
            step = abs(step)

        if start_pwm > end_pwm and step > 0:
            step = -step

        pwm = start_pwm

        try:
            while (step > 0 and pwm <= end_pwm) or \
                (step < 0 and pwm >= end_pwm):

                print(f"PWM: {pwm}")

                self.pi.set_servo_pulsewidth(self.pin, pwm)

                t.sleep(time_per_speed)

                pwm += step

        finally:
            # Stop motor at end
            self.pi.set_servo_pulsewidth(self.pin, 0)

        print("Motor stopped")


