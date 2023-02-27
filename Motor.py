import Jetson.GPIO as GPIO
from PID_VelocityControl import PID_VelocityControl

class Motor():
    def __init__(self, Pins):
        self.pins = Pins
        self.pid = PID_VelocityControl(kp = 0.35, ki = 0.000003, kd = 100)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pins[0], GPIO.OUT, initial=GPIO.HIGH) # HIGH PIN
        GPIO.setup(self.pins[1], GPIO.OUT, initial=GPIO.LOW) # LOW PIN
        GPIO.setup(self.pins[2], GPIO.OUT, initial=GPIO.HIGH) # PWM PIN
        self.pwm = GPIO.PWM(self.pins[2], 50)
        self.pwm.start(0)
        self.encoder = 0
        self.velocity = 0
    def setSpeed(self, encoder, target):
        self.encoder = encoder
        self.pid.calPID(target=target, encoder = self.encoder)
        self.velocity = self.pid.setVelocity()
        self.pwm.ChangeDutyCycle(self.velocity)
    def __del__(self):
        self.pwm.stop()