import Jetson.GPIO as GPIO
import serial
from PID_VelocityControl import PID_VelocityControl

class Motor():
    def __init__(self, serial_port, Pins):
        self.serial_port = serial_port 
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
    def setSpeed(self):
        self.setEncoder()
        self.pid.calPID(100, self.encoder)
        self.velocity = self.pid.setVelocity()
        self.pwm.ChangeDutyCycle(self.velocity)
    def setEncoder(self):
        try:
            self.encoder = self.serial_port.read_all()
            if self.encoder:
                self.encoder = int(self.encoder.decode())
                # print(R_encoder)
        except serial.SerialException as e:
            print(e)
        except ValueError as e:
            print(e)
    def __del__(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.serial_port.close()