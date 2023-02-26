import threading
import Jetson.GPIO as GPIO
import serial
from PID_VelocityControl import PID_VelocityControl
R_PWM_PIN = 32
R_HIGH_PIN = 37
R_LOWER_PIN = 35
L_PWM_PIN = 33
L_HIGH_PIN = 36
L_LOWER_PIN = 38

ser = serial.Serial(port="/dev/ttyUSB0",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout = 0.11,
        )


def main():
    global ser
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(R_HIGH_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(R_LOWER_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(R_PWM_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(L_HIGH_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(L_LOWER_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(L_PWM_PIN, GPIO.OUT, initial=GPIO.HIGH)
    rp = GPIO.PWM(R_PWM_PIN, 50)
    r_motor = PID_VelocityControl(kp = 0.35, ki = 0.000003, kd = 100)
    rp.start(0)
    R_encoder = 0
    #ser.open()
    print("PWM running. Press CTRL+C to exit.")
    while True:
        try:
        
            #target = input()
            R_encoder = ser.read_all()
            if R_encoder:
                R_encoder = int(R_encoder.decode())
                print(R_encoder)
                #r_motor.calPID(100, R_encoder)
                #velocity = r_motor.setVelocity()
                #rp.ChangeDutyCycle(velocity)
        #except KeyboardInterrupt:
        #    print("interrupt")
        except serial.SerialException as e:
            print(e)
        except ValueError as e:
            print(e)
    p.stop()
        # GPIO.remove_event_detect(ENCODER_R_A)
    GPIO.cleanup()
    ser.close()
   
if __name__ == '__main__':
    main()
