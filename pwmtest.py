import sys
sys.path.append("/usr/local/lib/python3.8/site-packages")
import cv2
import time
import utilis
import Jetson.GPIO as GPIO
import serial
PWM_PIN = 32
HIGH_PIN = 37
LOWER_PIN = 35
ENCODER_R_A = 21
ENCODER_R_B = 22


def main():
    ser = serial.Serial(port="/dev/ttyUSB0",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout = 0.009,
        )
    global counter
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(HIGH_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LOWER_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PWM_PIN, GPIO.OUT, initial=GPIO.HIGH)
    p = GPIO.PWM(PWM_PIN, 50)
    val = 50
    incr = 5
    p.start(val)
    R_encoder = 0
    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            R_encoder = ser.read_all()
            if R_encoder:
                R_encoder = int(R_encoder.decode())
                print(R_encoder)
            pass
    except KeyboardInterrupt:
        print("interrupt")
    finally:
        #p.stop()
        # GPIO.remove_event_detect(ENCODER_R_A)
        GPIO.cleanup()
        ser.close()
    
def map()
    
if __name__ == '__main__':
    main()
