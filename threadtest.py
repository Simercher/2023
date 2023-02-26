import Jetson.GPIO as GPIO
import threading
import serial
from Motor import Motor
from PID_VelocityControl import PID_VelocityControl

serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )
R_encoder = L_encoder = 0
flag = False

def rMotor():
    global R_encoder
    global flag
    pins = [37, 35, 32]
    motor = Motor(Pins = pins)
    try:
        while not flag:
            motor.setSpeed(R_encoder)
            #print("rr")
        print("r close")
    finally:
        print("delete right motor")
        del motor
        
def lMotor():
    global L_encoder
    global flag
    pins = [36, 38, 33]
    motor = Motor(Pins = pins)
    try:
        while not flag:
            motor.setSpeed(L_encoder)
            #print("ll")
        print("l close")
    finally:
        print("delete left motor")
        del motor

def main():
    global serial_port
    global R_encoder
    global L_encoder
    global flag
    r = threading.Thread(target = rMotor, daemon = False)
    l = threading.Thread(target = lMotor, daemon = False)
    msgs = []
    r.start()
    l.start()
    print(r.native_id, l.native_id)
    try:
        while not flag:
            try:
                msg = serial_port.read_all()
                if msg:
                    msgs = msg.decode().split()
                    R_encoder = int(msgs[0])
                    L_encoder = int(msgs[1])
                    print(msgs)
            except serial.SerialException as e:
                print(e)
            except KeyboardInterrupt as e:
                print(e)
                flag = True
    finally:
        r.join()
        l.join()
        print(flag)
        serial_port.close()
        GPIO.cleanup()
    
if __name__ == "__main__":
    main()

