import numpy as np
import sys
sys.path.append("/usr/local/lib/python3.8/site-packages")
import cv2
import serial
import time
import threading
from Motor import Motor

serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )
R_encoder = L_encoder = 0
flag = False
r_speed = l_speed = 0

def rMotor():
    global R_encoder
    global flag
    global r_speed
    pins = [37, 35, 32]
    motor = Motor(Pins = pins)
    try:
        while not flag:
            motor.setSpeed(R_encoder, )
            #print("rr")
        print("r close")
    finally:
        print("delete right motor")
        del motor
        
def lMotor():
    global L_encoder
    global flag
    global l_speed
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

def map(turn):
  if turn > 320:
    return ((turn - 320)/15)**2
  elif turn < 320:
    return -((turn - 320)/15)**2
  else:
    return 0
    
def toHSV(img):
     img_copy = img.copy()
     img_HSV = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
     return img_HSV

def getHsvMask(img):
     lowerWhite = np.array([0, 0, 230])
     upperWhite = np.array([180, 30, 255])
     maskWhite = cv2.inRange(img, lowerWhite, upperWhite)

     return maskWhite

def cv(img):
    img = toHSV(img)
    img = getHsvMask(img)

    left_deviation = 0
    for i in range(300,0,-1):
        if (img[200][i] == 255):
            left_deviation = i
            break

    right_deviation = 640
    for i in range(340,640):
        if (img[200][i] == 255):
            right_deviation = i
            break
    
    left_deviation1 = 0
    for i in range(300,0,-1):
        if (img[300][i] == 255):
            left_deviation1 = i
            break
    right_deviation1 = 640
    for i in range(340,640):
        if (img[300][i] == 255):
            right_deviation1 = i
            break

    vertical_deviation = 0
    for i in range(300, 0, -1):
        if (img[i][320] == 255):
            vertical_deviation = i
            break

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.line(img, (300, vertical_deviation), (340, vertical_deviation), (0,0,255),3)
    cv2.line(img, (int(left_deviation1), 280), (int(left_deviation1), 320), (0,0,255),3)
    cv2.line(img, (int(right_deviation1), 280), (int(right_deviation1), 320), (0,0,255),3)
    cv2.line(img, (int(left_deviation), 180), (int(left_deviation), 220), (0,0,255),3)
    cv2.line(img, (int(right_deviation), 180), (int(right_deviation), 220), (0,0,255),3)
    a = right_deviation + left_deviation
    b = right_deviation1 + left_deviation1
    c = vertical_deviation
    #print(c)
    #if c > 0:
    if not a / 2  > 320 and b / 2 > 320:
       turn = int((a * 0 + b * 2)/1 / 2)
    elif not a / 2 < 320 and b / 2 < 320:
        b -= 320
        turn = int((a * 0 + b * 2)/ 2)
    else:
        print(1)
        turn = int((a * 1 + b * 0)/1 / 2)
    # else:
    #    turn = 320
    # turn = int((right_deviation + left_deviation) / 2)
    cv2.line(img, (320, 400), (turn, 400), (255,0,0),3)

    return img, map(turn)


def main():
    global serial_port
    global R_encoder
    global L_encoder
    global flag
    global r_speed
    global l_speed

    r = threading.Thread(target = rMotor, daemon = False)
    l = threading.Thread(target = lMotor, daemon = False)
    msgs = []
    r.start()
    l.start()
    cap = cv2.VideoCapture(-1)
    try:
        while not flag:
            ret, frame = cap.read()
            if ret:
                frame1 = frame.copy()
                frame1, turn = cv(frame1)
                cv2.imshow('video',frame1)
                try:
                    serial_port.open()
                    if turn >= 20:
                        turn = 35
                    elif turn <= -20:
                        turn = -35
                    msg = serial_port.read_all()
                    if msg:
                        msgs = msg.decode().split()
                        R_encoder = int(msgs[0])
                        L_encoder = int(msgs[1])
                        print(msgs)
                    if turn < 0:	
                            r_speed = 70
                            l_speed = 70 + turn
                    elif turn > 0:
                        r_speed = 70 - turn
                        l_speed = 70
                    else:
                        r_speed = 70
                        l_speed = 70
                except serial.SerialException as e:
                    print(e)
                except KeyboardInterrupt:
                    flag = True
                    print("Exiting Program")
                except Exception as exception_error:
                    print("Error occurred. Exiting Program")
                    print("Error: " + str(exception_error))
                finally:
                    pass
            else:
                break
            if cv2.waitKey(10) == ord('q'):
                break
        serial_port.close()
        cap.release()
        cv2.destroyAllWindows()
    finally:
        r.join()
        l.join()
        print(flag)
        serial_port.close()
        GPIO.cleanup()

if __name__ == '__main__':
    main()