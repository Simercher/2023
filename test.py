import numpy as np
import sys
sys.path.append("/usr/local/lib/python3.8/site-packages")
import cv2
import serial

def map(turn):
    return (turn - 320) * (120 - 0) / (470 - 320)

def drawLine(img, left_deviation, right_deviation):
    cv2.line(img, (0, 100), (300, 100), (0,255,0),3)
    cv2.line(img, (340, 100), (640, 100), (0,255,0),3)
    cv2.line(img, (320, 370), (320, 430), (255,255,255),3)
    cv2.line(img, (0, 70), (0, 130), (0,255,0),3)
    cv2.line(img, (300, 70), (300, 130), (0,255,0),3)
    cv2.line(img, (340, 70), (340, 130), (0,255,0),3)
    cv2.line(img, (640, 70), (640, 130), (0,255,0),3)
    cv2.line(img, (left_deviation, 80), (left_deviation, 120), (0,0,255),3)
    cv2.line(img, (right_deviation, 80), (right_deviation, 120), (0,0,255),3)
    turn = int((right_deviation + left_deviation)/2)
    cv2.line(img, (320, 400), (turn, 400), (255,0,0),3)
    return turn

def calTurn(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    kernel_size = 3
    img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    low_threshold = 75
    high_threshold = 150
    img = cv2.Canny(img, low_threshold, high_threshold)
    
    left_deviation = 0
    for i in range(300,0,-1):
        if (img[100][i] == 255):
            left_deviation = i
            break
    
    right_deviation = 640
    for i in range(340,640):
        if (img[100][i] == 255):
            right_deviation = i
            break
    turn = drawLine(img, left_deviation, right_deviation)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    print(turn)
    return img, turn

def main():
    serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )
    
    cap = cv2.VideoCapture(-1)
    width = 480
    height = 640
    turn_list = []
    while True:
        ret, frame = cap.read()
        if ret:
            frame, turn = calTurn(frame)
            turn_list.append(turn)
            cv2.imshow('video',frame)
            if len(turn_list) >= 6 :
                turn = sum(turn_list) / len(turn_list)
                turn_list.clear()
                try:
                    # Send a message to the Arduino
                    serial_port.open()
                    msg = '0 0'
                    if turn > 320:
                        msg = str(int(120 - map(turn))) + '&120\n'
                    elif turn < 320:
                        msg = '120&' + str(int(120 + map(turn))) + '\n'
                    else:
                        msg = '120&120\n'
                    serial_port.write(msg.encode())
                    print(msg)
                except KeyboardInterrupt:
                    print("Exiting Program")
                except Exception as exception_error:
                    print("Error occurred. Exiting Program")
                    print("Error: " + str(exception_error))
                finally:
                    serial_port.close()
        if cv2.waitKey(10) == ord('q'):
             break
    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()