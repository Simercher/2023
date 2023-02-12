import numpy as np
# import sys
# sys.path.append("/usr/local/lib/python3.8/site-packages")
import cv2
# import serial

def map(turn):
    return (turn - 32) * (120 - 0) / (40 - 32)

def drawLine(img, left_deviation, right_deviation):
    cv2.line(img, (0, 10), (30, 10), (0,255,0),3)
    cv2.line(img, (34, 10), (64, 10), (0,255,0),3)
    cv2.line(img, (32, 37), (32, 43), (255,255,255),3)
    cv2.line(img, (0, 7), (0, 13), (0,255,0),3)
    cv2.line(img, (30, 7), (30, 13), (0,255,0),3)
    cv2.line(img, (34, 7), (34, 13), (0,255,0),3)
    cv2.line(img, (64, 7), (64, 13), (0,255,0),3)
    cv2.line(img, (left_deviation, 8), (left_deviation, 12), (0,0,255),3)
    cv2.line(img, (right_deviation, 8), (right_deviation, 12), (0,0,255),3)
    turn = int((right_deviation + left_deviation)/2)
    cv2.line(img, (32, 40), (turn, 40), (255,0,0),1)
    return turn

def toHSV(img):
     img_copy = img.copy()
     img_HSV = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
     return img_HSV

def getHsvMask(img):
     lowerWhite = np.array([0, 0, 221])
     upperWhite = np.array([180, 30, 255])
     maskWhite = cv2.inRange(img, lowerWhite, upperWhite)

     return maskWhite

def calTurn(img):
#     img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
#     kernel_size = 3
#     img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
#     low_threshold = 75
#     high_threshold = 150
#     img = cv2.Canny(img, low_threshold, high_threshold)
    img_HSV = toHSV(img)
    img = getHsvMask(img_HSV)
    
    left_deviation = 0
    for i in range(30,0,-1):
        if (img[10][i] == 255):
            left_deviation = i
            break
    
    right_deviation = 64
    for i in range(34,64):
        if (img[10][i] == 255):
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
    
    cap = cv2.VideoCapture(1)
    width = 48
    height = 64
    turn_list = []
    while True:
        ret, frame = cap.read()
        frame = cv2.resize(frame, (64, 48))
        # print(frame.shape[0], frame.shape[1])
        if ret:
            cv2.imshow('frame',frame)
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
                    if turn > 32:
                        msg = str(int(120 - map(turn))) + '&120\n'
                    elif turn < 32:
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
                    pass
        if cv2.waitKey(10) == ord('q'):
             break
    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
