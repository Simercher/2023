import numpy as np
import sys
sys.path.append("/usr/local/lib/python3.8/site-packages")
import cv2
import serial

def drawpoint(img, points):
     picture = img
     for i in range(4):
          cv2.circle(picture, (int(points[i][0]), int(points[i][1])), 15, (0, 0, 255), cv2.FILLED)
     
     return picture

def passFunction(x):
     pass

def initializeTrackerBar(img):
     windowName = 'TrackerBar'
     cv2.namedWindow(windowName)
     cv2.resizeWindow(windowName, 640, 480)
     cv2.createTrackbar('Width Top', windowName, 200, img.shape[1]//2, passFunction)
     cv2.createTrackbar('Height Top', windowName, 10, img.shape[0], passFunction)
     cv2.createTrackbar('Width Bottom', windowName, 200, img.shape[1]//2, passFunction)
     cv2.createTrackbar('Height Bottom', windowName, 100, img.shape[0], passFunction)
     cv2.createTrackbar('HUE Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('HUE Max', windowName, 255, 255, passFunction)
     cv2.createTrackbar('SAT Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('SAT Max', windowName, 255, 255, passFunction)
     cv2.createTrackbar('VALUE Min', windowName, 0, 255, passFunction)
     cv2.createTrackbar('VALUE Max', windowName, 255, 255, passFunction)

def ValTrackers(img):
     windowName = 'TrackerBar'
     widthTop = 0#cv2.getTrackbarPos('Width Top', windowName) #0
     heightTop = 130#cv2.getTrackbarPos('Height Top', windowName)# 130
     widthBottom = 0#cv2.getTrackbarPos('Width Bottom', windowName) #0
     heightBottom = 250#cv2.getTrackbarPos('Height Bottom', windowName)#250

     points = np.float32([(widthBottom, heightBottom), (widthTop, heightTop), (img.shape[1] - widthTop, heightTop), (img.shape[1] - widthBottom, heightBottom)])
     return points

def getROI(img):
     height = img.shape[0]
     width = img.shape[1]
     triangle = np.array([ValTrackers(img)], dtype=np.int32)
     black_img = np.zeros_like(img)
     mask = cv2.fillPoly(black_img, triangle, 255)
     masked_img = cv2.bitwise_and(img, mask)
     return masked_img

def map(turn):
    return int((turn - 0) * (120 - 0) / (25 - 0))

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

def canny(img):
    # img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # kernel_size = 3
    # img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    low_threshold = 75
    high_threshold = 150
    img = cv2.Canny(img, low_threshold, high_threshold)
    return img

def getLane(frame):
    turn_list = []
    img_HSV = toHSV(frame.copy())
    img = getHsvMask(img_HSV)
    edges = canny(img.copy())
    img_roi = getROI(edges)
    cv2.imshow('roi', img_roi)
    lines = cv2.HoughLinesP(img_roi, 1, np.pi / 180, 30,minLineLength=20,maxLineGap=10)
    try:
        for line in lines:
            # newlines1 = lines[:, 0, :]
            # print ("line["+str(i-1)+"]=",line)
            if line is None:
                continue
            x1,y1,x2,y2 = line[0]  
            cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2) 

            x1 = float(x1)
            x2 = float(x2)
            y1 = float(y1)
            y2 = float(y2)
            # print ("x1=%s,x2=%s,y1=%s,y2=%s" % (x1, x2, y1, y2))
            if x2 - x1 == 0:
                result=90
            elif y2 - y1 == 0 :
                result=0
            else:
                k = -(y2 - y1) / (x2 - x1)
                result = np.arctan(k) * 57.29577
            turn_list.append(result)
    except Exception:
        pass
    cv2.imshow("line_detect",frame)
    try:
        return sum(turn_list) / len(turn_list)
    except Exception:
        return sum(turn_list) / (len(turn_list)+1)

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
    if not cap.isOpened():
          print("Cannot open camera")
          exit()
    ret, frame = cap.read()
    # initializeTrackerBar(frame)
    while True:
        ret, frame = cap.read()
        if ret:
            turn_list.append(getLane(frame))
            if len(turn_list) >= 6 :
                turn = sum(turn_list) / len(turn_list)
                turn_list.clear()
                if turn > 25:
                    turn = 25
                elif turn < -25:
                    turn = -25
                print(turn)
                try:
                    # Send a message to the Arduino
                    serial_port.open()
                    msg = '0 0'
                    if turn > 0:
                        msg = str(int(120 - map(turn))) + '&120\n'
                    elif turn < 0:
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
