import numpy as np
import sys
sys.path.append("/usr/local/lib/python3.8/site-packages")
import cv2
import serial
import os
import time

def map(turn):
  if turn > 320:
    return ((turn - 320)/15)**2
  elif turn < 320:
    return -((turn - 320)/15)**2
  else:
    return 0
#def map(turn):
#    if turn - 320 >= 0:
#        return (turn - 320) / 320 * 70 
#    else:
#        return (turn - 320) / 320 * 70
    
def toHSV(img):
     img_copy = img.copy()
     img_HSV = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
     return img_HSV

def getHsvMask(img):
     lowerWhite = np.array([0, 0, 230])
     upperWhite = np.array([180, 30, 255])
     maskWhite = cv2.inRange(img, lowerWhite, upperWhite)

     return maskWhite

def main(img):
    #img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    #kernel_size = 3
    #img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    
    img = toHSV(img)
    img = getHsvMask(img)
    low_threshold = 100
    high_threshold = 150
    #img = cv2.Canny(img, low_threshold, high_threshold)

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
    
    #cv2.line(img, (int(left_deviation1), 230), (int(left_deviation1), 270), (0,0,255),3)
    #cv2.line(img, (int(right_deviation1), 230), (int(right_deviation1), 270), (0,0,255),3)

#    left_deviation2 = 0
#    for i in range(300,0,-1):
#        if (img[330][i] == 255):
#            left_deviation2 = i
#            break
    vertical_deviation = 0
    for i in range(300, 0, -1):
        if (img[i][320] == 255):
            vertical_deviation = i
            break

    #cv2.line(img, (int(left_deviation2), 280), (int(left_deviation2), 320), (0,0,255),3)
    #cv2.line(img, (int(right_deviation2), 280), (int(right_deviation2), 320), (0,0,255),3)

    #left_deviation /= 3
    #right_deviation /= 3
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    #cv2.line(img, (0, 200), (300, 200), (0,255,0),3)
    #cv2.line(img, (340, 200), (640, 200), (0,255,0),3)
    #cv2.line(img, (320, 370), (320, 430), (255,255,255),3)
    #cv2.line(img, (0, 70), (0, 130), (0,255,0),3)
    #cv2.line(img, (300, 70), (300, 130), (0,255,0),3)
    #cv2.line(img, (340, 70), (340, 130), (0,255,0),3)
    #cv2.line(img, (640, 70), (640, 130), (0,255,0),3)
    #cv2.line(img, (0, 310), (0, 350), (0,0,255),3)
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
    #if not a / 2  > 320 and b / 2 > 320:
    #    turn = int((a * 0 + b * 2)/1 / 2)
    #elif not a / 2 < 320 and b / 2 < 320:
    #    b -= 320
    #    turn = int((a * 0 + b * 2)/ 2)
    #else:
        #print(1)
    #    turn = int((a * 1 + b * 0)/1 / 2)
    #else:
    #    turn = 320
    turn = int((right_deviation + left_deviation) / 2)
    cv2.line(img, (320, 400), (turn, 400), (255,0,0),3)

    return img, map(turn), c

cap = cv2.VideoCapture(-1)
width = 480
height = 640

serial_port = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    )

clip = "001"
img_num = 1
pre_turn = 0
#os.system(f"rm -rf ./photo/{clip}")
#os.system(f"mkdir ./photo/{clip}")
start = end = time.time()
serial_port.write('170&170\n'.encode())
while end - start < 0.5:
    ret, frame = cap.read()
    if ret:
        frame1 = frame.copy()
        frame1, turn, c = main(frame1)
        print(c)
        cv2.imshow('video',frame1)
        try:
            # Send a message to the Arduino
            #print(turn)
            #serial_port.open()
            if turn >= 20:
                turn = 90
            elif turn <= -20:
                turn = -90
            img_num_str = str(img_num).zfill(5)
            #cv2.imwrite(f'./photo/{clip}/{img_num_str}_{round(turn, 2)}.jpg', frame)
            img_num += 1
            # msg = '0&0\n'
            # if turn < 0:	
            #     msg = str(int(170 + turn) + 0) + '&170\n'
            # elif turn > 0:
            #     msg = '170&' + str(int(170 - turn) + 0) + '\n'
            # else:
            #     msg = '170&170\n'
            # serial_port.write(msg.encode())
             #print(msg)
        except KeyboardInterrupt:
            print("Exiting Program")
        except Exception as exception_error:
            print("Error occurred. Exiting Program")
            print("Error: " + str(exception_error))
        finally:
            #serial_port.close()
            pass
    else:
        break
    if cv2.waitKey(10) == ord('q'):
        break
    end = time.time()
print(end - start)
serial_port.write('0&0\n'.encode())
cap.release()
cv2.destroyAllWindows()
