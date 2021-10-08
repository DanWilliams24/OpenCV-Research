import cv2
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
import socket
import sys

developerWindow = 'Developer Window'
lower = [0,0,0]
upper = [255,255,255]
def callback(x):
    lower[0] = int(cv2.getTrackbarPos('lowH', developerWindow))
    upper[0] = int(cv2.getTrackbarPos('highH', developerWindow))
    lower[1] = int(cv2.getTrackbarPos('lowS', developerWindow))
    upper[1] = int(cv2.getTrackbarPos('highS', developerWindow))
    lower[2] = int(cv2.getTrackbarPos('lowV', developerWindow))
    upper[2] = int(cv2.getTrackbarPos('highV', developerWindow))
    iterations = cv2.getTrackbarPos('FPS', developerWindow)

def formatOutput():
    lowerStr = str(lower[0]) + " " + str(lower[1]) + " " + str(lower[2])
    upperStr = str(upper[0]) + " " + str(upper[1]) + " " + str(upper[2])
    return (lowerStr, upperStr)
def defineTrackbars():
    # create trackbars for color change
    cv2.createTrackbar('lowH',developerWindow,lower[0],179,callback)
    cv2.createTrackbar('highH',developerWindow,upper[0],179,callback)

    cv2.createTrackbar('lowS',developerWindow,lower[1],255,callback)
    cv2.createTrackbar('highS',developerWindow,upper[1],255,callback)

    cv2.createTrackbar('lowV',developerWindow,lower[2],255,callback)
    cv2.createTrackbar('highV',developerWindow,upper[2],255,callback)
    print("[INFO] Setting up Trackers...")


img1 = cv2.imread('opencv-corner-detection-sample.jpg',1)
cv2.namedWindow(developerWindow)
defineTrackbars()

cv2.imshow(developerWindow,img1)
port = 6666
try: 
    s = socket.socket()
    print("Created Socket Successfully")
    s.bind(('',port ))
    print("Socket binded to %s" %(port))
    s.listen(5)
    print("Listening on port %s" %(port))
    while True:
        c, addr = s.accept()
        print("Client connected from:",addr)

        while True:
            #cv2.imshow(developerWindow,img1)
            fLow, fUp = formatOutput()
            #print(fLow,fUp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                c.send(fLow.encode())
                c.send(fUp.encode())
                print("Data Sent")
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                c.send("STOP".encode())
                c.close()
                break
            c.send("".encode())
        key = cv2.waitKey(1) & 0xFF
except socket.error as error:
    print("Problem occurred when connecting to the host")
    print(error)
    sys.exit()


cv2.waitKey(0)
cv2.destroyAllWindows()
