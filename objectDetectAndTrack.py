import sys
import time

import cv2
import imutils
import numpy as np
from imutils.video import FPS, VideoStream

vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()
area_limit = 700
cv2.namedWindow('Tracking')
cv2.namedWindow('Threshold')
LOWER = [0, 105, 255]
UPPER = [179, 255, 255]
frame0 = 0

tracker = cv2.TrackerKCF_create()
tracked = False
bbox = 0
objectFound = False

def callback():
    LOWER[0] = cv2.getTrackbarPos('lowH', 'Threshold')
    UPPER[0] = cv2.getTrackbarPos('highH', 'Threshold')
    LOWER[1] = cv2.getTrackbarPos('lowS', 'Threshold')
    #upper[1] = cv2.getTrackbarPos('highS', 'Threshold')
    LOWER[2] = cv2.getTrackbarPos('lowV', 'Threshold')
    #upper[2] = cv2.getTrackbarPos('highV', 'Threshold')
    iterations = cv2.getTrackbarPos('FPS', 'Threshold')

def defineTrackbars():
    # create trackbars for color change
    cv2.createTrackbar('lowH','Threshold',LOWER[0],179,callback)
    cv2.createTrackbar('highH','Threshold',UPPER[0],179,callback)

    cv2.createTrackbar('lowS','Threshold',LOWER[1],255,callback)
    cv2.createTrackbar('highS','Threshold',UPPER[1],255,callback)

    cv2.createTrackbar('lowV','Threshold',LOWER[2],255,callback)
    cv2.createTrackbar('highV','Threshold',UPPER[2],255,callback)
    print("[INFO] Setting up Trackers...")



def trackContours(frame):
    contours,hierachy = cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnt = 0
    if len(contours) > 0:
        cnt = contours[0]
    #print(contours)
    M = cv2.moments(cnt)
    area = M['m00']
    coords = (0,0)
    dimensions = (0,0)
    if area > area_limit:
        print(M)
        x,y,w,h = cv2.boundingRect(cnt)
        coords = (x,y)
        print(x,y,w,h)
        dimensions = (w,h)
    return coords,dimensions
    
def processFrame(frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array(LOWER)
        upper_yellow = np.array(UPPER)
        frame = cv2.inRange(frame,lower_yellow,upper_yellow)
        cv2.imshow('Threshold',frame)
        kernel = np.ones((5,5),np.uint8)
        frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
        # update the FPS counter
        fps.update()
        key = cv2.waitKey(1) & 0xFF
        return frame

def isSuitableFrame(frame):
    frame = processFrame(frame)
    contours,hierachy = cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnt = 0
    if len(contours) > 0:
        cnt = contours[0]
    else:
        return False
    M = cv2.moments(cnt)
    area = M['m00']
    if area > area_limit:
        return True
    else:
        return False
    
def initTracker(view,point,size):
    bbox = (point[0],point[1],size[0],size[1])
    #print(bbox)
    tracked = tracker.init(view, bbox)


   
print("[INFO] Start video stream...")
defineTrackbars()
done = False
while not done:
    while not objectFound:
        frame = vs.read()
        frame = imutils.resize(frame, width=700)

        timer = cv2.getTickCount()
        cv2.imshow('Tracking',frame)
        objectFound = isSuitableFrame(frame)
        if objectFound:
            #print('good')
            frame0 = frame
            frame0 = processFrame(frame0)
            coords, dimension = trackContours(frame0)
            initTracker(frame0,coords,dimension)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
                cv2.destroyAllWindows()
                vs.stop()
                sys.exit()

    while True:

        frame = vs.read()
        frame = imutils.resize(frame, width=700)
        timer = cv2.getTickCount()
        frame0 = processFrame(frame)
        tracked, bbox = tracker.update(frame0)
        if tracked:
            
            #successfully tracked
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            cv2.putText(frame, "Tracking Object", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            
        else:
            cv2.putText(frame, "Tracking Object Failure", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            #print('Reset')
            objectFound = False
            break
        #print('test')
        cv2.imshow('Tracking',frame)
     
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
                break

        # update the FPS counter
        fps.update()
    print("exit")
# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
print("[INFO] Lower bound detected values: {}".format(LOWER))
print("[INFO] Upper bound detected values: {}".format(UPPER))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
sys.exit()
