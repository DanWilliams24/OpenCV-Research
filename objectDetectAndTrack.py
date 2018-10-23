from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
import sys

vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()

#cv2.namedWindow('Detector')

lower = [22, 116, 255]
upper = [68, 256, 256]
frame0 = 0

tracker = cv2.TrackerCSRT_create()
tracked = False
bbox = 0
objectFound = False

def callback(x):
    lower[0] = cv2.getTrackbarPos('lowH', 'Detector')
    upper[0] = cv2.getTrackbarPos('highH', 'Detector')
    lower[1] = cv2.getTrackbarPos('lowS', 'Detector')
    #upper[1] = cv2.getTrackbarPos('highS', 'Detector')
    lower[2] = cv2.getTrackbarPos('lowV', 'Detector')
    #upper[2] = cv2.getTrackbarPos('highV', 'Detector')
    iterations = cv2.getTrackbarPos('FPS', 'Detector')

def defineTrackbars():
    # create trackbars for color change
    cv2.createTrackbar('lowH','Detector',lower[0],179,callback)
    cv2.createTrackbar('highH','Detector',upper[0],179,callback)

    cv2.createTrackbar('lowS','Detector',lower[1],255,callback)
    cv2.createTrackbar('highS','Detector',upper[1],255,callback)

    cv2.createTrackbar('lowV','Detector',lower[2],255,callback)
    cv2.createTrackbar('highV','Detector',upper[2],255,callback)
    cv2.createTrackbar('FPS','Detector',0,40,callback)
    print("[INFO] Setting up Trackers...")



def trackContours(frame):
    _, contours,hierachy = cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnt = 0
    if len(contours) > 0:
        cnt = contours[0]

    M = cv2.moments(cnt)
    area = M['m00']
    coords = (0,0)
    dimensions = (0,0)
    if area > 500:
        print(M)
        x,y,w,h = cv2.boundingRect(cnt)
        coords = (x,y)
        print(x,y,w,h)
        dimensions = (w,h)
    return coords,dimensions
    
def processFrame(frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array(lower)
        upper_yellow = np.array(upper)
        frame = cv2.inRange(frame,lower_yellow,upper_yellow)
        cv2.imshow('Threshold',frame)
        kernel = np.ones((5,5),np.uint8)
        frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
 

        return frame

def isSuitableFrame(frame):
    frame = processFrame(frame)
    _, contours,hierachy = cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnt = 0
    if len(contours) > 0:
        cnt = contours[0]
    else:
        return False
    M = cv2.moments(cnt)
    area = M['m00']
    if area > 500:
        return True
    else:
        return False
    
def initTracker(view,point,size):
    bbox = (point[0],point[1],size[0],size[1])
    print(bbox)
    tracked = tracker.init(view, bbox)


   
print("[INFO] Start video stream...")
#defineTrackbars()


while not objectFound:
    frame = vs.read()
    frame = imutils.resize(frame, width=600)
    cv2.imshow('Tracking',frame)
    objectFound = isSuitableFrame(frame)
    if(objectFound):
        print('good')
        frame0 = frame
        frame0 = processFrame(frame0)
        coords, dimension = trackContours(frame0)
        initTracker(frame,coords,dimension)

while True:

    frame = vs.read()
    frame = imutils.resize(frame, width=600)
    timer = cv2.getTickCount()
    
    tracked, bbox = tracker.update(frame)
    if tracked:
        
        #successfully tracked
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        
    else:
        cv2.putText(frame, "Tracking Object Failure", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

 
    cv2.imshow('Tracking',frame)
 
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
            break

    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
print("[INFO] Lower bound detected values: {}".format(lower))
print("[INFO] Upper bound detected values: {}".format(upper))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()

