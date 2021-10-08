from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2


class Buffer():
    def __init__(self, frame0):
        self.frames = [frame0]
        
    def addFrame(self,frame):
        self.frames.append(frame)


iterations = 0
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
print("[INFO] Start video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()

cv2.namedWindow('Detector')

lower = [3,127,77]
upper = [19,256,256]
defineTrackbars()


def autoFilterByROI(ROI):
    pass

def getROI(frame):
    saliency = cv2.StaticSaliencyFineGrained_create()
    (success, saliencyMap) = saliency.computeSaliency(frame)
    
    return cv2.threshold(saliencyMap,0,255,cv2.THRESHBINARY | cv2.THRESH_OTSU)[1]

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
        dimensions = (x+w,y+h)
        #print(coords)
        #print(dimensions)
    return coords,dimensions
    
    
count1 = 0
while True:
    if count1 == iterations:
        count1 = 0
        frame = vs.read()
     
        frame = imutils.resize(frame, width=900)
        original = frame
        #cv2.imshow("Original", frame)
        #Step 1: RGB to HSV
        frame = cv2.GaussianBlur(frame,(5,5),0)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
        #cv2.imshow("HSV", frame)
        lower_yellow = np.array(lower)
        upper_yellow = np.array(upper)
        #lower_yellow = np.array([20,100,100])
        #upper_yellow = np.array([100,255,255])
        #Step 2: Filter colors of HSV
        frame = cv2.inRange(frame,lower_yellow,upper_yellow)
        cv2.imshow('Threshold', frame)
        kernel = np.ones((5,5),np.uint8)
        frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)

        #ret,thresh = cv2.threshold(frame,127,255,0)
        #cv2.imshow('Thresh', thresh)
        _, contours, hierarchy = cv2.findContours(frame.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            #frame = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
            coords, dimensions = trackContours(frame)
            original = cv2.rectangle(original,coords,dimensions,(0,255,0),2)
            
        
        cv2.imshow('Detector',original)
        #cv2.imshow("Yellow Detector", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
                break

        # update the FPS counter
        fps.update()
    else:
        count1 +=1 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
print("[INFO] Lower bound detected values: {}".format(lower))
print("[INFO] Upper bound detected values: {}".format(upper))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop() 
