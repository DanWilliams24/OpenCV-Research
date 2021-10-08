from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2

#TODO create Rect Class
#TODO create buffer functions
#TODO create 
class Rect:
    def __init__(self,x=0,y=0,width=0,height=0):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def br(self):
        return (self.x+self.width,self.y+self.height)

    def tl(self):
        return (self.x,self.y)

    def contains(self,point):
        return (self.x <= point[0] and point[0] < self.x + self.width and self.y <= point[1] and point[1] < self.y +self.height)

    def getCenter(self):
        return (self.x + (self.width/2),self.y + (self.height/2))

    def area(self):
        return self.width*self.height


iterations = 0
def callback(x):
    lower[0] = cv2.getTrackbarPos('lowH', 'Threshold')
    upper[0] = cv2.getTrackbarPos('highH', 'Threshold')
    lower[1] = cv2.getTrackbarPos('lowS', 'Threshold')
    upper[1] = cv2.getTrackbarPos('highS', 'Threshold')
    lower[2] = cv2.getTrackbarPos('lowV', 'Threshold')
    upper[2] = cv2.getTrackbarPos('highV', 'Threshold')
    iterations = cv2.getTrackbarPos('FPS', 'Threshold')

def defineTrackbars():
    # create trackbars for color change
    cv2.createTrackbar('lowH','Threshold',lower[0],255,callback)
    cv2.createTrackbar('highH','Threshold',upper[0],255,callback)
    cv2.createTrackbar('lowS','Threshold',lower[1],255,callback)
    cv2.createTrackbar('highS','Threshold',upper[1],255,callback)

    cv2.createTrackbar('lowV','Threshold',lower[2],255,callback)
    cv2.createTrackbar('highV','Threshold',upper[2],255,callback)
    print("[INFO] Setting up Trackers...")
print("[INFO] Start video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()
BOUNDS = 100
cv2.namedWindow('Detector')
cv2.namedWindow('Threshold')
counter = 0
lower = [17,90,57]
upper = [60,255,255]
tempBuff = []
#lower = [23,58,9] #green puzzle block
#upper = [100,255,211] #green puzzle block
buffer = []
defineTrackbars()
def initTracker(view):
    bboxes = []
    for i in range(len(buffer)):
        bboxes[i] = (buffer[i].x,buffer[i].y,buffer[i].width,buffer[i].height)
        
    #print(bbox)
    tracked = tracker.init(view, bbox)

tracker = cv2.TrackerKCF_create()



def beginTrackingLoop(frame):
    initTracker(frame,)
def cannyROI(subFrame):
    pass

def isWithinBox(box1,box2):
    return box1.contains(box2.br()) & box1.contains(box2.tl())

def averageBoxes(box,box1):
    return Rect(int((box1.x + box.x)/2),int((box1.y + box.y)/2),int((box1.width + box.width)/2),int((box1.height + box.height)/2))
def trackContours(frame):
    _, contours,hierachy = cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnt = 0
    allCoords = []
    allDimensions = []
    if len(contours) > 0:
        cnt = contours[0]
    for i in range(len(contours)):
        cnt = contours[i]
        M = cv2.moments(cnt)
        area = M['m00']
        coords = (0,0)
        dimensions = (0,0)
        if area > 500:
            #print(M)
            x,y,w,h = cv2.boundingRect(cnt)
            coords = (x,y)
            dimensions = (x+w,y+h)
            #print(coords)
            #print(dimensions)
##            if(startTrack == 0):
##                startTrack += 1
##                initTracker(frame,coords,dimensions)
            allCoords.append(coords)
            allDimensions.append(dimensions)
    return allCoords,allDimensions
             
def tryBoxFromBuffer(box):
    for i in range(len(buffer)):
        if isWithinBox(buffer[i],box) or isWithinBox(box,buffer[i]):
            box = averageBoxes(box,buffer[i])
            tempBuff.append(box)
            return box
        
    return box

count1 = 0
#Start Concurrent Tracking
def processFrame(frame):
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    #Step 1: RGB to HSV
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array(lower)
    upper_yellow = np.array(upper)
    #cv2.imshow('No Median Blur', frame)
    #frame = cv2.medianBlur ( frame,3,frame);
    #cv2.imshow('Blur', frame)
    #Step 2: Filter colors of HSV
    frame = cv2.inRange(frame,lower_yellow,upper_yellow)
    
    kernel = np.ones((5,5),np.uint8)
    frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    cv2.imshow('Threshold', frame)
    return frame
def relativePos(frame,box):
    (h,w) = frame.shape[:2]
    message = ""
    x,y = box.getCenter()
    if x > (w/2)+BOUNDS:
        message = "RIGHT"
    elif x < (w/2)-BOUNDS: 
        message = "LEFT"
    else:
        message = "CENTER"
    return message
while True:
    frame = vs.read()
    (h, w) = frame.shape[:2]
    frame = imutils.resize(frame, width=800)
    original = frame
    if counter == 5:
        counter = 0
        beginTrackingLoop(frame)
        continue
    frame = processFrame(frame)
    _, contours, hierarchy = cv2.findContours(frame.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        #frame = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        coords, dimensions = trackContours(frame)
        for i in range(min(len(coords),len(dimensions))):
            x_coord = coords[i][0]
            y_coord = coords[i][1]
            x_dim = dimensions[i][0]
            y_dim = dimensions[i][1]
            boundingBox = Rect(x_coord,y_coord,x_dim-x_coord,y_dim-y_coord)
            if boundingBox:
                boundingBox = tryBoxFromBuffer(boundingBox)
                #print(boundingBox.br(), boundingBox.tl())
                original = cv2.rectangle(original,boundingBox.tl(),boundingBox.br(),(0,255,0),2)
                message = relativePos(frame,boundingBox)
                cv2.putText(original, message, boundingBox.tl(), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(original, str(boundingBox.width) + " x " + str(boundingBox.height), boundingBox.br(), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0), 2, cv2.LINE_AA)
                #do stuff with location
        
        buffer = tempBuff
        tempBuff = []
        if buffer:
            counter += 1
    cv2.imshow('Detector',original)
    #cv2.imshow("Yellow Detector", frame)
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
