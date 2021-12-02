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

#used to set the bounds values when the track bar is moved
def callback(x):
    lower[0] = cv2.getTrackbarPos('lowH', 'Threshold')
    upper[0] = cv2.getTrackbarPos('highH', 'Threshold')
    lower[1] = cv2.getTrackbarPos('lowS', 'Threshold')
    upper[1] = cv2.getTrackbarPos('highS', 'Threshold')
    lower[2] = cv2.getTrackbarPos('lowV', 'Threshold')
    upper[2] = cv2.getTrackbarPos('highV', 'Threshold')
    iterations = cv2.getTrackbarPos('FPS', 'Threshold')


#Defines the trackbars on init
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

#set up video stream and init a few variables
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()
BOUNDS = 100

#Create two named windows, one for the detector and one for the thresholded values
cv2.namedWindow('Detector')
cv2.namedWindow('Threshold')



lower = [15,126,82]
upper = [47,227,255]
tempBuff = []
#lower = [23,58,9] #green puzzle block
#upper = [100,255,211] #green puzzle block
buffer = []
defineTrackbars()

#Not used during program operation
def initTracker(view,point,size):
    bbox = (point[0],point[1],size[0],size[1])
    #print(bbox)
    tracked = tracker.init(view, bbox)
    #move to global scope when in use
    tracker = cv2.TrackerKCF_create()


def cannyROI(subFrame):
    pass

def isWithinBox(box1,box2):
    return box1.contains(box2.br()) & box1.contains(box2.tl())

def averageBoxes(box,box1):
    return Rect(int((box1.x + box.x)/2),int((box1.y + box.y)/2),int((box1.width + box.width)/2),int((box1.height + box.height)/2))
def trackContours(frame):
    _, contours,hierachy = cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnt = 0
    if len(contours) > 0:
        largeCNT = None
        maxArea = 0
        for i in range(len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            area = M['m00']
            if maxArea < area:
                maxArea = area
                largeCNT = cnt
        coords = (0,0)
        dimensions = (0,0)
        x,y,w,h = cv2.boundingRect(largeCNT)
        coords = (x,y)
        dimensions = (x+w,y+h)
        return coords,dimensions
    return None

def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)
            
def tryBoxFromBuffer(box):
    for i in range(len(buffer)):
        if isWithinBox(buffer[i],box) or isWithinBox(box,buffer[i]):
            box = averageBoxes(box,buffer[i])
            tempBuff.append(box)
            return box
    return box

count1 = 0
#Process frame and return it so that objects can be detected
def processFrame(frame):
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    #Step 1: RGB to HSV
 
    lower_yellow = np.array(lower)
    upper_yellow = np.array(upper)
    
    #frame = cv2.medianBlur ( frame,3,frame);
    #cv2.imshow('Blur', frame)
    #Step 2: Filter colors of HSV
    frame = cv2.inRange(frame,lower_yellow,upper_yellow)
    
    kernel = np.ones((5,5),np.uint8)
    frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
 
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

def switchToTracking(original):
    if(len(buffer) > 50):
        #possible breakpoint: If original is passed completely by value, then the new label wont show up on the screen
        cv2.putText(original, "Tracking with", boundingBox.tl(), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0), 2, cv2.LINE_AA)

while True:
    frame = vs.read()
    (h, w) = frame.shape[:2]
    frame = imutils.resize(frame, width=400)
    original = frame
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    frame = processFrame(frame)
    cv2.imshow('Threshold', frame)
    _, contours, hierarchy = cv2.findContours(frame.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        #frame = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        coords, dimensions = trackContours(frame)
        boundingBox = Rect(coords[0],coords[1],dimensions[0]-coords[0],dimensions[1]-coords[1])
        if boundingBox:
            boundingBox = tryBoxFromBuffer(boundingBox)
            original = cv2.rectangle(original,boundingBox.tl(),boundingBox.br(),(0,255,0),2)
            message = relativePos(frame,boundingBox)
            cv2.putText(original, message, boundingBox.tl(), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(original, str(boundingBox.width) + " x " + str(boundingBox.height), boundingBox.br(), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 255, 0), 2, cv2.LINE_AA)
            #do stuff with location
        buffer = tempBuff
        tempBuff = []
    original = imutils.resize(original, width=800)
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
