import cv2
import sys
import imutils
from imutils.video import VideoStream
from imutils.video import FPS
import time
import numpy as np
# Set up tracker.
# Instead of MIL, you can also use

tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
tracker_type = tracker_types[2]
fgbg = cv2.createBackgroundSubtractorMOG2()
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()

if tracker_type == 'BOOSTING':
    tracker = cv2.TrackerBoosting_create()
if tracker_type == 'MIL':
    tracker = cv2.TrackerMIL_create()
if tracker_type == 'KCF':
    tracker = cv2.TrackerKCF_create()
if tracker_type == 'TLD':
    tracker = cv2.TrackerTLD_create()
if tracker_type == 'MEDIANFLOW':
    tracker = cv2.TrackerMedianFlow_create()
if tracker_type == 'GOTURN':
    tracker = cv2.TrackerGOTURN_create()
if tracker_type == 'MOSSE':
    tracker = cv2.TrackerMOSSE_create()
if tracker_type == "CSRT":
    tracker = cv2.TrackerCSRT_create()

def getROI(frame):
    saliency = cv2.saliency.StaticSaliencyFineGrained_create()
    (success, saliencyMap) = saliency.computeSaliency(frame)
    
    return cv2.threshold(saliencyMap, 0, 255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

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
    


# Read first frame.
frame = vs.read()
frame = imutils.resize(frame, width=600)

# Define an initial bounding box


# Uncomment the line below to select a different bounding box
coords, dimensions = trackContours(getROI(frame))

bbox = (coords[0],coords[1],dimensions[0],dimensions[1])

# Initialize tracker with first frame and bounding box
ok = tracker.init(frame, bbox)

while True:
    # Read a new frame
    frame = vs.read()
    frame = imutils.resize(frame, width=600)
    saliency = getROI(frame)
    
    fgmask = fgbg.apply(frame)
    # Start timer
    timer = cv2.getTickCount()

    # Update tracker
    ok, bbox = tracker.update(getROI(frame))

    # Calculate Frames per second (FPS)
    #fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        cv2.putText(frame, "Admin", p1, cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),2)
        cv2.rectangle(fgmask, p1, p2, (255,0,0), 2, 1)
        cv2.putText(fgmask, "Admin", p1, cv2.FONT_HERSHEY_SIMPLEX, 0.75,(255,0,0),2)
    else :
        # Tracking failure
        cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # Display tracker type on frame
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);
 
    # Display FPS on frame
    #cv2.putText(frame, "FPS : " + str(fps.fps()), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

    # Display result
    cv2.imshow("Tracking", frame)
    cv2.imshow("Other", saliency)
    #cv2.imshow("Test", fgmask)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
            break

    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))


# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
