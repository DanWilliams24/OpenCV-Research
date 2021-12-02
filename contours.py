import numpy as np
import imutils
import cv2

import time
from imutils.video import VideoStream
from imutils.video import FPS

print("[INFO] Start video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()

cv2.namedWindow('Detector')

while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=600)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(frame,127,255,0)
    im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    frame = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    cv2.imshow("once and done", frame)
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
