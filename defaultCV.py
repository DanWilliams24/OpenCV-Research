'''from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2

vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()

while True:
    img = vs.read()
    img = cv2.GaussianBlur(img,(5,5),0)
    img = imutils.resize(img, width=800)
    plt.subplot(121),plt.imshow(img),plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(blur),plt.title('Blurred')
    plt.xticks([]), plt.yticks([])
    plt.show()
 
    
    
    
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
'''