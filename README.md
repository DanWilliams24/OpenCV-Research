# OpenCV-Research
This is an experimentation project for OpenCV object detection, recognition and tracking 

## Notes:

Constant Detection: ObjectTracking.py
  - Allows for an accurate object tracking, but it is quite inefficient due to detection calculations being run every frame.
  
Constant Tracking: ObjTrackTest.py
  - Program runs at a higher FPS, while maintaining a decent object detection. When the object currently being tracked is lost, the program generally fails to recover since it cannot detect where the object is on the frame. This makes this method unrealistic whenever the object leaves the camera's view.

Initial Detection and Subsequent Tracking: objectDetectAndTrack.py
  - This mix of the tracking/detection programs, conducts detection calculations until a proper frame with the object can be found via color thresholding. When the object has been successfully detected, detection calculations are stopped, and it is tracked using a tracking algorithm to run more efficiency frame to frame. Overall the best approach of all currently tested methods. 
