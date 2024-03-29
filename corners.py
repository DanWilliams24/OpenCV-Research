import cv2
import numpy as np
from matplotlib import pyplot as plt
imgTest = cv2.imread('blackandwhite.jpg',0)
img = cv2.imread('blackandwhite.jpg',0)
img = np.float32(img)
corners = cv2.cornerHarris(img,2,3,0.04)
 
corners = cv2.cornerHarris(img,2,3,0.04)


plt.subplot(1), plt.imshow(imgTest ,cmap = 'jet')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])

plt.subplot(2,2,1), plt.imshow(corners ,cmap = 'jet')
plt.title('Harris Corner Detection'), plt.xticks([]), plt.yticks([])
 
img2 = cv2.imread('blackandwhite.jpg')
corners2 = cv2.dilate(corners, None, iterations=3)
img2[corners2 > 0.01*corners2.max()] = [255,0,0]
 
plt.subplot(2,2,2),plt.imshow(img2,cmap = 'gray')
plt.title('Canny Edge Detection'), plt.xticks([]), plt.yticks([])
 
plt.show()
