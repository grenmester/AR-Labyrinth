import cv2
import numpy as np

# initialize video camera and take color image
cap = cv2.VideoCapture(1)

ret1, img1 = cap.read()

surf1 = cv2.xfeatures2d.SURF_create(400)

# We set it to some 50000. Remember, it is just for representing in picture.
# In actual cases, it is better to have a value 300-500
#surf.hessianThreshold = 50000

# Again compute keypoints and check its number.
kp1, des1 = surf1.detectAndCompute(img1,None)

img1_kp = cv2.drawKeypoints(img1,kp1,None,(255,0,0),4)

cv2.imshow('frame', img1_kp)

cv2.waitKey(0)

#take second point

ret2, img2 = cap.read()

surf2 = cv2.xfeatures2d.SURF_create(400)

# We set it to some 50000. Remember, it is just for representing in picture.
# In actual cases, it is better to have a value 300-500
#surf.hessianThreshold = 50000

# Again compute keypoints and check its number.
kp2, des2 = surf2.detectAndCompute(img2,None)

img2_kp = cv2.drawKeypoints(img2,kp2,None,(255,0,0),4)

cv2.imshow('frame',img2_kp)

#calculate homography
kp1 = map(lambda x: x.pt, kp1)
kp2 = map(lambda x: x.pt, kp2)

print(kp1,kp2)
h, status = cv2.findHomography(np.array(kp1), np.array(kp2))

# Warp source image to destination based on homography
im_out = cv2.warpPerspective(img1, h, (img1.shape[1],img2.shape[0]))

cv2.imshow("Source Image", im1)
cv2.imshow("Destination Image", im2)
cv2.imshow("Warped Source Image", im_out)

cv2.waitKey(0)
