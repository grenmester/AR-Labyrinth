import numpy as np
import cv2
import time,math

MIN_MATCH_COUNT = 10

def getComponents(normalised_homography):
  '''((translationx, translationy), rotation, (scalex, scaley), shear)'''
  a = normalised_homography[0,0]
  b = normalised_homography[0,1]
  c = normalised_homography[0,2]
  d = normalised_homography[1,0]
  e = normalised_homography[1,1]
  f = normalised_homography[1,2]

  p = math.sqrt(a*a + b*b)
  r = (a*e - b*d)/(p)
  q = (a*d+b*e)/(a*e - b*d)

  translation = (c,f)
  scale = (p,r)
  shear = q
  theta = math.atan2(b,a)

  return (translation, theta, scale, shear)

cap = cv2.VideoCapture(1)

ret1, img1 = cap.read() # queryImage
cv2.imshow("a",img1)
cv2.waitKey(0)
ret2, img2 = cap.read() # trainImage

# Initiate SURF detector

#surf = cv2.xfeatures2d.SURF_create(5000)
#SIFT may be better
surf = cv2.xfeatures2d.SURF_create()

# find the keypoints and descriptors with SURF
kp1, des1 = surf.detectAndCompute(img1,None)

start = time.time()
kp2, des2 = surf.detectAndCompute(img2,None)
end = time.time()

print("Time to detect second SURF")
print(end - start)


FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)

bf = cv2.BFMatcher()

matches = bf.knnMatch(des1,des2,k=2)

# store all the good matches as per Lowe's ratio test.
good = []
for m,n in matches:
    if m.distance < 0.7*n.distance:
        good.append(m)

if len(good)>MIN_MATCH_COUNT:
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    start = time.time()
    M, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC,5.0)
    end = time.time()

    print("Time to homografy")
    print(end - start)
    matchesMask = mask.ravel().tolist()

    # print(img1.shape)
    h,w,_ = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)



else:
    print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    matchesMask = None

draw_params = dict(matchColor = (0,255,0), # draw matches in green color
               singlePointColor = None,
               matchesMask = matchesMask, # draw only inliers
               flags = 2)


img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

cv2.imshow("frame",img3)
cv2.waitKey(0)

print(M)

translation, rotation, scale, shear = getComponents(M)

print(translation,rotation, scale,shear)

yaw = rotation * (180/math.pi)
pitch = math.acos(scale[0]) * (180/math.pi)
roll = math.acos(scale[1]) * (180/math.pi)

print("Pitch: {}".format(pitch))
print("Roll: {}".format(roll))
print("Yaw: {}".format(yaw))

cv2.waitKey(0)
