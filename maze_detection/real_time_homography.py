import numpy as np
import cv2
import math

MIN_MATCH_COUNT = 10
SAMPLES = 5
MAX_DELTA= 15
pitch_list = [0 for x in range(SAMPLES)]
roll_list = [0 for x in range(SAMPLES)]

def capture_frame(cap):
    while True:
        ret,img = cap.read()

        cv2.imshow("Press C to capture",img)

        if cv2.waitKey(1) & 0xFF == ord('c'):
            return img

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
img1 = capture_frame(cap) # queryImage
p=0
r=0
while True:
    ret2, img2 = cap.read() # trainImage

    # Initiate SURF detector

    #surf = cv2.xfeatures2d.SURF_create(5000)
    #SIFT may be better
    surf = cv2.xfeatures2d.SURF_create()

    # find the keypoints and descriptors with SURF
    kp1, des1 = surf.detectAndCompute(img1,None)

    kp2, des2 = surf.detectAndCompute(img2,None)


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

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

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

    #print(M)

    translation, rotation, scale, shear = getComponents(M)

    #print(translation,rotation, scale,shear)

    # yaw = rotation * (180/math.pi)
    # print("Yaw: {}".format(yaw))
    if(scale[0] > 2 or scale[0] < 0):
        pitch = None
    elif(scale[0]>=1):
        pitch = math.acos(2 - scale[0]) * (180/math.pi)
    else:
        #toward us is positive
        pitch = -1 * math.acos(scale[0]) * (180/math.pi)

    # if pitch:``
        # print("PiTcH", pitch)

    if(scale[1] > 2 or scale[1] < 0):
        roll = None
    elif(scale[1]>=1):
        roll = math.acos(2 - scale[1]) * (180/math.pi)
    else:
        #toward us is positive
        roll = -1 * math.acos(scale[1]) * (180/math.pi)

    # if roll:
    #     print("ROLl:",roll)

    if pitch and abs(pitch - np.mean(pitch_list)) > MAX_DELTA:
        pitch_list[p] = pitch
        p = (p+1) % SAMPLES
        # print("Pitch", np.mean(pitch_list))
    if roll and abs(roll - np.mean(roll_list)) > MAX_DELTA:
        roll_list[r] = roll
        r = (r+1) % SAMPLES
        print("Roll", np.mean(roll_list))

    #
    #     # print("Pitch: {}".format(pitch))
    # if(scale[1] < 1):
    #     #away from us is negative
    #     print(scale[1])
    #     roll = -1 * math.acos(scale[1]) * (180/math.pi)
    #     # print("Roll: {}".format(roll))
    # else:
    #     #toward us is positive
    #     print(scale[1])
    #     roll = math.acos(2 - scale[1]) * (180/math.pi)
    #     # print("Roll: {}".format(roll))

    # print("Pitch: {0:.2f} Roll: {1:.2f}".format(pitch,roll))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
