import numpy as np
import cv2

MIN_SIZE = 500
MAX_SIZE = 10000

#Initialize camera frame and display output of contours

cap = cv2.VideoCapture(1)
ret, frame = cap.read()

# Our operations on the frame come here
img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(img,127,255,1)

img2, contours,h = cv2.findContours(thresh,1,2)

cv2.imshow('frame',img2)


def get_dist(vert1,vert2):
    return ((vert1[0] + vert2[0])**2 + (vert1[1] + vert2[1])**2)**.5

def is_large_enough(vertices):
    #dist = get_dist(vertices[0][0],vertices[1][0])
    #print(dist)
    area = cv2.contourArea(vertices)
    print(area)
    return (MIN_SIZE < area < MAX_SIZE)

wait_time = 1

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(img,127,255,1)

    img2, contours,h = cv2.findContours(thresh,1,2)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        #print len(approx)
        # if len(approx)==5:
        #     print "pentagon"
        #     cv2.drawContours(img,[cnt],0,255,-1)
        # elif len(approx)==3:
        #     print "triangle"
        #     cv2.drawContours(img,[cnt],0,(0,255,0),-1)
        if (len(approx)==4) and is_large_enough(approx):
            cv2.drawContours(img,[cnt],0,(0,0,255),-1)
            print(get_dist(approx[0][0],approx[1][0]))
            cv2.imshow('frame',img)
        # elif len(approx) == 9:
        #     print "half-circle"
        #     cv2.drawContours(img,[cnt],0,(255,255,0),-1)
        # elif len(approx) > 15:
        #     print "circle"
        #     cv2.drawContours(img,[cnt],0,(0,255,255),-1)

    # Display the resulting frame
    key = cv2.waitKey(wait_time)
    if key & 0xFF == ord('q'):
        break
    if key & 0xFF == ord('p'):
        if(wait_time == 0):
            wait_time = 1
        else:
            wait_time = 0
    print("\n")

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
