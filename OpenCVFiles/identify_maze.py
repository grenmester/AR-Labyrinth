import numpy as np
import cv2

def get_dist(vert1,vert2):
    return ((vert1[0] + vert2[0])**2 + (vert1[1] + vert2[1])**2)**.5

def is_large_enough(vertices):
    #dist = get_dist(vertices[0][0],vertices[1][0])
    #print(dist)
    area = cv2.contourArea(vertices)
    print(area)
    return (MIN_SIZE < area < MAX_SIZE)

MIN_SIZE = 200
MAX_SIZE = 5000

#Initialize camera frame and display output of contours

cap = cv2.VideoCapture(1)

cnts = []

for i in range(100):
    ret, frame = cap.read()

    # Our operations on the frame come here
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(gray, 100, 200)

    img2, contours,h = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]

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
            cnts.append(cnt)
            print(approx)
            print("\n")
            cv2.imshow('frame',img)
        # elif len(approx) == 9:
        #     print "half-circle"
        #     cv2.drawContours(img,[cnt],0,(255,255,0),-1)
        # elif len(approx) > 15:
        #     print "circle"
        #     cv2.drawContours(img,[cnt],0,(0,255,255),-1)

    # When everything done, release the capture
    #cv2.waitKey(-1)
    cv2.imshow('frame',img)

for cnt in cnts:
    cv2.drawContours(img,[cnt],0,(0,0,255),-1)
cv2.imshow('frame',img)
cv2.waitKey(0)
cap.release()
cv2.destroyAllWindows()
