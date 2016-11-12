import cv2
import numpy as np

def capture_frame(cap):
    while True:
        ret,img = cap.read()

        cv2.imshow("Press C to capture",img)

        if cv2.waitKey(1) & 0xFF == ord('c'):
            return img

cap = cv2.VideoCapture(1)
img = capture_frame(cap) # queryImage
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)

minLineLength = 100
maxLineGap = 10
lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
print(lines)
# for i in lines:
#     rho=i[0][0]
#     theta= i[0][1]
#     a = np.cos(theta)
#     b = np.sin(theta)
#     x0 = a*rho
#     y0 = b*rho
#     x1 = int(x0 + 1000*(-b))
#     y1 = int(y0 + 1000*(a))
#     x2 = int(x0 - 1000*(-b))
#     y2 = int(y0 - 1000*(a))
#
#     cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
for i in lines:
    x1 = i[0][0]
    y1 = i[0][1]
    x2 = i[0][2]
    y2 = i[0][3]
    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
cv2.imshow("Lines detected",img)
cv2.waitKey(0)
