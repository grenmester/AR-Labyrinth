import numpy as np
import cv2

def capture_frame(cap):
    while True:
        ret,img = cap.read()

        cv2.imshow("Press C to capture",img)

        if cv2.waitKey(1) & 0xFF == ord('c'):
            return img

# initialize video camera and take color image
cap = cv2.VideoCapture(1)

image = capture_frame(cap)

# cv2.imshow('frame',img)

# if cv2.waitKey(1) & 0xFF == ord('q'):
#     break



# define the list of boundaries
boundaries = [([0,0,0],[100,100,100])
 #([80, 120, 200], [100, 150, 255]) # boundary for red
#([200, 180, 150], [255, 210, 190]) # boundary for blue
 #([25, 146, 190], [62, 174, 250])
#([50, 0, 0], [200, 100, 100])
]

# loop over the boundaries
for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask = mask)

    # show the images
cv2.imshow("mask",mask)

cv2.destroyAllWindows()

boxes = []
def on_mouse(event, x, y, flags, params):

    if event == cv2.EVENT_LBUTTONDOWN:
         print 'Start Mouse Position: '+str(x)+', '+str(y)
         sbox = [x, y]
         boxes.append(sbox)
         # print sbox

    elif event == cv2.EVENT_LBUTTONUP:
        print 'End Mouse Position: '+str(x)+', '+str(y)
        ebox = [x, y]
        boxes.append(ebox)
        print boxes
        crop = img[boxes[-2][1]:boxes[-1][1],boxes[-2][0]:boxes[-1][0]]

        cv2.imshow('crop',crop)
        res = cv2.resize(crop,(50, 50), interpolation = cv2.INTER_CUBIC)
        print(res)
        cv2.imshow('resized', res)
        for row in range(len(res)):
            res[row] = [res[row][i] > 100 for i in range(len(res[row]))]
        print(res)

        np.savetxt("maze.txt",res)

        cv2.waitKey(0)

img = mask
# img = cv2.blur(img, (3,3))
# img = cv2.resize(img, None, fx = 0.25,fy = 0.25)

cv2.namedWindow('real image')
cv2.setMouseCallback("real image", on_mouse,0)
cv2.imshow('real image', img)
cv2.waitKey(0)
