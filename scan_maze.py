import numpy as np
import cv2
import math

#video camera
cap = cv2.VideoCapture(1)

#constants
GRAVITY = 9.8
MIN_MATCH_COUNT = 10
SAMPLES = 10
MAX_DELTA= 15

def capture_frame(cap):
    '''
    Press c to capture a still frame from the live webcam feed
    '''
    while True:
        ret,img = cap.read()

        cv2.imshow("Press C to capture",img)

        if cv2.waitKey(1) & 0xFF == ord('c'):
            return img

def getComponents(normalised_homography):
    '''
    Denormalize homographical transform matrix into components.
    Takes normalized linear transformation (matrix[2][2]=1) and return components
    ((translation_x, translation_y), rotation, (scale_x, scale_y), shear)
    '''

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

def crop(img):
    '''
    Crop an image by clicking on the top left corner and dragging down to the bottom right corner
    '''

    # TODO: Make function return cropped image without using global variable to store image from callback func
    boxes = []
    def on_mouse(event, x, y, flags, params):


        #  TODO: Add simple logic to allow click to start from any of the four corners
        if event == cv2.EVENT_LBUTTONDOWN:
             print 'Start Mouse Position: '+str(x)+', '+str(y)
             sbox = [x, y]
             boxes.append(sbox)
             # print sbox

        elif event == cv2.EVENT_LBUTTONUP:
            print 'End Mouse Position: '+str(x)+', '+str(y)
            ebox = [x, y]
            boxes.append(ebox)
            crop = img[boxes[-2][1]:boxes[-1][1],boxes[-2][0]:boxes[-1][0]]

            cv2.imshow('crop',crop)
            res = cv2.resize(crop,(50, 50), interpolation = cv2.INTER_CUBIC)
            cv2.imshow('resized', res)
            # for row in range(len(res)):
            #     res[row] = [res[row][i] > 100 for i in range(len(res[row]))]
            global cropped_image
            cropped_image = res


    cv2.namedWindow('real image')
    cv2.setMouseCallback("real image", on_mouse,0)
    cv2.imshow('real image', img)

    while True:
        if cv2.waitKey(0) & 0xFF == ord("q"):
            return cropped_image

def fuzzy_mode(coords,delta):
    '''
    Returns the rough position from list of coordinates provided
    '''

    coords = extract_coords(coords,255)
    print(coords)
    largest_mode = 0
    mode_coord = []
    for coord in coords:
        mode = len(filter(lambda x: abs(x[0] - coord[0]) <= delta, coords))
        if(mode > largest_mode):
            mode_coord = coord
            largest_mode = mode
    return mode_coord

def convert_coord(x,y,width,height,new_width,new_height):
    return (math.floor(new_width*x/width),math.floor(new_height*y/height))

def extract_coords(mask,matching_color):
    print(mask)
    coords = []
    for i in range(len(mask)):
        for j in range(len(mask[0])):
            if(mask[i][j] == matching_color):
                coords.append((j,i))

    return coords

def average_point(mask):
    '''
    Returns the mean point from a set of coordinates
    '''
    coords = extract_coords(mask,255)
    print(coords)
    sum_x = sum_y = 0
    for coord in coords:
        sum_x += coord[0]
        sum_y += coord[1]

    return (sum_x/len(coords),sum_y/len(coords))  # TODO: Check that we have more than 0 coordinates


def get_mask(image,lower_bound,upper_bound):
    '''
    Returns a mask of the pixels with RGB values between the provided upper and lower bounds
    '''
    # create NumPy arrays from the boundaries
    lower_bound = np.array(lower_bound, dtype = "uint8")
    upper_bound = np.array(upper_bound, dtype = "uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(image, lower_bound, upper_bound)
    return mask

def get_maze(image):
    '''
    Returns a mask of the black lines found in the image
    '''
    #between (0,0,0) and (100,100,100) is black
    mask = get_mask(image,[0,0,0],[100,100,100])

    return mask

def get_start(image):
    '''
    returns the mean of the red coordinates as the start point for the ball
    '''
    #range for red
    mask = get_mask(image,[80, 120, 200], [100, 150, 255])
    return average_point(mask)

def get_end(image):
    '''
    returns the approximate blue region that is the end of the maze
    Note: this is not fully functional yet
    '''
    #range for blue
    mask = get_mask(image,[200, 180, 150], [255, 210, 190])
    return fuzzy_mode(mask,5)

def get_acceleration(image = None):
    '''
    Calculates the rolling mean of the tilt of the paper using homography and writes the output to a file.
    ompares the live camera feed with the static image argument.
    '''
    if(not image):
        image = start_image

    pitch_list = [0 for x in range(SAMPLES)]
    roll_list = [0 for x in range(SAMPLES)]
    p=0
    r=0

    ret2, img2 = cap.read() # comparison image

    cv2.imshow("Labyrinth",img2)
    cv2.waitKey(1)

    # Initiate SURF detector
    surf = cv2.xfeatures2d.SURF_create()

    # find the keypoints and descriptors with SURF
    kp1, des1 = surf.detectAndCompute(image,None)

    kp2, des2 = surf.detectAndCompute(img2,None)


    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)

    bf = cv2.BFMatcher()

    # Match the two sets of features
    matches = bf.knnMatch(des1,des2,k=2) #TODO: Once the bug with FLANN matching is fixed, switch to that

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    M = None
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

    else:
        print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
        matchesMask = None

    # Use trig to extrapolate pitch and roll from changes in scale
    if(M is not None and M.any()): # TODO: Check that this catches correctly
        translation, rotation, scale, shear = getComponents(M)

        if(scale[0] > 2 or scale[0] < 0):
            pitch = 0
        elif(scale[0]>=1):
            pitch = math.acos(2 - scale[0]) * (180/math.pi)
        else:
            #toward us is positive
            pitch = -1 * math.acos(scale[0]) * (180/math.pi)


        if(scale[1] > 2 or scale[1] < 0):
            roll = 0
        elif(scale[1]>=1):
            roll = math.acos(2 - scale[1]) * (180/math.pi)
        else:
            #toward us is positive
            roll = -1 * math.acos(scale[1]) * (180/math.pi)

        pitch_list[p] = pitch
        roll_list[r] = roll

        try:
            acceleration = (sum(roll_list)/SAMPLES ,sum(pitch_list)/SAMPLES)
        except Exception as e:
            acceleration = (0,0)
        print(roll,pitch)
        print(acceleration)

        f = open("input/acceleration.txt","w")
        f.write(str(acceleration))
        f.close()


def init():
    '''
    Takes in the initial image of the map, crops it, and finds the start point
    Walls and start point are output as files
    '''
    global start_image
    start_image = capture_frame(cap)

    global cropped_image
    cropped_image = crop(start_image)

    maze = get_maze(cropped_image)
    cv2.imshow("maze",cropped_image)
    cv2.waitKey(0)
    try:
        start_point = get_start(start_image)
    except:
        start_point = (10,10)
    height,width = len(start_image),len(start_image[0])
    start_point = convert_coord(start_point[0],start_point[1],width,height,50,50)
    print(start_point)

    f = open("input/start_point.txt","w")
    f.write(str(start_point))
    f.close()

    np.savetxt("input/maze.txt",maze)


if(__name__ == "__main__"):
    init()

    cv2.destroyAllWindows()
    cv2.waitKey(1)

    while True:
        get_acceleration()
