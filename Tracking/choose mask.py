import cv2
import numpy as np
import freenect

def get_video():
    return freenect.sync_get_video()[0][:, :, ::-1]  # RGB -> BGR




def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('result')

# Starting with 100's to prevent error while masking
h,s,v = 100,100,100

# Creating track bar
cv2.createTrackbar('hmin', 'result',0,179,nothing)
cv2.createTrackbar('smin', 'result',0,255,nothing)
cv2.createTrackbar('vmin', 'result',0,255,nothing)

cv2.createTrackbar('hmax', 'result',179,179,nothing)
cv2.createTrackbar('smax', 'result',255,255,nothing)
cv2.createTrackbar('vmax', 'result',255,255,nothing)


while(1):

    frame=frame=get_video()

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    hmin = cv2.getTrackbarPos('hmin','result')
    smin = cv2.getTrackbarPos('smin','result')
    vmin = cv2.getTrackbarPos('vmin','result')

    hmax = cv2.getTrackbarPos('hmax','result')
    smax = cv2.getTrackbarPos('smax','result')
    vmax = cv2.getTrackbarPos('vmax','result')

    # Normal masking algorithm
    lower_blue = np.array([hmin,smin,vmin])
    upper_blue = np.array([hmax,smax,vmax])

    mask = cv2.inRange(hsv,lower_blue, upper_blue)

    result = cv2.bitwise_and(frame,frame,mask = mask)

    cv2.imshow('result',result)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break



cv2.destroyAllWindows()
