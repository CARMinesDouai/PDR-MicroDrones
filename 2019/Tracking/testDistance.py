# This is a script to evaluate the distance between two circles/forms on the kinect camera

import numpy as np
import argparse
import imutils
import cv2
import itertools
from math import atan2,degrees
import freenect
import time
#import matplotlib.pyplot as plt


def get_depth():
    return freenect.sync_get_depth()[0]


def get_video():
    return freenect.sync_get_video()[0][:, :, ::-1]  # RGB -> BGR


def getAngleOfLineBetweenTwoPoints(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    return degrees(atan2(yDiff, xDiff))#eventuellement -np.pi/2.

#returns the square of the AB distance
def distSquared(a,b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

if __name__=="__main__":
    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space
    greenLower = (51,69,100) #35, 32, 6
    greenUpper = (91,255,236)#85, 255, 255
    blueLower = (94,116,100)
    blueUpper = (131,255,255)

    resize_width=600
    #original is 640
    frame=frame=get_video()
    frame = imutils.resize(frame, width=resize_width)
    frame_width = resize_width
    frame_height= len(frame)
    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
    out = cv2.VideoWriter('output.avi',cv2.VideoWriter.fourcc('M','J','P','G'), 10, (frame_width,frame_width))
    # comme on va resize la width a 600, la heigth va suivre : ancienne_height*600/ancienne_width
while True:

    frame=get_video()
    depth_frame = get_depth()
    # resize the frame
    frame = imutils.resize(frame, width=resize_width)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, blueLower, blueUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    mask2 = cv2.inRange(hsv, greenLower, greenUpper)
    mask2 = cv2.erode(mask2, None, iterations=2)
    mask2 = cv2.dilate(mask2, None, iterations=2)

    p1, p2 = [0,0], [0,0]

    # Detect blue point and calculate its center
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]

    centroid = None
    centroidList=[]

    if len(cnts) == 1:
        ((x1, y1), r1) = cv2.minEnclosingCircle(cnts[0])
        p1 = [x1,y1]

    # Detect green point and calculate its center
    cnts = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]

    centroid = None
    centroidList=[]

    if len(cnts) == 1:
        ((x2, y2), r2) = cv2.minEnclosingCircle(cnts[0])
        p2 = [x2,y2]

    if p1!=[0,0] and p2!=[0,0]:
        dist = distSquared(p1,p2)

        print("distance : "+str(dist))
        
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("c"):
         exit_process()
