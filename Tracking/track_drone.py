# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
import numpy as np
import argparse
import imutils
import cv2
import itertools
from math import atan2,degrees



def areAligned(l):
     produit_croix=abs((c[2][1]-c[0][1])*(c[1][0]-c[0][0])-(c[2][0]-c[0][0])*(c[1][1]-c[0][1]))
     return (produit_croix<2000)


#returns the square of the AB distance
def distSquared(a,b):
    return (a[0] - b[0])**2 + (a[1] - b[1])**2


#returns the index of the point which is between the 2 others
def getCenterIndex(l):
    d01 = distSquared(l[0],l[1])
    d02 = distSquared(l[0],l[2])
    d12 = distSquared(l[1],l[2])
    if d01>d02 and d01>d12:
        return (2,d01)
    elif d02>d01 and d02>d12:
        return (1,d02)
    else:
        return (0,d12)


def getAngleOfLineBetweenTwoPoints(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    return degrees(atan2(yDiff, xDiff)-np.pi/2.)#probleme : va de -270 a +90


#place the points in the right order : first the center and last the bottom point
#also returns the distance between the 2 other points 
#and the angle of the figure 
def getFigure(c, centroidList):
     # get the center point and the distance between the most far away points in c
     (centerIndex,maxDistSquared)=getCenterIndex(c)  
     
     #put the center at the beginning of the list c 
     c[0],c[centerIndex]=c[centerIndex],c[0]  
     
     """
     Focal length F:
     F = (Perceived lenght in px * Height in cm) / real Length in cm 
     attention notre distance est au carré !
     
     pour calculer ensuite : 
     H = F * L / P
     """
    
     for i in range(4):
         if centroidList[i] not in c:
              bottomIndex=i 
          
     #add the bottom point at the end of centroidList
     c.append(centroidList[bottomIndex])
     angle = getAngleOfLineBetweenTwoPoints(c[0],c[3])
     return (c,maxDistSquared,angle)
      




                     

if __name__=="__main__":
    
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
    	help="path to the (optional) video file")

    args = vars(ap.parse_args())
    
    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space
    greenLower = (25, 70, 6)#(29, 86, 6)
    greenUpper = (80, 255, 255)#(64, 255, 255)

    
    # if a video path was not supplied, grab the reference
    # to the webcam
    if not args.get("video", False):
        camera = cv2.VideoCapture(0)
        frame_width = int(camera.get(3))
        frame_height = int(camera.get(4))
    
    # otherwise, grab a reference to the video file
    else:
        camera = cv2.VideoCapture(args["video"])
     
    # Check if camera opened successfully
    if (camera.isOpened() == False): 
        print("Unable to read camera feed")
     
    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
    out = cv2.VideoWriter('output.avi',cv2.cv.CV_FOURCC('M','J','P','G'), 10, (600,600*frame_height/frame_width))
    # comme on va resize la width à 600, la heigth va suivre : ancienne_height*600/ancienne_width
    
    
    # keep looping until break
    while True:
        # grab the current frame
        (grabbed, frame) = camera.read()
    
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if args.get("video") and not grabbed:
            print("break")
            break

        # resize the frame, blur it, and convert it to the HSV color space
        """                
        utile??????????????????
        """
        frame = imutils.resize(frame, width=600)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
    
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    		 cv2.CHAIN_APPROX_SIMPLE)[-2]
        centroid = None
        centroidList=[]
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid (barycentre)
            cnts.sort(key=cv2.contourArea, reverse=True)
            c=cnts[0:4]
    
            for ci in c:
                ((x, y), radius) = cv2.minEnclosingCircle(ci)
                M = cv2.moments(ci)
                #get the centroid (barycentre) of the shape
                centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
    
            	  # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    #cv2.circle(mask, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    #cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
                    #add the centroid to the list
                    centroidList.append(centroid)
                   
                   
            if len(centroidList)==4:
                for c in itertools.permutations(centroidList,3):
                    if areAligned(c):
                         (centroidList,maxDistSquared,angle)=getFigure(list(c),list(centroidList))
                         """   position in centroidlist :
                                 ?   0   ?
                                     3
                         """   
                         
                         #print all useful information
                         cv2.putText(frame,
                                'taille : '+str(maxDistSquared), 
                                (10,50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type
                         
                         cv2.putText(frame,
                                'angle : '+str(angle), 
                                (10,70), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type                     
                         
                         cv2.line(frame, centroidList[0], centroidList[2], (0,255,0),2) # BGR order
                         cv2.circle(frame, centroidList[0], 5, ( 255, 0,0), -1)#bleu foncé
                         cv2.circle(frame, centroidList[3], 5, ( 220, 200,0), -1)#bleu clair
                         """
                         cv2.line(mask, centroidList[0], centroidList[2], (0,255,0),2) # BGR order
                         cv2.circle(mask, centroidList[0], 5, ( 255, 0,0), -1)#bleu foncé
                         cv2.circle(mask, centroidList[3], 5, ( 220, 200,100), -1)#bleu clair
                         """
                         #break?????
                        
                        
                        
        # write the frame to output video
        out.write(frame)
        # show the frame to our screen
        cv2.imshow("Frame", frame)
        #cv2.imshow("Frame", mask)
        
        
        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
    		break
    
    out.release()
    
    # cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows()