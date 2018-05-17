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
import freenect



def get_depth():
    return freenect.sync_get_depth()[0]


def get_video():
    return freenect.sync_get_video()[0][:, :, ::-1]  # RGB -> BGR


def getAligned(centroidList):
     min=2000
     cmin=None
     for c in itertools.permutations(centroidList,3):
          produit_croix=abs((c[2][1]-c[0][1])*(c[1][0]-c[0][0])-(c[2][0]-c[0][0])*(c[1][1]-c[0][1]))
	  if(produit_croix<min):
	       cmin=c
	       min=produit_croix
     return (cmin)


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
    return degrees(atan2(yDiff, xDiff))#eventuellement -np.pi/2.


#place the points in the right order : first the center and last the bottom point
#also returns the distance between the 2 other points 
#and the angle of the figure 
def getFigure(c, centroidList):
     # get the center point and the distance between the most far away points in c
     (centerIndex,maxDistSquared)=getCenterIndex(c)  
     
     #put the center at the beginning of the list c 
     c[0],c[centerIndex]=c[centerIndex],c[0]      

     for i in range(4):
         if centroidList[i] not in c:
              bottomIndex=i 
          
     #add the bottom point at the end of centroidList
     c.append(centroidList[bottomIndex])
     angle = getAngleOfLineBetweenTwoPoints(c[0],c[3])
     
     #put the right point at c[1], left at c[2]
     if(0<(c[3][0]-c[0][0])*(c[1][1]-c[0][1])-(c[3][1]-c[0][1])*(c[1][0]-c[0][0])):
          c[1],c[2]=c[2],c[1]
     
     return (c,maxDistSquared,angle)
      




                     

if __name__=="__main__":
    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space
    greenLower = (55, 50, 6)
	#pas restrictif 25 50 6
	#tres restrictif : 50 80 30
    greenUpper = (85, 255, 255)

    resize_width=900 #original is 640
    frame=frame=get_video()
    frame = imutils.resize(frame, width=resize_width)
    frame_width = resize_width


     # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
    out = cv2.VideoWriter('output.avi',cv2.cv.CV_FOURCC('M','J','P','G'), 10, (frame_width,frame_width))
    # comme on va resize la width a 600, la heigth va suivre : ancienne_height*600/ancienne_width
    
    
    # keep looping until break
    while True:
	frame=get_video()
	depth_frame = get_depth()
        # resize the frame
        frame = imutils.resize(frame, width=resize_width)

	#brouiller l'image ne change pas grand chose, a tester...
        #blurred = cv2.GaussianBlur(frame, (11, 11), 0)

	#convert it to the HSV color space
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
	#cv2.drawContours(frame,cnts,-1, (0,0,255),1)
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
                if radius > 1:
		    #print(hsv[int(y)][int(x)])
                    
		    #draw the circle and centroid on the frame,
                    #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                    #cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
                    
		    #add the centroid to the list
                    centroidList.append(centroid)
                   
                   
            if len(centroidList)==4:
                aligned=getAligned(centroidList)
                if(aligned!=None):    
                         (centroidList,maxDistSquared,angle)=getFigure(list(aligned),list(centroidList))
                         """   position in centroidlist :
                                 2   0   1
                                     3
                         """                            

 			 #get the depth of each centroid
			 depth_frame = imutils.resize(depth_frame, width=resize_width)
			 depths=[]
			 for i in centroidList:
			       depths.append(depth_frame[i[1]][i[0]])
			 
			 
			 #compute the angles of the drone
			 roulis=(int(depths[1])-int(depths[2]))
			 tangage=(int(depths[0])-int(depths[3]))*2
			 distance=int(depths[0])


                         #print all useful information
                         cv2.putText(frame,
                                'centre : '+str(centroidList[0]), 
                                (10,30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type

                         cv2.putText(frame,
                                'distance : '+str(depths[0]), 
                                (10,50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type
                         
                         cv2.putText(frame,
                                'roulis : '+str(roulis), 
                                (10,70), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type   

			 cv2.putText(frame,
                                'tangage : '+str(tangage), 
                                (10,90), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type
			 cv2.putText(frame,
                                'lacet : '+str(angle), 
                                (10,110), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type                    
                         
                         #cv2.line(frame, centroidList[1], centroidList[2], (0,255,0),2) # BGR order
                         cv2.circle(frame, centroidList[0], 2, ( 255,0,0 ), -1)#blue : center
			 cv2.circle(frame, centroidList[1], 2, ( 0,255,0 ), -1)#green : right
                         cv2.circle(frame, centroidList[3], 2, ( 0,0,255 ), -1)#Red : bottom

			 

                         
			
			       
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
    
    cv2.destroyAllWindows()
