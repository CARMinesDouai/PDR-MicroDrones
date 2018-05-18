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
import pygame, serial, time


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
      

def exit_process():
    # close the connection
    arduino.close()
    # re-open the serial port which will also reset the Arduino Uno an
    # this forces the quadcopter to power off when the radio loses conection. 
    arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=.01)  #We use the port COM6 and a data rate of 115200 bits per seconds
    arduino.close()
    # close it again so it can be reopened the next time it is run.  			       
    out.release()
    # cleanup the camera and close any open windows
    cv2.destroyAllWindows()
    pygame.display.quit
    pygame.quit()



def toTarget(x,y,base_aileron,base_elevator,centre):
    aileron=base_aileron
    elevator=base_elevator
    delta_aileron=x-centre[0]
    if (abs(delta_aileron)<50):
        aileron=base_aileron+np.sign(delta_aileron)*10
    elif (abs(delta_aileron)>100):

        aileron=base_aileron+np.sign(delta_aileron)*50
    elif (abs(delta_aileron)<100):
        aileron=base_aileron+delta_aileron/2
    print("da :"+str(delta_aileron))
    delta_elevator=y-centre[1]
    if (abs(delta_elevator)<50):
        elevator=base_elevator+np.sign(delta_elevator)*20
    elif (abs(delta_elevator)>100):
        elevator=base_elevator-delta_elevator/9
    elif (abs(delta_elevator)<100):
        elevator=base_elevator-delta_elevator/4
    print("de :"+str(delta_elevator))
    return(aileron,elevator,(delta_aileron+delta_elevator)/2)

def getTarget(mask2,frame):
    cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
    		      cv2.CHAIN_APPROX_SIMPLE)[-2]
    # only proceed if at least one contour was found
    if len(cnts2) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid (barycentre)
        cnts2.sort(key=cv2.contourArea, reverse=True)
        ci=cnts2[0]
        ((x, y), radius) = cv2.minEnclosingCircle(ci)
        M = cv2.moments(ci)
        #get the centroid (barycentre) of the shape
        centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 0:
	        #draw the circle on the frame,
                cv2.circle(frame, (int(x), int(y)), int(radius),(0,0, 255), 2)
        return(centroid[0],centroid[1])

                     

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
    out = cv2.VideoWriter('output.avi',cv2.cv.CV_FOURCC('M','J','P','G'), 10, (frame_width,frame_width))
    # comme on va resize la width a 600, la heigth va suivre : ancienne_height*600/ancienne_width
    
    
    pygame.init()
    base_throttle=1000
    base_aileron=1500
    base_elevator=1492
    base_rudder=1500  #yaw (lacet), rotates the drone

    throttle=base_throttle
    aileron=base_aileron
    elevator=base_elevator
    rudder=base_rudder

    tg=10
    ag=10
    eg=10
    rg=5
    screen=pygame.display.set_mode((300,100))

    started=False
    targetX=frame_width/2
    targetY=frame_height/2
    gotTarget=(targetX,targetY)

    try:
        arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=.01)
        time.sleep(1) #give the connection a second to settle
        command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
                 # string commands to the Arduino are prefaced with  [PC]           
        
        arduino.write(command+"\n")


	# keep looping until break
    	while True:
	     data = arduino.readline()
             if data:
                #String responses from Arduino Uno are prefaced with [AU]
                  print "[AU]: "+data 
        
             for event in pygame.event.get():
                  if event.type == pygame.QUIT:
                      exit_process()
	     
            	 # User pressed down on a key
                  if event.type == pygame.KEYDOWN:
                      if event.key == pygame.K_ESCAPE:
                          print "[PC]: ESC exiting"
                          exit_process()
                      elif event.key ==pygame.K_RETURN:
			  started=not started
			  print(started)
                      elif event.key ==pygame.K_t:
			  targetX,targetY=frame_width/4,frame_height/4
			  print("target")

                      elif event.key ==pygame.K_u:
			  targetX,targetY=gotTarget
			  print("target acquired")

                      elif event.key == pygame.K_z:
		          if throttle<1200:
                              throttle=1200
			  else:
			      throttle+=tg
                      elif event.key == pygame.K_s:
                          throttle-=tg

                      elif event.key == pygame.K_q:
                          rudder-=rg 
                      elif event.key == pygame.K_d:
                          rudder+=rg

                      
                      elif event.key== pygame.K_UP:
                          elevator+=eg
		      elif event.key== pygame.K_DOWN:
                          elevator-=eg

                      elif event.key== pygame.K_RIGHT:
                          aileron+=ag
                      elif event.key== pygame.K_LEFT:
                          aileron-=ag
                          
                      elif event.key== pygame.K_SPACE:  #Arret d'urgence
                          throttle=base_throttle
			  aileron=base_aileron
			  elevator=base_elevator
			  rudder=base_rudder
                  
                 


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
             mask = cv2.inRange(hsv, blueLower, blueUpper)
             mask = cv2.erode(mask, None, iterations=2)
             mask = cv2.dilate(mask, None, iterations=2)

             mask2 = cv2.inRange(hsv, greenLower, greenUpper)
             mask2 = cv2.erode(mask2, None, iterations=2)
             mask2 = cv2.dilate(mask2, None, iterations=2)
             
             gotTarget=getTarget(mask2, frame)
                
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
                     if radius > 0:
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
			

			 #set the target coordinates
			 target_angle = 60

			 #change the commands of the drone in order to reach the target coordinates
			 if started:
                             
                             
                             delta_angle=((angle+180-target_angle)%360)-180
			    
			     if (abs(delta_angle)<90):
				 rudder=base_rudder-delta_angle/2
			     elif(abs(delta_angle)>90):
			     	 rudder=base_rudder-delta_angle/3
			     else:
				 rudder=base_rudder-delta_angle/5
			     #print(delta_angle)
			     

                             if abs( delta_angle) < 20:
                                 (aileron,elevator,distance)=toTarget(targetX,targetY,base_aileron,base_elevator,centroidList[0])
                             if distance<2:
                                 throttle=1000
                             """delta_aileron=frame_width/2-centroidList[0][0]
			     if (abs(delta_aileron)>50):
			         aileron=base_aileron+delta_aileron/2
                             if (abs(delta_aileron)<50):
			         aileron=base_aileron+delta_aileron*3
			     print("da :"+str(delta_aileron))
                             
                             delta_elevator=frame_width/2-centroidList[0][1]
			     if (abs(delta_elevator)>50):
			         elevator=base_elevator-delta_elevator/10
                             if (abs(delta_elevator)<50):
			         elevator=base_elevator-delta_elevator/5
			     #print("de :"+str(delta_elevator))
"""





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
  
			 cv2.putText(frame,
                                "[PC]: "+command, 
                                (10,130), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type                    
                 	 cv2.putText(frame,
                                "[AU]: "+data, 
                                (10,150), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5,#font size
                                (0,0,0),
                                2)#line type                    
                         
                         #cv2.line(frame, centroidList[1], centroidList[2], (0,255,0),2) # BGR order
                         cv2.circle(frame, centroidList[0], 2, ( 255,0,0 ), -1)#blue : center
			 cv2.circle(frame, centroidList[1], 2, ( 0,255,0 ), -1)#green : right
                         cv2.circle(frame, centroidList[3], 2, ( 0,0,255 ), -1)#Red : bottom

                         cv2.circle(frame, (targetX, targetY), 5,(0,0, 255), 2)
		 else:
		     if(started):
		         print("hors de vue")
		         throttle=base_throttle
		         aileron=base_aileron
		         elevator=base_elevator
		         rudder=base_rudder

             if started:	 
	         command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
                 # string commands to the Arduino are prefaced with  [PC]           
                 arduino.write(command+"\n")


	     
        # write the frame to output video
             out.write(frame)                 
       			 
        # show the frame to our screen
             cv2.imshow("Frame", frame)
             #cv2.imshow("Frame", mask)
        
        
             key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
             if key == ord("c"):
    	          exit_process()
                         
			
    finally:
        # close the connection
         arduino.close()
        # re-open the serial port which will also reset the Arduino Uno and
        # this forces the quadcopter to power off when the radio loses conection. 
         arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=.01)  #We use the port COM6 and a data rate of 115200 bits per seconds
         arduino.close()
        # close it again so it can be reopened the next time it is run.  			       







    
    
    out.release()
    
    # cleanup the camera and close any open windows
    
    cv2.destroyAllWindows()
    pygame.display.quit
    pygame.quit()
