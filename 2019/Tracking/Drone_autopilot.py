# This is the main script of the project.

### STEPS ###
# 1 - Run the script in a shell
# 2 - Turn on the drone. It should be connected to Arduino. If not, reset the arduino board
# 3 - Click in the pygame window and check if the drone and the target are detected by the kinect
# 4 - Press u to acquire the target then enter. The drone should try to go to the target step by step.


# import the necessary packages
import numpy as np
import argparse
import imutils
import cv2
import itertools
from math import atan2,degrees,radians
import freenect
import pygame, serial, time


def get_depth():
    return freenect.sync_get_depth()[0]


def get_video():
    return freenect.sync_get_video()[0][:, :, ::-1]  # RGB -> BGR

# Return the list of the 3 most aligned elements of centroidList
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

# return an angle between -180 degrees and 180 degrees where p1 is considered as origin and p2 turning around p1
def getAngleOfLineBetweenTwoPoints(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    return degrees(atan2(yDiff, xDiff))#eventuellement -np.pi/2.


# c est la liste des centres des cercles sur le drone
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

# Acquire the target on the screen
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
    
    # Initialisation of the frame
    resize_width=600 
    #original is 640
    frame=frame=get_video()
    frame = imutils.resize(frame, width=resize_width)
    frame_width = resize_width
    frame_height= len(frame)
    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
    out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_width))
    
    
    pygame.init()
    base_throttle=1000	# Throttle-> force d'elevation
    base_aileron=1500	# Aileron-> Penche le drone vers la gauche/droite pour aller a gauche/droite
    base_elevator=1492	# Elevator-> Penche le drone vers l'avant/arriere pour aller vers l'avant/arriere
    base_rudder=1500  	# Rudder-> Tourne le drone sur lui-meme

    throttle=base_throttle
    aileron=base_aileron
    elevator=base_elevator
    rudder=base_rudder
    
    # Constantes d'incrementation du throttle, aileron, elevator, rudder
    tg=10
    ag=10
    eg=10
    rg=5
    screen=pygame.display.set_mode((300,100))

    started=False

    # Au depart, la cible est au centre de l'image
    targetX=frame_width/2
    targetY=frame_height/2
    gotTarget=(targetX,targetY)

    try:
	    # Tentative de connection au drone via arduino (sinon on reset la carte)
        arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=.01)
        time.sleep(1) #give the connection a second to settle
        command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
        # string commands to the Arduino are prefaced with  [PC]           
        
        arduino.write(command+"\n")


	# keep looping until break
    	while True:
	     # On affiche les infos qu'a recu la carte Arduino
	     data = arduino.readline()
             if data:
                #String responses from Arduino Uno are prefaced with [AU]
                  print "[AU]: "+data 
             
             # Si on clique sur la croix on quitte tout
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
    
             # Construction de deux masques, un qui conserve le bleu, l'autre le vert
             mask = cv2.inRange(hsv, blueLower, blueUpper)
             mask = cv2.erode(mask, None, iterations=2)
             mask = cv2.dilate(mask, None, iterations=2)

             mask2 = cv2.inRange(hsv, greenLower, greenUpper)
             mask2 = cv2.erode(mask2, None, iterations=2)
             mask2 = cv2.dilate(mask2, None, iterations=2)
             
	     # On identifie la cible (cercle vert) avec le masque 2
             gotTarget=getTarget(mask2, frame)
                
             # On recupere la liste des contours obtenus par application du masque bleu
	     # Un contour est une liste de points formant une figure fermee
             cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    		      cv2.CHAIN_APPROX_SIMPLE)[-2]


	     #cv2.drawContours(frame,cnts,-1, (0,0,255),1)
             centroid = None
             centroidList=[]
        
             # On conserve les 4 plus gros contours
             if len(cnts) > 0:
                 cnts.sort(key=cv2.contourArea, reverse=True)
                 c=cnts[0:4]
    
                 for ci in c:
		     # Plus petit cercle contenant le contour
                     ((x, y), radius) = cv2.minEnclosingCircle(ci)
                     M = cv2.moments(ci)
                     #get the centroid (barycentre) of the shape
		     # Le centroide est le barycentre du contour (ici avec des cercles bleus c'est
		     # quasiment le centre du minEnclosingCircle
                     centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
    
            	       # only proceed if the radius meets a minimum size
                     if radius > 0:
		         #print(hsv[int(y)][int(x)])
                    
		         #draw the circle and centroid on the frame,
                         #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                         #cv2.circle(frame, centroid, 5, (0, 0, 255), -1)
                    
		         #add the centroid to the list
                         centroidList.append(centroid)
                   
                 # S'il y a ' centroides c'est qu'on voit le drone
                 if len(centroidList)==4:
                     aligned=getAligned(centroidList)
                     if(aligned!=None):    
                         (centroidList,maxDistSquared,angle)=getFigure(list(aligned),list(centroidList))
                         """   position in centroidlist :
                                 2   0   1
                                     3
                         """                            
                 

			 #change the commands of the drone in order to reach the target coordinates
			 #on essaye d'envoyer le drone sur la cible
			 if started:
		             
			     angle_center_target = getAngleOfLineBetweenTwoPoints(centroidList[0], [targetX, targetY])
			     delta_angle = angle-angle_center_target
			     distance = np.sqrt(distSquared(centroidList[0], [targetX, targetY]))
			     #on evalue l'orientation du drone par rapport a la cible (avec un angle et on fait tourner par a-coups le drone pour l'alligner avec la cible
			     if (abs(delta_angle)>13 and distance>75):
				    # le drone n'est pas aligne avec la cible, on prepare une rotation
				    # On a calcule la vitesse angulaire comme etant de w=1,07rad/s
				    # pour un rudder de 1550.
				    rudder = rudder - 50*np.sign(delta_angle)
				    throttle = 1200
				    elevator = 1500
				    command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
		 		    arduino.write(command+"\n")
				    #le drone decole et tourne dans le bon sens le temps du time.sleep
				    time.sleep(radians(abs(delta_angle))/1.7)

				    #on a aligne le drone, on le fais atterir.
				    throttle = 1000
				    rudder = 1500
				    elevator = 1500
				    command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)    
				    arduino.write(command+"\n")
				    #le drone fait une pause pendant le time.sleep, le temps de se poser avant de recommmencer
				    time.sleep(1)

			     # on passe dans le else si le drone est correctement alligne avec la cible.
			     # on va donc le faire avancer tout droit
			     else:
			         
				 if distance>75 :
				    # sleeptime est le temps durant lequel le drone avancera tout droit,
 				    # plus le drone est loin de la cible, plus sleeptime est grand.
				    # On multiplie par une constante (0,0017) qui depend des conditions
				    # de l'experience (la notion de distance est relative a la distance
				    # entre la camera kinect et le drone (distance est la distance en px
				    # qui separe les barycentres du drone et de la cible)
				    largeur_ecran = 570 #dans les conditions de notre experience en px
				    vitesse_drone = 22 #22 cm par seconde a la camera
				    longueur_piste = 85 #la longuer reele est de 85cm
				    #sleeptime = 0.0017*distance + 0.2 ancienne tehcnique
				    sleeptime = (longueur_piste/vitesse_drone)*(distance/largeur_ecran)/2 
				    print("sleeptime : "+str(sleeptime))
				    elevator = 1550
				    throttle = 1200
				    command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
		 		    arduino.write(command+"\n")

				    time.sleep(sleeptime)
				    #le drone a assez avance, on l'arrete et recommence les tests pour voir si il est effectivement arrive
				    throttle = 1000
				    elevator = 1500
				    command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)    
				    arduino.write(command+"\n")
				    time.sleep(1)

                         cv2.line(frame, centroidList[1], centroidList[2], (0,255,0),2) # BGR order
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
