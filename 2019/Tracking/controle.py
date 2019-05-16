    # -*- coding: utf-8 -*-
import pygame, serial, time


THROTTLE_MID=1350
ELEVATOR_MID=1500
AILERON_MID=1500
RUDDER_MID=1500


def main():
    pygame.init()
    throttle=1200
    aileron=1500
    elevator=1500
    rudder=1500  #yaw, rotates the drone

    tg=200
    ag=1
    eg=10
    rg=50
    screen=pygame.display.set_mode((300,100))
    try:
        arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=.01)
        time.sleep(1) #give the connection a second to settle
        command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
                 # string commands to the Arduino are prefaced with  [PC]           
        print "[PC]: "+command 
        arduino.write(command+"\n")
        while True:
            data = arduino.readline()
            if data:
                #String responses from Arduino Uno are prefaced with [AU]
                print "[AU]: "+data 
        
            for event in pygame.event.get():
                 if event.type == pygame.QUIT:
                     return("fin")
    
     
            # User pressed down on a key
                 if event.type == pygame.KEYDOWN:
                     if event.key == pygame.K_ESCAPE:
                         print "[PC]: ESC exiting"
                         return()
                     elif event.key ==pygame.K_RETURN:
                         print "[PC]: Enter"
                     elif event.key == pygame.K_w:
                         throttle+=tg
                     elif event.key == pygame.K_a:
                         rudder-=rg         
                     elif event.key == pygame.K_s:
                         throttle-=tg
                     elif event.key == pygame.K_d:
                         rudder+=rg
                     elif event.key== pygame.K_DOWN:
                         elevator-=eg
                     elif event.key== pygame.K_UP:
                         elevator+=eg
                     elif event.key== pygame.K_RIGHT:
                         aileron+=ag
                     elif event.key== pygame.K_LEFT:
                         aileron-=ag
                         
                     elif event.key== pygame.K_SPACE:  #Arrêt d'urgence
                         throttle=1000
                         aileron=1000
                         elevator=1000
                         rudder=1000
                 command="%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
                 # string commands to the Arduino are prefaced with  [PC]           
                 print "[PC]: "+command 
                 arduino.write(command+"\n")
                 
                     
                         
    finally:
        # close the connection
        arduino.close()
        # re-open the serial port which will also reset the Arduino Uno and
        # this forces the quadcopter to power off when the radio loses conection. 
        arduino=serial.Serial('/dev/ttyACM0', 115200, timeout=.01)  #We use the port COM6 and a data rate of 115200 bits per seconds
        arduino.close()
        # close it again so it can be reopened the next time it is run.  
                            

main()
pygame.display.quit
pygame.quit()
