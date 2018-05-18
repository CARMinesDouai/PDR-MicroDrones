# PDR-MicroDrones

## Set up
    Python 2.7 with openCV, libfreenect, and pygame
    Arduino 1.8.5
    Arduino Uno board
    Camera Kinect 1 with drivers installed


## Drones used
    JJRC H20 with color patches


## How to launch the drone controller
    open Arduino IDE
    sudo chmod a+rw /dev/ttyACM0
    upload the code to the arduino
    execute the file track_drone.py
    plug in the battery
    reset the arduino board
    click on the pygame window
    press Enter to start the autopilot. It is set to the center of the screen by default
    place a green patch on the ground and press u to set it as a target, and press enter
    You can control the drone, great job!
    



