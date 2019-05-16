# PDR-MicroDrones

## Set up
    Python 2.7 with openCV, libfreenect, and pygame
    Arduino 1.8.5
    Arduino Uno board
    nRF24 transmitter/receiver
    Camera Kinect 1 with drivers installed


## Drones used
    JJRC H20 with 4 color patches (blue patches file can be found in Images directory)


## How to launch the drone controller
    open Arduino IDE
    sudo chmod a+rw /dev/ttyACM0
    upload the code to the arduino
    execute the file Drone_autopilot.py
    turn on the drone. It should connect to the arduino board. If not, please reset the arduino board
    click on the pygame window
    place a green patch on the ground and press u to set it as a target, and press enter
    the drone goes to the target, great job!
    



