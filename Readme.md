
# Autonomous Nano Drone

Drones use GPS to localise themselves in the surrounding environment. But GPS fails to work indoors hence it is limited to outdoor applications. This package uses an exterinsic sensor for localisation of drone. This is done using a previously implemented localisation technique "Whycon" which uses exterinsic sensors for data.

Using this package, a drone can be made to Autonomously navigate its environment which is as big as field view of camera. This package also makes the transmitter system obsolete. Emergency controls are available on Terminal GUI.

Drone can be armed/disarmed using the command prompt. The GUI also shows the feedback of important parameters like Battery Voltage, FC Commands, and Motor Values.

The Drone is supposed to have an onboard Flight Computer ( RPI Zero 2W ) for communicating with ground control system. The Flight Computer communicates with Flight Controller using Multi Wii Serial Protocol(MSP)

## Specs of Drone Used for Implementation

1. Kit: IFlight Protek25 Cinematic Drone
2. FC: Matek Sys F405
3. Firmware: INav

# Setting Up
1. Install ros (Tested on noetic)
2. Clone the contents of src folder and paste them to `<workspace>/src/` and then run catkin_make
3. Download the YAMSpy Folder and paste it into your RPI
4. Go to `src/scripts/mqtt.py` and edit the IP address and replace it by your RPI's IP
5. On Ground Control System:
    `roslaunch nano_drone main.launch`
6. SSH to RPI using `ssh pi@raspberrypi@local` or `ssh pi@<IP of RPI>`
7. On Pi, go to `YAMSpy/scripts/` run `GroundControl.py` "IN FULL SCREEN"
8. Use `a` to Arm, `d` to Disarm and `q` to Quit



### Steps to install YAMSpy

## In RPI:
1. Install pip3  --> sudo apt-get python3-pip
		     sudo apt-get update
		
2. Install YAMSPy --> sudo python3 setup.py install

## Other Dependencies

1. Install Mosquitto --> sudo apt-get install mosquitto mosquitto-clients
2. Install mqtt --> pip3 install paho-mqtt

## Facing Problems? 
Notion: https://www.notion.so/2640226478d842368cb303143df706fd?v=b8651a3620444e8395444f0a94484085
