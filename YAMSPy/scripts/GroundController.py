"""simpleUI.py: Simple UI (toy one really) to test YAMSPy using a SSH connection.

Copyright (C) 2020 Ricardo de Azambuja

This file is part of YAMSPy.

YAMSPy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

YAMSPy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with YAMSPy.  If not, see <https://www.gnu.org/licenses/>.


WARNING:
This UI is not fit for flying your drone, it is only useful for testing stuff in a safe place, ok?

Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).

Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.


TODO:
1) The integrator makes it too hard to control things (it winds up). Probably a non-linear thing would be helpful here...
2) Everything is far from optimal so it could be improved... but it's a simpleUI example after all ;)
"""

__author__ = "Ricardo de Azambuja"
__copyright__ = "Copyright 2020, MISTLab.ca"
__credits__ = [""]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Ricardo de Azambuja"
__email__ = "ricardo.azambuja@gmail.com"
__status__ = "Development"


import time
import curses
from collections import deque
from itertools import cycle
# from torch import roll

from yamspy import MSPy

import datetime
from yamspy import MSPy
import threading
import paho.mqtt.client as mqtt 
import paho.mqtt.publish as publish#import library
# from mqtt_client import MQTT, ALTITUDE

# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10
throttle = 0
roll_ = 0
pitch = 0
ALTITUDE = 0
x = 0
y = 0
tem_throttle = 0
pid = [[0],[0],[0]]
error = [0]*3
ARMED = False
PID_t = [0,0,0]
PID_r = [0,0,0]
PID_p = [0,0,0]
gains = [[0,0,0],[0,0,0],[0,0,0]]

detected = -1

#
# On Linux, your serial port will probably be something like
# /dev/ttyACM0 or /dev/ttyS0 or the same names with numbers different from 0
#
# On Windows, I would expect it to be 
# COM1 or COM2 or COM3...
#
# This library uses pyserial, so if you have more questions try to check its docs:
# https://pyserial.readthedocs.io/en/latest/shortintro.html
#
#
SERIAL_PORT = "/dev/serial0"

CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

# This order is the important bit: it will depend on how your flight controller is configured.
# Below it is considering the flight controller is set to use AETR.
# The names here don't really matter, they just need to match what is used for the CMDS dictionary.
# In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes AUX1, AUX2...
CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

def run_curses(external_function):
    result=1

    try:
        # get the curses screen window
        screen = curses.initscr()

        # turn off input echoing
        curses.noecho()

        # respond to keys immediately (don't wait for enter)
        curses.cbreak()

        # non-blocking
        screen.timeout(0)

        # map arrow keys to special values
        screen.keypad(True)

        screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'm' to change mode, 'a' to arm, 'd' to disarm and arrow keys to control", curses.A_BOLD)
        
        result = external_function(screen)

    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result==1:
            print("An error occurred... probably the serial port is not available ;)")

def keyboard_controller(screen):


    global CMDS, CMDS_ORDER, ARMED
    # "print" doesn't work with curses, use addstr instead
    controller = Controller()
    try:
        screen.addstr(15, 0, "Connecting to the FC...")

        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1

            screen.addstr(15, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()
            screen.move(1,0)

            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            if board.INAV:
                command_list.append('MSPV2_INAV_ANALOG')
                command_list.append('MSP_VOLTAGE_METER_CONFIG')

            for msg in command_list: 
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            if board.INAV:
                cellCount = board.BATTERY_STATE['cellCount']
            else:
                cellCount = 0 # MSPV2_INAV_ANALOG is necessary
            min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

            screen.addstr(15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
            screen.clrtoeol()
            screen.addstr(15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
            screen.addstr(16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
            screen.addstr(16, 50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
            screen.addstr(17, 0, "boardName: {}".format(board.CONFIG['boardName']))
            screen.addstr(17, 50, "name: {}".format(board.CONFIG['name']))


            slow_msgs = cycle(["MSP_ATTITUDE",'MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            while True:
                # publish.single("plot", error, hostname="192.168.43.226")
                controller.pid(float(ALTITUDE))
                start_time = time.time()

                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer
                

                #
                # Key input processing
                #

                #
                # KEYS (NO DELAYS)
                #
                if char == ord('q') or char == ord('Q'):
                    break

                elif char == ord('d') or char == ord('D'):
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['aux1'] = 1000

                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    time.sleep(0.5)
                    break

                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux1'] = 1800

                #
                # The code below is expecting the drone to have the
                # modes set accordingly since everything is hardcoded.
                #
                elif char == ord('m') or char == ord('M'):
                    if CMDS['aux2'] <= 1300:
                        cursor_msg = 'Horizon Mode...'
                        CMDS['aux2'] = 1500
                    elif 1700 > CMDS['aux2'] > 1300:
                        cursor_msg = 'Flip Mode...'
                        CMDS['aux2'] = 2000
                    elif CMDS['aux2'] >= 1700:
                        cursor_msg = 'Angle Mode...'
                        CMDS['aux2'] = 1000

                elif char == ord('w') or char == ord('W'):
                    CMDS['throttle'] = CMDS['throttle'] + 10 if CMDS['throttle'] + 10 <= 2000 else CMDS['throttle']
                    cursor_msg = 'W Key - throttle(+):{}'.format(CMDS['throttle'])

                elif char == ord('e') or char == ord('E'):
                    CMDS['throttle'] = CMDS['throttle'] - 10 if CMDS['throttle'] - 10 >= 1000 else CMDS['throttle']
                    cursor_msg = 'E Key - throttle(-):{}'.format(CMDS['throttle'])

                elif char == curses.KEY_RIGHT:
                    CMDS['roll'] = CMDS['roll'] + 10 if CMDS['roll'] + 10 <= 2000 else CMDS['roll']
                    cursor_msg = 'Right Key - roll(-):{}'.format(CMDS['roll'])

                elif char == curses.KEY_LEFT:
                    CMDS['roll'] = CMDS['roll'] - 10 if CMDS['roll'] - 10 >= 1000 else CMDS['roll']
                    cursor_msg = 'Left Key - roll(+):{}'.format(CMDS['roll'])

                elif char == curses.KEY_UP:
                    CMDS['pitch'] = CMDS['pitch'] + 10 if CMDS['pitch'] + 10 <= 2000 else CMDS['pitch']
                    cursor_msg = 'Up Key - pitch(+):{}'.format(CMDS['pitch'])

                elif char == curses.KEY_DOWN:
                    CMDS['pitch'] = CMDS['pitch'] - 10 if CMDS['pitch'] - 10 >= 1000 else CMDS['pitch']
                    cursor_msg = 'Down Key - pitch(-):{}'.format(CMDS['pitch'])

                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                #
                # SLOW MSG processing (user GUI)
                #
                if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()

                    next_msg = next(slow_msgs) # circular list

                    # Read info from the FC
                    if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                        
                    if next_msg == 'MSP_ANALOG':
                        voltage = board.ANALOG['voltage']
                        voltage_msg = ""
                        if min_voltage < voltage <= warn_voltage:
                            voltage_msg = "LOW BATT WARNING"
                        elif voltage <= min_voltage:
                            voltage_msg = "ULTRA LOW BATT!!!"
                        elif voltage >= max_voltage:
                            voltage_msg = "VOLTAGE TOO HIGH"

                        screen.addstr(8, 0, "Battery Voltage: {:2.2f}V".format(board.ANALOG['voltage']))
                        screen.clrtoeol()
                        screen.addstr(8, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                        screen.clrtoeol()

                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = board.bit_check(board.CONFIG['mode'],0)
                        screen.addstr(5, 0, "ARMED: {}".format(ARMED), curses.A_BOLD)
                        screen.clrtoeol()

                        screen.addstr(5, 50, "armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
                        screen.clrtoeol()

                        screen.addstr(6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                        screen.clrtoeol()
                        screen.addstr(6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                        screen.clrtoeol()

                        screen.addstr(7, 0, "mode: {}".format(board.CONFIG['mode']))
                        screen.clrtoeol()

                        screen.addstr(7, 50, "Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))
                        screen.clrtoeol()


                    elif next_msg == 'MSP_MOTOR':
                        screen.addstr(9, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_RC':
                        screen.addstr(10, 0, "RC Channels Values: {}".format(board.RC['channels']))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_ATTITUDE':
                        screen.addstr(17, 0, f"Altitude : {ALTITUDE}; X: {x}; Y: {y}" )
                        screen.clrtoeol()

                    screen.addstr(11, 0, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000, 1/(sum(average_cycle)/len(average_cycle))))

                    # Commanded values of T, R, P                                                
                    screen.clrtoeol()
                    screen.addstr(18, 0, "Command : {}".format([throttle, roll_, pitch]))
                    screen.clrtoeol()

                    # Value of gains for Throttle, Roll and Pitch
                    screen.addstr(19, 0, "PID-Throttle : {}".format(gains[0]))
                    screen.clrtoeol()
                    screen.addstr(20, 0, "PID-Roll : {}".format(gains[1]))
                    screen.clrtoeol()
                    screen.addstr(21, 0, "PID-Pitch : {}".format(gains[2]))
                    screen.clrtoeol()

                    # Error Values for Throttle, Roll and Pitch
                    # screen.addstr(21, 0, "Error : {}".format([error_t,error_r,error_p]))
                    screen.addstr(22, 0, f"Error-T: {round(error[0],3)}; Error-R: {round(error[1],3)}; Error-P: {round(error[2],3)}")
                    screen.clrtoeol()

                    # Value of PID for throttle after multiplied by gains
                    screen.addstr(23, 0, "P_t : {}".format(PID_t[0]))
                    screen.clrtoeol()
                    screen.addstr(24, 0, "I_t : {}".format(PID_t[1]))
                    screen.clrtoeol()
                    screen.addstr(25, 0, "D_t : {}".format(PID_t[2]))
                    screen.clrtoeol()

                    # Value of PID for roll after multiplied by gains
                    screen.addstr(23, 20, "P_r : {}".format(PID_r[0]))
                    screen.clrtoeol()
                    screen.addstr(24, 20, "I_r : {}".format(PID_r[1]))
                    screen.clrtoeol()
                    screen.addstr(25, 20, "D_r : {}".format(PID_r[2]))
                    screen.clrtoeol()

                    # Value of PID for pitch after multiplied by gains
                    screen.addstr(23, 40, "P_p : {}".format(PID_p[0]))
                    screen.clrtoeol()
                    screen.addstr(24, 40, "I_p : {}".format(PID_p[1]))
                    screen.clrtoeol()
                    screen.addstr(25, 40, "D_p : {}".format(PID_p[2]))
                    screen.clrtoeol()
                    if int(detected) == 0:
                        CMDS['aux1'] = 1000
                        screen.addstr(14, 0, "DRONE NOT FOUND!!!",curses.A_BOLD + curses.A_BLINK)
                        screen.clrtoeol()
                    else:
                        screen.addstr(14, 0, "                  ")
                        screen.clrtoeol()
                    
                    screen.addstr(3, 0, cursor_msg)
                    screen.clrtoeol()
                    

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()
    except:
        raise
    finally:
        screen.addstr(5, 0, "Disconneced from the FC!")
        screen.clrtoeol()
    



class Controller():
    
    def __init__(self):

        self.previous_error = [[0],[0],[0]] # Throttle, Pitch, Roll 
        self.array = [[0],[0],[0]] # Throttle, Pitch, Roll
        self.ARM = False

    def pid(self,altitude,setpoints_=[6,0.935,-0.3132]): # [Throttle_setpoint, pitch_setpoint, Roll_setpoint]
        global throttle, gains, error, PID_t, PID_r, PID_p, roll_, pitch

        self.setpoints = setpoints_

        gains[0] = [float(a) for a in pid[0]] # Throttle
        gains[1] = [float(b) for b in pid[1]] # Pitch
        gains[2] = [float(c) for c in pid[2]] # Roll

        error[0] = -(self.setpoints[0] - round(altitude,1)) # Throttle
        error[1] = -(self.setpoints[1] - round(x,1)) # Pitch
        error[2] = -(self.setpoints[2] - round(y,1)) # Roll
        
        if ARMED and altitude != 0 and int(detected) != 0:
        # if True:            
            self.array[0].append(error[0]) # Throttle
            self.array[1].append(error[1]) # Pitch
            self.array[2].append(error[2]) # Roll

            # Throttle Calculation
            # print(self.array)
            PID_t[0] = gains[0][0] * error[0]
            if throttle < 1550 or throttle > 1150:
                PID_t[1] = sum(self.array[0])*gains[0][1]
            PID_t[2] = gains[0][2]*(error[0]-self.previous_error[0][-1])
            throttle = sum(PID_t) + 1400
            # throttle = 1000

            # Pich Calculation
            PID_p[0] = gains[1][0] * error[1]
            if roll_ < 1550 or roll_ > 1450:
                PID_p[1] = sum(self.array[1])*gains[1][1]
            PID_p[2] = gains[1][2]*(error[1]-self.previous_error[1][-1])
            pitch = sum(PID_p) + 1500

            # Roll Calculation
            PID_r[0] = gains[2][0] * error[2]
            if pitch < 1550 or pitch > 1450:
                PID_r[1] = sum(self.array[2])*gains[2][1]
            PID_r[2] = gains[2][2]*(error[2]-self.previous_error[2][-1])
            roll_ = sum(PID_r) + 1500

            if throttle >= 1550:
                throttle = 1550
            elif throttle <= 1150:
                throttle = 1150
            
            if roll_ >= 1550:
                roll_ = 1550
            elif roll_ <= 1450:
                roll_ = 1450

            if pitch >= 1550:
                pitch = 1550
            elif pitch <= 1450:
                pitch = 1450
            
            CMDS['throttle'] = throttle
            CMDS['pitch'] = pitch
            CMDS['roll'] = roll_

            self.previous_error[0].append(error[0])
            self.previous_error[1].append(error[1])
            self.previous_error[2].append(error[2])

            if len(self.previous_error[0]) > 20:
                self.previous_error[0].pop(0)
                self.previous_error[1].pop(0)
                self.previous_error[2].pop(0)
            
        else:

            CMDS['throttle'] = 900
            CMDS['roll'] = 1500
            CMDS['pitch'] = 1500


        

class MQTT():
    def __init__(self, mqtt_server_, mqtt_path_):
        self.client = mqtt.Client()
        self.client.on_connect = self.onConnect
        self.client.on_message = self.onMessage
        self.MQTT_SERVER = mqtt_server_
        self.MQTT_PATH = mqtt_path_
        self.ALTITUDE = 0

    def connectClient(self):
        self.client.connect(self.MQTT_SERVER)

    def onMessage(self,client, userdata, msg):
        global ALTITUDE, pid, x, y, detected
        val = str(msg.payload)[2:-1].split(',')
        # print(val)
        y = float(val[11])
        x = float(val[10])
        ALTITUDE = float(val[9])
        pid[0] = val[:3]
        pid[2] = val[3:6]
        pid[1] = val[6:9]
        detected = val[12]

    def onConnect(self,client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
 
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe(self.MQTT_PATH)


if __name__ == "__main__":

    # client = mqtt.Client()
    # client.on_connect = on_connect
    # client.on_message = on_message
    # # client.on_subscribe = on_subscribe
    # client.connect(MQTT_SERVER)
    # g = GUI_LOOP("/dev/serial0")

    
    mqtt_client = MQTT("localhost","pid")
    mqtt_client.connectClient()

    t1 = threading.Thread(target=mqtt_client.client.loop_forever)
    t1.start()
    t2 = threading.Thread(target=run_curses, args=(keyboard_controller,))
    t2.start()
