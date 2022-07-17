from yamspy import MSPy
import time
import threading
import paho.mqtt.client as mqtt #import library

import time
# from mqtt_client import MQTT
from yamspy import MSPy
# from gui import GUI_LOOP, MQTT, Controller
import threading
# from dump import dum_send, ARMED
# from simpleUI import run_curses, keyboard_controller

CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10



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
ALTITUDE = 0

import time
import curses
from collections import deque
from itertools import cycle
import serial
from yamspy import MSPy
import threading
# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10

ARMED = False
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

        self.ALTITUDE = float(str(msg.payload)[2:-1])
        # print(ALTITUDE)

    def onConnect(self,client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
 
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe(self.MQTT_PATH)




def dum_send():

    global ARMED, CMDS, CMDS_ORDER
    # "print" doesn't work with curses, use addstr instead
    try:
        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1

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

            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            while True:
                print(CMDS["aux1"])
                start_time = time.time()

                #
                # Key input processing
                #

                #
                # KEYS (NO DELAYS)
                #

                #
                # The code below is expecting the drone to have the
                # modes set accordingly since everything is hardcoded.


    
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

     
                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = board.bit_check(board.CONFIG['mode'],0)

                    elif next_msg == 'MSP_MOTOR':
                        pass

                    elif next_msg == 'MSP_RC':
                        pass
                    

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()

    except Exception as e:
        print(e)

class Controller():
    
    def __init__(self):
        self.gains = [1500,0,0]

        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.previous_error = 0
        # self.brd = board_
        self.I = 0
        self.ARM = False

    def arm(self):
        global CMDS, CMDS_ORDER
        CMDS['aux1'] = 1800
        ARM = True
        # self.CMDS['throttle'] = 800
        
        # if self.brd.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
        #     dataHandler = self.brd.receive_msg()
        #     self.brd.process_recv_data(dataHandler)
       

    def disarm(self):
        # CMDS['throttle'] = 900
        global CMDS, CMDS_ORDER
        CMDS['aux1'] = 1000
        # if self.brd.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
        #     dataHandler = self.brd.receive_msg()
        #     self.brd.process_recv_data(dataHandler)
        ARM = False
    def pid(self,setpoints_=1):

        self.setpoints = setpoints_

        self.now = time.time()
        self.last_time = 0 

        self.elapsed_time = self.now - self.last_time

        if (self.elapsed_time > 0.005):

            error = self.setpoints - ALTITUDE

            self.P = self.gains[0] * error
            self.I = self.I + self.gains[1] * error * self.elapsed_time
            self.D = self.gains[2]*(error-self.previous_error)/self.elapsed_time

            throttle = self.P + self.I + self.D

            if throttle >= 2000:
                self.I = self.old_i
                throttle = 2000
            
            # if U <= 1000:
            #     self.I = self.old_i
            #     U = 1000


            self.previous_error = error
            self.last_time = self.now

            print(-throttle)
            self.CMDS['throttle'] = abs(int(throttle))
            # if self.board.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
            #     dataHandler = self.board.receive_msg()
            #     self.board.process_recv_data(dataHandler)
            #     self.ARM = True
            # print(self.CMDS['aux1'])


if __name__ == "__main__":

    # client = mqtt.Client()
    # client.on_connect = on_connect
    # client.on_message = on_message
    # # client.on_subscribe = on_subscribe
    # client.connect(MQTT_SERVER)
    # g = GUI_LOOP("/dev/serial0")

    controller = Controller()
    mqtt_client = MQTT("localhost","test_channel")
    mqtt_client.connectClient()

    t1 = threading.Thread(target=mqtt_client.client.loop_forever)
    t1.start()
    t1 = threading.Thread(target=dum_send)
    t1.start()
    
    
    while True:
        # print(CMDS[''])
        time.sleep(5)
        controller.arm()
        # if ARMED:
        #     print("Hurray")

        if KeyboardInterrupt:
            print("HI")
