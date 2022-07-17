from curses.ascii import alt
import time
import curses
from collections import deque
from itertools import cycle
from yamspy import MSPy
from controller import Controller, ALTITUDE
import paho.mqtt.client as mqtt #import library
ALTITUDE = 0


CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 800,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

class MQTT():
    def __init__(self, mqtt_server_, mqtt_path_):
        self.client = mqtt.Client()
        self.client.on_connect = self.onConnect
        self.client.on_message = self.onMessage
        self.MQTT_SERVER = mqtt_server_
        self.MQTT_PATH = mqtt_path_
        # self.ALTITUDE = 0

    def connectClient(self):
        self.client.connect(self.MQTT_SERVER)

    def onMessage(self,client, userdata, msg):
        global ALTITUDE
        ALTITUDE = float(str(msg.payload)[2:-1])
        # print(ALTITUDE)

    def onConnect(self,client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
 
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe(self.MQTT_PATH)

class Controller():
    
    def __init__(self, board_, cmd_, cmd_order_):
        self.gains = [1500,0,0]

        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500
        self.previous_error = 0
        self.brd = board_
        self.I = 0
        self.CMDS = cmd_
        self.CMDS_ORDER = cmd_order_

        self.ARM = False

    def arm(self):
        self.CMDS['aux1'] = 1800
        self.CMDS['throttle'] = 800
        
        if self.brd.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
            dataHandler = self.brd.receive_msg()
            self.brd.process_recv_data(dataHandler)
       

    def disarm(self):
        # CMDS['throttle'] = 900
        self.CMDS['aux1'] = 1000
        if self.brd.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
            dataHandler = self.brd.receive_msg()
            self.brd.process_recv_data(dataHandler)
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
            return throttle
            # print(-throttle)
            # self.CMDS['throttle'] = abs(int(throttle))
            # if self.board.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
            #     dataHandler = self.board.receive_msg()
            #     self.board.process_recv_data(dataHandler)
            #     self.ARM = True
            # print(self.CMDS['aux1'])

class GUI_LOOP():
    def __init__(self, serial_):

        self.serial = serial_
        self.ARM = False
        self.NO_OF_CYCLES_AVERAGE_GUI_TIME = 10
        self.CTRL_LOOP_TIME = 1/100
        self.SLOW_MSGS_LOOP_TIME = 1/5

        self.CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 900,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

        self.CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2'] 



    def run_curses(self):
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
            
            result = self.keyboard_controller(screen)

        
        finally:
            # shut down cleanly
            curses.nocbreak(); screen.keypad(0); curses.echo()
            curses.endwin()
            if result==1:
                print("An error occurred... probably the serial port is not available ;)")


    def keyboard_controller(self, screen): 
        
        try:
            screen.addstr(15, 0, "Connecting to the FC...")

            with MSPy(device = self.serial, loglevel = "WARNING", baudrate=115200) as board:
                if board == 1:
                    return 1
                controller = Controller(board, self.CMDS, self.CMDS_ORDER)
                
                screen.addstr(15, 0, "Connecting to the FC... connected!")
                screen.clrtoeol()
                screen.move(1,0)

                average_cycle = deque([0]*self.NO_OF_CYCLES_AVERAGE_GUI_TIME)

                command_list = ['MSP_BATTERY_STATE', 'MSP_BOXNAMES']

                if board.INAV:
                    # command_list.append('MSPV2_INAV_ANALOG')
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

                slow_msgs = cycle(['MSP_ALTITUDE','MSP_ANALOG','MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])
                
                cursor_msg = ""
                last_loop_time = last_slow_msg_time = last_cycleTime = time.time()

                while True:
                    
                    start_time = time.time()

                    char = screen.getch()
                    curses.flushinp()

                    if char == ord('q') or char == ord('Q'):
                        break

                    elif char == ord('d') or char == ord('D'):
                        cursor_msg = 'Sending Disarm command...'
                        self.CMDS['aux1'] = 1000

                    elif char == ord('r') or char == ord('R'):
                        screen.addstr(3, 0, 'Sending Reboot command...')
                        screen.clrtoeol()
                        board.reboot()
                        time.sleep(0.5)
                        break

                    elif char == ord('a') or char == ord('A'):
                        cursor_msg = 'Sending Arm command...'
                        self.CMDS['aux1'] = 1800     
                        self.ARM = True     

                    # elif char == curses.KEY_UP:
                    #     self.CMDS['throttle'] = self.CMDS['throttle'] + 10 if self.CMDS['throttle'] + 10 <= 2000 else self.CMDS['throttle']
                    #     cursor_msg = 'UP Key - throttle(+):{}'.format(self.CMDS['throttle'])

                    
                    # elif char == curses.KEY_DOWN:
                    #     self.CMDS['throttle'] = self.CMDS['throttle'] - 10 if self.CMDS['throttle'] - 10 >= 1000 else self.CMDS['throttle']
                    #     cursor_msg = 'DOWN Key - throttle(-):{}'.format(self.CMDS['throttle'])  
                    # 
                    if self.ARM:
                        # self.CMDS['throttle'] = controller.pid(ALTITUDE)
                        screen.addstr(18, 0, "PID: {}".format(controller.pid()), curses.A_BOLD)
                        screen.clrtoeol()   
                        # print()
                    # if ARMED == True:
                    # self.CMDS['throttle'] = control((-2.69 - altitude), 0, 0)[0]
                    # self.CMDS['roll'] = 1500
                    # self.CMDS['pitch'] = 1500
                    # print(ARMED)
                    # print(control((-2.69 - altitude), 0, 0)[0])

                    if (time.time()-last_loop_time) >= self.CTRL_LOOP_TIME:
                        last_loop_time = time.time()
                        # Send the RC channel values to the FC
                        if board.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)

                    if (time.time()-last_slow_msg_time) >= self.SLOW_MSGS_LOOP_TIME:
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

                        elif next_msg == 'MSP_ALTITUDE':
                            screen.addstr(15, 0, "ALTITUDE: {}".format(ALTITUDE), curses.A_BOLD)
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

                        screen.addstr(11, 0, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000,
                                                                                                    1/(sum(average_cycle)/len(average_cycle))))
                        screen.clrtoeol()

                        screen.addstr(3, 0, cursor_msg)
                        screen.clrtoeol()
                        

                    end_time = time.time()
                    last_cycleTime = end_time-start_time
                    if (end_time-start_time)<self.CTRL_LOOP_TIME:
                        time.sleep(self.CTRL_LOOP_TIME-(end_time-start_time))
                        
                    average_cycle.append(end_time-start_time)
                    average_cycle.popleft()
                    
        except Exception as e:
            print(e)
        finally:
            screen.addstr(5, 0, "Disconneced from the FC!")
            screen.clrtoeol()