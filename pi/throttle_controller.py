
import time
from mqtt_client import MQTT
from controller import Controller
from yamspy import MSPy
# from gui import GUI_LOOP, MQTT, Controller
import threading
from dump import dum_send, ARMED
# from simpleUI import run_curses, keyboard_controller





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


if __name__ == "__main__":

    # client = mqtt.Client()
    # client.on_connect = on_connect
    # client.on_message = on_message
    # # client.on_subscribe = on_subscribe
    # client.connect(MQTT_SERVER)
    # g = GUI_LOOP("/dev/serial0")

    controller = Controller(CMDS, CMDS_ORDER)
    mqtt_client = MQTT("localhost","test_channel")
    mqtt_client.connectClient()

    # t1 = threading.Thread(target=mqtt_client.client.loop_forever)
    # t1.start()
    t2 = threading.Thread(target=dum_send, args = (CMDS, CMDS_ORDER))
    t2.start()
    # time.sleep(10)
    controller.arm()
    while True:
        if ARMED:
            print("Hurray")

        if KeyboardInterrupt:
            controller.disarm()

    
    # with MSPy(device = "/dev/serial0", loglevel = "WARNING", baudrate=115200) as board:
    #         if board == 1:
    #             raise("Error on board connection")

    #         controller = Controller(board,CMDS, CMDS_ORDER)
    #         while True:
    #             controller.arm()
    #             print("Arming....")



    # t1 = threading.Thread(target=mqtt_client.loop_forever)
    # t2 = threading.Thread(target=run_curses, args=(keyboard_controller,))
    # run_curses(keyboard_controller)
    
    # t2.start()


