import paho.mqtt.client as mqtt #import library

ALTITUDE = 0

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
        print(self.ALTITUDE)

    def onConnect(self,client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
 
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe(self.MQTT_PATH)




if __name__ == "__main__":
    mqtt_obj = MQTT("localhost","test_channel")
    mqtt_obj.connectClient()
        
        
        


        
