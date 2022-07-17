#! /usr/bin/env python3

# from inspect import stack
import paho.mqtt.publish as publish
import rospy
from geometry_msgs.msg import PoseArray, Pose
import time
from whycon.msg import PidTune

MQTT_SERVER = "192.168.43.30"
MQTT_PATH = "test_channel"
MQTT_PID = "pid"

z = 0
Kp = 0
Ki = 0
Kd = 0
def pose_callback(msg):
    global z
    z = msg.poses[0].position.z
    # q = msg.header.stamp.nsecs
def pid_callback(msg):
    global Kp, Ki, Kd
    Kp = msg.Kp
    Ki = msg.Ki
    Kd = msg.Kd

def main():
    global z
    rospy.init_node("mqtt")
    rospy.Subscriber("/whycon/poses", PoseArray, pose_callback)
    rospy.Subscriber("/pid", PidTune, pid_callback)
    
    while not rospy.is_shutdown():
        gains = str(Kp) +"-"+ str(Ki) +"-"+ str(Kd) +"-"+ str(z)
        # publish.single(MQTT_PATH, z, hostname=MQTT_SERVER) #send data continuously every 3 seconds
        publish.single(MQTT_PID, gains, hostname=MQTT_SERVER) #send data continuously every 3 seconds

if __name__ == "__main__":
    main()   

