#! /usr/bin/env python3

# from inspect import stack
import paho.mqtt.publish as publish
import rospy
from geometry_msgs.msg import PoseArray, Pose

from whycon.msg import Coordinates
from std_msgs.msg import Float64, Int16

MQTT_SERVER = "192.168.43.75"
MQTT_PATH = "test_channel"
MQTT_PID = "pid"

x = 0
y = 0
z = 0

pid_throttle = [0,0,0]
pid_roll = [0,0,0]
pid_pitch = [0,0,0]

def pose_callback(msg):
    global z,x,y
    x = msg.poses[0].position.x
    y = msg.poses[0].position.y
    z = msg.poses[0].position.z

def throttle_callback(msg):
    global pid_throttle
    pid_throttle[0] = msg.Kp
    pid_throttle[1] = msg.Ki
    pid_throttle[2] = msg.Kd

def pitch_callback(msg):
    global pid_pitch
    pid_pitch[0] = msg.Kp
    pid_pitch[1] = msg.Ki
    pid_pitch[2] = msg.Kd


def roll_callback(msg):
    global pid_roll
    pid_roll[0] = msg.Kp
    pid_roll[1] = msg.Ki
    pid_roll[2] = msg.Kd

def main():
    global z,x,y
    rospy.init_node("mqtt")

    pub = rospy.Publisher("error_alt", Float64, queue_size=1000)
    pub1 = rospy.Publisher("error_roll", Float64, queue_size=1000)
    pub2 = rospy.Publisher("error_pitch", Float64, queue_size=1000)
    pub_zero = rospy.Publisher("zero", Int16, queue_size=1000)

    rospy.Subscriber("/whycon/poses", PoseArray, pose_callback)
    rospy.Subscriber("/throttle", Coordinates, throttle_callback)
    rospy.Subscriber("/pitch", Coordinates, pitch_callback)
    rospy.Subscriber("/roll", Coordinates, roll_callback)

    error_alt = Float64()
    error_roll = Float64()
    error_pitch = Float64()

    zero = Int16()

    setpoint = [6,0.935,-0.3132]

    while not rospy.is_shutdown():
        gains = str(pid_throttle[0]) +","+ str(pid_throttle[1]) +","+ str(pid_throttle[2]) +","+ str(pid_roll[0]) +","+ str(pid_roll[1]) +","+ str(pid_roll[2]) +","+ str(pid_pitch[0]) +","+ str(pid_pitch[1]) +","+ str(pid_pitch[2]) +","+ str(round(z,4)) +","+ str(round(x,4)) +","+ str(round(y,4))
        
        error_alt.data = setpoint[0] - z
        error_pitch.data = setpoint[1] - x
        error_roll.data = setpoint[2] - y
        zero.data = 0

        pub.publish(error_alt)
        pub1.publish(error_pitch)
        pub2.publish(error_roll)
        pub_zero.publish(zero)
        
        publish.single(MQTT_PID, gains, hostname=MQTT_SERVER) #send data continuously

if __name__ == "__main__":
    main()   

