#!/usr/bin/env python3

from tkinter import *
import rospy
from pid_tune.msg import PidTune

pid_value = PidTune()

def set_value():
  global pid_value
  pid_value.Kp = scale.get()
  pid_value.Ki = scale1.get()
  pid_value.Kd = scale2.get()
 
  send_pid.publish(pid_value)

root = Tk()

scale = Scale(root, orient='horizontal', from_=0, to=2000, label= 'Kp',width = "50", length = "1500",troughcolor="red")
scale1 =Scale(root, orient='horizontal', from_=0.0, to=1000, label= 'Ki',width = "50", length = "1500",troughcolor="green")
scale2 =Scale(root, orient='horizontal', from_=0, to=2000, label= 'Kd',width = "50", length = "1500", troughcolor="blue")
# scale3 =Scale(root, orient='horizontal', from_=0, to=300, label= 'Kp_z',width = "50", length = "500", troughcolor="gray")
scale.pack()
scale1.pack()
scale2.pack()
# scale3.pack()

Button(root, text='set_value', command=set_value).pack()



if __name__ == "__main__":
    rospy.init_node('tune_pid')
    send_pid = rospy.Publisher('/pid', PidTune, queue_size=1000, latch=True)

    root.mainloop()