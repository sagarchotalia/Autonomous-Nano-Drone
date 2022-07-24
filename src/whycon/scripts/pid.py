#!/usr/bin/env python3

from tkinter import *
import rospy
from whycon.msg import Coordinates

throttle = Coordinates()
pitch = Coordinates()
roll = Coordinates()

def set_value():
  global throttle, pitch
  throttle.Kp = float(t1.get())
  throttle.Ki = float(t2.get())
  throttle.Kd = float(t3.get())

  pitch.Kp = float(p1.get())
  pitch.Ki = float(p2.get())
  pitch.Kd = float(p3.get())

  roll.Kp = float(r1.get())
  roll.Ki = float(r2.get())
  roll.Kd = float(r3.get())
  

 
  throttle_pub.publish(throttle)
  pitch_pub.publish(pitch)
  roll_pub.publish(roll)

root = Tk()



kp_throttle = Label(root, text = "Kp_t")
t1 = Entry(root) 
t1.insert(0, 0)

ki_throttle = Label(root,text =  "Ki_t")
t2 = Entry(root) 
t2.insert(0, 0)
# entry2.grid(row=1, column=0)
kd_throttle = Label(root, text = "Kd_t")
t3 = Entry(root) 
t3.insert(0, 0)


kp_pitch = Label(root, text = "Kp_p")
p1 = Entry(root) 
p1.insert(0, 0)

ki_pitch = Label(root,text =  "Ki_p")
p2 = Entry(root) 
p2.insert(0, 0)
# entry2.grid(row=1, column=0)
kd_pitch = Label(root, text = "Kd_p")
p3 = Entry(root) 
p3.insert(0, 0)
# entry3.grid(row=2, column=0)


kp_roll = Label(root, text = "Kp_r")
r1 = Entry(root) 
r1.insert(0, 0)

ki_roll = Label(root,text =  "Ki_r")
r2 = Entry(root) 
r2.insert(0, 0)
# entry2.grid(row=1, column=0)
kd_roll = Label(root, text = "Kd_r")
r3 = Entry(root) 
r3.insert(0, 0)

# kp_label.pack()
# entry1.pack()
# ki_label.pack()
# entry2.pack()
# kd_label.pack()
# entry3.pack()
# scale3.pack()

kp_pitch.grid(row=0, column=2,padx=5, pady=5)
ki_pitch.grid(row=1, column=2,padx=5, pady=5)
kd_pitch.grid(row=2, column=2,padx=5, pady=5)
p1.grid(row=0, column=3)
p2.grid(row=1, column=3)
p3.grid(row=2, column=3)

kp_throttle.grid(row=0, column=0,padx=5, pady=5)
ki_throttle.grid(row=1, column=0,padx=5, pady=5)
kd_throttle.grid(row=2, column=0,padx=5, pady=5)
t1.grid(row=0, column=1)
t2.grid(row=1, column=1)
t3.grid(row=2, column=1)

kp_roll.grid(row=0, column=4,padx=5, pady=5)
ki_roll.grid(row=1, column=4,padx=5, pady=5)
kd_roll.grid(row=2, column=4,padx=5, pady=5)
r1.grid(row=0, column=5)
r2.grid(row=1, column=5)
r3.grid(row=2, column=5)

Button(root, text='set_value', command=set_value).grid(row=3, column=2)



if __name__ == "__main__":
    rospy.init_node('tune_pid')
    throttle_pub = rospy.Publisher('/throttle', Coordinates, queue_size=1000, latch=True)
    pitch_pub = rospy.Publisher('/pitch', Coordinates, queue_size=1000, latch=True)
    roll_pub = rospy.Publisher('/roll', Coordinates, queue_size=1000, latch=True)
    root.mainloop()