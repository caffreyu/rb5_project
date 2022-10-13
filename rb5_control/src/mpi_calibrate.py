#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Joy

joy_msg = Joy()
joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

rospy.init_node("mpi_calibrate")
pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)

rospy.loginfo("Publishing empty message")
pub_joy.publish(joy_msg)
rospy.sleep(2)

key, cmd, time = sys.argv[1 :]
cmd, time = float(cmd), float(time)

if key == 'w':
    joy_msg.axes[1] = -cmd

if key == 'a':
    joy_msg.axes[0] = -cmd
    
if key == 's':
    joy_msg.axes[1] = cmd

if key == 'd':
    joy_msg.axes[0] = cmd

if key == 'q':
    joy_msg.axes[2] = cmd

if key == 'e':
    joy_msg.axes[2] = -cmd
    
rospy.loginfo("Publishing command message")
pub_joy.publish(joy_msg)
rospy.sleep(time)

joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
pub_joy.publish(joy_msg)
    