#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from copy import deepcopy
import numpy as np
from std_msgs.msg import Int8
from math import atan2

# processing waypoints
with open('/home/caffreyu/FA22/cse276a/src/rb5_ros/rb5_control/src/waypoints.txt') as f:
    waypoints_file = f.readlines()

waypoints_string_ls = [ele[: -1].split(',') for ele in waypoints_file]
waypoints_ls = []

for string_ls in waypoints_string_ls:
    waypoints_ls.append([float(ele) for ele in string_ls])

speed_ls = [0.5, 0.5, 0.5]
x_speed, y_speed, theta_speed = speed_ls

speed_cmd = [0.5, 0.5, 0.25]
x_cmd, y_cmd, theta_cmd = speed_cmd

rospy.init_node("auto_driving")
pub_auto = rospy.Publisher("/auto", Joy, queue_size = 1)
current_state = [0, 0, 0]

joy_msg = Joy()
joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

def move_y(delta_y):
    delta_time = abs(delta_y) / y_speed
    curr_msg = deepcopy(joy_msg)
    direction = 1 if delta_y > 0 else -1
    curr_msg.axes[1] = direction * y_cmd
    pub_auto.publish(curr_msg)
    rospy.loginfo(f"sleeping for {delta_time}")
    rospy.sleep(delta_time)

def move_x(delta_x):
    delta_time = abs(delta_x) / x_speed
    curr_msg = deepcopy(joy_msg)
    direction = 1 if delta_x > 0 else -1
    curr_msg.axes[0] = direction * x_cmd
    pub_auto.publish(curr_msg)
    rospy.loginfo(f"sleeping for {delta_time}")
    rospy.sleep(delta_time)

def move_theta(delta_theta):
    delta_time = abs(delta_theta) / theta_speed
    curr_msg = deepcopy(joy_msg)
    direction = 1 if delta_theta > 0 else -1
    curr_msg.axes[2] = direction * theta_cmd
    pub_auto.publish(curr_msg)
    rospy.loginfo(f"sleeping for {delta_time}")
    rospy.sleep(delta_time)

while waypoints_ls:

    data = rospy.wait_for_message("/auto_stop", Int8)
    if data.data != 1: break

    w_x, w_y, w_theta = waypoints_ls.pop(0)
    if w_theta < 0:
        w_theta = -w_theta + 3.14
    c_x, c_y, c_theta = current_state
    delta_x, delta_y, delta_theta = w_x - c_x, w_y - c_y, w_theta - c_theta
    
    if delta_theta == delta_y == delta_x == 0:
        rospy.loginfo("passed")
        continue
    # slide
    if delta_theta == delta_y == 0 and delta_x != 0:
        rospy.loginfo("slide")
        move_x(delta_x)
        continue
    # move in single y direction:
    elif delta_x == 0:
        rospy.loginfo("head in y")
        move_y(delta_y)
    else:
        moving_direction = atan2(delta_y, delta_x)
        if moving_direction < 0: moving_direction += 2 * 3.14
        delta_moving_direction = moving_direction - c_theta
        rospy.loginfo("head to moving direction")
        move_theta(delta_moving_direction)
        delta_theta = w_theta - delta_moving_direction
        moving_distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
        rospy.loginfo("head in y in moving direction")
        move_y(moving_distance)
    
    rospy.loginfo("adjust direction")
    move_theta(delta_theta)
    current_state = [w_x, w_y, w_theta]

pub_auto.publish(joy_msg)


