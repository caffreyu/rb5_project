#!/usr/bin/env python
from command_driving import CommandDriving
import rospy
from std_msgs.msg import Int8

# processing waypoints
with open('/root/ros_ws/src/rb5_project/rb5_control/src/waypoints.txt') as f:
    waypoints_file = f.readlines()

# with open('/home/hay024/ros_ws/src/rb5_project/rb5_control/src/waypoints.txt') as f:
#     waypoints_file = f.readlines()

waypoints_string_ls = [ele[: -1].split(',') for ele in waypoints_file]
waypoints_string_ls[-1][-1] = '0'
waypoints_ls = []

for string_ls in waypoints_string_ls:
    waypoints_ls.append([float(ele) for ele in string_ls])

# waypoints_ls = waypoints_ls[: 3]

current_state = [0.0, 0.0, 0.0]

while waypoints_ls:
    switch = rospy.wait_for_message("/switch", Int8)
    if switch.data != 1:
        break

    command_state = waypoints_ls.pop(0)
    command_driver = \
        CommandDriving(
            current_state=current_state,
            command_state=command_state,
        )
    command_driver.run()
    current_state = command_state
    
