#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from copy import deepcopy
from math import sqrt, atan2

joy_msg = Joy()
joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

speed_x, speed_y, speed_theta_cw, speed_theta_ccw = [1, 1, 1.9, 1.9]
command_x, command_y, command_theta = [1, 1.2, 0.5]

class CommandDriving:
    """Class that follows the giving waypoint command."""
    
    def __init__(
        self, 
        current_state,
        command_state = [0.0, 0.0, 0.0],
    ):
        """
        Initialize the CommandDriving class.
        
        Input:
            current_state: current state of the robot.
            command_state: command waypoint for the robot. 
        """
        assert len(current_state) == 3, \
            "Length of current_state is off"
        assert len(command_state) == 3, \
            "Length of command_state is off"
        self.current_state, self.command_state = current_state, command_state   
        self.current_x, self.current_y, self.current_theta = current_state
        self.command_x, self.command_y, self.command_theta = command_state
        self.pub = rospy.Publisher('/joy', Joy, queue_size = 5)
        
        rospy.init_node("command_driving")
        self.pub.publish(joy_msg)
        rospy.sleep(0.5)
    
    def move(self, distance):
        """Move the robot in forward / reverse direction."""
        if distance != 0:
            rospy.loginfo("Moving the robot")
            delta_time = abs(distance) / speed_x
            rospy.loginfo("Moving for " + str(delta_time))
            current_msg = deepcopy(joy_msg)
            current_msg.axes[1] = -command_x * distance / abs(distance)
            self.pub.publish(current_msg)
            rospy.sleep(delta_time)
    
    def slide(self, distance):
        """Slide the robot in right / left direction."""
        if distance != 0:
            rospy.loginfo("Sliding the robot")
            delta_time = abs(distance) / speed_y
            rospy.loginfo("Sliding for " + str(delta_time))
            current_msg = deepcopy(joy_msg)
            current_msg.axes[0] = -command_y * distance / abs(distance)
            self.pub.publish(current_msg)
            rospy.sleep(delta_time)
    
    def rotate(self, angle):
        """Rotate the robot in clockwise / counter clockwise direction."""
        if angle != 0:
            rospy.loginfo("Rotating the robot")
            if angle < 0: delta_time = abs(angle) / speed_theta_ccw
            else: delta_time = abs(angle) / speed_theta_cw
            rospy.loginfo("Rotating for " + str(delta_time))
            current_msg = deepcopy(joy_msg)
            current_msg.axes[2] = -command_theta * angle / abs(angle)
            self.pub.publish(current_msg)
            rospy.sleep(delta_time)
    
    def run(self):
        """Move the robot from current state to command state."""
        delta_x, delta_y, delta_theta = \
            [ele[0] - ele[1] 
             for ele in zip(self.command_state, self.current_state)]
        print (delta_x, delta_y, delta_theta)
        if self.current_theta % 3.14 == 0:
            # robot facing x / -x direction
            # x directional movement will be moving
            # y directional movement will be sliding
            self.move(delta_x)
            self.slide(delta_y)
        elif self.current_theta % 1.57 == 0:
            # robot facing y / -y direction
            # x directional movement will be sliding
            # y directional movement will be moving
            self.move(delta_y)
            self.move(delta_x)
        else:
            # rotate the robot to the desired direction first
            # then move to the command position
            current_cmd_angle = atan2(delta_y, delta_x)
            angle = current_cmd_angle - self.current_theta
            delta_theta = current_cmd_angle - self.command_theta
            distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
            self.rotate(angle)
            self.move(distance)
        
        self.rotate(delta_theta)
        self.pub.publish(joy_msg)
        rospy.sleep(1.0)
        
