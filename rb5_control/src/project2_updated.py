#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseArray
from pid_controller import PIDcontroller
from tf.transformations import euler_from_quaternion

waypoints = np.array(
    [
        [0.5, 0.0, 0.0],
        [0.5, 1.0, np.pi],
        [0.0, 0.0, 0.0],
    ],
)

april_tag_refs = np.array(
    [
        [-0.3, 0, 0],
        [-0.5, 0, 0],
        [-0.8, 0, 0],
    ]
)

class AutoController:

    def __init__(self, pid_constrants):
        kp, ki, kd = pid_constrants
        self.open_reached = False
        self.closed_reached = False
        self.index = 0
        self.pid = PIDcontroller(kp, ki, kd)
        self.pub = rospy.Publisher('/twist', Twist, queue_size = 1)
    
    def set_current_state(self, current_state):
        self.current_state = current_state
    
    def get_current_state(self):
        return self.current_state

    def set_target_state(self):
        if self.index > waypoints.shape[0]: pass
        if not (self.open_reached or self.closed_reached):
            self.pid.setTarget(waypoints[self.index])
        elif self.open_reached:
            self.pid.setTarget(april_tag_refs[self.index])
    
    def genTwistMsg(self, desired_twist):
        """
        Convert the twist to twist msg.
        """
        twist_msg = Twist()
        twist_msg.linear.x = desired_twist[0] 
        twist_msg.linear.y = desired_twist[1] 
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = desired_twist[2]
        return twist_msg
    
    def coord(self, twist, current_state):
        J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                    [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                    [0.0,0.0,1.0]])
        return np.dot(J, twist)
    
    # def get_target_state(self):
    #     return self.target_state
        
    def move_cb(self, data):
        poses_array = data.poses
        self.set_target_state()
        if len(poses_array) == 0:
            if(np.linalg.norm(self.pid.getError(self.get_current_state(), self.pid.target)) > 0.05): 
                update_value = self.pid.update(self.get_current_state())
                twist_msg = self.genTwistMsg(
                    self.coord(
                        twist = update_value, 
                        current_state = self.get_current_state(),
                    )
                )
                self.pub.publish(twist_msg)
                rospy.sleep(0.05)
                self.set_current_state(self.get_current_state() + update_value)
            else:
                rospy.loginfo("OPEN LOOP REACHED")
                self.open_reached = True      
        else:
            # Seperate this part into a single function
            curr_pose = poses_array[0].position
            curr_quat = poses_array[0].orientation
            curr_x, curr_z = curr_pose.x, curr_pose.z
            _, curr_r, _ = euler_from_quaternion(
                [
                    curr_quat.w,
                    curr_quat.x,
                    curr_quat.y,
                    curr_quat.z,
                ]
            )
            curr_state = np.array([-curr_z, curr_x, curr_r])
            # self.set_current_state(curr_state)
            
            if np.any(self.pid.getError(curr_state, self.pid.target) > 0.10):
                update_value = self.pid.update(curr_state)
                twist_msg = self.genTwistMsg(
                    self.coord(
                        twist = update_value, 
                        current_state = curr_state,
                    )
                )
                self.pub.publish(twist_msg)
                rospy.sleep(0.05)
            else:
                rospy.loginfo("CLOSED LOOP REACHED")
                self.closed_reached = True
                if self.index < waypoints.shape[0]:
                    self.set_current_state(waypoints[self.index])
                    self.index += 1
                else:
                    self.pub.publish(self.genTwistMsg(np.array(0.0, 0.0, 0.0)))


auto_controller = AutoController([0.1, 0.02, 0.05])
auto_controller.set_current_state(np.array([0.0, 0.0, 0.0]))

if __name__ == "__main__":
    rospy.init_node('project2')

    rospy.Subscriber('/april_poses', PoseArray, auto_controller.move_cb, queue_size = 1)

    rospy.spin()


