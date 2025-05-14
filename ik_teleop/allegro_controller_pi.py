import rospy
import os
import numpy as np
from copy import copy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
import time


class AllegroController:
    def __init__(self):
        self.node_name = 'allegro_controller'
        try:
            rospy.init_node(self.node_name)
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.node_name}')
            pass

        # rospy.Subscriber('/kth_franka_plant/in/inspire_cmd', JointState, self._sub_callback_delta_cmd, queue_size=1)
        rospy.Subscriber('/replay/inspire_cmd', JointState, self._sub_callback_delta_cmd, queue_size=1)
        rospy.Subscriber('/replay/inspire_cmd_absolute', JointState, self._sub_callback_absolute_cmd, queue_size=1)
        rospy.Subscriber('/inspire_hand/joint_state', JointState, self._sub_callback_joint_state, queue_size=1)
        self.pub_angles = rospy.Publisher('/inspire_hand/goal_angles', JointState, queue_size=1)
        self.pub_delta_goal_angles = rospy.Publisher('/inspire_hand/goal_angles_delta', JointState, queue_size=1)
        
        self.first_goal_received = False
        self.absolute = False

        rospy.loginfo(f'{self.node_name}: Initialized!')


    def _sub_callback_joint_state(self, data):
        self.angle_act = data.position
        if len(self.angle_act) != 6:
            rospy.logerr("Received incorrect number of angles. Expected 6.")
            return
        if not self.first_goal_received:
            self.goal_angles = self.angle_act

        if not self.absolute:
            goal_angles_msg = JointState()
            goal_angles_msg.position = [angle for angle in self.goal_angles]
            
            self.pub_angles.publish(goal_angles_msg)


    def _sub_callback_delta_cmd(self, data):
        self.absolute = False
        delta_goal_angles = data.position
        if not self.first_goal_received:
            self.first_goal_received = True
        print(f"self.angle_act: {self.angle_act}")
        print(f"delta_goal_angles: {delta_goal_angles}")
        self.goal_angles = tuple(a + b for a, b in zip(delta_goal_angles, self.angle_act))
        print(f"DLT===================self.angle_act: {self.goal_angles}")


    def _sub_callback_absolute_cmd(self, data):
        self.absolute = True
        goal_angles = data.position
        if not self.first_goal_received:
            self.first_goal_received = True
        goal_angles_msg = JointState()

        goal_angles_msg.position = [angle for angle in goal_angles]
        print(f"ABS===================self.angle_act: {goal_angles}")

        self.pub_angles.publish(goal_angles_msg)



    
    def run(self):
        try:
            rospy.spin()
        except Exception as e:
            rospy.loginfo(f"{self.node_name}: Exception occurred: {e}")
        finally:
            rospy.loginfo(f"{self.node_name}: Terminated.")



if __name__ == '__main__':
    node_name = 'allegro_controller'
    allegro_controller = AllegroController()
    allegro_controller.run()
