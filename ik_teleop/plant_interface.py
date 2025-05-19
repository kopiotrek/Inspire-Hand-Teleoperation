import rospy
import os
import numpy as np
from copy import copy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
import time


class InspireController:
    def __init__(self):
        self.node_name = 'inspire_controller'
        try:
            rospy.init_node(self.node_name)
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.node_name}')
            pass

        rospy.Subscriber('/kth_franka_plant/in/inspire_cmd', JointState, self._sub_callback_delta_cmd, queue_size=1)
        # rospy.Subscriber('/replay/inspire_cmd', JointState, self._sub_callback_delta_cmd, queue_size=1)
        rospy.Subscriber('/replay/inspire_cmd_absolute', JointState, self._sub_callback_absolute_cmd, queue_size=1)
        rospy.Subscriber('/inspire_hand/joint_state', JointState, self._sub_callback_joint_state, queue_size=1)
        self.pub_angles = rospy.Publisher('/inspire_hand/goal_angles', JointState, queue_size=1)
        
        self.first_goal_received = False
        self.absolute = False
        self.prev_goal_angles = None

        rospy.loginfo(f'{self.node_name}: Initialized!')

    def _sub_callback_joint_state(self, data):
        self.angle_act = data.position
        if len(self.angle_act) != 6:
            rospy.logerr("Received incorrect number of angles. Expected 6.")
            return
        if not self.first_goal_received:
            self.goal_angles = self.angle_act
            self.prev_goal_angles = self.angle_act
            return

        if not self.absolute:
            if self.prev_goal_angles is None:
                self.prev_goal_angles = self.angle_act  # Fallback init

            self.goal_angles = list(a + b for a, b in zip(self.delta_goal_angles, self.prev_goal_angles))
            for i in range(0,len(self.goal_angles)):
                if self.goal_angles[i] > 1000:
                    self.goal_angles[i] = 1000
                elif self.goal_angles[i] < 0:
                    self.goal_angles[i] = 0
            print(f"DLT===================self.angle_act: {self.goal_angles}")

            goal_angles_msg = JointState()
            goal_angles_msg.position = [angle for angle in self.goal_angles]
            self.pub_angles.publish(goal_angles_msg)

            # Update previous goal for next delta application
            self.prev_goal_angles = copy(self.goal_angles)


    def _sub_callback_delta_cmd(self, data):
        self.absolute = False
        self.delta_goal_angles = data.position
        if not self.first_goal_received:
            self.first_goal_received = True


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
    node_name = 'inspire_controller'
    inspire_controller = InspireController()
    inspire_controller.run()
