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
        self.finger_types = ['index', 'middle', 'ring', 'thumb']
        self.current_joint_state = JointState()
        self.processes = []  # Track subprocesses

        # To track publishing frequency
        self.last_publish_time = None  # Timestamp of the last message
        self.publish_timestamps = []  # List of (timestamp, frequency)


        self.pub_delta_goal_angles = rospy.Publisher('/inspire_hand/goal_angles_delta', JointState, queue_size=1)
        
        self.paused = False
        rospy.Subscriber('/hand_tracking/pause', Bool, self._pause_callback, queue_size=1)
        rospy.Subscriber('/motion_retargetting/goal_angles_raw', Float32MultiArray, self._sub_callback_goal_angles, queue_size=1)
        rospy.Subscriber('/inspire_hand/joint_state', JointState, self._sub_callback_joint_state, queue_size=1)
        self.pub_angles = rospy.Publisher('/inspire_hand/goal_angles', JointState, queue_size=1)
        
        self.scaling_factors = [1.4,1.3,1.3,1.3,4,5]
        self.offset_factors = [0,0,0,0,0,-1.7]
        self.first_goal_received = False
        self.prev_goal_angles = None

        rospy.loginfo(f'{self.node_name}: Initialized!')

    def _pause_callback(self, msg):
        self.paused = msg.data


    def _sub_callback_goal_angles(self, data):
        # Calculate average and lowest frequency from the last 10 seconds
        # print("callback goal!!")
        # Rest of the code for publishing desired joint states
        try:
            self.goal_angles = data.data
            if len(self.goal_angles) != 6:
                rospy.logerr("Received incorrect number of angles. Expected 6.")
                return
            self.first_goal_received = True
            # Apply scaling and offset
            self.goal_angles = np.add(self.goal_angles, self.offset_factors)
            self.goal_angles = np.multiply(self.goal_angles, self.scaling_factors)

            # Check if angles are within the range [0, π]
            for i, angle in enumerate(self.goal_angles):
                if angle < 0 or angle > np.pi:
                    self.goal_angles[i] = np.clip(angle, 0, np.pi)

            # Convert to new scale (assuming radians to degrees or another scale)
            self.goal_angles = np.multiply(self.goal_angles, np.pi*100)
            
            # Adjust goal angles by subtracting from 1000
            self.goal_angles = 1000 - np.round(self.goal_angles).astype(int)
            # rospy.loginfo(f'++++++++ self.goal_angles {self.goal_angles}')

            # self.pub_angles.publish(self.goal_angles)

        except:
            rospy.loginfo(f'{self.node_name}: WARN: Goal angles missing! Waiting...')
            time.sleep(1)
            pass
	
    def _sub_callback_joint_state(self, data):
        self.angle_act = data.position
        if len(self.angle_act) != 6:
            rospy.logerr("Received incorrect number of angles. Expected 6.")
            return
        if not self.first_goal_received:
            self.goal_angles = self.angle_act
            return

        if self.paused:
            return

        # Compute delta between current and previous goal angles
        if self.prev_goal_angles is not None:
            delta_goal_angles = tuple(self.goal_angles[i] - self.prev_goal_angles[i] for i in range(len(self.goal_angles)))

            # Publish delta_goal_angles
            delta_goal_angles_msg = JointState()
            delta_goal_angles_msg.position = [angle for angle in delta_goal_angles]
            self.pub_delta_goal_angles.publish(delta_goal_angles_msg)

        # Publish current goal angles
        goal_angles_msg = JointState()
        goal_angles_msg.position = [angle for angle in self.goal_angles]
        self.pub_angles.publish(goal_angles_msg)

        # Update previous goal angles
        self.prev_goal_angles = copy(self.goal_angles)

    def _calculate_recent_frequency_stats(self, current_time):
        # Keep only the data from the last 10 seconds
        self.publish_timestamps = [
            (timestamp, frequency) for timestamp, frequency in self.publish_timestamps
            if current_time - timestamp <= 10
        ]

        if self.publish_timestamps:
            # Extract frequencies from the last 10 seconds
            frequencies = [freq for _, freq in self.publish_timestamps]
            average_frequency = np.mean(frequencies)
            lowest_frequency = np.min(frequencies)

            # Log or publish the stats
            #rospy.loginfo(f"{self.node_name}: Average frequency (last 10s): {average_frequency:.2f} Hz")
            #rospy.loginfo(f"{self.node_name}: Lowest frequency (last 10s): {lowest_frequency:.2f} Hz")


    def _sub_callback_index_delta_cmd(self, data):
        self.index_delta_cmd = data.position
        self.index_mutex = True

    def _sub_callback_middle_delta_cmd(self, data):
        self.middle_delta_cmd = data.position
        self.middle_mutex = True

    def _sub_callback_ring_delta_cmd(self, data):
        self.ring_delta_cmd = data.position
        self.ring_mutex = True

    def _sub_callback_thumb_delta_cmd(self, data):
        self.thumb_delta_cmd = data.position
        self.thumb_mutex = True

    
    def run(self):
        try:
            rospy.spin()
        except Exception as e:
            rospy.loginfo(f"{self.node_name}: Exception occurred: {e}")
        finally:
            rospy.loginfo(f"{self.node_name}: Terminated.")



if __name__ == '__main__':
    node_name = 'inspire_controller'
    #rospy.init_node(node_name)
    inspire_controller = InspireController()
    inspire_controller.run()
    # Set up signal handler for Ctrl+C


    # try:
    #     rospy.spin()
    # except Exception as e:
    #     rospy.loginfo(f"{node_name}: Exception occurred: {e}")
    # finally:
    #     rospy.loginfo(f"{node_name}: Terminated.")
