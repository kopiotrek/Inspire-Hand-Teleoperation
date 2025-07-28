import rospy
from copy import copy
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState
import time
import argparse

class InspireController:
    def __init__(self, args):
        self.node_name = 'inspire_controller'
        try:
            rospy.init_node(self.node_name)
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.node_name}')
            pass

        rospy.Subscriber('/kth_franka_plant/in/inspire_cmd', JointState, self._sub_callback_delta_cmd, queue_size=1)
        rospy.Subscriber('/kth_franka_plant/in/grasp_signal', Float32, self._sub_callback_grasp_signal, queue_size=1)
        rospy.Subscriber('/kth_franka_plant/in/index_signal', Float32, self._sub_callback_index_signal, queue_size=1)
        rospy.Subscriber('/kth_franka_plant/in/index_middle_signal', Float32, self._sub_callback_index_middle_signal, queue_size=1)
        rospy.Subscriber('/kth_franka_plant/in/thumb_signal', Float32, self._sub_callback_thumb_signal, queue_size=1)
        rospy.Subscriber('/activate_button', Bool, self._sub_callback_activate_button, queue_size=1)
        rospy.Subscriber('/release_button', Bool, self._sub_callback_release_button, queue_size=1)
        rospy.Subscriber('/quest/noding_gesture', Bool, self._sub_callback_nodding_gesture, queue_size=1)
        rospy.Subscriber('/replay/inspire_cmd_absolute', JointState, self._sub_callback_absolute_cmd, queue_size=1)
        rospy.Subscriber('/inspire_hand/joint_state', JointState, self._sub_callback_joint_state, queue_size=1)
        self.pub_angles = rospy.Publisher('/inspire_hand/goal_angles', JointState, queue_size=0)
        self.pub_grasp_state = rospy.Publisher('/grasp_state', Bool, queue_size=0)

        self.first_goal_received = False
        self.absolute = False
        self.prev_goal_angles = None
        self.new_goal_received = False
        self.grasped = False
        self.activate_button_pressed = False
        self.mode = 'index'
        self.thumb_xtra_var = False
        self.mode_treshold = 0.8
        self.finger_signal_tolerance = 0.8
        self.index_signal = 0.0
        self.index_middle_signal = 0.0
        self.thumb_signal = 0.0
        self.stop_checking_mode = False

        if self.mode == 'index_middle':
            self.grasp_treshold = 0.8
        elif self.mode == 'thumb':
            self.grasp_treshold = 0.8
        elif self.mode == 'index':
            self.grasp_treshold = 0.8
        print(f"self.mode {self.mode}")
        self.home_robot()
        rospy.loginfo(f'{self.node_name}: Initialized!')

    def home_robot(self):
        #              little  ring    middle  index   t.flex  t.rotate
        goal_angles = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 0.0]
        goal_angles_msg = JointState()
        goal_angles_msg.position = [angle for angle in goal_angles]
        while self.pub_angles.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for subscribers to /inspire_hand/goal_angles...")
            rospy.sleep(0.1)
        self.pub_angles.publish(goal_angles_msg)
        rospy.loginfo(f"{self.node_name}: Home position")

    def _sub_callback_joint_state(self, data):
        self.angle_act = data.position
        if len(self.angle_act) != 6:
            rospy.logerr("Received incorrect number of angles. Expected 6.")
            return
        if not self.first_goal_received:
            self.goal_angles = list(self.angle_act)

            # self.goal_angles = self.angle_act
            self.prev_goal_angles = self.goal_angles
            return

        if self.grasped:
            return
        
        if not self.absolute and self.new_goal_received:
            if self.prev_goal_angles is None:
                self.prev_goal_angles = self.angle_act  # Fallback init

            self.goal_angles = list(a + b for a, b in zip(self.delta_goal_angles, self.prev_goal_angles))
            for i in range(0,len(self.goal_angles)):
                if self.goal_angles[i] > 1000:
                    self.goal_angles[i] = 1000
                elif self.goal_angles[i] < 0:
                    self.goal_angles[i] = 0
            # print(f"DLT===================self.angle_act: {self.goal_angles}")

            goal_angles_msg = JointState()
            goal_angles_msg.position = [angle for angle in self.goal_angles]
            # print(goal_angles_msg)

            self.pub_angles.publish(goal_angles_msg)
            self.new_goal_received = False

            # Update previous goal for next delta application
            self.prev_goal_angles = copy(self.goal_angles)

    def _sub_callback_nodding_gesture(self, data):
        if data.data:
            if self.grasped:
                self.activate_button_pressed = ~self.activate_button_pressed 
                if self.activate_button_pressed:
                    for _ in range(10):
                        if self.mode == 'index_middle':
                            self.pub_angles.publish(self.activate_index_middle_goal_angles(self.activate_button_pressed))
                        elif self.mode == 'thumb':
                            self.pub_angles.publish(self.activate_thumb_goal_angles(self.activate_button_pressed))
                        elif self.mode == 'index':
                            self.pub_angles.publish(self.activate_index_goal_angles(self.activate_button_pressed))
                        time.sleep(0.02)
                    self.new_goal_received = False
                else:
                    if self.mode == 'index_middle':
                        self.pub_angles.publish(self.activate_index_middle_goal_angles(self.activate_button_pressed))
                    elif self.mode == 'thumb':
                        self.pub_angles.publish(self.activate_thumb_goal_angles(self.activate_button_pressed))
                    elif self.mode == 'index':
                        self.pub_angles.publish(self.activate_index_goal_angles(self.activate_button_pressed))
                    self.new_goal_received = False
        else:
            goal_angles = copy(self.prev_goal_angles)
            print(goal_angles)
            goal_angles[0] = 1000
            goal_angles[1] = 1000
            goal_angles[2] = 1000
            goal_angles[3] = 1000
            goal_angles[4] = 1000
            goal_angles[5] = 0
            self.prev_goal_angles = goal_angles
            goal_angles_msg = JointState()
            goal_angles_msg.position = [angle for angle in goal_angles]
            for _ in range(30):  # 10 Hz * 3 seconds = 30 iterations
                self.pub_angles.publish(goal_angles_msg)
                time.sleep(0.1)  # 1 / 10 Hz
            self.new_goal_received = False
            self.grasped = False
            self.thumb_xtra_var = False
            self.stop_checking_mode = False
            

    def _sub_callback_activate_button(self, data):
        if data.data:
            self.activate_button_pressed = True
            for _ in range(10):
                if self.mode == 'index_middle':
                    self.pub_angles.publish(self.activate_index_middle_goal_angles(self.activate_button_pressed))
                elif self.mode == 'thumb':
                    self.pub_angles.publish(self.activate_thumb_goal_angles(self.activate_button_pressed))
                elif self.mode == 'index':
                    self.pub_angles.publish(self.activate_index_goal_angles(self.activate_button_pressed))
                time.sleep(0.1)
            self.new_goal_received = False
        else:
            self.activate_button_pressed = False
            if self.mode == 'index_middle':
                self.pub_angles.publish(self.activate_index_middle_goal_angles(self.activate_button_pressed))
            elif self.mode == 'thumb':
                self.pub_angles.publish(self.activate_thumb_goal_angles(self.activate_button_pressed))
            elif self.mode == 'index':
                self.pub_angles.publish(self.activate_index_goal_angles(self.activate_button_pressed))
            self.new_goal_received = False

    def _sub_callback_release_button(self, data):
        if data.data:
            goal_angles = copy(self.prev_goal_angles)
            goal_angles[0] = 1000
            goal_angles[1] = 1000
            goal_angles[2] = 1000
            goal_angles[3] = 1000
            goal_angles[4] = 1000
            goal_angles[5] = 0
            self.prev_goal_angles = goal_angles
            goal_angles_msg = JointState()
            goal_angles_msg.position = [angle for angle in goal_angles]
            for _ in range(30):  # 10 Hz * 3 seconds = 30 iterations
                self.pub_angles.publish(goal_angles_msg)
                time.sleep(0.1)  # 1 / 10 Hz
            self.new_goal_received = False
            self.grasped = False
            self.thumb_xtra_var = False
            self.stop_checking_mode = False



    def activate_index_middle_goal_angles(self, activate):
        goal_angles_msg = JointState()
        goal_angles = copy(self.prev_goal_angles)
        if activate:
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 0
            goal_angles[3] = 0
            goal_angles_msg.position = [angle for angle in goal_angles]
            return goal_angles_msg
        else:
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 1000
            goal_angles[3] = 1000
            goal_angles_msg.position = [angle for angle in goal_angles]
            return goal_angles_msg


    def activate_thumb_goal_angles(self, activate):
        goal_angles_msg = JointState()
        goal_angles = copy(self.prev_goal_angles)

        #                little  ring    middle  index   t.flex  t.rotate
        # goal_angles = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 0.0]

        if activate:
            if self.thumb_xtra_var == False:
                goal_angles[0] = 0
                goal_angles[1] = 0
                goal_angles[2] = 0
                goal_angles[3] = 0
                goal_angles[4] = 1000
                goal_angles[5] = 800
                goal_angles_msg.position = [angle for angle in goal_angles]
                self.pub_angles.publish(goal_angles_msg)
                time.sleep(.5)
                self.thumb_xtra_var = True
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 0
            goal_angles[3] = 0
            goal_angles[4] = 0
            goal_angles[5] = 800
            goal_angles_msg.position = [angle for angle in goal_angles]
            return goal_angles_msg

        else:
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 0
            goal_angles[3] = 0
            goal_angles[4] = 1000
            if self.thumb_xtra_var == False:
                goal_angles[5] = 0
            else:
                goal_angles[5] = 800
            goal_angles_msg.position = [angle for angle in goal_angles]
            return goal_angles_msg


    def activate_index_goal_angles(self, activate):
        goal_angles_msg = JointState()
        goal_angles = copy(self.prev_goal_angles)
        if activate:
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 0
            goal_angles[3] = 0
            goal_angles_msg.position = [angle for angle in goal_angles]
            return goal_angles_msg
        else:
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 0
            goal_angles[3] = 1000
            goal_angles_msg.position = [angle for angle in goal_angles]
            return goal_angles_msg


    def _sub_callback_grasp_signal(self, data):
        # self.grasped = True
        if data.data > self.grasp_treshold and self.grasped is False:
            self.grasped = True
            msg = Bool()
            msg.data = True
            self.pub_grasp_state.publish(msg)
            time.sleep(.5)
            self.stop_checking_mode = True
            if self.mode == 'index_middle':
                self.pub_angles.publish(self.activate_index_middle_goal_angles(False))
            elif self.mode == 'thumb':
                self.pub_angles.publish(self.activate_thumb_goal_angles(False))
            elif self.mode == 'index':
                self.pub_angles.publish(self.activate_index_goal_angles(False))
            self.new_goal_received = False


    def _sub_callback_index_signal(self, data):
        if not self.stop_checking_mode:
            self.index_signal = data.data
            if self.index_signal > self.mode_treshold:
                if self.index_signal > self.index_middle_signal and self.index_signal > self.thumb_signal:
                    self.mode = 'index'

    def _sub_callback_index_middle_signal(self, data):
        if not self.stop_checking_mode:
            self.index_middle_signal = data.data
            if self.index_middle_signal > self.mode_treshold:
                if self.index_middle_signal > self.index_signal and self.index_middle_signal > self.thumb_signal:
                    self.mode = 'index_middle'

    def _sub_callback_thumb_signal(self, data):
        if not self.stop_checking_mode:
            self.thumb_signal = data.data
            if self.thumb_signal > self.mode_treshold:
                if self.thumb_signal > self.index_middle_signal and self.thumb_signal > self.index_signal:
                    self.mode = 'thumb'


            


    def _sub_callback_delta_cmd(self, data):
        self.absolute = False
        self.new_goal_received = True
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

        self.pub_angles.publish(goal_angles_msg)

    def run(self):
        try:
            rospy.spin()
        except Exception as e:
            rospy.loginfo(f"{self.node_name}: Exception occurred: {e}")
        finally:
            rospy.loginfo(f"{self.node_name}: Terminated.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Controller used with plant interface")
    args = parser.parse_args()

    node_name = 'inspire_controller'
    inspire_controller = InspireController(args)
    inspire_controller.run()
