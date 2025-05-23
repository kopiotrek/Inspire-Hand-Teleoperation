import rospy
from copy import copy
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import JointState


class InspireController:
    def __init__(self):
        self.node_name = 'inspire_controller'
        try:
            rospy.init_node(self.node_name)
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.node_name}')
            pass

        rospy.Subscriber('/kth_franka_plant/in/inspire_cmd', JointState, self._sub_callback_delta_cmd, queue_size=1)
        rospy.Subscriber('/kth_franka_plant/in/grasp_signal', Float32, self._sub_callback_grasp_signal, queue_size=1)
        rospy.Subscriber('/button', Bool, self._sub_callback_button, queue_size=1)
        rospy.Subscriber('/replay/inspire_cmd_absolute', JointState, self._sub_callback_absolute_cmd, queue_size=1)
        rospy.Subscriber('/inspire_hand/joint_state', JointState, self._sub_callback_joint_state, queue_size=1)
        self.pub_angles = rospy.Publisher('/inspire_hand/goal_angles', JointState, queue_size=1)
        
        self.first_goal_received = False
        self.absolute = False
        self.prev_goal_angles = None
        self.new_goal_received = False
        self.grasped = False
        self.button_pressed = False
        self.grasp_treshold = 0.2
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

        if self.grasped:
            return
        
        if self.button_pressed:
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
            self.pub_angles.publish(goal_angles_msg)
            self.new_goal_received = False

            # Update previous goal for next delta application
            self.prev_goal_angles = copy(self.goal_angles)

    def _sub_callback_button(self, data):
        if data.data:
            self.button_pressed = True
            goal_angles = copy(self.prev_goal_angles)
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 0
            goal_angles[3] = 0
            goal_angles_msg = JointState()
            goal_angles_msg.position = [angle for angle in goal_angles]
            self.pub_angles.publish(goal_angles_msg)
            self.new_goal_received = False
        else:
            self.button_pressed = False
            self.button_pressed = True
            goal_angles = copy(self.prev_goal_angles)
            goal_angles[0] = 0
            goal_angles[1] = 0
            goal_angles[2] = 1000
            goal_angles[3] = 1000
            goal_angles_msg = JointState()
            goal_angles_msg.position = [angle for angle in goal_angles]
            self.pub_angles.publish(goal_angles_msg)
            self.new_goal_received = False

            

    def _sub_callback_grasp_signal(self, data):
        if data.data > self.grasp_treshold:
            self.grasped = True
        # else:
            # self.grasped = False


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
        # print(f"ABS===================self.angle_act: {goal_angles}")

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
