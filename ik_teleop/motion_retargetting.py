#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from xml.etree import ElementTree as ET
import numpy as np
from ik_teleop.teleop_utils.constants import *
from copy import deepcopy as copy
from ik_teleop.teleop_utils.vectorops import *
import time 
from ik_teleop.teleop_utils.files import *
from visualization_msgs.msg import Marker, MarkerArray


ROBOT_JOINTS_RAW = {
    'index': [3],
    'middle': [2],
    'ring': [1],
    'little': [0],
    'thumb': [4, 5]
}

ROBOT_KEYPOINTS_COUNT = 6

class AllegroRetargetingOptimizer:
    def __init__(self):
        self.node_name = 'motion_retargetting'
        try:
            rospy.init_node(self.node_name)
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.node_name}')
            pass


        rospy.Subscriber('/hand_tracking/joint_poses', PoseArray, callback=self._get_XR_joints_poses, queue_size=1)
        self.pub_mod = rospy.Publisher('/hand_tracking/keypoints_transformed', PoseArray, queue_size=10)
        self.pub_marker_mod = rospy.Publisher('/hand_tracking/marker_keypoints_transformed', MarkerArray, queue_size=10)
        self.pub = rospy.Publisher('/hand_tracking/keypoints', PoseArray, queue_size=10)
        self.pub_marker = rospy.Publisher('/hand_tracking/marker_keypoints', MarkerArray, queue_size=10)

        self.bound_info = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_bounds.yaml"))
        self.linear_scaling_factors = self.bound_info['linear_scaling_factors']
        self.finger_coords = []
        self.finger_orientations = []
        self.robot_coords = []
        self.robot_coords_array = []
        self.keypoint_translation_array = []
        # self.knuckle_points = (OCULUS_JOINTS['knuckles'][3], OCULUS_JOINTS['knuckles'][0])

        #####################################
        # self.urdf_file = get_path_in_package("robot/assets/allegro_hand_right.urdf")
        # rospy.Subscriber('/allegroHand/tf', TFMessage, self.tf_callback)
        # self.tree = ET.parse(self.urdf_file)
        self.allegro_keypoints_pose_array = []

        rospy.loginfo(f"{self.node_name}: Initialized!")





    def _get_XR_joints_poses(self, msg):
        finger_coords_array = []
        finger_orientation_array = []
        if len(msg.poses) < OCULUS_NUM_KEYPOINTS:
            rospy.loginfo(f"{self.node_name}: ERROR: not enough joints received")
            return
        for i in range(OCULUS_NUM_KEYPOINTS):
            finger_coords_array.append(self.position_to_array(msg.poses[i].position))     
        self.finger_coords_array_np = np.array(finger_coords_array)
        for i in range(OCULUS_NUM_KEYPOINTS):
            finger_orientation_array.append(self.orientation_to_array(msg.poses[i].orientation)) 
        self.finger_orientation_array_np = np.array(finger_orientation_array)

        self.transform_keypoints()
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            index = self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle = self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring = self.finger_coords_array_np[OCULUS_JOINTS['ring']],
            little = self.finger_coords_array_np[OCULUS_JOINTS['little']],
            thumb = self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
        )
        
        self.pub_marker.publish(self.create_marker_array_msg(1,0,0))
        self.pub.publish(self.create_pose_array_msg())

        self.get_keypoint_difference()

        self.align_hand_to_robot()

        self.pub_marker_mod.publish(self.create_marker_array_msg(0,1,0))
        self.pub_mod.publish(self.create_pose_array_msg())

    def create_pose_array_msg(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = "palm_link"
        for finger in self.finger_coords:
            for joint in range(0, len(self.finger_coords[finger])):
                pose = Pose()
                pose.position.x = self.finger_coords[finger][joint][0]
                pose.position.y = self.finger_coords[finger][joint][1]
                pose.position.z = self.finger_coords[finger][joint][2]
                pose.orientation.x = self.finger_orientations[finger][joint][0]
                pose.orientation.y = self.finger_orientations[finger][joint][1]
                pose.orientation.z = self.finger_orientations[finger][joint][2]
                pose.orientation.w = self.finger_orientations[finger][joint][3]
                pose_array_msg.poses.append(pose)
        return pose_array_msg
        
    def create_marker_msg(self, position, r=1.0, g=0.0, b=0.0, a=.5, scale=0.02):
        marker = Marker()
        marker.header.frame_id = "palm_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        return marker

    def create_marker_array_msg(self, r, g, b):
        marker_array_msg = MarkerArray()
        marker_id = 0
        for finger in self.finger_coords:
            for joint in self.finger_coords[finger]:
                marker = self.create_marker_msg(joint, r, g, b)
                marker.id = marker_id
                marker_array_msg.markers.append(marker)
                marker_id += 1
        return marker_array_msg

    def _translate_coords(self, coords):
        return copy(coords) - coords[12]
        
    def _get_coord_frame(self, wrist_coord, index_knuckle_coord, little_knuckle_coord):
        z_axis = normalize_vector(wrist_coord)
        x_axis = normalize_vector(index_knuckle_coord - little_knuckle_coord)
        y_axis = normalize_vector(np.cross(z_axis, x_axis))
        
        return [y_axis, x_axis, -z_axis] # Change from left-handed unity system to right-handed
    


    def transform_keypoints(self):
        self.finger_coords_array_np = self._translate_coords(self.finger_coords_array_np)
        original_coord_frame = self._get_coord_frame(
            self.finger_coords_array_np[OCULUS_JOINTS['wrist'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['index'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['ring'][0]]
        )

        if np.linalg.det(original_coord_frame) == 0:
            rospy.logerr("Original coord frame is singular and cannot be inverted")
            return

        try:
            # Compute the rotation matrix
            rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T

            # Transform positions
            finger_coords = (rotation_matrix @ self.finger_coords_array_np.T).T

            # Transform orientations
            transformed_orientations = []
            for quaternion in self.finger_orientation_array_np:
                rotation_matrix_quat = quaternion_matrix(quaternion)[:3, :3]
                transformed_matrix = rotation_matrix @ rotation_matrix_quat
                full_transform_matrix = np.eye(4)
                full_transform_matrix[:3, :3] = transformed_matrix
                transformed_quat = quaternion_from_matrix(full_transform_matrix)
                transformed_orientations.append(transformed_quat)

            self.finger_coords_array_np = finger_coords
            self.finger_orientation_array_np = np.array(transformed_orientations)
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing rotation matrix: {e}")

        # Update finger coordinate dictionary
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            thumb=self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_coords_array_np[OCULUS_JOINTS['ring']],
        )
        # Update finger orientation dictionary
        self.finger_orientations = dict(
            wrist=self.finger_orientation_array_np[OCULUS_JOINTS['wrist']],
            thumb=self.finger_orientation_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_orientation_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_orientation_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_orientation_array_np[OCULUS_JOINTS['ring']],
        )

        

    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    def orientation_to_array(self, orientation):
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    def calculate_finger_angles(self, finger_type, finger_joint_coords, metacarpals_coords, curr_angles, moving_avg_arr):
        if finger_type == 'index':
            idx = 1
        elif finger_type == 'middle':
            idx = 2
        elif finger_type == 'ring':
            idx = 3
        elif finger_type == 'little':
            idx = 4
        angle = calculate_angle(
            metacarpals_coords[idx],
            finger_joint_coords[1],
            finger_joint_coords[3]
        )
        print(f"finger_joint_coords {finger_joint_coords}")

        angle * self.linear_scaling_factors[idx]
        return angle * self.linear_scaling_factors[idx]

    def calculate_joint_1_angle(self, thumb_joint_coords):

        origin = thumb_joint_coords[1]
        reference_point = thumb_joint_coords[1].copy()
        reference_point[2] += 1

        vector_origin_to_index = reference_point - origin
        vector_origin_to_thumb = thumb_joint_coords[2] - origin

        if np.linalg.norm(vector_origin_to_index) == 0 or np.linalg.norm(vector_origin_to_thumb) == 0:
            print("One of the vectors is zero, unable to compute angle.")
            return np.nan

        z_axis = np.cross(vector_origin_to_index, vector_origin_to_thumb)
        if np.linalg.norm(z_axis) == 0:
            print("Cross product resulted in zero vector; vectors might be parallel.")
            return np.nan

        z_axis /= np.linalg.norm(z_axis)
        x_axis = vector_origin_to_index / np.linalg.norm(vector_origin_to_index)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_thumb)

        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

        return angle

    def calculate_joint_2_angle(self, thumb_joint_coords):

        origin = thumb_joint_coords[2]
        vector_origin_to_joint_2 = thumb_joint_coords[3] - origin
        vector_origin_to_joint_0 = thumb_joint_coords[1] - origin

        z_axis = np.cross(vector_origin_to_joint_2, vector_origin_to_joint_0)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = vector_origin_to_joint_2 / np.linalg.norm(vector_origin_to_joint_2)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint_0)
        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

        return 3.14-angle

    def calculate_joint_3_angle(self, thumb_joint_coords):
        origin = thumb_joint_coords[2]

        vector_origin_to_tip = thumb_joint_coords[3] - origin
        vector_origin_to_joint = thumb_joint_coords[1] - origin

        z_axis = np.cross(vector_origin_to_tip, vector_origin_to_joint)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = vector_origin_to_tip / np.linalg.norm(vector_origin_to_tip)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint)
        angle = 3.14 - np.arctan2(vector_in_plane[1], vector_in_plane[0])
        if angle < 0:
            angle = 0
        return angle



    def calculate_thumb_angles(self, thumb_joint_coords, curr_angles, moving_avg_arr):

        calc_finger_angles = []
        # joint 1
        angle = self.calculate_joint_1_angle(thumb_joint_coords)
        print(f"angle1 {angle}")
        angle -= 2.3
        # print(f"angle1 {angle}")
        # time.sleep(0.1)
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[1])
        
        # joint 2
        angle = self.calculate_joint_2_angle(thumb_joint_coords)
        angle += 0.2
        # print(f"angle2 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[2])


        # joint 3
        angle = self.calculate_joint_3_angle(thumb_joint_coords)
        angle -= 0.2
        # print(f"angle3 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[3])
        
        angle = -calculate_angle_z(
            [1.0,0.0,0.0],
            [0.0,0.0,0.0],
            thumb_joint_coords[1]
        )
        # angle = 0.263
        # angle = 1.396
        angle += 2.8
        # print(f"angle0 {angle}")
        calc_finger_angles.append(angle * self.rotatory_thumb_scaling_factors[0])

        filtered_angles = self._get_filtered_thumb_angles("thumb", calc_finger_angles, curr_angles, moving_avg_arr)
        # print(f"filtered_angles {filtered_angles}")
        return filtered_angles

    def run(self):
        try:
            rospy.spin()
        except Exception as e:
            rospy.loginfo(f"{self.node_name}: Exception occurred: {e}")
        finally:
            rospy.loginfo(f"{self.node_name}: Terminated.")



if __name__ == "__main__":
    listener = AllegroRetargetingOptimizer()
    listener.run()
