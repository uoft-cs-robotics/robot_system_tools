import cv2
import rospy
from tf import TransformListener
import tf.transformations as tf_utils
from ._data_collector import RobotPoseCollector
from ...logger import logger
"""
queries ROS TF tree to get ee frame
"""
class ROSRobotPoseCollector(RobotPoseCollector):
    def __init__(self, ee_frame, base_frame, init_node = True):
        if(init_node):
            try:
                rospy.init_node("tf_listener")
            except rospy.exceptions.ROSException as e:
                print("Node has already been initialized, not doing a new one for tf listener")

        self.tf_listener = TransformListener()
        self.ee_frame = ee_frame
        self.base_frame = base_frame

    
    def get_ee_frame(self,):
        t = self.tf_listener.getLatestCommonTime(self.base_frame, self.ee_frame)
        ee_position, ee_quaternion = self.tf_listener.lookupTransform(self.base_frame, self.ee_frame, t)
        ee_rotation_matrix = tf_utils.quaternion_matrix(ee_quaternion)
        ee_rotation_matrix = ee_rotation_matrix[0:3, 0:3]
        return cv2.Rodrigues(ee_rotation_matrix)[0], ee_position


    