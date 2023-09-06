import time
import numpy as np
import threading
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from .camera import RGBCamera
from ..logger import logger

class ROSCamera(RGBCamera):
    def __init__(self, image_topic_name, camera_info_topic_name, init_node = True):
        if(init_node):
            try:
                rospy.init_node("image_subscriber", anonymous=True)
            except rospy.exceptions.ROSException as e:
                print("Node has already been initialized, not doing a new one for image susbscriber")
        self.rgb_image_msg = None
        self.camera_info_msg = None
        self.cv_bridge = CvBridge()
        rospy.Subscriber(image_topic_name, Image, self.rgb_image_callback)
        rospy.Subscriber(camera_info_topic_name, CameraInfo, self.camera_info_callback)
        while(self.rgb_image_msg is None):
            logger.info("Waiting for image msg")
            time.sleep(1)
            if (rospy.is_shutdown()):
                return
        camera_matrix, dist_coeffs = self.get_camera_params(self.camera_info_msg)
        RGBCamera.__init__(self, camera_matrix = camera_matrix,
                                dist_coeffs = dist_coeffs)

    def rgb_image_callback(self, rgb_img_msg):       
        if self.camera_info_msg is None:
            return 
        self.rgb_image_msg = rgb_img_msg

    def camera_info_callback(self, camera_info_msg):  
        if(self.camera_info_msg is not None):
            return
        self.camera_info_msg = camera_info_msg        

    def get_current_rgb_frame(self,):
        color_image = self.cv_bridge.imgmsg_to_cv2(self.rgb_image_msg)
        return color_image

    def get_camera_params(self, camera_info_msg):
        dist_coeffs = np.array(camera_info_msg.D[:4])
        camera_matrix = np.array(camera_info_msg.K).reshape(3,3)
        return camera_matrix, dist_coeffs
    
    