import numpy as np
import cv2
import matplotlib.pyplot as plt
from .aruco_board import ArucoBoard
from ..robot_camera_calibration._calibration_data_utils import tf_from_rvectvec, rvectvec_from_tf
from ..logger import logger
from ._fiducial import TagDetection

class Object:
    def __init__(self, object_name,
                        tag, 
                        object2tag_offset = np.eye(4)):
        self.object_name = object_name
        self.tag = tag
        self.object2tag_offset = object2tag_offset
    
    def get_object_pose(self, rvec_tag2world, tvec_tag2world):
        tf_tag2world = tf_from_rvectvec(rvec_tag2world, tvec_tag2world)
        return rvectvec_from_tf(np.matmul(tf_tag2world, self.object2tag_offset))

    def compute_object_pose(self, color_image, camera, debug_image = False):
        rvec_tag2cam, tvec_tag2cam, tag_detection = self.get_tag_pose(color_image, camera, debug_image = debug_image)
        rvec_object2cam, tvec_object2cam = self.get_object_pose(rvec_tag2cam, tvec_tag2cam)
        
        if debug_image:
            self.tag.draw_estimated_frame(color_image = color_image,
                                        camera = camera,
                                        rvec = rvec_object2cam,
                                        tvec = tvec_object2cam)
            plt.imshow(color_image)
            plt.show(block=False)
            plt.pause(2.5)
            plt.close()   

        return rvec_object2cam, tvec_object2cam, tag_detection

    def get_tag_pose(self, color_image, camera, debug_image = False):
        corners, ids, _ = self.tag.detect_markers(color_image)
        if ids is None:
            logger.debug("No tag markers found in this image")
            return None,  None, None
        image_gray =  cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        self.tag.refine_corners(image_gray, corners)
        if debug_image:
            self.tag.draw_detections(color_image = color_image,
                                corners = corners)

        obj_points, img_points = self.tag.get_matched_object_points(corners, ids)
        rvec, tvec = self.tag.estimate_pose(obj_points, img_points, camera)
        reproj_error, error_variance = self.tag.compute_reprojection_error(rvec,
                                                            tvec,
                                                            obj_points,
                                                            img_points,
                                                            camera)   
  

        return rvec, tvec, TagDetection(reproj_error_mean=reproj_error, 
                                        reproj_error_variance=error_variance,
                                        matched_img_pts=img_points,
                                        matched_obj_pts=obj_points)                                                      