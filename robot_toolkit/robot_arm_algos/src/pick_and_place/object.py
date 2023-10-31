import numpy as np
import cv2
import matplotlib.pyplot as plt
from ..robot_camera_calibration._calibration_data_utils import tf_from_rvectvec
from ..logger import logger
from ..tags_detection._fiducial import TagDetection

def transform_2_object_frame(cam2base_tf, obj_rvec_in_cam, obj_tvec_in_cam, ee_rvec_in_base, ee_tvec_in_base):
    obj2cam_tf = tf_from_rvectvec(obj_rvec_in_cam, obj_tvec_in_cam)
    grasp2base_tf = tf_from_rvectvec(ee_rvec_in_base, ee_tvec_in_base)
    grasp2cam_tf = np.matmul(np.linalg.inv(cam2base_tf), grasp2base_tf)
    grasp2obj_tf = np.matmul(np.linalg.inv(obj2cam_tf), grasp2cam_tf)
    return grasp2obj_tf

def plot_image(image, blocking = False, transient = True, transient_time = 3.5):
    plt.imshow(image)
    plt.show(block=blocking)
    if transient:
        plt.pause(transient_time)
        plt.close() 
    
class Object:
    def __init__(self, object_name,
                        tag=None,
                        grasp_predictor=None,
                        pose_estimator=None):
        self.object_name = object_name
        self.tag = tag
        self.pose_estimator = pose_estimator
        self.grasp_predictor = grasp_predictor
        assert(not(tag is None and pose_estimator is None and grasp_predictor is None)), "You need to provide a pose estimator or grasp predictor if an AR tag is not attached to the object."

    def get_object_pose(self, color_image, camera, debug_image = False, use_tag_to_find_pose = True):
        if self.pose_estimator is None or use_tag_to_find_pose: 
            return self.get_tag_pose(color_image = color_image, 
                                camera = camera, 
                                debug_image = debug_image)
        return self.pose_estimator.estimate_pose()# if using a pose estimator 

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
        logger.info(f"Reprojection error with mean = {reproj_error}, variance = {error_variance}")
        
        if debug_image:
            self.tag.draw_detections(color_image = color_image,
                                corners = corners)
            self.tag.draw_estimated_frame(color_image = color_image,
                                        camera = camera,
                                        rvec = rvec,
                                        tvec = tvec)
            plot_image(color_image)
            
        return rvec, tvec, TagDetection(reproj_error_mean=reproj_error, 
                                        reproj_error_variance=error_variance,
                                        matched_img_pts=img_points,
                                        matched_obj_pts=obj_points)                                                      
        

    # def get_object_pose(self, rvec_tag2world, tvec_tag2world):
    #     tf_tag2world = tf_from_rvectvec(rvec_tag2world, tvec_tag2world)
    #     return rvectvec_from_tf(np.matmul(tf_tag2world, self.object2tag_offset))

    # def compute_object_pose(self, color_image, camera, debug_image = False):
    #     rvec_tag2cam, tvec_tag2cam, tag_detection = self.get_tag_pose(color_image, camera, debug_image = debug_image)
    #     rvec_object2cam, tvec_object2cam = self.get_object_pose(rvec_tag2cam, tvec_tag2cam)
        
    #     if debug_image:
    #         self.tag.draw_estimated_frame(color_image = color_image,
    #                                     camera = camera,
    #                                     rvec = rvec_object2cam,
    #                                     tvec = tvec_object2cam)
    #         plt.imshow(color_image)
    #         plt.show(block=False)
    #         plt.pause(2.5)
    #         plt.close()   

    #     return rvec_object2cam, tvec_object2cam, tag_detection        