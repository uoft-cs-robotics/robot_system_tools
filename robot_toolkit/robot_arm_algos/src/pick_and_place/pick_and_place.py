"""
Pick n Place example with one robot and one camera with known robot camera extrinsics.

Author: Ruthrash Hari
Date: 2/10/23 
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
import dataclasses
from ..robot_camera_calibration._calibration_data_utils import tf_from_rvectvec, rvectvec_from_tf
from ..logger import logger
from ..config_reader import read_yaml_file

def get_offset_from_pose(pose, offset_dist = 0.15, offset_in_obj_frame = True):
    offsetvect_poseframe = np.array([0.0, 0.0, offset_dist])
    offset_pose = pose.copy()  
    if offset_in_obj_frame:
        grasp2robotbase_rot = offset_pose[0:3, 0:3]
        offsetvect_robotbaseframe = np.matmul(grasp2robotbase_rot, -1.0 * offsetvect_poseframe)
    else:
        offsetvect_robotbaseframe = offsetvect_poseframe
    offset_pose[0:3,-1] += offsetvect_robotbaseframe
    return offset_pose

def plot_image(image, blocking = True, transient = False, transient_time = 3.5):
    plt.imshow(image)
    plt.show(block=blocking)
    if transient:
        plt.pause(transient_time)
        plt.close() 
        
def draw_estimated_frame(color_image, camera, rvec, tvec):
    cv2.drawFrameAxes(color_image, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
    return color_image    

class PickAndPlace:
    def __init__(self, robot_arm_object, camera, cam_extrinsics, camera_in_hand = True):
        self.robot_arm = robot_arm_object
        self.camera = camera
        self.cam_extrinsics = cam_extrinsics#either cam2base: camera in environment or cam2gripper: camera in hand
        self.camera_in_hand = camera_in_hand 
        
    def get_cam2base_tf(self,):
        if(self.camera_in_hand):        
            ee2base_tf = self.robot_arm.get_ee_frame()
            return np.matmul(ee2base_tf, self.cam_extrinsics)
        else: 
            return self.cam_extrinsics
    
    def pick_object(self, object_, grasps_file, grasp_name = None):
        logger.debug(f"performing grasps for object{object_.object_name}")
        grasp_pose = self.get_grasp_pose(object_ = object_,
                                         grasps_file = grasps_file,
                                         grasp_name = grasp_name)
        pre_grasp_pose = get_offset_from_pose(pose = grasp_pose, offset_in_obj_frame = True)
        post_grasp_pose = get_offset_from_pose(pose = grasp_pose, offset_in_obj_frame = False)
        self.robot_arm.open_gripper()
        self.robot_arm.go_to_ee_pose(pre_grasp_pose)
        self.robot_arm.go_to_ee_pose(grasp_pose)
        self.robot_arm.close_gripper()
        self.robot_arm.go_to_ee_pose(post_grasp_pose)
        return grasp_pose
    
    def place_object(self, place_pose):
        
        place_pose[2,3] += 0.05
        preplace_pose = get_offset_from_pose(pose = place_pose, offset_in_obj_frame = True)
        postplace_pose = get_offset_from_pose(pose = place_pose, offset_in_obj_frame = False)
        self.robot_arm.go_to_ee_pose(preplace_pose)
        self.robot_arm.go_to_ee_pose(place_pose)
        self.robot_arm.open_gripper()
        self.robot_arm.go_to_ee_pose(postplace_pose)
        return place_pose
        
    def get_grasp_pose(self, object_, grasps_file, grasp_name=None):
        pre_recorded_grasps = read_yaml_file(grasps_file)
        if grasp_name is None:
            desired_grasp2obj_tf = list(pre_recorded_grasps["grasps"].values())[0]
        else: 
            desired_grasp2obj_tf = pre_recorded_grasps["grasps"][grasp_name]           

        image = self.camera.get_current_rgb_frame()
        obj2cam_rvec, obj2cam_tvec, _ =  object_.get_object_pose(color_image = image,
                                                        camera = self.camera)
        
        viz_image = draw_estimated_frame(image, self.camera, obj2cam_rvec, obj2cam_tvec)
        
        
        obj2cam_tf = tf_from_rvectvec(obj2cam_rvec, obj2cam_tvec)

        
        grasp_rvec, grasp_tvec = rvectvec_from_tf(np.matmul(obj2cam_tf, desired_grasp2obj_tf))
        viz_image = draw_estimated_frame(viz_image, self.camera, grasp_rvec,grasp_tvec)
        plot_image(viz_image)
        
        
        grasp_pose_in_base = np.matmul(self.get_cam2base_tf(),
                                    np.matmul(obj2cam_tf, desired_grasp2obj_tf))       
        return grasp_pose_in_base