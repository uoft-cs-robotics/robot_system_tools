"""
Pick n Place example with one robot and one camera with known robot camera extrinsics.

Author: Ruthrash Hari
Date: 2/10/23 
"""
import numpy as np
import dataclasses
from ..robot_camera_calibration._calibration_data_utils import tf_from_rvectvec
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
    
    def pick_object(self, object, grasps_file, grasp_name = None):
        logger.debug(f"performing grasps for object{dataclasses.asdict(object.tag.fiducial_data)}")
        grasp_pose = self.get_grasp_pose(object = object,
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
        
    def get_grasp_pose(self, object, grasps_file, grasp_name=None):
        pre_recorded_grasps = read_yaml_file(grasps_file)
        if grasp_name is None:
            desired_grasp2obj_tf = list(pre_recorded_grasps["grasps"].values())[0]
        else: 
            desired_grasp2obj_tf = pre_recorded_grasps["grasps"][grasp_name]           
             
        obj2cam_rvec, obj2cam_tvec, _ =  object.get_object_pose(color_image = self.camera.get_current_rgb_frame(),
                                                        camera = self.camera)        
        obj2cam_tf = tf_from_rvectvec(obj2cam_rvec, obj2cam_tvec)
        grasp_pose_in_base = np.matmul(self.get_cam2base_tf(),
                                    np.matmul(obj2cam_tf, desired_grasp2obj_tf))       
        return grasp_pose_in_base
        







































      
    # def grasp_object(self,):
    #     pass
            
    # def grasp(self, object, grasps_file, grasp_name=None):
    #     self.robot_arm.open_gripper()
    #     pre_recorded_grasps = read_yaml_file(grasps_file)
    #     if grasp_name is None:
    #         desired_grasp2obj_tf = list(pre_recorded_grasps["grasps"].values())[0]
    #     else: 
    #         desired_grasp2obj_tf = pre_recorded_grasps["grasps"][grasp_name]
    #     obj2cam_rvec, obj2cam_tvec, _ =  object.get_object_pose(color_image = self.camera.get_current_rgb_frame(),
    #                                                     camera = self.camera)        
    #     obj2cam_tf = tf_from_rvectvec(obj2cam_rvec, obj2cam_tvec)
    #     grasp_pose_in_base = np.matmul(self.get_cam2base_tf(),
    #                                    np.matmul(obj2cam_tf, desired_grasp2obj_tf))
    #     grasp_pose_in_base[2,3] += 0.2
    #     self.robot_arm.go_to_ee_pose(grasp_pose_in_base)
        
    #     grasp_pose_in_base[2,3] -= 0.2
    #     self.robot_arm.go_to_ee_pose(grasp_pose_in_base)        
    #     self.robot_arm.close_gripper()

        

#get_grasp_pose 
#compute_pregrasp_pose
#get pickup pose  
#get_place_pose
#compute_preplace_pose


























































'''

    def detect_object(self, object_, debug_image = True):
        return object_.compute_object_pose(color_image = self.camera_object.get_current_rgb_frame(),
                            camera = self.camera_object,
                            debug_image = debug_image)
    
    def detect_object_robot_base_frame(self, object_, camera_in_hand = True, debug_image = True):
        rvec_obj2cam, tvec_obj2cam, tag_det =  self.detect_object(object_, debug_image=debug_image)
        logger.debug(f"reprojection error = {tag_det.reproj_error_mean}, error_variance = {tag_det.reproj_error_variance}")                                                                                  
        tf_obj2cam = tf_from_rvectvec(rvec_obj2cam, tvec_obj2cam)
        if camera_in_hand:
            tf_gripper2base = self.robot_arm.get_ee_frame()
            tf_cam2base = np.matmul(tf_gripper2base, self.cam_extrinsics)
            tf_obj2robot_base = np.matmul(tf_cam2base, tf_obj2cam)
        else:
            tf_obj2robot_base = np.matmul(self.cam_extrinsics, tf_obj2cam)
        rvec_obj2base, tvec_obj2base = rvectvec_from_tf(tf_obj2robot_base)
        logger.debug(f"object in robot frame:, translation = {tvec_obj2base}, rotation = {rvec_obj2base}")        

        return (rvec_obj2base , tvec_obj2base), tag_det

    def pick_object(self, object_, camera_in_hand = True, debug_image = True):
        self.robot_arm.open_gripper()
        (rvec_obj2base, tvec_obj2base), tag_det =  self.detect_object_robot_base_frame(object_, 
                                                                                    camera_in_hand = camera_in_hand, 
                                                                                    debug_image = debug_image)
        pose_fpy = self.robot_arm.fpy_object.get_pose()

        grasp_rvec = rvec_obj2base.copy()
        grasp_tvec = tvec_obj2base.copy()
        
        # go to pregrasp pose
        pregrasp_rvec, pregrasp_tvec = self.get_pregrasp_pose(rvec_obj2base, tvec_obj2base)
        pose_fpy.translation = pregrasp_tvec
        pose_fpy.rotation = tf_from_rvectvec(pregrasp_rvec, pregrasp_tvec)[0:3,0:3]
        self.robot_arm.go_to_ee_pose(pose_fpy)

        # go to grasp pose
        pose_fpy.translation = grasp_tvec
        pose_fpy.rotation = tf_from_rvectvec(grasp_rvec, grasp_tvec)[0:3,0:3]        
        self.robot_arm.go_to_ee_pose(pose_fpy)
        
        # grasp
        self.robot_arm.close_gripper()

        # go to post grasp pose 
        postgrasp_rvec, postgrasp_tvec = self.get_postgrasp_pose(grasp_rvec, grasp_tvec)
        pose_fpy.translation = postgrasp_tvec
        pose_fpy.rotation = tf_from_rvectvec(postgrasp_rvec, postgrasp_tvec)[0:3,0:3]
        self.robot_arm.go_to_ee_pose(pose_fpy)
        return (rvec_obj2base, tvec_obj2base), tag_det

    def get_pregrasp_pose(self, rvec_obj2base, tvec_obj2base):
        tvec_pregrasp = tvec_obj2base.copy()
        tvec_pregrasp[2] += 0.15
        rvec_pregrasp = rvec_obj2base.copy()
        return rvec_pregrasp, tvec_pregrasp
    
    def get_postgrasp_pose(self, grasp_rvec, grasp_tvec):
        tvec_postgrasp = grasp_tvec.copy()
        tvec_postgrasp[2] += 0.15
        rvec_postgrasp = grasp_rvec.copy()
        return rvec_postgrasp, tvec_postgrasp

    
    def place_object(self, place_pose_rvec, place_pose_tvec ):
        # todo check if object exists in gripper
        pose_fpy = self.robot_arm.fpy_object.get_pose()
        # go to pre place pose
        preplace_rvec, preplace_tvec = self.get_preplace_pose(place_pose_rvec, place_pose_tvec)
        pose_fpy.translation = preplace_tvec    
        pose_fpy.rotation = tf_from_rvectvec(preplace_rvec, preplace_tvec)[0:3, 0:3]
        self.robot_arm.go_to_ee_pose(pose_fpy)
        
        # go to place pose 
        pose_fpy.translation = place_pose_tvec  
        pose_fpy.rotation = tf_from_rvectvec(place_pose_rvec, place_pose_tvec)[0:3, 0:3]
        self.robot_arm.go_to_ee_pose(pose_fpy)        
        
        # un grasp
        self.robot_arm.open_gripper()

        # go to post place pose
        postplace_rvec, postplace_tvec = self.get_postplace_pose(place_pose_rvec, place_pose_tvec)
        pose_fpy.translation = postplace_tvec
        pose_fpy.rotation = tf_from_rvectvec(postplace_rvec, postplace_tvec)[0:3,0:3]
        self.robot_arm.go_to_ee_pose(pose_fpy)
        
        return
    
    def get_preplace_pose(self, place_rvec, place_tvec):
        tvec_preplace = place_tvec.copy()
        tvec_preplace[2] += 0.15
        rvec_preplace = place_rvec.copy()
        return rvec_preplace, tvec_preplace
    
    def get_postplace_pose(self, place_rvec, place_tvec):
        tvec_postplace = place_tvec.copy()
        tvec_postplace[2] += 0.15
        rvec_postplace = place_rvec.copy()
        return rvec_postplace, tvec_postplace
'''