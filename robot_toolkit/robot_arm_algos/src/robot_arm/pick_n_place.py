
import numpy as np
from ..robot_camera_calibration._calibration_data_utils import tf_from_rvectvec, rvectvec_from_tf
from ..logger import logger
"""
Pick n Place example with one robot and one camera wit known robot camera extrinsics
object to be picked and placed is detected with an aruco tag object with a known offset to object centroid
"""
class PickNPlace:
    def __init__(self, robot_arm_object, camera_object, cam_extrinsics):
        self.robot_arm = robot_arm_object
        self.camera_object = camera_object
        self.cam_extrinsics = cam_extrinsics#either cam2base: camera in environment or cam2gripper: camera in hand
    
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
