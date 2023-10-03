import numpy as np
from robot_arm_algos.src.robot_camera_calibration._calibration_data_utils import tf_from_rvectvec, rvectvec_from_tf
from robot_arm_algos.src.robot_arm.robot_frankapy import RobotFrankaPy
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.pick_and_place.object import Object
from robot_arm_algos.src.pick_and_place.pick_and_place import PickAndPlace
from robot_arm_algos.src.config_reader import read_yaml_file
from robot_arm_algos.src.logger import logger

def main():
    aruco_board_data = ArucoBoardData(dictionary="DICT_6X6_1000",
                                marker_length = 0.0225,
                                marker_separation = 0.0025,
                                n_rows = 2,
                                n_cols = 2,
                                ids = np.array([0,1,2,3]))  
    aruco_board_tag = ArucoBoard(aruco_board_data)  
    object_ = Object(object_name = "cube 4 with arucoboard tag",
                    tag = aruco_board_tag)   
    camera = RealSenseCamera(camera_id='145522066546')
    robot_arm_object = robot_arm_object = RobotFrankaPy(init_node = True, 
                                                        with_franka_gripper = True)   
    robot_arm_object.fpy_object.reset_joints()      
    
    extrinsics_dict = read_yaml_file("cam2.yaml")
    extrinsics = extrinsics_dict["HAND_EYE_DANIILIDIS"]  
    picknplace = PickAndPlace(robot_arm_object = robot_arm_object,
                              camera = camera, 
                              cam_extrinsics = extrinsics, 
                              camera_in_hand = True)
    # picknplace.grasp(object = object_,
    #                 grasps_file =  "tests/data/grasps.yaml")
    grasp_pose = picknplace.pick_object(object = object_,
                    grasps_file =  "tests/data/grasps.yaml")
    grasp_pose[0,3] += 0.1    
    picknplace.place_object(place_pose = grasp_pose)
          
if __name__ == "__main__":
    main()        