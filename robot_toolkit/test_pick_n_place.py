import numpy as np
from robot_arm_algos.src.robot_camera_calibration._calibration_data_utils import read_cam_robot_extrinsics
from robot_arm_algos.src.robot_arm.robot_frankapy import RobotFrankaPy
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.pick_and_place.object import Object
from robot_arm_algos.src.pick_and_place.pick_and_place import PickAndPlace
from robot_arm_algos.src.config_reader import read_yaml_file
from robot_arm_algos.src.logger import logger
from robot_arm_algos.src.inference.megapose6d import MegaPose6D

test_config = {
    "extrinsics_file" : "robot_arm_algos/robot_camera_extrinsics.yaml",
    "grasps_file": "tests/data/grasps_powerdrill.yaml",
    "aruco_board_data": ArucoBoardData(dictionary="DICT_6X6_1000",
                                marker_length = 0.0225,
                                marker_separation = 0.0025,
                                n_rows = 2,
                                n_cols = 2,
                                ids = [0,1,2,3]),  
    "object_name" : "powerdrill",
    "object_dir_path_base": "robot_arm_algos/src/inference/objects/",
    "camera_in_hand": True
    }


def main():
    obj_dir_path = test_config["object_dir_path_base"] + test_config["object_name"]
    megapose = MegaPose6D(object_name = test_config["object_name"],
                          obj_dir_path = obj_dir_path)
    
    aruco_board_tag = ArucoBoard(test_config["aruco_board_data"])  
    # object_ = Object(object_name = "cube 4 with arucoboard tag",
    #                 tag = aruco_board_tag)   
    object_ = Object(object_name = test_config["object_name"],
                    tag = None,
                    pose_estimator = megapose)    
    
    
    camera = RealSenseCamera()
    robot_arm_object = robot_arm_object = RobotFrankaPy(init_node = True, 
                                                        with_franka_gripper = True)   
    robot_arm_object.fpy_object.reset_joints()      
    extrinsics = read_cam_robot_extrinsics(extrinsics_file_name = test_config["extrinsics_file"])
    picknplace = PickAndPlace(robot_arm_object = robot_arm_object,
                              camera = camera, 
                              cam_extrinsics = extrinsics, 
                              camera_in_hand = test_config["camera_in_hand"])
    # picknplace.grasp(object = object_,
    #                 grasps_file =  "tests/data/grasps.yaml")
    grasp_pose = picknplace.pick_object(object_ = object_,
                    grasps_file =  test_config["grasps_file"])
    grasp_pose[0,3] += 0.1    
    picknplace.place_object(place_pose = grasp_pose)
          
if __name__ == "__main__":
    main()        