import numpy as np
from robot_arm_algos.src.robot_camera_calibration._calibration_data_utils import tf_from_rvectvec, rvectvec_from_tf
from robot_arm_algos.src.robot_arm.robot_frankapy import RobotFrankaPy
from robot_arm_algos.src.camera.ros_camera import ROSCamera
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.tags_detection.object import Object
from robot_arm_algos.src.robot_arm.pick_n_place import PickNPlace
from robot_arm_algos.src.config_reader import read_yaml_file
from robot_arm_algos.src.logger import logger
def main():
    # define a camera object
    ros_camera = ROSCamera(image_topic_name = "/camera/color/image_raw",
                        camera_info_topic_name = "/camera/color/camera_info",
                        init_node = True)  

    # define object class for an object to detect and grasp
    aruco_board_data = ArucoBoardData(dictionary="DICT_6X6_1000",
                                    marker_length = 0.0225,
                                    marker_separation = 0.0025,
                                    n_rows = 2,
                                    n_cols = 2,
                                    ids = np.array([16,17,18,19]))     
    aruco_board_tag = ArucoBoard(aruco_board_data)                           
    object2tag_offset = np.eye(4)
    object2tag_offset[0:3, -1] = np.array([aruco_board_data.marker_length + aruco_board_data.marker_separation/2.0,
                                        aruco_board_data.marker_length + aruco_board_data.marker_separation/2.0,
                                        0.05 / 2.0])# half of cube's height
    object_ = Object(object_name = "cube 4 with arucoboard tag",
                    tag = aruco_board_tag,
                    object2tag_offset=object2tag_offset)   
    # define a robot_arm object for pick and place         
    robot_arm_object = robot_arm_object = RobotFrankaPy(init_node = False, 
                                                        with_franka_gripper = True)   
    robot_arm_object.fpy_object.reset_joints()                                              

    # load camera to robot base extrinsics
    file_ = "robot_arm_algos/robot_camera_extrinsics.yaml"
    extrinsics_dict = read_yaml_file(file_)
    cam2gripper_extrinsics = extrinsics_dict["HAND_EYE_TSAI"]

    # Pick N Place object
    picknplace = PickNPlace(robot_arm_object = robot_arm_object,
                            camera_object = ros_camera,
                            cam_extrinsics = cam2gripper_extrinsics)
    
    (pick_rvec, pick_tvec), tag_det = picknplace.pick_object(object_, camera_in_hand = True)
    logger.info(f"Picked object at {pick_rvec}, {pick_tvec}")

    picknplace.place_object(pick_rvec, pick_tvec)

    logger.info(f"Placed the object back at where we picked it")

if __name__ == "__main__":
    main()        