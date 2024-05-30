import numpy as np
import matplotlib.pyplot as plt
import cv2
from robot_arm_algos.src.logger import logger
from robot_arm_algos.src.camera.ros_camera import ROSCamera
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.tags_detection.object import Object    

def main():
    ros_camera = ROSCamera(image_topic_name = "/camera/color/image_raw",
                        camera_info_topic_name = "/camera/color/camera_info",
                        init_node = True)   
    aruco_board_data = ArucoBoardData(dictionary="DICT_6X6_1000",
                                    marker_length = 0.0225,
                                    marker_separation = 0.0025,
                                    n_rows = 2,
                                    n_cols = 2,
                                    ids = [16,17,18,19])  
    aruco_board_tag = ArucoBoard(aruco_board_data) 
    object2tag_offset = np.eye(4)
    object2tag_offset[0:3, -1] = np.array([aruco_board_data.marker_length + aruco_board_data.marker_separation/2.0,
                                        aruco_board_data.marker_length + aruco_board_data.marker_separation/2.0,
                                        0.05 / 2.0])
    object_ = Object(object_name = "cube with arucoboard tag",
                    tag = aruco_board_tag,
                    object2tag_offset=object2tag_offset)                                     
    color_image = ros_camera.get_current_rgb_frame() 
    rvec_object2cam, tvec_object2cam, tag_detection = object_.compute_object_pose(color_image,
                                                                                ros_camera,
                                                                                debug_image = True)
    logger.info(f"reprojection error = {tag_detection.reproj_error_mean}, error_variance = {tag_detection.reproj_error_variance}")                                                                                  

if __name__ == "__main__":
    main()    