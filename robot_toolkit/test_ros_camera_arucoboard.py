import numpy as np
import matplotlib.pyplot as plt
import cv2
from robot_arm_algos.src.logger import logger
from robot_arm_algos.src.camera.ros_camera import ROSCamera
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData


def main():
    ros_camera = ROSCamera(image_topic_name = "/camera/color/image_raw",
                            camera_info_topic_name = "/camera/color/camera_info",
                            init_node = True)
    aruco_board_data = ArucoBoardData(dictionary="DICT_6X6_1000",
                                    marker_length = 0.025,
                                    marker_separation = 0.005,
                                    n_rows = 5,
                                    n_cols = 7)
    aruco_board = ArucoBoard(aruco_board_data)
    color_image = ros_camera.get_current_rgb_frame()                            
    corners, ids, _  = aruco_board.detect_markers(color_image)
    image_gray =  cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)    
    aruco_board.refine_corners(image_gray, corners)
    obj_points, img_points = aruco_board.get_matched_object_points(corners, ids)
    rvec, tvec = aruco_board.estimate_pose(obj_points, img_points, ros_camera)
    reproj_error, error_variance = aruco_board.compute_reprojection_error(rvec,
                                                        tvec,
                                                        obj_points,
                                                        img_points,
                                                        ros_camera)    
    logger.info(f"reprojection error = {reproj_error}, error_variance = {error_variance}")
    aruco_board.draw_detections(color_image, corners)
    aruco_board.draw_estimated_frame(color_image, ros_camera, rvec, tvec)
    plt.imshow(color_image)
    plt.show(block=False)
    plt.pause(4.0)
    plt.close()

if __name__ == "__main__":
    main()    