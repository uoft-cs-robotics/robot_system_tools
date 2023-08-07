import numpy as np
import cv2
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.camera.camera import Camera
from robot_arm_algos.src.logger import *
from robot_arm_algos.src.robot_camera_calibration.data_collector.arucoboard_data_collector import ArucoBoardDataCollector

def main():
    camera_matrix = np.array([[909.34179688, 0.0, 643.8762207 ],
                            [0.0, 908.06414795, 348.60513306],
                            [0.0, 0.0, 1.0]])
    dist_coeffs = np.array([0.0,0.0,0.0,0.0])                            
    camera = Camera(camera_matrix, dist_coeffs)
    aruco_board_data = ArucoBoardData(dictionary="DICT_6X6_1000",
                                    marker_length = 0.025,
                                    marker_separation = 0.005,
                                    n_rows = 5,
                                    n_cols = 7)
    aruco_board = ArucoBoard(aruco_board_data)
    color_img = cv2.imread("tests/data/aruco_board_test.png")

    data_collector = ArucoBoardDataCollector(aruco_board_data)
    print(data_collector.get_tag_frame(color_img, camera))

if __name__ == "__main__":
    main()


    
