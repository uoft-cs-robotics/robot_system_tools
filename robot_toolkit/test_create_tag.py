import numpy as np
from robot_arm_algos.src.tags_detection.create_aruco_tag import *

if __name__ == "__main__":
    # create_and_save_arucotag()
    create_and_save_arucoboard(output_image_file="tests/data/aruco_board_2x2.png",
                                dictionary = "DICT_6X6_1000", 
                                n_rows = 2,
                                n_cols = 2, 
                                marker_length = 0.0225, 
                                marker_separation = 0.0025, 
                                ids = np.array([16,17,18,19])
                            )

