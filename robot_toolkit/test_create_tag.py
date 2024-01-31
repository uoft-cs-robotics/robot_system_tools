import numpy as np
from robot_arm_algos.src.tags_detection.create_aruco_tag import *

if __name__ == "__main__":
    # create_and_save_arucotag()
    # create_and_save_arucoboard(output_image_file="tests/data/aruco_board_2x2.png",
    #                             dictionary = "DICT_6X6_1000", 
    #                             n_rows = 2,
    #                             n_cols = 2, 
    #                             marker_length = 0.0225, 
    #                             marker_separation = 0.0025, 
    #                             ids = np.array([12,13,14,15])

    #                             #ids = np.array([16,17,18,19])
    #                         )
    tag_id = 1
    create_and_save_arucotag(output_image_file=f"tests/data/aruco_tag_{tag_id}.png",
                            dictionary = "DICT_4X4_1000", 
                            id_ = tag_id)

                            #ids = np.array([16,17,18,19])
                        

    create_and_save_arucoboard(output_image_file="tests/data/aruco_board_5x4.jpg",
                                dictionary = "DICT_6X6_1000", 
                                n_rows = 4,
                                n_cols = 5, 
                                marker_length = 0.04, 
                                marker_separation = 0.005, 
                                #ids = np.array([16,17,18,19])
                            )