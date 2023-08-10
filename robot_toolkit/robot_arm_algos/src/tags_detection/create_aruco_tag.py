import cv2 
from ._fiducial import ARUCO_DICT
from ...src.logger import logger

def create_and_save_arucotag(output_image_file, dictionary, id_ = 0):
    try:
        cv_dictionary = ARUCO_DICT[dictionary]
    except KeyError:
        logger.error(f"{dictionary} that was provided doesn't exist in the list of options: {list(ARUCO_DICT.keys())}" )
        return
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv_dictionary)
    color_image = cv2.aruco.generateImageMarker(aruco_dict, id_, sidePixels=3300)
    return cv2.imwrite(output_image_file, color_image)

def create_and_save_arucoboard(output_image_file, dictionary, n_rows, n_cols, marker_length, marker_separation, ids = None):
    try:
        cv_dictionary = ARUCO_DICT[dictionary]
    except KeyError:
        logger.error(f"{dictionary} that was provided doesn't exist in the list of options: {list(ARUCO_DICT.keys())}" )
        return
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv_dictionary)
    board = cv2.aruco.GridBoard((n_rows, n_cols), 
                                marker_length, 
                                marker_separation, 
                                aruco_dict,
                                ids = ids)    
    color_image = None
    color_image = board.generateImage((3300,3300), color_image)
    return cv2.imwrite(output_image_file, color_image)