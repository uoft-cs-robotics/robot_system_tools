from dataclasses import dataclass
import numpy as np
from abc import ABC, abstractmethod
import cv2
from ..camera.camera import RGBCamera

@dataclass
class TagDetection:
    reproj_error_mean: float
    reproj_error_variance: float
    matched_obj_pts: np.ndarray
    matched_img_pts: np.ndarray

class Fiducial(ABC):
    def __init__(self, marker_type):
        self.fiducial_type = marker_type
    
    @abstractmethod
    def detect_markers(self, color_image):
        pass
    
    @abstractmethod
    def estimate_pose(self, img_points, 
							obj_points, 
							camera:RGBCamera):
        pass

    @abstractmethod
    def compute_reprojection_error(self, rvec, 
										tvec, 
										obj_points, 
										img_points, 
										camera:RGBCamera):
        pass	


ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000
}

# for april tag
# https://pyimagesearch.com/2020/12/28/determining-aruco-marker-type-with-opencv-and-python/
