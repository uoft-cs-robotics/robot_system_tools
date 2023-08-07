from typing import Optional
from dataclasses import dataclass
import numpy as np
import cv2 
from ._fiducial import Fiducial, ARUCO_DICT
from ..camera.camera import RGBCamera
from ..logger import logger

@dataclass
class ArucoBoardData:
    dictionary: str
    marker_length: float
    marker_separation: float 
    n_rows: int
    n_cols: int
    ids: Optional[np.ndarray] = None

class ArucoBoard(Fiducial):
    def __init__(self, aruco_board_data:ArucoBoardData):
        Fiducial.__init__(self, "aruco_board")
        self.create_aruco_detector(aruco_board_data)

    def create_aruco_detector(self, aruco_board_data):
        try:
            dictionary = ARUCO_DICT[aruco_board_data.dictionary]
        except KeyError:
            logger.error(f"{aruco_board_data.dictionary} is unknown dictionary.")

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.board = cv2.aruco.GridBoard((aruco_board_data.n_rows, aruco_board_data.n_cols), 
                                        aruco_board_data.marker_length, 
                                        aruco_board_data.marker_separation, 
                                        self.aruco_dict,
                                        ids = aruco_board_data.ids)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, 
                                                self.aruco_params)
    
    def refine_corners(self, image, corners):
        winSize = [5, 5]
        zeroZone = [-1, -1]
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 30, 0.001)
        for corner in corners: 
            cv2.cornerSubPix(image, corner, winSize, zeroZone, criteria)

    def detect_markers(self, color_image):
        image_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(image_gray)
        self.refine_corners(image_gray, corners)
        if ids is None:
            # logger.error("No corners detected in this image")
            return None, None, None
        return corners, ids, rejectedImgPoints
        
    def get_matched_object_points(self, corners, ids):
        obj_points = None; img_points = None
        obj_points, img_points = self.board.matchImagePoints(corners, 
                                                            ids, 
                                                            obj_points, 
                                                            img_points)
        return obj_points, img_points          

    def estimate_pose(corners, obj_points, img_points, camera:RGBCamera):
        rvec = None
        tvec = None
        retval, rvec, tvec = cv2.solvePnP(obj_points, 
                                        img_points, 
                                        camera.camera_matrix, 
                                        rvec, 
                                        tvec)
        return rvec, tvec

    """
    returns reprojection error mean and variance computed from each pair of 2D-3D correspondences
    """
    def compute_reprojection_error(self, rvec, tvec, obj_points, img_points, camera):
        errors = []  
        for img_point, obj_point in zip(img_points, obj_points):
            proj_img_point, _ = cv2.projectPoints(obj_point,
                                                rvec,
                                                tvec,
                                                camera.camera_matrix,
                                                camera.dist_coeffs) 
            error = cv2.norm(np.squeeze(img_point), np.squeeze(proj_img_point), cv2.NORM_L2)     
            errors.append(error)
        return np.mean(errors), np.var(errors)   

    def draw_detections(self, color_image, corners):
        cv2.aruco.drawDetectedMarkers(color_image, corners, borderColor=(0, 0, 255))
        return color_image
        
    def draw_estimated_frame(self, color_image, camera, rvec, tvec):
        cv2.drawFrameAxes(color_image, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.1)
        return color_image
                                        