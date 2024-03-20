from typing import Optional
from dataclasses import dataclass
import numpy as np
import cv2
from ._fiducial import Fiducial, ARUCO_DICT
from ..logger import logger


@dataclass 
class ArucoTagData:
    dictionary: str
    marker_length: float
    tag_id: Optional[int] = 0

class ArucoTag(Fiducial):
    def __init__(self, aruco_tag_data:ArucoTagData):
        Fiducial.__init__(self, "aruco_tag", aruco_tag_data)
        self.create_aruco_tag_detector(aruco_tag_data)
        self.marker_length = aruco_tag_data.marker_length
        self.tag_id = aruco_tag_data.tag_id
    
    def create_aruco_tag_detector(self, aruco_tag_data):
        try:
            dictionary = ARUCO_DICT[aruco_tag_data.dictionary]
        except KeyError:
            logger.error(f"{aruco_tag_data.dictionary} is unknown dictionary.") 

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def detect_markers(self, color_image):
        image_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(image_gray)
        self.refine_corners(image_gray, corners)
        if ids is None:
            logger.error("No corners detected in this image")
            return None, None, None
        if(self.tag_id not in np.squeeze(ids)):
            logger.debug(f"marker with id = {self.tag_id} not found in this image, only ids = {np.squeeze(ids)} are found")
            return None, None, None            
        return corners, ids, rejectedImgPoints        

    def estimate_pose(self, obj_points, img_points, camera):
        rvec = None
        tvec = None
        retval, rvec, tvec = cv2.solvePnP(obj_points,
                                        img_points,
                                        camera.camera_matrix,
                                        rvec,
                                        tvec)
        return rvec, tvec

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

    def refine_corners(self, image, corners):
        winSize = [5, 5]
        zeroZone = [-1, -1]
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 30, 0.001)
        for corner in corners: 
            cv2.cornerSubPix(image, corner, winSize, zeroZone, criteria)
    
    def get_matched_object_points(self, corners):
        obj_points = np.array([[-self.marker_length/2.0, self.marker_length/2.0, 0.0],
                    [self.marker_length/2.0, self.marker_length/2.0, 0.0],
                    [self.marker_length/2.0, -self.marker_length/2.0, 0.0],
                    [-self.marker_length/2.0, -self.marker_length/2.0, 0.0]], dtype=np.float32)
        return obj_points, np.squeeze(corners)

    def draw_detections(self, color_image, corners):
        cv2.aruco.drawDetectedMarkers(color_image, corners, borderColor=(0, 0, 255))
        return color_image
        
    def draw_estimated_frame(self, color_image, camera, rvec, tvec):
        cv2.drawFrameAxes(color_image, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.1)
        return color_image
                                                