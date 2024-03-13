from typing import Optional
from dataclasses import dataclass
import numpy as np
import cv2
from ._fiducial import Fiducial, ARUCO_DICT
from ..logger import logger


@dataclass 
class ArucoTagData:
    """DataClass for defining the ArucoTag
    
    Args:
        dictionary (str): defines the dictionary of the aruco marker 
        marker_length (float): edge length of the square aruco marker in meters
        tag_id(int, Optional): You can specify the ID of the marker to be rendered in the ArucoTag. Defaults to 0
    """    
    dictionary: str
    marker_length: float
    tag_id: Optional[int] = 0

class ArucoTag(Fiducial):
    """Implementation of the abstract Fiducial Class for ArucoTag pattern
    """    
    def __init__(self, aruco_tag_data:ArucoTagData):
        """ArucoTag Class Constructor

        Args:
            aruco_tag_data (ArucoTagData): ArucoTagData data class object that defines the parameters of the arucotag pattern.
        """
        Fiducial.__init__(self, "aruco_tag")
        self.create_aruco_tag_detector(aruco_tag_data)
        self.marker_length = aruco_tag_data.marker_length
        self.tag_id = aruco_tag_data.tag_id
    
    def create_aruco_tag_detector(self, aruco_tag_data):
        """Instantiates aruco_tag detector function to detect aruco marker in the image 

        Args:
            aruco_tag_data (ArucoTagData): ArucoTagData data class object that defines the parameters of the arucotag pattern
        
        Raises: 
            Exception: KeyError if the requested dictionary doesn't exist in available dictionaries of aruco markers.
        """        
        try:
            dictionary = ARUCO_DICT[aruco_tag_data.dictionary]
        except KeyError:
            logger.error(f"{aruco_tag_data.dictionary} is unknown dictionary.") 

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def detect_markers(self, color_image):
        """Perform aruco marker detection in the image.

        Args:
            color_image (numpy array): 3 channel RGB image

        Returns:
            numpy array: numpy array of 4 corners for each aruco marker of the arucotag detected in the image
            numpy array: numpy array of ids of the markers corresponding to the 'corners'
            numpy array: contains the imgPoints of those squares whose inner code has not a correct codification. Useful for debugging purposes.
        """
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
        """Estimates pose of the arucotag in the camera frame

        Args:
            corners (numpy array):  of 4 corners for each aruco marker of the arucotag detected in the image
            obj_points (numpy array): 3D locations of the markers's corners defined in the arucotag's frame
            img_points (numpy array): corresponding 2D locations of the markers' corners in the image
            camera (RGBCamera): Camera Object that has the intrinsics and distortion parameters.

        Returns:
            numpy array: 3x1 angle-axis rotation vector of the arucotag's pose in the camera frame.
            numpy array: 3x1 translation vector of the arucotag's pose in the camera frame.
        """        
        rvec = None
        tvec = None
        retval, rvec, tvec = cv2.solvePnP(obj_points,
                                        img_points,
                                        camera.camera_matrix,
                                        rvec,
                                        tvec)
        return rvec, tvec

    def compute_reprojection_error(self, rvec, tvec, obj_points, img_points, camera):
        """Computes reprojection error mean and variance from each pair of 2D-3D correspondences

        Args:
            rvec (numpy array): 3x1 angle-axis rotation vector of the arucotag's pose in the camera frame.
            tvec (numpy array): 3x1 translation vector of the arucotag's pose in the camera frame.
            obj_points (numpy array): 3D locations of the markers's corners defined in the arucotag's frame
            img_points (numpy array): corresponding 2D locations of the markers' corners in the image
            camera (RGBCamera): Camera Object that has the intrinsics and distortion parameters.
        
        Returns: 
            float: reprojection error mean from each pair of 2D-3D correspondences
            float: reprojection error variance from each pair of 2D-3D correspondences
        """        
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
        """Performs subpixel refinement of the detected corners in the image. Refined subpixel values are stored in 'corners' variable. 

        Args:
            image (numpy array): 1 channel grayscale image
            corners (numpy array): numpy array of 4 corners for each aruco marker of the arucotag detected in the image
        """        
        winSize = [5, 5]
        zeroZone = [-1, -1]
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 30, 0.001)
        for corner in corners: 
            cv2.cornerSubPix(image, corner, winSize, zeroZone, criteria)
    
    def get_matched_object_points(self, corners):
        """Get the corresponding 3D locations of the markers' corners defined in the arucotag's frame

        Args:
            corners (numpy array):  of 4 corners for each aruco marker of the arucotag detected in the image
            ids (list): list of ids corresponding to 4 corners in the corners variable

        Returns:
            numpy array: 3D locations of the markers's corners defined in the arucotag's frame
            numpy array: corresponding 2D locations of the markers' corners in the image
        """        
        obj_points = np.array([[-self.marker_length/2.0, self.marker_length/2.0, 0.0],
                    [self.marker_length/2.0, self.marker_length/2.0, 0.0],
                    [self.marker_length/2.0, -self.marker_length/2.0, 0.0],
                    [-self.marker_length/2.0, -self.marker_length/2.0, 0.0]], dtype=np.float32)
        return obj_points, np.squeeze(corners)

    def draw_detections(self, color_image, corners):
        """Draw detections of the markers in the color image

        Args:
            color_image (numpy array): 3 channel RGB image on which the detections will be drawn
            corners (numpy array): 2D locations of all the corners detected in the image

        Returns:
            numpy array: 3 channel RGB image which has markers detections drawn on it.
        """        
        cv2.aruco.drawDetectedMarkers(color_image, corners, borderColor=(0, 0, 255))
        return color_image
        
    def draw_estimated_frame(self, color_image, camera, rvec, tvec):
        """Draws a frame of the aructag's frame based on its estimated pose in the camera frame.

        Args:
            color_image (numpy array): 3 channel RGB image on which the detections will be drawn
            camera (Camera): Camera Object that has the intrinsics and distortion parameters.
            rvec (numpy array): 3x1 angle-axis rotation vector of the arucotag's pose in the camera frame.
            tvec (numpy array): 3x1 translation vector of the arucotag's pose in the camera frame.

        Returns:
            numpy array: 3 channel RGB image which has aructag's estimated frame drawn on it.
        """        
        cv2.drawFrameAxes(color_image, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.1)
        return color_image
                                                