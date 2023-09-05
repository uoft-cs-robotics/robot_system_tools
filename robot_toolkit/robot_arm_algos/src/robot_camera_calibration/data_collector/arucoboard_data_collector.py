import numpy as np
import matplotlib.pyplot as plt
import cv2
from ...tags_detection._fiducial import TagDetection
from ._data_collector import CameraDataCollector
from ...tags_detection.aruco_board import ArucoBoard
from ...camera.camera import RGBCamera
from ...logger import logger

class ArucoBoardDataCollector(ArucoBoard, CameraDataCollector):
    def __init__(self, aruco_board_data, debug_detection_image = True):
        ArucoBoard.__init__(self, aruco_board_data)
        CameraDataCollector.__init__(self, self.fiducial_type)
        self.debug_detection_image = debug_detection_image
    
    def get_tag_frame(self, color_image, camera:RGBCamera):
        corners, ids, _  = self.detect_markers(color_image)
        if ids is None:
            return None, None, None
        image_gray =  cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        self.refine_corners(image_gray, corners)
        obj_points, img_points = self.get_matched_object_points(corners, ids)
        rvec, tvec = self.estimate_pose(obj_points, img_points, camera)
        reproj_error, error_variance = self.compute_reprojection_error(rvec,
                                                            tvec,
                                                            obj_points,
                                                            img_points,
                                                            camera)     
        if self.debug_detection_image:
            self.draw_detections(color_image = color_image,
                                corners = corners)
            self.draw_estimated_frame(color_image = color_image,
                                        camera = camera,
                                        rvec = rvec,
                                        tvec = tvec)
            plt.imshow(color_image)
            plt.show(block=False)
            plt.pause(2.5)
            plt.close()

        return rvec, tvec, TagDetection(reproj_error_mean=reproj_error, 
                                        reproj_error_variance=error_variance,
                                        matched_img_pts=img_points,
                                        matched_obj_pts=obj_points)

        