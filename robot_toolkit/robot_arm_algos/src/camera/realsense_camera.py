import numpy as np
import open3d as o3d
# import pyrealsense2 as rs
# from perception.realsense_sensor import RealSenseSensor
from .realsense_driver import RealSenseDriver
from .camera import RGBDCamera
from ..logger import logger

class RealSenseCamera(RGBDCamera):
    def __init__(self, camera_matrix = None, dist_coeffs = None, start_camera = True, camera_id = None):  
        try:
            if start_camera:
                if camera_id is None:
                    camera_id = RealSenseDriver.get_all_device_serials()[0]
                self.realsense_driver = RealSenseDriver(serial_number = camera_id)
                self.realsense_driver.start() 
        except Exception as e:
            logger.error(e)
            raise Exception(f"Failed to start intel RealSense")

        if camera_matrix is None and dist_coeffs is None:
            intr = self.realsense_driver.get_intrinsics()            
            camera_matrix = np.array([[intr.fx, 0.0, intr.cx], [0.0, intr.fy, intr.cy],[0.0,0.0,1.0]])
            dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0])

        self.start_camera = start_camera
        RGBDCamera.__init__(self, camera_matrix,
                                dist_coeffs,
                                camera_id) 
                                  
    def get_current_rgb_frame(self,):
        color_im, _ = self.realsense_driver.read()  
        return color_im
    
    def get_current_depth_frame(self,):
        _, depth_im = self.realsense_driver.read()    
        return depth_im
    
    def get_current_rgbd_frames(self,):
        color_im, depth_im = self.realsense_driver.read()    
        return color_im, depth_im
    
    def __del__(self):
        if self.start_camera:
            self.realsense_driver.stop()


# class RealSenseCamera(RGBDCamera):
#     def __init__(self, camera_id = None):  
#         ctx = rs.context()        
#         if camera_id is None:
#             try:
#                 camera_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
#             except IndexError:
#                 logger.error("No camera is connected")

#         self.sensor = RealSenseSensor(camera_id, frame="realsense", filter_depth=True)
#         self.sensor.start()
#         intr = self.sensor.color_intrinsics
#         camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
#         dist_coeffs = np.array([0.0,0.0,0.0,0.0])
#         RGBDCamera.__init__(self, camera_matrix,
#                                 dist_coeffs,
#                                 camera_id)    
#         for _ in range(20):
#             self.sensor.frames()          
    
#     def get_current_rgb_frame(self,):
#         color_im_, _ = self.sensor.frames()  
#         color_im = color_im_.raw_data
#         return color_im
    
#     def get_current_depth_frame(self,):
#         _, depth_im_ = self.sensor.frames()  
#         depth_im = depth_im_.raw_data
#         return depth_im
    
#     def get_current_rgbd_frames(self,):
#         color_im_, depth_im_ = self.sensor.frames()  
#         color_im = color_im_.raw_data
#         depth_im = depth_im_.raw_data  
#         return color_im, depth_im