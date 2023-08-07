import numpy as np
import pyrealsense2 as rs
from perception.realsense_sensor import RealSenseSensor
from .camera import RGBCamera
from ..logger import logger

class RealSenseCamera(RGBCamera):
    def __init__(self, camera_id = None):  
        ctx = rs.context()        
        if camera_id is None:
            try:
                camera_id = ctx.devices[0].get_info(rs.camera_info.serial_number)
            except IndexError:
                logger.error("No camera is connected")

        self.sensor = RealSenseSensor(camera_id, frame="realsense", filter_depth=True)
        self.sensor.start()
        intr = self.sensor.color_intrinsics
        camera_matrix = np.array([[intr._fx, 0.0, intr._cx], [0.0, intr._fy, intr._cy],[0.0,0.0,1.0]])
        dist_coeffs = np.array([0.0,0.0,0.0,0.0])
        RGBCamera.__init__(self, camera_matrix,
                                dist_coeffs,
                                camera_id)              

    
    def get_current_rgb_frame(self,):
        color_im_, _ = self.sensor.frames()  
        color_im = color_im_.raw_data
        return color_im