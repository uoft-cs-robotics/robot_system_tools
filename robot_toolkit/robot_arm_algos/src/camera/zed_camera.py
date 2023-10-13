from camera import RGBDCamera
import pyzed.sl as sl
import numpy as np
from typing import Bool, Any

class ZEDCamera(RGBDCamera):
    def __init__(self, camera_matrix: np.array = None, dist_coeffs: np.array = None, start_camera: bool = True, camera_id: Any = None):  
        try:
            if start_camera:
                self.zed = sl.Camera()
                self.init_params = sl.InitParameters()
                self.init_params.coordinate_units = sl.UNIT.METER
                err = self.zed.open(init_params)
                if err != sl.ERROR_CODE.SUCCESS:
                    raise Exception("Camera Open : "+repr(err)+". Cannot open camera.")
                    exit()
        except Exception as e:
            logger.error(e)
            raise Exception(f"Failed to initialize ZED camera")

        if camera_matrix is None and dist_coeffs is None:
            calibration_param=zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
            intr_list=[calibration_param.fx, calibration_param.fy, calibration_param.cx, calibration_param.cy]
            camera_matrix = np.array([[calibration_param.fx,                      0,  calibration_param.cx],
                                    [                   0,   calibration_param.fy,  calibration_param.cy],
                                    [                   0,                      0,                      1]])
            dist_coeffs =np.array([0.0,0.0,0.0,0.0])

        self.image = sl.Mat()
        self.right_image = sl.Mat()
        self.depth_map = sl.Mat()
        self.start_camera = start_camera
        super().__init__(self, camera_matrix,
                                dist_coeffs,
                                camera_id=None) 

    def grab_images(self) -> None:
        self.zed.grab()
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT) 
        self.zed.retrieve_image(self.right_image, sl.VIEW.RIGHT)
        self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH) 
        return

    def get_current_rgb_frame(self,) -> np.array:
        _ = self.grab_images()
        return self.image
    
    def get_secondary_rgb_frame(self,) -> np.array:
        _ = self.grab_images()
        return self.right_image
    
    def get_current_depth_frame(self,) -> np.array:
        _ = self.grab_images()
        return self.depth_map
    
    def get_current_rgbd_frames(self,) -> tuple(np.array, np.array):
        _ = self.grab_images()
        return self.image, self.depth_map
