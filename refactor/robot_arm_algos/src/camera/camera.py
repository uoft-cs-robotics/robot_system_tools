from abc import ABC, abstractmethod
class Camera:
    def __init__(self, camera_matrix, dist_coeffs, camera_id = 1):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.camera_id = camera_id#use realsense ids to use pyrealsense      

class RGBCamera(ABC, Camera):
    def __init__(self, camera_matrix, dist_coeffs, camera_id = 1):
        Camera.__init__(self, camera_matrix,
                            dist_coeffs,
                            camera_id)
    
    @abstractmethod
    def get_current_rgb_frame(self,):
        pass

class RGBDCamera(RGBCamera):
    def __init__(self, camera_matrix, dist_coeffs):
        RGBCamera.__init__(self, camera_matrix,
                                    dist_coeffs)
    
    @abstractmethod
    def get_current_rgbd_frame(self,):
        pass

    @abstractmethod
    def get_current_depth_frame(self,):
        pass 