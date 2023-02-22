import pyrealsense2 as rs
# works for both intel D435 and D415
class RealSenseUtils: 
    def __init__(self, 
                rgb_width=640, 
                rgb_height=480,
                rgb_fps=30, 
                depth_width=640,
                depth_height=480,
                depth_fps=30):
        self.config = rs.config()
        self.pipeline = rs.pipeline()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))            
        found_rgb
