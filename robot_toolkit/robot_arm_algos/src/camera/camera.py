import open3d as o3d
from abc import ABC, abstractmethod
class Camera:
    def __init__(self, camera_matrix, dist_coeffs, camera_id = 1):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.camera_id = camera_id#use realsense ids to use pyrealsense     
        self.fx = camera_matrix[0,0]
        self.fy = camera_matrix[1,1]
        self.cx = camera_matrix[0,2]
        self.cy =  camera_matrix[1,2]

class RGBCamera(ABC, Camera):
    def __init__(self, camera_matrix, dist_coeffs, camera_id = 1):
        Camera.__init__(self, camera_matrix,
                            dist_coeffs,
                            camera_id)
    
    @abstractmethod
    def get_current_rgb_frame(self,):
        pass

class RGBDCamera(RGBCamera):
    def __init__(self, camera_matrix, dist_coeffs, camera_id):
        RGBCamera.__init__(self, camera_matrix,
                                    dist_coeffs,
                                    camera_id)
    @abstractmethod
    def get_current_rgbd_frames(self,):
        pass

    @abstractmethod
    def get_current_depth_frame(self,):
        pass 

    def get_pointcloud_rgbd(self, color_im = None, depth_im = None):
        if color_im is None and depth_im is None:
            color_im, depth_im = self.get_current_rgbd_frames()
        assert(depth_im.shape[1] == color_im.shape[1])
        assert(depth_im.shape[0] == color_im.shape[0])
        width, height = depth_im.shape[1], depth_im.shape[0]         
        o3d_depth = o3d.geometry.Image(depth_im)
        o3d_color = o3d.geometry.Image(color_im)
        o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, convert_rgb_to_intensity=False, depth_scale = 1.0)
        o3d_camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        o3d_camera_intrinsics.set_intrinsics(width = width, 
                                        height = height,
                                        fx = self.fx,
                                        fy = self.fy,
                                        cx = self.cx,
                                        cy = self.cy)         
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(o3d_rgbd, o3d_camera_intrinsics)                                        
        return pcd
    
    # def get_pointcloud_rgbd(self,):
    #     color_im, depth_im = self.get_current_rgbd_frames()
    #     return self.get_rgb_pointcloud(color_im, depth_im)    
    
    def get_pointcloud_depth(self, depth_im = None):
        if depth_im is None:
            depth_im = self.get_current_depth_frame()
        width, height = depth_im.shape[1], depth_im.shape[0]        
        o3d_depth = o3d.geometry.Image(depth_im)
        o3d_camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
        o3d_camera_intrinsics.set_intrinsics(width = width, 
                                        height = height,
                                        fx = self.fx,
                                        fy = self.fy,
                                        cx = self.cx,
                                        cy = self.cy)         
        pcd_depth = o3d.geometry.PointCloud.create_from_depth_image(o3d_depth, o3d_camera_intrinsics)
        return pcd_depth
    
    # def get_pointcloud_depth(self,):
    #     depth_im = self.get_current_depth_frame()
    #     return self.get_pointcloud_depth(depth_im)