from abc import ABC, abstractmethod
from dataclasses import dataclass
import open3d as o3d
from ..logger import logger
'''
#todo: more abstract classes to a different file
'''
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

@dataclass
class BBox:
    xmin: int
    ymin: int 
    xmax: int
    ymax: int
        
import numpy as np        
import cv2
def get_bbox_annotations(rgb_image): 
    # Mouse callback function
    global click_list
    positions, click_list = [], []
    def callback(event, x, y, flags, param):
        if event == 1: click_list.append((x,y))
    cv2.namedWindow('img')
    cv2.setMouseCallback('img', callback)
    logger.info("Click the top left corner of the desired bounding box first")
    logger.info("Click the bottom right corner of the desired bounding box next")
    logger.info("press esc to stop bounding box annotation")
    # img = cv2.imread('/home/stephen/Desktop/test214.png')

    # Mainloop - show the image and collect the data
    while True:
        cv2.imshow('img', rgb_image)    
        # Wait, and allow the user to quit with the 'esc' key
        k = cv2.waitKey(1)
        # If user presses 'esc' break 
        if k == 27: break        
    cv2.destroyAllWindows()    
    bbox = BBox(xmin = click_list[0][0], ymin = click_list[0][1],
                xmax = click_list[1][0], ymax = click_list[1][1])
  
    return bbox, rgb_image[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax]

def get_segmap_from_bbox(image, bbox):
    output = np.zeros(np.shape(image)[:-1])
    output[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax] = 255
    return output

def get_segmap_from_bbox_with_depth(rgb_image, depth_image, bbox):
    # output = np.zeros(np.shape(image)[:-1])
    output = np.zeros(np.shape(depth_image))
    cropped_depth = depth_image[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax]
    avg_depth = np.mean(cropped_depth)
    output[depth_image < avg_depth ] = 255 
    output[:bbox.ymin, :] = 0
    output[bbox.ymax:, :] = 0
    output[:, :bbox.xmin] = 0
    output[:, bbox.xmax:] = 0    
    return output