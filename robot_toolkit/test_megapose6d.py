import numpy as np
import cv2
import open3d as o3d
import timeit

import matplotlib.pyplot as plt
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.inference.megapose6d import MegaPose6D, rvectvec_from_tf
from robot_arm_algos.src.logger import logger
# from robot_arm_algos.src.robot_camera_calibration._calibration_data_utils import rvectvec_from_tf

def draw_estimated_frame(color_image, camera, rvec, tvec):
    cv2.drawFrameAxes(color_image, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
    return color_image

    
def main(): 
    rs_camera = RealSenseCamera()
    object_name = "powerdrill"
    obj_dir_path = "robot_arm_algos/src/inference/objects/"+object_name
    
    megapose = MegaPose6D(object_name = object_name,
                          obj_dir_path = obj_dir_path)
    rvec, tvec, output = megapose.estimate_pose(rs_camera)
    rvec1, tvec1 = rvectvec_from_tf(output.poses[0].cpu().detach().numpy())
    rvec2, tvec2 = rvectvec_from_tf(output.poses_input[0].cpu().detach().numpy())
    rgb_image, depth_image = rs_camera.get_current_rgbd_frames()
    pcd = rs_camera.get_pointcloud_rgbd(rgb_image, depth_image)
    
    rgb_image = draw_estimated_frame(rgb_image, rs_camera, rvec1, tvec1)
    
    plt.imshow(rgb_image)
    plt.show(block=True)


    
if __name__ == "__main__":
    main()     