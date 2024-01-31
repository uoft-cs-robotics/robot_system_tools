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
    # try:
    #     object_mesh = o3d.io.read_triangle_mesh(obj_dir_path + "/meshes/" + object_name + "/textured.obj")
    # except: 
    #     object_mesh = o3d.io.read_point_cloud(obj_dir_path + "/meshes/" + object_name + "/textured.obj")
        
    # o3d.visualization.draw_geometries([object_mesh], point_show_normal=False)

    megapose = MegaPose6D(object_name = object_name,
                          obj_dir_path = obj_dir_path)
    # megapose = MegaPose6D(object_name = "soupcan",
    #                       obj_dir_path = "robot_arm_algos/src/inference/objects/soupcan")    
    # plt.imshow(rs_camera.get_current_rgb_frame())
    # plt.show(block=False)
    # plt.pause(4.0)
    # plt.close()
    start = timeit.timeit()
    logger.info("hello")

    rvec, tvec, output = megapose.estimate_pose(rs_camera)
    end = timeit.timeit()
    logger.info(end - start)    
    print(output.poses[0])
    print(output.poses_input[0])
    rvec1, tvec1 = rvectvec_from_tf(output.poses[0].cpu().detach().numpy())
    rvec2, tvec2 = rvectvec_from_tf(output.poses_input[0].cpu().detach().numpy())
    rgb_image, depth_image = rs_camera.get_current_rgbd_frames()
    pcd = rs_camera.get_pointcloud_rgbd(rgb_image, depth_image)
    
    rgb_image = draw_estimated_frame(rgb_image, rs_camera, rvec1, tvec1)
    
    # new_image = draw_estimated_frame(new_image, rs_camera, rvec1, tvec1)
    plt.imshow(rgb_image)
    plt.show(block=True)

    # object_mesh = o3d.io.read_triangle_mesh(obj_dir_path + "/meshes/" + object_name + "/textured.obj")

    # object_mesh.transform(output.poses[0].cpu().detach().numpy())
    
    # frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(output.poses[0].cpu().detach().numpy())
    # o3d.visualization.draw_geometries([pcd, frame_mesh, object_mesh], point_show_normal=False)
    
if __name__ == "__main__":
    main()     