import numpy as np
import matplotlib.pyplot as plt
import cv2 
import open3d as o3d
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera

def main():
    rs_camera = RealSenseCamera()
    color_im, depth_im = rs_camera.get_current_rgbd_frames()
    # width, height = color_im.shape[1], color_im.shape[0]

    # o3d_depth = o3d.geometry.Image(depth_im)
    # o3d_color = o3d.geometry.Image(color_im)

    # o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, convert_rgb_to_intensity=False)
    # camera_intrinsics = o3d.camera.PinholeCameraIntrinsic()
    # camera_intrinsics.set_intrinsics(width = width, 
    #                                 height = height,
    #                                 fx = rs_camera.fx,
    #                                 fy = rs_camera.fy,
    #                                 cx = rs_camera.cx,
    #                                 cy = rs_camera.cy)
    pcd = rs_camera.get_pointcloud_rgbd(color_im, depth_im)
    o3d.visualization.draw_geometries([pcd], point_show_normal=False)
    plt.figure()
    plt.subplot(1,2,1)
    plt.imshow(color_im)
    plt.subplot(1,2,2)
    plt.imshow(depth_im)
    plt.show()    
    np.save('tests/data/rgb.npy', color_im)
    np.save('tests/data/depth.npy', depth_im)

if __name__ == "__main__":
    main()
