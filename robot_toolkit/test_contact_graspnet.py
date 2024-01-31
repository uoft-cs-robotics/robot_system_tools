import numpy as np
import open3d as o3d
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.inference.contact_graspnet import ContactGraspNet as cgn
import robot_arm_algos.src.inference._contact_graspnet_config_utils as config_utils


test_config = {
    "use_depth" : False,
    "checkpoint_dir" : "/root/git/scratchpad/scene_test_2048_bs3_hor_sigma_0025"
}

def main(): 
    rs_camera = RealSenseCamera()
    global_config = config_utils.load_config(test_config["checkpoint_dir"], batch_size=1, arg_configs=[])
    cgn_ = cgn(global_config, test_config["checkpoint_dir"] )
    grasps, scores, contact_points = cgn_.generate_grasps(rs_camera, use_depth_for_seg = test_config["use_depth"])
    
    # sorted_grasps = [x for _, x in sorted(zip(grasps[255.0], scores[255.0])) ]
    sorted_grasps = grasps[255.0][np.argsort(scores[255.0])]
    plot_grasps = sorted_grasps[:3]
    plot_grasp_frames = []
    for plot_grasp in plot_grasps:
        frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(plot_grasp)
        plot_grasp_frames.append(frame_mesh)
        
    rgb_image, depth_image = rs_camera.get_current_rgbd_frames()
    pcd = rs_camera.get_pointcloud_rgbd(rgb_image, depth_image)
    plot_grasp_frames.append(pcd)
    o3d.visualization.draw_geometries(plot_grasp_frames)

    
    
if __name__ == "__main__":
    main()       