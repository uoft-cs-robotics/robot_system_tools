from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.inference.contact_graspnet import ContactGraspNet as cgn
import robot_arm_algos.src.inference._contact_graspnet_config_utils as config_utils

def main(): 
    rs_camera = RealSenseCamera()
    checkpoint_dir = "/root/git/scratchpad/scene_test_2048_bs3_hor_sigma_0025"
    global_config = config_utils.load_config(checkpoint_dir, batch_size=1, arg_configs=[])
    cgn_ = cgn(global_config, checkpoint_dir )
    cgn_.generate_grasps(rs_camera)
    
if __name__ == "__main__":
    main()       