import numpy as np
import matplotlib.pyplot as plt
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.inference.dope import DOPEPoseDetection as DOPE

def main(): 
    rs_camera = RealSenseCamera()
    dope_obj = DOPE(dope_config_pose_file = "config/dope_config_pose.yaml" )
    plt.imshow(rs_camera.get_current_rgb_frame())
    plt.show(block=False)
    plt.pause(4.0)
    plt.close()
    dope_obj.estimate_pose(rs_camera)
    
if __name__ == "__main__":
    main()    
    
    
#pip3 install open3d matplotlib opencv pyrealsens2 
# sudo apt-get install ros-noetic-vision-msgs