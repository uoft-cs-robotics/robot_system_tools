import argparse
import rospy
from robot_arm_algos.src.robot_camera_calibration.robot_camera_calibrator import RobotCameraCalibrator 
from robot_arm_algos.src.robot_arm.robot_frankapy import RobotFrankaPy
from robot_arm_algos.src.logger import logger
from robot_arm_algos.src.config_reader import get_robot_camera_calib_config, read_yaml_file

def main(args):
    rospy.init_node("robot_camera_calibration")

    tag_pose_collector, robot_pose_collector, camera, config_dict = get_robot_camera_calib_config(args.config_file) 

    robot_arm_object = None
    if(config_dict["move_robot_automatically"]):
        robot_arm_object = robot_arm_object = RobotFrankaPy(init_node = False, 
                                                            with_franka_gripper = config_dict["with_gripper"])   
        # robot_arm_object.fpy_object.reset_joints() 

    robot_camera_calibrator = RobotCameraCalibrator(robot_pose_collector,
                                                    tag_pose_collector,
                                                    camera,
                                                    output_file = config_dict["output_file"],
                                                    flag_camera_in_hand = config_dict["camera_in_hand"],
                                                    collect_data_and_calibrate = not(config_dict["only_calibration"]),
                                                    move_robot_automatically = config_dict["move_robot_automatically"],
                                                    robot_arm_object = robot_arm_object,
                                                    n_data = config_dict["n_data"],
                                                    fg_data_folder_path = config_dict["fg_optim_data_file_name"],
                                                    calib_data_file_name = config_dict["calib_data_file_name"])

    robot_camera_calibrator.run_calibration()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Robot Camera Calibration')
    parser.add_argument("--config_file", 
                        required = True, 
                        type = str, 
                        help = "path to config YAML file containing parameters and arguments required for running robot camera calibration")
    args = parser.parse_args()
    main(args)