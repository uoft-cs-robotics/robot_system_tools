import numpy as np
from robot_arm_algos.src.pick_and_place.object import Object
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.pick_and_place.record_grasps import RecordGrasps
from robot_arm_algos.src.robot_camera_calibration.data_collector.zmq_robot_pose_collector import ZmqRobotPoseCollector
from robot_arm_algos.src.inference.megapose6d import MegaPose6D

test_config = {
    "zmq_ip": "192.168.0.3",
    "zmq_port": "2233",
    "extrinsics_file" : "robot_arm_algos/robot_camera_extrinsics.yaml",    
    "grasps_file": "tests/data/grasps_powerdrill.yaml",    
    "camera_in_hand": True,    
    "object_name" : "powerdrill",
    "object_dir_path_base": "robot_arm_algos/src/inference/objects/" 
}


def main():
    
    obj_dir_path = test_config["object_dir_path_base"] + test_config["object_name"]
    megapose = MegaPose6D(object_name = test_config["object_name"],
                          obj_dir_path = obj_dir_path)
    
    

    object_ = Object(object_name = test_config["object_name"],
                    tag = None,
                    pose_estimator = megapose)          
    camera = RealSenseCamera()
    zmq_robot_pose_collector = ZmqRobotPoseCollector(test_config["zmq_ip"], test_config["zmq_port"])
    rg = RecordGrasps(object = object_,
                      camera = camera,
                      extrinsics_file = test_config["extrinsics_file"],
                      robot_pose_collector = zmq_robot_pose_collector,
                      camera_in_hand = test_config["camera_in_hand"],
                      output_file = test_config["grasps_file"])
    rg.run_grasp_recorder()

if __name__ == "__main__":
    main()      