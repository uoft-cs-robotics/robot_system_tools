import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

import rospy
from tests_common import *
from tests_common import SynchMessages
import matplotlib.pyplot as plt
import time
import tf.transformations as tf_utils 


test_config = {
    "square_length": 0.1,
    "dt" : 0.001,
    "velocity" : 0.08,
    "cycles" : 4,
    #unused at the moment
    "roll_deg": 10, 
    "pitch_deg": 10, 
    "yaw_deg": 10
    #####
}

if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()
    synch_messages_object = SynchMessages(commands_topic_name=FC.DEFAULT_SENSOR_PUBLISHER_TOPIC)

    rospy.loginfo('Generating Trajectory')
    p0 = fa.get_pose()

    X1 = p0.translation.copy()
    X2 = p0.translation.copy()
    X1[1] -= test_config["square_length"]/2.0
    X2[1] += test_config["square_length"]/2.0
    X3 = X2.copy() 
    X3[2] -= test_config["square_length"]

    X4 = X1.copy()
    X4[2] -= test_config["square_length"]

    waypoints = generate_square_pose_traj(X1, X2, X3, X4, velocity=test_config["velocity"], 
                                        square_length=test_config["square_length"], 
                                        dt=test_config["dt"], 
                                        cycles=test_config["cycles"])

    p0.translation = waypoints[0]
    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / test_config["dt"])

    rospy.loginfo('Going to initial point...')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(p0)
    time.sleep(2.0)
    rospy.loginfo('Starting test...')
    fa.goto_pose(p0,  buffer_time=1000000000,  dynamic=True)
    init_time = rospy.Time.now().to_time()
    for i in range(1, len(waypoints)):
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, timestamp=timestamp, 
            position=waypoints[i], quaternion=p0.quaternion
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
        )
        ros_msg.header.stamp = rospy.Time.now()       
        pub.publish(ros_msg)
        rate.sleep()

    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    rospy.loginfo('Done')

    poses_cmd = []; poses_measured = []
    d_poses_cmd = []; d_poses_measured = []
    cmds, robot_states = synch_messages_object.get_synched_messages()
    print(len(cmds), len(robot_states))
    for cmd, robot_state in zip(cmds, robot_states):
        deserialized_msg = PosePositionSensorMessage.FromString(cmd.trajectoryGeneratorSensorData.sensorData )
        poses_cmd.append(deserialized_msg.position) 
        poses_measured.append(np.array([robot_state.O_T_EE[12],
                                robot_state.O_T_EE[13],
                                robot_state.O_T_EE[14]]))
    plot_ee_level(poses_measured, poses_cmd)
        
        

                        
