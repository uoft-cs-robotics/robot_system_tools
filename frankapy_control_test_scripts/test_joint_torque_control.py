import numpy as np

from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import  JointPositionSensorMessage, ShouldTerminateSensorMessage, TorqueControllerSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk

import rospy

from control_test_utils import *
from control_test_utils import SynchMessages

test_config = {
    "with_gripper" : False
}

if __name__ == "__main__":
    fa = FrankaArm(with_gripper = test_config["with_gripper"])
    fa.reset_joints()
    synch_messages_object = SynchMessages()
    
    rospy.loginfo('Generating Trajectory')
    joints_0 = fa.get_joints()

    T = 5
    dt = 0.001
    ts = np.arange(0, T, dt)  
    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)

    rospy.loginfo('Publishing joints trajectory...')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.apply_joint_torques([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=T, buffer_time=10)

    rospy.loginfo('Try moving the robot now...')
    init_time = rospy.Time.now().to_time()
    for i in range(2, len(ts)):   
        joint_torque_proto_msg =  TorqueControllerSensorMessage(
            id=i, timestamp=rospy.Time.now().to_time() - init_time, 
            joint_torques_cmd=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        ros_msg = make_sensor_group_msg(           
            feedback_controller_sensor_msg=sensor_proto2ros_msg(
                joint_torque_proto_msg, SensorDataMessageType.JOINT_TORQUE)                
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
    qs_cmd = []; qs_measured = []
    dqs_cmd = []; dqs_measured = []

    cmds_q, robot_qs = synch_messages_object.get_synched_messages()
    for cmd, measured_state in zip(cmds_q, robot_qs):
        deserialized_msg =  TorqueControllerSensorMessage.FromString(cmd.feedbackControllerSensorData.sensorData)
        print(deserialized_msg.joint_torques_cmd)
        qs_cmd.append(deserialized_msg.joint_torques_cmd)
        qs_measured.append(measured_state.q)
        dqs_cmd.append(measured_state.dq_d)
        dqs_measured.append(measured_state.dq)


