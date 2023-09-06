import numpy as np

from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage,  CartesianImpedanceSensorMessage
from frankapy.proto import JointImpedanceFeedbackControllerMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk

import rospy

from control_test_utils import *
from control_test_utils import SynchMessages

# test_config = {
#     "amp" : 
# }

if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()
    synch_messages_object = SynchMessages()
    
    rospy.loginfo('Generating Trajectory')
    joints_0 = fa.get_joints()

    T = 5
    dt = 0.001
    ts = np.arange(0, T, dt)
    joints_traj = [[joints_0[0],
                   joints_0[1],
                   joints_0[2],
                   joints_0[3],
                   joints_0[4],
                   joints_0[5],
                   joints_0[6]] for t in ts]
                   
    # z_stiffness_traj = [min_jerk(0, 10, t, T) for t in ts]
    
    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)

    rospy.loginfo('Publishing joints trajectory...')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_joints(joints_traj[1], duration=T, dynamic=True, buffer_time=10, use_impedance=False, block=False)
    init_time = rospy.Time.now().to_time()
    print(FC.DEFAULT_TRANSLATIONAL_STIFFNESSES)
    for i in range(2, len(ts)):
        # print(joints_traj[i][3])
        traj_gen_proto_msg = JointPositionSensorMessage(
            id=i, timestamp=rospy.Time.now().to_time() - init_time, 
            joints=joints_traj[i]
        )
        fb_ctrlr_proto = CartesianImpedanceSensorMessage(
            id=i, timestamp=rospy.Time.now().to_time() - init_time,
            translational_stiffnesses=[0,0,0],#FC.DEFAULT_TRANSLATIONAL_STIFFNESSES[:2] + [z_stiffness_traj[i]],
            rotational_stiffnesses=[0,0,0]#FC.DEFAULT_ROTATIONAL_STIFFNESSES
        )        
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(
                fb_ctrlr_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)                
        )
        ros_msg.header.stamp = rospy.Time.now()
        # rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
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
        deserialized_msg = JointPositionSensorMessage.FromString(cmd.trajectoryGeneratorSensorData.sensorData )
        deserialized_msg_stiffnes_z = CartesianImpedanceSensorMessage.FromString(cmd.feedbackControllerSensorData.sensorData)
        # print(deserialized_msg.joints[3])
        print(deserialized_msg_stiffnes_z)
        qs_cmd.append(deserialized_msg.joints)
        qs_measured.append(measured_state.q)
        dqs_cmd.append(measured_state.dq_d)
        dqs_measured.append(measured_state.dq)
    plot_joint_level(qs_real=qs_measured,
                    qs_commmanded=qs_cmd, 
                    dqs_real=dqs_measured, 
                    dqs_commanded=dqs_cmd)


    # robot_states = synch_messages_object.get_non_synched_messages()

    # for measured_state in robot_states: 
    #     qs_cmd.append(measured_state.q_d)
    #     qs_measured.append(measured_state.q)
    #     dqs_cmd.append(measured_state.dq_d)
    #     dqs_measured.append(measured_state.dq)
    # plot_joint_level(qs_real=qs_measured,
    #                 qs_commmanded=qs_cmd, 
    #                 dqs_real=dqs_measured, 
    #                 dqs_commanded=dqs_cmd)