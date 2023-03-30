import numpy as np
import rospy 
import message_filters
from  franka_interface_msgs.msg import RobotState
from franka_interface_msgs.msg import SensorDataGroup
import matplotlib.pyplot as plt


def square_coordinates_yz_plane(current_pose, length=0.1):
    square_coordinates = []
    square_coordinates.append(np.array([current_pose[0], current_pose[1]-length/2, current_pose[2]]))
    square_coordinates.append(np.array([current_pose[0], current_pose[1]+length/2, current_pose[2]]))
    square_coordinates.append(np.array([current_pose[0], current_pose[1]+length/2, current_pose[2]-length]))
    square_coordinates.append(np.array([current_pose[0], current_pose[1]-length/2, current_pose[2]-length]))    
    return square_coordinates
    
def generate_sinusoidal_delta_joint_angle(time):
    delta_angle = np.pi/8.0 * (1- np.cos(np.pi/2.5 * time))
    return delta_angle 

def generate_square_pose_traj(X1, X2, X3, X4, velocity, square_length, dt=0.001):
    line1 = generate_straight_line_traj(X1, X2, velocity, square_length, dt)
    line2 = generate_straight_line_traj(X2, X3, velocity, square_length, dt)
    line3 = generate_straight_line_traj(X3, X4, velocity, square_length, dt)
    line4 = generate_straight_line_traj(X4, X1, velocity, square_length, dt)

    return line1 + line2 + line3 + line4 

def generate_straight_line_traj(X1, X2, velocity,square_length, dt=0.001): 
    alpha = 0.0
    d_alpha = (velocity*dt/square_length)
    waypoints = []    
    while(alpha <= 1.0):
        waypoints.append(X1*(1-alpha) + X2*alpha)
        alpha += d_alpha
    waypoints.append(X2)
    print("X1 ", X1, waypoints[0])
    print("X2", X2, waypoints[-1])
    return waypoints

def plot_joint_level(qs_real, qs_commmanded, dqs_real, dqs_commanded, show_plot=True): 
    assert(len(qs_real)==len(qs_commmanded))
    assert(len(dqs_real)==len(dqs_commanded))
    fig, axs = plt.subplots(7,2)
    fig.suptitle('Vertically stacked joint space subplots')
    for i in range(7):
        x_axis = []
        y_axis_measured = []; y_axis_cmd = []
        count = 0
        for q_real, q_cmd in zip(qs_real, qs_commmanded):
            y_axis_measured.append(q_real[i])
            y_axis_cmd.append(q_cmd[i])
            x_axis.append(count)
            count += 1
        axs[i, 0].plot(x_axis, y_axis_measured,color='r', label='measured')
        axs[i, 0].plot(x_axis, y_axis_cmd ,color='g', label='commaded')
        axs[i, 0].set_title('joint_{}'.format(i))
    for i in range(7):
        x_axis = []
        y_axis_measured = []; y_axis_cmd = []
        count = 0
        for dq_real, dq_cmd in zip(dqs_real, dqs_commanded):
            y_axis_measured.append(dq_real[i])
            y_axis_cmd.append(dq_cmd[i])
            x_axis.append(count)
            count += 1
        axs[i, 1].plot(x_axis, y_axis_measured, color='r', label='measured')
        axs[i, 1].plot(x_axis, y_axis_cmd, color='g', label='commaded')
        axs[i, 1].set_title('joint_{}'.format(i))    
    plt.legend()
    plt.show()

def plot_ee_level(poses_measured, poses_cmd, show_plot=True): 
    assert(len(poses_measured)==len(poses_cmd))
    fig, axs = plt.subplots(3,2)
    fig.suptitle('Vertically stacked cartesian space subplots')
    for i in range(3):
        x_axis = []
        y_axis_measured = []; y_axis_cmd = []
        count = 0
        for pose_measured, pose_cmd in zip(poses_measured, poses_cmd):
            y_axis_measured.append(pose_measured[i])
            y_axis_cmd.append(pose_cmd[i])
            x_axis.append(count)
            count += 1
        axs[i,0].plot(x_axis, y_axis_measured,color='r', label='measured')
        axs[i,0].plot(x_axis, y_axis_cmd,color='g', label='commanded')
        axs[i,0].set_title('positions')
        print(len(y_axis_measured), len(y_axis_cmd))
    plt.legend()
    plt.show()  


class SynchMessages:
    def __init__(self,
                robot_state_topic_name="/robot_state_publisher_node_1/robot_state",
                commands_topic_name="/franka_ros_interface/sensor",
                ):
      
        #synch topics based on their time stamps
        self.robot_state_sync_sub = message_filters.Subscriber(robot_state_topic_name, 
                                                        RobotState, 
                                                        queue_size=1)
        self.command_state_sync_sub = message_filters.Subscriber(commands_topic_name, 
                                                            SensorDataGroup, 
                                                            queue_size=1)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_state_sync_sub, self.command_state_sync_sub], 100,0.1) 
        self.ts.registerCallback(self.SynchCallback)

        # self.only_robot_state_sub = rospy.Subscriber(robot_state_topic_name, RobotState, self.RobotStateCB)       
        self.cmd_states = []
        self.robot_states = []
        self.non_synched_robot_states = []

    def get_synched_messages(self,):
        return self.cmd_states, self.robot_states

    def get_non_synched_messages(self,):
        return self.non_synched_robot_states

    def get_synched_q(self,):
        cmd_qs = []
        robot_qs = []
        for cmd_msg, robot_msg in zip(self.cmd_states, self.robot_states):
            cmd_qs.append(cmd_msg)
            robot_qs.append(robot_msg.q)

    def SynchCallback(self,robot_state_msg, command_state_msg):
        self.cmd_states.append(command_state_msg)
        self.robot_states.append(robot_state_msg)
        #print("yes") #uncomment to check if synching of messages works


    # def RobotStateCB(self,robot_state_msg):
    #     self.non_synched_robot_states.append(robot_state_msg)
    #     print("j 3:", robot_state_msg.q[3])
