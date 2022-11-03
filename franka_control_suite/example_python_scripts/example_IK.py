#!/usr/bin/env python3
import zmq 
import numpy as np
import tf.transformations as tr
import copy
def main():
    context = zmq.Context() 
    command_pub_socket = context.socket(zmq.PUB)
    command_pub_socket.bind("tcp://127.0.0.1:2069")

    ee_state_sub_socket = context.socket(zmq.SUB)
    ee_state_sub_socket.setsockopt(zmq.CONFLATE, True)
    ee_state_sub_socket.connect("tcp://127.0.0.1:2096")
    ee_state_sub_socket.setsockopt(zmq.SUBSCRIBE, b'')

    message = ee_state_sub_socket.recv()
    ee_now = np.frombuffer(message).astype(np.float32)
    ee_now_mat = np.reshape(ee_now, (4,4), order='F')
    current_xyz = ee_now_mat[:3,-1]  
    first_xyz = copy.copy(current_xyz)
    q = tr.quaternion_from_matrix(ee_now_mat)

    send_data = np.zeros(7)
    goal_1_xyz = np.array([ee_now_mat[0,-1],
                        ee_now_mat[1,-1] + 0.3,
                        ee_now_mat[2,-1]])
    while(np.linalg.norm(goal_1_xyz - current_xyz) > 0.05):
        send_data[3:] = q 
        send_data[0:3] = goal_1_xyz
        send_data = np.array(send_data).astype(np.float64)
        command_pub_socket.send(send_data.tobytes())
        message = ee_state_sub_socket.recv()
        ee_now = np.frombuffer(message).astype(np.float32)
        ee_now_mat = np.reshape(ee_now, (4,4), order='F')
        current_xyz = ee_now_mat[:3,-1]    


    send_data = np.zeros(7)
    goal_1_xyz = np.array([first_xyz[0],
                        first_xyz[1] + 0.3,
                        first_xyz[2] + 0.2])
    while(np.linalg.norm(goal_1_xyz - current_xyz) > 0.05):
        send_data[3:] = q 
        send_data[0:3] = goal_1_xyz
        send_data = np.array(send_data).astype(np.float64)
        command_pub_socket.send(send_data.tobytes())
        message = ee_state_sub_socket.recv()
        ee_now = np.frombuffer(message).astype(np.float32)
        ee_now_mat = np.reshape(ee_now, (4,4), order='F')
        current_xyz = ee_now_mat[:3,-1]


    send_data = np.zeros(7)
    goal_1_xyz = np.array([first_xyz[0],
                        first_xyz[1],
                        first_xyz[2] + 0.2])
    while(np.linalg.norm(goal_1_xyz - current_xyz) > 0.05):
        send_data[3:] = q 
        send_data[0:3] = goal_1_xyz
        send_data = np.array(send_data).astype(np.float64)
        command_pub_socket.send(send_data.tobytes())
        message = ee_state_sub_socket.recv()
        ee_now = np.frombuffer(message).astype(np.float32)
        ee_now_mat = np.reshape(ee_now, (4,4), order='F')
        current_xyz = ee_now_mat[:3,-1]        


    send_data = np.zeros(7)
    goal_1_xyz = np.array([first_xyz[0]+0.2,
                        first_xyz[1],
                        first_xyz[2]])
    while(np.linalg.norm(goal_1_xyz - current_xyz) > 0.05):
        send_data[3:] = q 
        send_data[0:3] = goal_1_xyz
        send_data = np.array(send_data).astype(np.float64)
        command_pub_socket.send(send_data.tobytes())
        message = ee_state_sub_socket.recv()
        ee_now = np.frombuffer(message).astype(np.float32)
        ee_now_mat = np.reshape(ee_now, (4,4), order='F')
        current_xyz = ee_now_mat[:3,-1]   


    send_data = np.zeros(7)
    goal_1_xyz = np.array([first_xyz[0],
                        first_xyz[1],
                        first_xyz[2]])
    while(np.linalg.norm(goal_1_xyz - current_xyz) > 0.05):
        send_data[3:] = q 
        send_data[0:3] = goal_1_xyz
        send_data = np.array(send_data).astype(np.float64)
        command_pub_socket.send(send_data.tobytes())
        message = ee_state_sub_socket.recv()
        ee_now = np.frombuffer(message).astype(np.float32)
        ee_now_mat = np.reshape(ee_now, (4,4), order='F')
        current_xyz = ee_now_mat[:3,-1]   
if __name__ == "__main__":
    main()

