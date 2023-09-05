#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
file_name_leader = "/home/ruthrash/Desktop/log_leader.txt"
file_name_follower = "/home/ruthrash/Desktop/log_follower.txt"


def read_logs_and_store_dictionary(file_name:str, joint_space_flag=True):
    logs_dict = dict() 
    file_ = open(file_name, 'r')
    lines = file_.readlines()
    if (joint_space_flag):
        logs_dict['q_now'] = list(); logs_dict['dq_now'] = list(); 
        logs_dict['q_desired'] = list(); logs_dict['dq_desired'] = list(); 
        for line in lines: 
            line = line.strip()
            datas = line.split(',')
            q_now = list(map(np.float64,datas[:7]))
            q_desired = list(map(np.float64, datas[7:14]))
            dq_now = list(map(np.float64, datas[14:21]))
            dq_desired = list(map(np.float64, datas[21:28]))
            logs_dict['q_now'].append(q_now)
            logs_dict['q_desired'].append(q_desired)
            logs_dict['dq_now'].append(dq_now)
            logs_dict['dq_desired'].append(dq_desired)
    else: 
        logs_dict['position_now'] = list(); logs_dict['position_desired'] = list(); 
        logs_dict['twist_now'] = list(); logs_dict['twist_desired'] = list(); 
        for line in lines:
            line =line.strip()
            datas = line.split(',')
            position_now = list(map(np.float64,datas[:3]))
            position_desired = list(map(np.float64,datas[3:6]))
            twist_now = list(map(np.float64,datas[6:12]))
            twist_desired = list(map(np.float64,datas[12:18]))
            logs_dict['position_now'].append(position_now)
            logs_dict['position_desired'].append(position_desired)
            logs_dict['twist_now'].append(twist_now)
            logs_dict['twist_desired'].append(twist_desired)         

    return  logs_dict

def plot_data(data_dict_leader:dict,data_dict_follower:dict, joint_space_flag=True, velocity_flag=False):
    if joint_space_flag:
        fig, axs = plt.subplots(7,2)
        fig.suptitle('Vertically stacked joint space subplots')
        for i in range(7):
            x = []
            ynow = []; ydesired = []
            count = 0
            for q_now, q_desired in zip(data_dict_leader['q_now'], data_dict_leader['q_desired']):
                ynow.append(q_now[i])
                ydesired.append(q_desired[i])
                x.append(count)
                count += 1
            axs[i, 0].plot(x, ynow,color='r', label='leader')
            # axs[i, 0].plot(x, ydesired,color='g', label='desired')
            axs[i, 0].set_title('joint_{}'.format(i))
        for i in range(7):
            x = []
            ynow = []; ydesired = []
            count = 0
            for q_now, q_desired in zip(data_dict_follower['q_now'], data_dict_follower['q_desired']):
                ynow.append(q_now[i])
                ydesired.append(q_desired[i])
                x.append(count)
                count += 1
            axs[i, 0].plot(x, ynow,color='b', label='follower')
            # axs[i, 0].plot(x, ydesired,color='g', label='desired')
            axs[i, 0].set_title('joint_{}'.format(i))    
        for i in range(7):
            x = []
            ynow = []; ydesired = []
            count = 0
            for q_now, q_desired in zip(data_dict_leader['dq_now'], data_dict_leader['dq_desired']):
                ynow.append(q_now[i])
                ydesired.append(q_desired[i])
                x.append(count)
                count += 1
            axs[i, 1].plot(x, ynow,color='r', label='leader')
            # axs[i, 0].plot(x, ydesired,color='g', label='desired')
            axs[i, 1].set_title('joint_{}'.format(i))
        for i in range(7):
            x = []
            ynow = []; ydesired = []
            count = 0
            for q_now, q_desired in zip(data_dict_follower['dq_now'], data_dict_follower['dq_desired']):
                ynow.append(q_now[i])
                ydesired.append(q_desired[i])
                x.append(count)
                count += 1
            axs[i, 1].plot(x, ynow,color='b', label='follower')
            # axs[i, 0].plot(x, ydesired,color='g', label='desired')
            axs[i, 1].set_title('joint_{}'.format(i))                        
        # for i in range(7):
        #     x = []
        #     ynow = []; ydesired = []
        #     count = 0
        #     for dq_now, dq_desired in zip(data_dict['dq_now'], data_dict['dq_desired']):
        #         ynow.append(dq_now[i])
        #         ydesired.append(dq_desired[i])
        #         x.append(count)
        #         count += 1
        #     axs[i, 1].plot(x, ynow,color='r', label='now')
        #     axs[i, 1].plot(x, ydesired,color='g', label='desired')
        #     axs[i, 1].set_title('joint_{}'.format(i))
    else:
        fig, axs = plt.subplots(3,2)
        fig.suptitle('Vertically stacked cartesian space subplots')
        for i in range(3):
            x = []
            ynow = []; ydesired = []
            count = 0
            for p_now, p_desired in zip(data_dict['position_now'], data_dict['position_desired']):
                ynow.append(p_now[i])
                ydesired.append(p_desired[i])
                x.append(count)
                count += 1
            axs[i, 0].plot(x, ynow,color='r', label='now')
            axs[i, 0].plot(x, ydesired,color='g', label='desired')
            axs[i, 0].set_title('position')
        for i in range(3):
            x = []
            ynow = []; ydesired = []
            count = 0
            for twist_now, twist_desired in zip(data_dict['twist_now'], data_dict['twist_desired']):
                ynow.append(twist_now[i])
                ydesired.append(twist_desired[i])
                x.append(count)
                count += 1
            axs[i, 1].plot(x, ynow,color='r', label='now')
            axs[i, 1].plot(x, ydesired,color='g', label='desired')
            axs[i, 1].set_title('velocity')            
    plt.legend()
    plt.show()

joint_space_flag = True
log_dict_leader= read_logs_and_store_dictionary(file_name_leader, joint_space_flag=joint_space_flag)
log_dict_follower= read_logs_and_store_dictionary(file_name_follower, joint_space_flag=joint_space_flag)

plot_data(log_dict_leader, log_dict_follower, joint_space_flag=joint_space_flag)


