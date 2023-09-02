#!/usr/bin/env python
from jiaming_task_planner import GMM

import numpy as np
import os
import sys
import rospkg
import rospy
import moveit_commander
from tqdm import tqdm
import matplotlib.pyplot as plt


if __name__ == "__main__":

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("arm")
    active_joint_names = move_group.get_active_joints()

    rospack = rospkg.RosPack()

    # Get the path of the desired package
    task_planner_package_path = rospack.get_path('task_planner')
    gmm_dir_path = task_planner_package_path + '/computed_gmms_dir/dpgmm/'
    gmm = GMM()
    gmm.load_distributions(gmm_dir_path)
    print(gmm._sklearn_gmm.weights_.shape)

    gmm_dataset_dir_path = '/tmp/'

    # load valid robot states
    valid_robot_states = np.load(gmm_dataset_dir_path + 'valid_robot_states.npy')

    # load robot joint names
    robot_joint_names = np.load(gmm_dataset_dir_path + 'joint_names.npy')

    # # get the index of active joint names in robot_joint_names
    active_joint_names_index = [robot_joint_names.tolist().index(active_joint_name) for active_joint_name in active_joint_names]

    # filter the valid robot states based on the active joint names index
    active_valid_robot_states = valid_robot_states[:, active_joint_names_index]

    # print active_valid_robot_states.shape

    gmm_id_of_valid_robot_states = gmm._sklearn_gmm.predict(active_valid_robot_states).tolist()

    # print gmm_id_of_valid_robot_states

    # find all files start with env_
    valid_count_dic = {}
    total_count_dic = {}
    # load the valid_tag.npy in it
    valid_tag = np.load('/tmp/valid_tag.npy').tolist()
    plt.hist(valid_tag)
    plt.show()
    for gmm_id, tag in zip(gmm_id_of_valid_robot_states, valid_tag):
        if gmm_id not in total_count_dic:
            total_count_dic[gmm_id] = 1
            if tag == 1:
                valid_count_dic[gmm_id] = 1
            else:
                valid_count_dic[gmm_id] = 0
        else:
            total_count_dic[gmm_id] += 1

            if tag == 1:
                valid_count_dic[gmm_id] += 1

    num_dist = len(gmm._sklearn_gmm.weights_)
    out_npy = np.zeros((num_dist))
    for i in range(num_dist):
        if i in valid_count_dic:
            # print(float(valid_count_dic[i])/ float(total_count_dic[i]), float(valid_count_dic[i]), float(total_count_dic[i]))
            out_npy[i] = float(valid_count_dic[i])/ float(total_count_dic[i])
        else:
            print(i)
            out_npy[i] = 0.0

    np.save("/tmp/gmm_weights.npy", out_npy)
    print(out_npy)
    plt.plot(out_npy)
    plt.show()
        # save the valid_count_dic and total_count_dic
        # np.save(gmm_dataset_dir_path + file_path + '/valid_count_dic.npy', valid_count_dic)
        # np.save(gmm_dataset_dir_path + file_path + '/total_count_dic.npy', total_count_dic)