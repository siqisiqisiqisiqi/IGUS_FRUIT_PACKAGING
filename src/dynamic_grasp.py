#!/usr/bin/env python3

import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import numpy as np
import rospy
import math


def constant_jerk_kinematic_model(target, home, vel, offset=50):
    '''
    unit in mm
    target is the position of the peach top surface center: np.array shape (3,)
    home is the position of the gripper: np.array shape (3,)
    vel is the velocity of the robot: float, mm/s
    t_sum is the estimation time used for the robot to complete the path
    '''
    offset = 50  # mm
    jerk = 2e5  # mm/s^3
    path_len = 2
    t_sum = 0  # s

    path = np.zeros([3, 3])
    path[0] = home
    path[1] = np.array([target[0], target[1], target[2] + offset])
    path[2] = target

    for i in range(path_len):
        dist = np.linalg.norm(path[i + 1] - path[i])
        t = math.sqrt(vel / jerk)
        d1 = 1 / 6 * jerk * (t**3)
        d2 = 1 / 3 * jerk * (t**3) + 1 / 2 * jerk * (t**3)
        d = d1 + d2
        delta_d = dist - 2 * d
        if delta_d > 0:
            t2 = delta_d / vel
            t_sum = t_sum + t2 + 4 * t
        else:
            t_sum = t_sum + 4 * (dist / (2 * jerk))**(1 / 3)

    p1 = 9.5472e-8
    p2 = -1.6314e-4
    p3 = -0.0501
    t_comp = p1 * (vel**2) + p2 * vel + p3
    t_sum = t_sum - path_len * t_comp
    return round(t_sum, 4)


def dynamic_path_estimation(peach_array, home, data_t, vel=500):
    # conveyor_vel = 43  # mm/s
    conveyor_vel = 75  # mm/s
    # conveyor_vel = 0

    # Step 1: check if the peach is in the certain region
    target_peach_list = []
    for peach in peach_array:
        y = peach[1]
        if y > 0 and y < 300:
            target_peach_list.append(peach)
    # If there is no peach in this area, don't do anything
    if len(target_peach_list) == 0:
        return None

    target_peach_array = np.array(target_peach_list)
    peach_index = np.argmin(target_peach_array, axis=0)
    target_peach = target_peach_array[peach_index[1]]

    # Step 2: estimate and plan the grasping point
    # Opt 1: grasp the peach at the center of the conveyor
    # picking_peach_goal = np.array([target_peach[0], 0, target_peach[2]])
    # t_robot = constant_jerk_kinematic_model(picking_peach_goal, home, vel)
    # y_dist = abs(target_peach[1])
    # t_conveyor = y_dist / conveyor_vel
    # seconds = rospy.get_time()
    # t_data_diff = seconds - data_t
    # t_diff = t_conveyor - t_robot - t_data_diff
    # if t_diff < 0:
    #     rospy.loginfo("Can't reach the peach!")
    #     return None
    # rospy.sleep(t_diff)
    # return picking_peach_goal

    # Opt 2: grasp the peach with minimum time
    # rospy.loginfo(f"target peach value is {target_peach}.")
    t_array = np.zeros(11)
    a = target_peach.reshape([1, -1])
    picking_peach_array = np.repeat(a, 11, axis=0)
    picking_peach_array[:, 1] = np.linspace(-150, 150, num=11)
    for index in range(11):
        picking_peach = picking_peach_array[index]
        t_robot = constant_jerk_kinematic_model(picking_peach, home, vel)
        y_dist = target_peach[1] - picking_peach[1]
        if y_dist < 0:
            t_array[index] = 1e4
            continue
        t_conveyor = y_dist / conveyor_vel
        t_sum = t_robot + t_conveyor
        t_array[index] = t_sum
    min_index = np.argmin(t_array)
    picking_peach = picking_peach_array[min_index]
    t_robot = constant_jerk_kinematic_model(picking_peach, home, vel)
    y_dist = target_peach[1] - picking_peach[1]
    if y_dist < 0:
        return None
    t_conveyor = y_dist / conveyor_vel
    seconds = rospy.get_time()
    t_data_diff = seconds - data_t
    t_diff = t_conveyor - t_robot - t_data_diff
    if t_diff > 0:
        rospy.sleep(t_diff)
    return picking_peach
