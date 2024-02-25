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


def dynamic_path_estimation(peach_array, home, data_t, vel=700):
    conveyor_vel = 43  # mm/s

    # Step 1: check if the peach is in the certain region
    target_peach = None
    for peach in peach_array:
        y = peach[1]
        if y > 100 and y < 300:
            target_peach = peach
            break
    # If there is no peach in this area, don't do anything
    if target_peach is None:
        return None
    # Step 2: estimate and plan the grasping point
    # Opt 1: grasp the peach at the center of the conveyor
    picking_peach_goal = np.array([target_peach[0], 0, target_peach[2]])
    t_robot = constant_jerk_kinematic_model(picking_peach_goal, home, vel)
    y_dist = abs(target_peach[1])
    t_conveyor = y_dist / conveyor_vel
    seconds = rospy.get_time()
    t_data_diff = seconds - data_t
    rospy.loginfo(f"t data difference is {t_data_diff}.")
    t_diff = t_conveyor - t_robot - t_data_diff
    if t_diff < 0:
        rospy.loginfo("Can't reach the peach!")
        return None
    rospy.sleep(t_diff)
    return picking_peach_goal

    # Opt 2: grasp the peach with minimum time
