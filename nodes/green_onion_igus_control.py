#!/usr/bin/env python3

import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import yaml
import rospy
import numpy as np
from numpy.linalg import norm
from std_msgs.msg import String

from green_onion_perception.msg import Keypoints
from src.igus_driver2 import IgusDriverEncoder
from igus_fruit_packaging.msg import RobotFeedback


class IgusController():
    def __init__(self):
        rospy.init_node("igus_controller")

        # Init green onion position subscribers
        rospy.Subscriber("/kpts_pose", Keypoints, self.get_kpts_data)

        # Init robot feedback subscribers
        rospy.Subscriber("/actual_position", RobotFeedback,
                         self.get_robot_data)

        # Init igus message publisher to control the robot
        self.robot_pub = rospy.Publisher("igus_message", String, queue_size=10)

        # Init igus driver
        self.encoder = IgusDriverEncoder()

        # Init the transformation between robot frame and world frame
        self.M = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.T = np.array([[54], [70], [0]])
        self.z_offset = 100

        # system home in millimeter
        self.home = np.array([0, 0, 250])
        # robot actual position
        self.actual_pos = [0, 0, 0]
        # system frequency
        self.rate = rospy.Rate(10)
        # gripper trigger to identify if it is successfully picking up the peach
        self.gripper_flag = False

    def get_kpts_data(self, data):
        grasp_points = []
        grasp_yaw = []
        try:
            for pose in data.grap_pose:
                grasp_point = [pose.x, pose.y, pose.z]
                grasp_points.append(grasp_point)
                grasp_yaw.append(pose.yaw)
            # change the unit from meter to millimeter
            self.grasp_points = np.array(grasp_points) * 1e3
            self.grasp_yaw = np.array(grasp_yaw)
            self.num = data.num
        except:
            rospy.loginfo("Can't detect the green onion")

    def get_robot_data(self, data):
        self.actual_pos = [data.x, data.y, data.z]
        din = data.digital_input
        if din[0] == 22:
            self.gripper_flag = True
        else:
            self.gripper_flag = False

    def data_coordinate_transformation(self):
        target_list = []
        for i in range(self.num):
            grasp_point = self.grasp_points[i].reshape(3, 1)
            target = self.M @ grasp_point + self.T
            target[2, 0] = target[2, 0] + self.z_offset
            target_list.append(target.squeeze())
        target_array = np.array(target_list)
        return target_array

    def robot_move(self, desired_position):
        # desired position in millimeter
        move_message = self.encoder.cartesian_move(desired_position)
        # In case losing the message
        for i in range(2):
            self.robot_pub.publish(move_message)
            rospy.sleep(0.05)
        rospy.sleep(0.02)
        while norm(np.array(self.actual_pos) - np.array(desired_position)) > 5:
            rospy.sleep(0.05)

    def gripper_open(self):
        _, message2 = self.encoder.gripper(0, 0)
        self.robot_pub.publish(message2)

    def gripper_open1(self):
        message1, _ = self.encoder.gripper(0, 1)
        self.robot_pub.publish(message1)

    def gripper_half_open(self):
        _, message2 = self.encoder.gripper(1, 1)
        self.robot_pub.publish(message2)

    def gripper_close(self):
        message1, _ = self.encoder.gripper(1, 0)
        self.robot_pub.publish(message1)
        while self.gripper_flag == False:
            continue

    def pick_and_place(self, target):
        target_offset = np.array(
            [target[0], target[1], target[2] + 50])
        self.robot_move(target_offset)
        self.gripper_open()
        target[2] = target[2] - 7
        self.robot_move(target)
        self.gripper_close()
        rospy.sleep(0.5)
        self.robot_move(self.home)
        self.robot_move(np.array([-190, -80, 140]))
        self.gripper_half_open()
        rospy.sleep(0.4)
        self.robot_move(self.home)
        self.gripper_open1()

    def run(self):
        rospy.loginfo(
            f"Robot will start in 3 seconds, make sure no one is around!")
        rospy.sleep(3)

        self.robot_move(self.home)
        self.gripper_open()

        while not rospy.is_shutdown():
            target_array = self.data_coordinate_transformation()
            rospy.loginfo(f"The target array is {target_array}.")

            for target in target_array:
                self.pick_and_place(target)
            break

            self.rate.sleep()


if __name__ == "__main__":
    print("start the igus controller")
    client = IgusController()
    rospy.sleep(1)
    client.run()
