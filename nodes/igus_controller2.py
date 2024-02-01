#!/usr/bin/env python3

import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import rospy
import numpy as np
from numpy.linalg import norm
from std_msgs.msg import String

from zed_3D_detection.msg import Box3d
from src.igus_driver2 import IgusDriverEncoder
from igus_fruit_packaging.msg import RobotFeedback


class IgusController():
    def __init__(self):
        rospy.init_node("igus_controller")
        # Init corners subscribers
        rospy.Subscriber("/corners_test", Box3d, self.get_corners_data)
        # Init robot feedback subscribers
        rospy.Subscriber("/actual_position", RobotFeedback,
                         self.get_robot_data)
        # Init igus driver
        self.encoder = IgusDriverEncoder()
        #
        self.robot_pub = rospy.Publisher("igus_message", String, queue_size=10)
        # from world frame to robot frame
        self.M = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        self.T = np.array([[-0.05753], [0.04226], [0.04]])
        # distance between the peach upper face center and tools
        self.z_offset = 0.15
        # system home in millimeter
        self.home = [0, 0, 300]
        # system frequency
        self.rate = rospy.Rate(10)
        # gripper trigger to identify if it is successfully picking up the peach
        self.gripper_flag = False

    def get_robot_data(self, data):
        self.actual_pos = [data.x, data.y, data.z]
        din = data.digital_input
        if din[0] == 22:
            self.gripper_flag = True
        else:
            self.gripper_flag = False
        rospy.loginfo(f"the gripper flag value is {self.gripper_flag}.")

    def get_corners_data(self, data):
        corner_data = []
        try:
            for b in data.corners_data:
                self.corners = np.array(b.data).reshape((8, 3))
                corner_data.append(self.corners)
            # CHANGE THE UNIT FROM CENTIMETER TO meter
            self.corner_data = np.array(corner_data) / 100
            self.num = self.corner_data.shape[0]
        except:
            rospy.loginfo(f"Not detect the peach!")

    def calculate_target(self):
        for i in range(self.num):
            corner = self.corner_data[i]
            corner_up = corner[:4, :]
            center_up = np.mean(corner_up, axis=0,
                                keepdims=True).T  # shape (3,1)
            target = self.M @ center_up + self.T
            target[2, 0] = target[2, 0] + self.z_offset
            # region constraints
            if target[0, 0] > -0.1:
                return target
        return None

    def robot_move(self, desired_position):
        move_message = self.encoder.cartesian_move(desired_position)
        # In case losing the message
        for i in range(2):
            self.robot_pub.publish(move_message)
            rospy.sleep(0.1)
        rospy.loginfo("successfully publish the data")
        rospy.sleep(0.2)
        while norm(np.array(self.actual_pos) - np.array(desired_position)) > 5:
            rospy.sleep(0.05)
        rospy.loginfo("successfully go to the desired position.")

    def gripper_open(self):
        message = self.encoder.gripper(0, 0)
        self.robot_pub.publish(message)
        # rospy.sleep(0.1)
        # self.move_pub.publish(message)

    def gripper_close(self):
        message = self.encoder.gripper(1, 0)
        self.robot_pub.publish(message)
        # rospy.sleep(0.1)
        # self.move_pub.publish(message)

    def run(self):
        position = [0, 0, 150]
        while not rospy.is_shutdown():
            self.robot_move(self.home)
            self.gripper_open()
            self.robot_move(position)
            self.gripper_close()
            rospy.sleep(0.1)
            while self.gripper_flag == False:
                continue
        rospy.sleep(1)
        rospy.loginfo("shutdown successfully")


if __name__ == "__main__":
    print("start the igus controller")
    client = IgusController()
    client.run()
