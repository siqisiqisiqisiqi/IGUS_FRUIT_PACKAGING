#!/usr/bin/env python3

import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)

import rospy
import yaml
import socket
import numpy as np
from std_msgs.msg import String, Int16

from zed_3D_detection.msg import Box3d


class IgusController():
    def __init__(self):
        rospy.init_node("igus_controller")
        # Init corners subscribers
        rospy.Subscriber("/corners_test", Box3d, self.get_corners_data)
        # from world frame to robot frame
        self.M = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        self.T = np.array([[0.205], [-0.936], [0]])
        # distance between the peach upper face center and tools
        self.z_offset = 0.15
        # system frequency
        self.rate = rospy.Rate(10)

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
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    client = IgusController()
    client.run()
