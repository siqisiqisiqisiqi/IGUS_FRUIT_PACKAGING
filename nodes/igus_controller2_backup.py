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

from zed_3D_detection.msg import Box3d
from src.igus_driver2 import IgusDriverEncoder
from igus_fruit_packaging.msg import RobotFeedback
from src.dynamic_grasp import dynamic_path_estimation


class IgusController():
    def __init__(self):
        rospy.init_node("igus_controller")
        # Init corners subscribers
        rospy.Subscriber("/corners_test", Box3d, self.get_corners_data)
        # Init robot feedback subscribers
        rospy.Subscriber("/actual_position", RobotFeedback,
                         self.get_robot_data)
        # Init container type subscriber
        rospy.Subscriber("/box_type", String, self.get_container_type)
        # Init igus message publisher to control the robot
        self.robot_pub = rospy.Publisher("igus_message", String, queue_size=10)
        # Init container status publisher to show on the gui
        self.sys_pub = rospy.Publisher("system_status", String, queue_size=10)
        # Init igus driver
        self.encoder = IgusDriverEncoder()
        # Init the container type
        self.M = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        # self.T = np.array([[-57.53], [42.26], [-30]])
        self.T = np.array([[-70], [53], [-30]])
        # distance between the peach upper face center and tools
        self.z_offset = 27
        # system home in millimeter
        self.home = np.array([0, 0, 250])
        self.actual_pos = [0, 0, 0]
        # system frequency
        self.rate = rospy.Rate(10)
        # gripper trigger to identify if it is successfully picking up the peach
        self.gripper_flag = False
        # init the corner data
        self.corner_data = None
        # Init the container info
        self.box_type = None
        self.position = None
        self.capacity = None
        # Reset status
        self.reset = False

    def get_container_type(self, data):
        container_type = data.data
        if container_type == "Reset":
            self.reset = True
        else:
            self.reset = False
            rospy.loginfo(f"container type is {container_type}.")
            # load the box info
            self.box_type = container_type
            path_file = f"{self.box_type}.yaml"
            with open(f"{parent}/config/{path_file}", "r") as f:
                box_info = yaml.safe_load(f)
            self.position = box_info["Position"]
            self.capacity = box_info["Capacity"][0]

    def get_robot_data(self, data):
        self.actual_pos = [data.x, data.y, data.z]
        din = data.digital_input
        if din[0] == 22:
            self.gripper_flag = True
        else:
            self.gripper_flag = False
        # rospy.loginfo(f"the gripper flag value is {self.gripper_flag}.")

    def get_corners_data(self, data):
        corner_data = []
        try:
            for b in data.corners_data:
                self.corners = np.array(b.data).reshape((8, 3))
                corner_data.append(self.corners)
            # CHANGE THE UNIT FROM CENTIMETER TO MILLIMETER
            self.corner_data = np.array(corner_data) * 10
            self.num = self.corner_data.shape[0]
            # rospy.loginfo(f"The peach position is {self.corner_data}.")
        except:
            rospy.loginfo(f"Not detect the peach!")

    def corner_data_transformation(self):
        target_list = []
        for i in range(self.num):
            corner = self.corner_data[i]
            corner_up = corner[:4, :]
            center_up = np.mean(corner_up, axis=0,
                                keepdims=True).T  # shape (3,1)
            target = self.M @ center_up + self.T
            target[2, 0] = target[2, 0] + self.z_offset
            target_list.append(target.squeeze())
        self.target_array = np.array(target_list)
        # rospy.loginfo(
        #     f"the shape of the target_array is {self.target_array.shape}.")

    def calculate_container_space(self):
        empty_position = [x + 1 for x in range(self.capacity)]
        pp_xy_array = self.target_array
        for key, value in self.position.items():
            xy = value[:2]
            dist = norm(pp_xy_array[:, :2] - xy, axis=1)
            dist_min = np.min(dist)
            if dist_min < 50:
                idx_peach = np.argmin(dist)
                empty_position.remove(key)
                pp_xy_array = np.delete(pp_xy_array, (idx_peach), axis=0)
        return empty_position, pp_xy_array

    def robot_move(self, desired_position):
        # rospy.loginfo(f"desired_position is {desired_position}.")
        # desired position in millimeter
        move_message = self.encoder.cartesian_move(desired_position)
        # In case losing the message
        for i in range(2):
            self.robot_pub.publish(move_message)
            rospy.sleep(0.05)
        rospy.loginfo("successfully publish the data")
        rospy.sleep(0.02)
        while norm(np.array(self.actual_pos) - np.array(desired_position)) > 5:
            # rospy.loginfo(f"actual position is {self.actual_pos}")
            rospy.sleep(0.05)
        # rospy.loginfo("successfully go to the desired position.")

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

    def run(self):
        self.robot_move(self.home)
        self.gripper_open()
        while not rospy.is_shutdown():

            if self.reset == False:
                if self.corner_data is not None and self.capacity is not None:

                    self.corner_data_transformation()
                    empty_position, peach_array = self.calculate_container_space()

                    rospy.loginfo(
                        f"number of peach in box is {4-len(empty_position)}")
                    if len(empty_position) == 0:
                        self.sys_pub.publish("Done")
                        rospy.loginfo("Finish the task!")
                    else:
                        self.sys_pub.publish("Run")

                    if len(empty_position) > 0 and peach_array.shape[0] > 0:
                        target = peach_array[0]
                        if target is not None:
                            # move over the peach
                            target_offset = np.array(
                                [target[0], target[1], target[2] + 50])
                            self.robot_move(target_offset)
                            self.gripper_open()
                            # move down to the peach
                            self.robot_move(target)
                            # close the gripper to pickup the peach
                            self.gripper_close()
                            # move the robot over the container
                            p = self.position[empty_position[0]]
                            container_position = np.array(
                                [p[0], p[1], p[2] + 55])
                            self.robot_move(container_position)
                            # move the robot to the container
                            self.robot_move(self.position[empty_position[0]])
                            # open the gripper to put the peach
                            self.gripper_half_open()
                            # move the robot to the home
                            self.robot_move(self.home)
                            self.gripper_open1()
            else:
                rospy.loginfo(f"Reset the system!")
                self.robot_move(self.home)
            self.rate.sleep()

        rospy.sleep(1)
        rospy.loginfo("shutdown successfully")


if __name__ == "__main__":
    print("start the igus controller")
    client = IgusController()
    rospy.loginfo(1)
    client.run()
