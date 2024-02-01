#!/usr/bin/env python3

import sys
import os
import re

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import rospy
import socket
from std_msgs.msg import String
from igus_fruit_packaging.msg import RobotFeedback


class IgusCommunication():
    def __init__(self, sock):
        super(IgusCommunication, self).__init__()

        rospy.init_node("igus_communication")
        # alive message
        messageAliveJog = "CRISTART 1234 ALIVEJOG 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 CRIEND"
        encodedAliveJog = messageAliveJog.encode('utf-8')
        self.arrayAliveJog = bytearray(encodedAliveJog)
        # init sock
        self.sock = sock
        # init the publisher
        self.pub = rospy.Publisher(
            'actual_position', RobotFeedback, queue_size=10)
        # init the subscriber
        rospy.Subscriber("/igus_message", String, self.message_callback)
        # cartesian position
        self.cartesian_position = None
        # digital input
        self.din = []
        # alive message frequency
        self.rate = rospy.Rate(20)

    def message_callback(self, data):
        # rospy.loginfo(f"receive the data is {data.data}")
        move_message = data.data
        encoded = move_message.encode('utf-8')
        move_array = bytearray(encoded)
        self.sock.sendall(move_array)

    def run(self):
        while not rospy.is_shutdown():
            self.sock.sendall(self.arrayAliveJog)
            data = self.sock.recv(1024).decode()
            bot_status = RobotFeedback()
            try:
                result1 = re.findall(r'DIN \d+', data, re.DOTALL)
                a = result1[0].split()
                din = int(a[1])
                self.din = [i + 1 for i,
                            j in enumerate(bin(din)[:1:-1]) if j == "1"]
                result2 = re.findall(
                    r'POSCARTROBOT(?: \-?\d+\.?\d+){3}', data, re.DOTALL)
                b = result2[0].split()
                self.cartesian_position = [float(ele) for ele in b[1:]]
                if len(self.din) == 0:
                    bot_status.digital_input = [-1]
                else:
                    bot_status.digital_input = self.din
                bot_status.x = self.cartesian_position[0]
                bot_status.y = self.cartesian_position[1]
                bot_status.z = self.cartesian_position[2]
                self.pub.publish(bot_status)
            except:
                pass
            self.rate.sleep()


if __name__ == "__main__":
    print("start the igus alive.")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = ('192.168.3.11', 3920)
    print("Connecting...")
    sock.connect(server_address)
    print("Connected")

    client = IgusCommunication(sock)

    client.run()
    sock.close()
    rospy.loginfo("Complete the task!")
