#!/usr/bin/env python3

import sys
import os
import re

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import rospy
import src.sock_connection as sock_connection
from igus_fruit_packaging.msg import RobotFeedback


class AliveDecode():
    def __init__(self, sock):
        super(AliveDecode, self).__init__()
        rospy.init_node("igus_alive")
        messageAliveJog = "CRISTART 1234 ALIVEJOG 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 CRIEND"
        encodedAliveJog = messageAliveJog.encode('utf-8')
        self.arrayAliveJog = bytearray(encodedAliveJog)
        self.sock = sock
        # init the publisher
        self.pub = rospy.Publisher(
            'actual_position', RobotFeedback, queue_size=10)
        # cartesian position
        self.cartesian_position = None
        # digital input
        self.din = []

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
                # rospy.loginfo(f"self.din is {self.din}")
                # rospy.loginfo(
                #     f"self.cartesian_position is {self.cartesian_position}")
                # bot_status.digital_input = [1.0, 2.0]
                if len(self.din) == 0:
                    bot_status.digital_input = [-1]
                else:
                    bot_status.digital_input = self.din
                bot_status.x = 0.0
                bot_status.y = 0.0
                bot_status.z = 100.0
                self.pub.publish(bot_status)

            except:
                pass

            # shutdown the thread
            rospy.sleep(0.05)


if __name__ == "__main__":
    print("start the igus alive.")
    sock_connection.init()
    client = AliveDecode(sock_connection.sock)
    client.run()
    sock_connection.sock.close()
    rospy.loginfo("Complete the task!")
