#!/usr/bin/env python3

import socket
import re
from numpy.linalg import norm
import numpy as np
import time
import rospy
from threading import Thread, Event

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Enter the IP address of the robot (192.168.3.11) here if you're not using a CPRog/iRC simulation
server_address = ('192.168.3.11', 3920)
print("Connecting...")
sock.connect(server_address)
print("Connected")


class AliveDecode(Thread):
    def __init__(self, event):
        super(AliveDecode, self).__init__()

        self.event = event
        # The ALIVEJOG message needs to be sent regularly (at least once a second) to keep the connection alive
        messageAliveJog = "CRISTART 1234 ALIVEJOG 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 CRIEND"
        encodedAliveJog = messageAliveJog.encode('utf-8')
        self.arrayAliveJog = bytearray(encodedAliveJog)

        # cartesian position
        self.cartesian_position = None
        # digital input
        self.din = []

    def run(self):
        global sock
        while True:
            sock.sendall(self.arrayAliveJog)
            data = sock.recv(1024).decode()
            try:
                result1 = re.findall(r'DIN \d+', data, re.DOTALL)
                a = result1[0].split()
                din = int(a[1])
                self.din = [i + 1 for i,
                            j in enumerate(bin(din)[:1:-1]) if j == "1"]
            except:
                pass
            try:
                result2 = re.findall(
                    r'POSCARTROBOT(?: \-?\d+\.?\d+){3}', data, re.DOTALL)
                b = result2[0].split()
                self.cartesian_position = [float(ele) for ele in b[1:]]
            except:
                pass
            # shutdown the thread
            if self.event.is_set():
                break
            rospy.sleep(0.05)


class IgusDriver():
    def __init__(self, sock):
        rospy.init_node("igus_driver")
        self.stop_event = Event()
        self.alive_client = AliveDecode(self.stop_event)
        self.sock = sock
        self.rate = rospy.Rate(10)
        rospy.sleep(1)
        self.alive_client.start()

    def cartesian_move(self, position):
        x = position[0]
        y = position[1]
        z = position[2]
        message = f"CRISTART 1234 CMD Move Cart {x} {y} {z} 0 0 0 0 0 0 100 CRIEND"
        encoded = message.encode('utf-8')
        move_array = bytearray(encoded)
        self.sock.sendall(move_array)
        actual_pos = self.alive_client.cartesian_position
        while norm(np.array(actual_pos) - np.array(position)) > 10:
            actual_pos = self.alive_client.cartesian_position
            rospy.sleep(0.1)
        rospy.loginfo("Successfully reach the point.")

    def gripper(self, D22=1, D23=1):
        if D22 == 0:
            message = "CRISTART 1234 CMD DOUT 22 false CRIEND"
        else:
            message = "CRISTART 1234 CMD DOUT 22 true CRIEND"
        encoded = message.encode('utf-8')
        Dout_array = bytearray(encoded)
        self.sock.sendall(Dout_array)

        if D23 == 0:
            message = "CRISTART 1234 CMD DOUT 23 false CRIEND"
        else:
            message = "CRISTART 1234 CMD DOUT 23 true CRIEND"
        encoded = message.encode('utf-8')
        Dout_array = bytearray(encoded)
        self.sock.sendall(Dout_array)

    def shutdown(self):
        self.stop_event.set()
        self.alive_client.join()
        self.sock.close()
        rospy.sleep(0.5)
        exit()


if __name__ == "__main__":
    driver = IgusDriver(sock)
